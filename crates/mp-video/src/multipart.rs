//! A multipart stream of pictures, one part per frame: what an MJPEG server sends and what
//! `CaptureMJPEG` reads, and what the GStreamer pipeline here writes (`multipartmux`, see
//! [`crate::gstreamer`]).
//!
//! Each part is headers - `Content-Type`, `Content-Length` - a blank line, `Content-Length`
//! bytes of picture and a line ending. The boundary line is read as a header line without a
//! colon and passed over, as the C#'s `getHeader` passes it over.
//! `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:56-80, 150-192, 226-246`

use std::io::{self, Read};

use crate::VideoError;

/// The longest header line read before the stream is taken to be something else: the C# reads
/// on until a line feed, which a stream out of step would not send for a long while.
const LONGEST_LINE: usize = 8 * 1024;

/// The largest part read: sixteen times a 4K RGBA frame, far past any picture a stream sends.
const LARGEST_PART: usize = 16 * 3840 * 2160 * 4;

/// `ReadLine`: bytes up to the line feed, read as Latin-1 (`(char) by`), with the line ending
/// taken off.
///
/// Divergence: the C# retries a failed read until five seconds have passed without a byte, and
/// then returns what it has; here a failed read, the end of the stream included, is an error,
/// since the reading thread's socket or pipe already times out or ends for good.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:56-80`
///
/// # Errors
///
/// The read failing, the stream ending, or a line longer than any header.
pub fn read_line(reader: &mut impl Read) -> io::Result<String> {
    let mut line = String::new();
    let mut byte = [0u8; 1];
    loop {
        reader.read_exact(&mut byte)?;
        line.push(char::from(byte[0]));
        if byte[0] == b'\n' {
            break;
        }
        if line.len() > LONGEST_LINE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "a header line with no end",
            ));
        }
    }
    // `sb.Replace("\r\n", "")`; a bare line feed is taken off too.
    let trimmed = line.trim_end_matches('\n').trim_end_matches('\r');
    Ok(trimmed.to_owned())
}

/// `getHeader`: lines up to a blank one, each split at its colons with the empty pieces dropped
/// and kept when that leaves a name and a value, both trimmed. A line with no colon (the
/// boundary) or with more than one (`http://...`) is passed over.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:226-246`
///
/// # Errors
///
/// As [`read_line`].
pub fn read_headers(reader: &mut impl Read) -> io::Result<Vec<(String, String)>> {
    let mut headers = Vec::new();
    loop {
        let line = read_line(reader)?;
        if line.is_empty() {
            return Ok(headers);
        }
        let items: Vec<&str> = line.split(':').filter(|item| !item.is_empty()).collect();
        if let [name, value] = items.as_slice() {
            headers.push((name.trim().to_owned(), value.trim().to_owned()));
        }
    }
}

/// One part: its headers, then `Content-Length` bytes, then the line that ends the data.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:150-192`
///
/// # Errors
///
/// A read failing ([`VideoError::Read`]), and the C#'s `int.Parse(getHeader(br)["Content-
/// Length"])` failing or giving nothing ("No mjpeg length header", [`VideoError::BadFrame`]):
/// either ends the C#'s loop.
pub fn read_part(reader: &mut impl Read) -> Result<Vec<u8>, VideoError> {
    let read = |why: io::Error| VideoError::Read(why.to_string());
    let headers = read_headers(reader).map_err(read)?;
    let length = headers
        .iter()
        .find(|(name, _)| name == "Content-Length")
        .and_then(|(_, value)| value.parse::<i64>().ok())
        .ok_or_else(|| VideoError::BadFrame("no Content-Length header".to_owned()))?;
    let length = usize::try_from(length)
        .ok()
        .filter(|length| *length > 0 && *length <= LARGEST_PART)
        .ok_or_else(|| VideoError::BadFrame("No mjpeg length header".to_owned()))?;
    let mut data = vec![0u8; length];
    reader.read_exact(&mut data).map_err(read)?;
    // "blank line at end of data"
    read_line(reader).map_err(read)?;
    Ok(data)
}

/// Where the first part starts in an HTTP answer whose `Content-Type` names no boundary: a
/// blank line, then lines up to the first that starts with `--`, or one of two characters or
/// fewer.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:112-125`
///
/// # Errors
///
/// As [`read_line`].
pub fn skip_to_boundary(reader: &mut impl Read) -> io::Result<()> {
    read_line(reader)?;
    loop {
        let line = read_line(reader)?;
        if line.starts_with("--") || line.chars().count() <= 2 {
            return Ok(());
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    /// What `multipartmux boundary=mpframe` writes, as gst-launch-1.0 1.24 wrote it for two
    /// 4 x 2 PNGs, the pictures cut short.
    fn two_parts() -> Vec<u8> {
        let mut stream = Vec::new();
        for picture in [&b"first"[..], &b"second!"[..]] {
            stream.extend_from_slice(b"--mpframe\r\nContent-Type: image/png\r\n");
            stream
                .extend_from_slice(format!("Content-Length: {}\r\n\r\n", picture.len()).as_bytes());
            stream.extend_from_slice(picture);
            stream.extend_from_slice(b"\r\n");
        }
        stream
    }

    #[test]
    fn parts_are_read_one_after_another() {
        let stream = two_parts();
        let mut reader = stream.as_slice();
        assert_eq!(read_part(&mut reader).unwrap(), b"first");
        assert_eq!(read_part(&mut reader).unwrap(), b"second!");
        assert!(matches!(read_part(&mut reader), Err(VideoError::Read(_))));
    }

    #[test]
    fn headers_are_split_as_get_header_splits_them() {
        let mut reader = &b"--b\r\nContent-Type: image/jpeg\r\nX-Url: http://a:1/b\r\nContent-Length:  12 \r\n\r\n"[..];
        let headers = read_headers(&mut reader).unwrap();
        assert_eq!(
            headers,
            [
                ("Content-Type".to_owned(), "image/jpeg".to_owned()),
                ("Content-Length".to_owned(), "12".to_owned()),
            ]
        );
    }

    #[test]
    fn a_part_without_a_length_ends_the_stream() {
        let mut reader = &b"--b\r\nContent-Type: image/jpeg\r\n\r\nxx\r\n"[..];
        assert_eq!(
            read_part(&mut reader).unwrap_err().to_string(),
            "bad frame: no Content-Length header"
        );
        let mut reader = &b"Content-Length: 0\r\n\r\n"[..];
        assert_eq!(
            read_part(&mut reader).unwrap_err().to_string(),
            "bad frame: No mjpeg length header"
        );
    }

    #[test]
    fn a_line_feed_alone_ends_a_line_too() {
        let mut reader = &b"a: b\nContent-Length: 1\n\nz\n"[..];
        assert_eq!(read_part(&mut reader).unwrap(), b"z");
    }

    #[test]
    fn a_line_with_no_end_is_refused() {
        let long = vec![b'x'; LONGEST_LINE + 10];
        assert!(read_line(&mut long.as_slice()).is_err());
    }

    #[test]
    fn the_first_boundary_is_found_when_the_content_type_names_none() {
        let mut reader =
            &b"\r\nproxy junk here\r\n--boundary\r\nContent-Length: 1\r\n\r\nq\r\n"[..];
        skip_to_boundary(&mut reader).unwrap();
        assert_eq!(read_part(&mut reader).unwrap(), b"q");
    }
}
