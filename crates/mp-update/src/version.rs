//! `System.Version`, as `CheckForUpdate` reads `version.txt`'s first line, locally and from the
//! channel, and compares them. `// C#: Utilities/Update.cs:155-171`

use std::cmp::Ordering;

/// `System.Version`: `major.minor[.build[.revision]]`, a missing part below zero when compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    /// `Major`.
    pub major: i32,
    /// `Minor`.
    pub minor: i32,
    /// `Build`, -1 when absent.
    pub build: i32,
    /// `Revision`, -1 when absent.
    pub revision: i32,
}

impl Version {
    /// `new Version(text)`: two to four non-negative numbers separated by points, as .NET parses
    /// them (white space around each is allowed).
    ///
    /// # Errors
    ///
    /// .NET's `ArgumentException`/`FormatException` messages for what is wrong.
    pub fn parse(text: &str) -> Result<Self, String> {
        let parts: Vec<&str> = text.split('.').collect();
        if parts.len() < 2 || parts.len() > 4 {
            return Err("Version string portion was too short or too long.".to_owned());
        }
        let mut numbers = [-1i32; 4];
        for (slot, part) in numbers.iter_mut().zip(parts.iter()) {
            let value: i32 = part
                .trim()
                .parse()
                .map_err(|_| "Input string was not in a correct format.".to_owned())?;
            if value < 0 {
                return Err(
                    "Version's parameters must be greater than or equal to zero.".to_owned(),
                );
            }
            *slot = value;
        }
        let [major, minor, build, revision] = numbers;
        Ok(Self {
            major,
            minor,
            build,
            revision,
        })
    }
}

impl PartialOrd for Version {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Version {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.major, self.minor, self.build, self.revision).cmp(&(
            other.major,
            other.minor,
            other.build,
            other.revision,
        ))
    }
}

impl std::fmt::Display for Version {
    /// `Version.ToString()`: as many parts as were given.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)?;
        if self.build >= 0 {
            write!(f, ".{}", self.build)?;
        }
        if self.revision >= 0 {
            write!(f, ".{}", self.revision)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_parse_and_compare_as_dotnet() {
        let v = Version::parse("1.3.80.0").unwrap();
        assert_eq!(v.to_string(), "1.3.80.0");
        assert_eq!(Version::parse(" 1.3 ").unwrap().to_string(), "1.3");
        assert!(Version::parse("1.3").unwrap() < Version::parse("1.3.0").unwrap());
        assert!(Version::parse("1.3.79.1").unwrap() < Version::parse("1.3.80.0").unwrap());
        assert!(Version::parse("1.10").unwrap() > Version::parse("1.9").unwrap());
        assert_eq!(
            Version::parse("1").unwrap_err(),
            "Version string portion was too short or too long."
        );
        assert_eq!(
            Version::parse("1.2.3.4.5").unwrap_err(),
            "Version string portion was too short or too long."
        );
        assert_eq!(
            Version::parse("1.x").unwrap_err(),
            "Input string was not in a correct format."
        );
        assert_eq!(
            Version::parse("1.-2").unwrap_err(),
            "Version's parameters must be greater than or equal to zero."
        );
    }
}
