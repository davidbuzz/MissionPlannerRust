//! Driving the bootloader.
//!
//! Ported from `ExtLibs/px4uploader/Uploader.cs` @ efb0801 (GPL-3.0-or-later).
//!
//! Generic over `Read + Write` rather than owning a serial port, so the other end of the
//! conversation can be a test. D13 requires the byte protocol proven against a mock bootloader
//! before a real board is flashed, and a type that can only talk to `/dev/ttyACM0` cannot be.

use crate::firmware::Firmware;
use crate::protocol::{
    BL_REV_MAX, BL_REV_MIN, Code, Info, PROG_MULTI_MAX, command, external_crc, get_info,
    program_multi, program_multi_external,
};
use std::io::{Read, Write};

/// Something that went wrong talking to a bootloader.
#[derive(Debug, thiserror::Error)]
pub enum UploaderError {
    /// The port could not be read or written.
    #[error("bootloader link: {0}")]
    Io(#[from] std::io::Error),
    /// A response did not begin with `INSYNC`.
    #[error("expected INSYNC, got {0:#04x}")]
    NotInSync(u8),
    /// The bootloader did not understand the command.
    #[error("the bootloader rejected the command as invalid")]
    Invalid,
    /// The bootloader understood and failed.
    #[error("the bootloader reported the operation failed")]
    Failed,
    /// The bootloader answered with something outside the protocol.
    #[error("expected OK, got {0:#04x}")]
    NotOk(u8),
    /// The bootloader speaks a revision this does not.
    #[error(
        "bootloader protocol revision {0} is outside the supported range {BL_REV_MIN}-{BL_REV_MAX}"
    )]
    UnsupportedRevision(u32),
    /// The firmware is for a different board.
    #[error("this firmware is for board {firmware}, the board reports {board}")]
    WrongBoard {
        /// What the firmware says it is for.
        firmware: u32,
        /// What the board says it is.
        board: u32,
    },
    /// The image will not fit.
    #[error("the firmware is {image} bytes and the board has room for {capacity}")]
    TooLarge {
        /// The image size.
        image: usize,
        /// What the board reports it can hold.
        capacity: usize,
    },
    /// The flash did not read back as the firmware that was written.
    #[error("the flash CRC is {reported:#010x} and the firmware's is {expected:#010x}")]
    CrcMismatch {
        /// What the firmware computes.
        expected: u32,
        /// What the board reports.
        reported: u32,
    },
    /// A block could not be framed.
    #[error("a firmware block of {0} bytes cannot be sent")]
    BadBlock(usize),
}

/// A conversation with a bootloader.
#[derive(Debug)]
pub struct Uploader<T> {
    port: T,
}

impl<T: Read + Write> Uploader<T> {
    /// Wraps a port that is already open and already at the bootloader's baud rate.
    pub const fn new(port: T) -> Self {
        Self { port }
    }

    /// Gives the port back.
    pub fn into_inner(self) -> T {
        self.port
    }

    /// Sends bytes.
    fn send(&mut self, bytes: &[u8]) -> Result<(), UploaderError> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    /// Reads exactly `count` bytes.
    fn recv(&mut self, buffer: &mut [u8]) -> Result<(), UploaderError> {
        self.port.read_exact(buffer)?;
        Ok(())
    }

    /// Reads a little-endian `i32`, as `__recv_int` does.
    fn recv_u32(&mut self) -> Result<u32, UploaderError> {
        let mut raw = [0u8; 4];
        self.recv(&mut raw)?;
        Ok(u32::from_le_bytes(raw))
    }

    /// Reads the two-byte reply that ends every command.
    ///
    /// `INSYNC` then `OK`. `INVALID` and `FAILED` are named separately because they mean different
    /// things to the caller: invalid is a bootloader too old for the command and usually has a
    /// fallback, failed is the operation genuinely not working.
    ///
    /// `// C#: ExtLibs/px4uploader/Uploader.cs:467-489`
    fn get_sync(&mut self) -> Result<(), UploaderError> {
        let mut byte = [0u8; 1];
        self.recv(&mut byte)?;
        if byte[0] != Code::InSync.byte() {
            return Err(UploaderError::NotInSync(byte[0]));
        }
        self.recv(&mut byte)?;
        match byte[0] {
            b if b == Code::Ok.byte() => Ok(()),
            b if b == Code::Invalid.byte() => Err(UploaderError::Invalid),
            b if b == Code::Failed.byte() => Err(UploaderError::Failed),
            other => Err(UploaderError::NotOk(other)),
        }
    }

    /// Asks whether anything is listening.
    pub fn sync(&mut self) -> Result<(), UploaderError> {
        self.send(&command(Code::GetSync))?;
        self.get_sync()
    }

    /// Reads one of the device info words.
    pub fn info(&mut self, info: Info) -> Result<u32, UploaderError> {
        self.send(&get_info(info))?;
        let value = self.recv_u32()?;
        self.get_sync()?;
        Ok(value)
    }

    /// Reads everything needed to decide whether a firmware can be flashed.
    pub fn identify(&mut self) -> Result<Board, UploaderError> {
        self.sync()?;
        let revision = self.info(Info::BootloaderRevision)?;
        if !(BL_REV_MIN..=BL_REV_MAX).contains(&revision) {
            return Err(UploaderError::UnsupportedRevision(revision));
        }
        Ok(Board {
            bootloader_revision: revision,
            board_id: self.info(Info::BoardId)?,
            board_revision: self.info(Info::BoardRevision)?,
            flash_size: self.info(Info::FlashSize)? as usize,
        })
    }

    /// Erases the program flash.
    ///
    /// The sync and the `GET_DEVICE` in front are not redundant: the C# carries the comment "fix
    /// for bootloader bug - must see a sync and a get_device", so some bootloaders refuse the
    /// erase without them. `// C#: ExtLibs/px4uploader/Uploader.cs:527-538`
    pub fn erase(&mut self) -> Result<(), UploaderError> {
        self.sync()?;
        let _ = self.info(Info::BootloaderRevision)?;
        self.send(&command(Code::ChipErase))?;
        self.get_sync()
    }

    /// Writes one block.
    fn program_block(&mut self, data: &[u8], external: bool) -> Result<(), UploaderError> {
        let frame = if external {
            program_multi_external(data)
        } else {
            program_multi(data)
        }
        .ok_or(UploaderError::BadBlock(data.len()))?;
        self.send(&frame)?;
        self.get_sync()
    }

    /// Writes a whole image, reporting progress as a fraction between 0 and 1.
    pub fn program(
        &mut self,
        image: &[u8],
        external: bool,
        mut progress: impl FnMut(f32),
    ) -> Result<(), UploaderError> {
        let blocks = image.chunks(PROG_MULTI_MAX);
        let total = blocks.len().max(1);
        for (index, block) in image.chunks(PROG_MULTI_MAX).enumerate() {
            self.program_block(block, external)?;
            #[allow(clippy::cast_precision_loss)] // block counts are in the thousands
            progress((index + 1) as f32 / total as f32);
        }
        Ok(())
    }

    /// Asks the board for its flash CRC and checks it against the firmware.
    ///
    /// Revision 3 and later. `// C#: ExtLibs/px4uploader/Uploader.cs:737-760`
    pub fn verify(&mut self, firmware: &Firmware, flash_size: usize) -> Result<(), UploaderError> {
        let expected = firmware.crc(flash_size);
        self.send(&command(Code::GetCrc))?;
        let reported = self.recv_u32()?;
        self.get_sync()?;
        if reported == expected {
            Ok(())
        } else {
            Err(UploaderError::CrcMismatch { expected, reported })
        }
    }

    /// Checks the external flash CRC.
    pub fn verify_external(&mut self, firmware: &Firmware) -> Result<(), UploaderError> {
        let expected = firmware.external_crc(firmware.declared_external_size);
        let length = u32::try_from(firmware.declared_external_size).unwrap_or(u32::MAX);
        self.send(&external_crc(length))?;
        let reported = self.recv_u32()?;
        self.get_sync()?;
        if reported == expected {
            Ok(())
        } else {
            Err(UploaderError::CrcMismatch { expected, reported })
        }
    }

    /// `currentChecksum`: whether the board already holds this firmware - its flash CRC over
    /// `fw_maxsize` against the file's, and the external flash's over the file's external image
    /// where the board has one. A bootloader before revision 3 cannot say, and the C# goes on to
    /// upload; that is `Ok(false)` here.
    ///
    /// # Errors
    ///
    /// The port failing: the C# tells the operator it lost communication with the board.
    /// `// C#: ExtLibs/px4uploader/Uploader.cs:806-866`
    pub fn same_firmware(
        &mut self,
        firmware: &Firmware,
        board: &Board,
    ) -> Result<bool, UploaderError> {
        if board.bootloader_revision < 3 {
            return Ok(false);
        }
        self.sync()?;
        let mut same = true;
        if board.flash_size > 0 {
            let expected = firmware.crc(board.flash_size);
            self.send(&command(Code::GetCrc))?;
            let reported = self.recv_u32()?;
            self.get_sync()?;
            if expected != reported {
                same = false;
            }
        }
        // `extf_maxsize`: `GET_DEVICE EXTF_SIZE`, which a board without external flash - or a
        // bootloader too old for the question - answers with an error the C# reads as 0.
        let external_size = self.info(Info::ExternalFlashSize).unwrap_or(0);
        if external_size > 0 {
            let length = u32::try_from(firmware.declared_external_size).unwrap_or(u32::MAX);
            let expected = firmware.external_crc(firmware.declared_external_size);
            self.send(&external_crc(length))?;
            let reported = self.recv_u32()?;
            self.get_sync()?;
            if expected != reported {
                same = false;
            }
        }
        Ok(same)
    }

    /// Leaves the bootloader and starts the firmware.
    ///
    /// The board stops answering the moment it obeys, so there is no reply to wait for - a
    /// `get_sync` here would always time out, and treating that timeout as a failure would report
    /// every successful flash as a failed one.
    /// `// C#: ExtLibs/px4uploader/Uploader.cs:652-660`
    pub fn reboot(&mut self) -> Result<(), UploaderError> {
        self.send(&command(Code::Reboot))
    }

    /// The whole sequence: identify, check, erase, program, verify, reboot.
    ///
    /// The checks come before the erase, and that ordering is the safety property. Erasing first
    /// and discovering afterwards that the firmware is for another board leaves a vehicle with no
    /// firmware at all and an operator who has to find the right file before it will fly again.
    pub fn upload(
        &mut self,
        firmware: &Firmware,
        mut progress: impl FnMut(f32),
    ) -> Result<Board, UploaderError> {
        let board = self.identify()?;
        if board.board_id != firmware.board_id {
            return Err(UploaderError::WrongBoard {
                firmware: firmware.board_id,
                board: board.board_id,
            });
        }
        if firmware.image.len() > board.flash_size {
            return Err(UploaderError::TooLarge {
                image: firmware.image.len(),
                capacity: board.flash_size,
            });
        }

        self.erase()?;
        self.program(&firmware.image, false, &mut progress)?;
        self.verify(firmware, board.flash_size)?;
        if !firmware.external_image.is_empty() {
            self.program(&firmware.external_image, true, &mut progress)?;
            self.verify_external(firmware)?;
        }
        self.reboot()?;
        Ok(board)
    }
}

/// What a board says about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Board {
    /// The bootloader protocol revision it speaks.
    pub bootloader_revision: u32,
    /// The board type, which the firmware's `board_id` must equal.
    pub board_id: u32,
    /// The board revision.
    pub board_revision: u32,
    /// How much firmware it can hold.
    pub flash_size: usize,
}
