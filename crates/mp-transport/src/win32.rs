//! Windows' own record of each COM port, read as Mission Planner reads it.
//!
//! `Win32DeviceMgmt.GetAllCOMPorts` asks SetupAPI for the present devices of two interface
//! classes - USB devices, for Windows 7's virtual COM ports, then COM ports, for Windows 10 - and
//! for each whose device key has a `PortName` takes `SPDRP_DEVICEDESC` as its description,
//! `SPDRP_HARDWAREID` as its hardware id (the multi-string's first,
//! `USB\VID_2DAE&PID_1016&REV_0200&MI_00`) and `DEVPKEY_Device_BusReportedDeviceDesc` as its
//! board, the name the device reports over USB ("CubeOrange"). The `serialport` crate gives none
//! of the three: its Windows product is `SPDRP_FRIENDLYNAME` ("USB Serial Device (COM4)"), which
//! names no board, and the id built from its VID and PID has no `&` after the PID for the C#'s
//! `VID_..&PID_..&` pattern - so the Install Firmware page found no board on Windows (the owner's
//! report, 2026-09-26, with the bench CubeOrange passed through to the VM).
//!
//! FFI, so `unsafe`, which the workspace denies elsewhere: allowed for this file alone by the
//! owner's choice of 2026-09-26, each call with what makes it sound.
//! `// C#: Utilities/Win32DeviceMgnt.cs:392-620`
#![allow(unsafe_code)]

use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
    DICS_FLAG_GLOBAL, DIGCF_DEVICEINTERFACE, DIGCF_PRESENT, DIREG_DEV, HDEVINFO,
    SP_DEVINFO_DATA, SPDRP_DEVICEDESC, SPDRP_HARDWAREID, SetupDiDestroyDeviceInfoList,
    SetupDiEnumDeviceInfo, SetupDiGetClassDevsW, SetupDiGetDevicePropertyW,
    SetupDiGetDeviceRegistryPropertyW, SetupDiOpenDevRegKey,
};
use windows_sys::Win32::Devices::Properties::{DEVPKEY_Device_BusReportedDeviceDesc, DEVPROPKEY};
use windows_sys::Win32::Foundation::INVALID_HANDLE_VALUE;
use windows_sys::Win32::System::Registry::{KEY_QUERY_VALUE, RegCloseKey, RegQueryValueExW};
use windows_sys::core::GUID;

use crate::enumerate::{WindowsDevice, utf16_to_first_nul};

/// `GUID_DEVINTERFACE_USB_DEVICE`. `// C#: Utilities/Win32DeviceMgnt.cs:398`
const USB_DEVICE: GUID = GUID::from_u128(0xA5DC_BF10_6530_11D2_901F_00C0_4FB9_51ED);
/// `GUID_DEVINTERFACE_COMPORT`. `// C#: Utilities/Win32DeviceMgnt.cs:401`
const COMPORT: GUID = GUID::from_u128(0x86E0_D1E0_8089_11D0_9CE4_0800_3E30_1F73);

/// The buffers the C# reads into: 1,024 bytes for a property, 256 characters for `PortName`.
const PROPERTY_BYTES: usize = 1024;
const PORT_NAME_BYTES: usize = 512;

/// `GetAllCOMPorts`: the USB devices' ports, then the COM ports' - a device of each class with a
/// `PortName`, in SetupAPI's order.
/// `// C#: Utilities/Win32DeviceMgnt.cs:392-409`
#[must_use]
pub(crate) fn com_ports() -> Vec<WindowsDevice> {
    let mut devices = class_devs(&USB_DEVICE);
    devices.extend(class_devs(&COMPORT));
    devices
}

/// `GetClassDevs`: each present device of an interface class, skipped when its `PortName`, its
/// description or its hardware id cannot be read - where the C# catches the exception and moves
/// to the next - and its board left unset when the bus did not report one.
/// `// C#: Utilities/Win32DeviceMgnt.cs:412-575`
fn class_devs(class: &GUID) -> Vec<WindowsDevice> {
    // SAFETY: `class` points at a GUID for the call's length; a null enumerator and a null
    // window are allowed; the handle returned is checked before use and destroyed below.
    let set: HDEVINFO = unsafe {
        SetupDiGetClassDevsW(
            class,
            std::ptr::null(),
            0,
            DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
        )
    };
    if set == INVALID_HANDLE_VALUE {
        return Vec::new();
    }
    let mut devices = Vec::new();
    for index in 0.. {
        let mut data = SP_DEVINFO_DATA {
            cbSize: u32::try_from(std::mem::size_of::<SP_DEVINFO_DATA>()).unwrap_or(u32::MAX),
            ClassGuid: GUID::from_u128(0),
            DevInst: 0,
            Reserved: 0,
        };
        // SAFETY: `set` is a live device information set and `data` a correctly sized
        // SP_DEVINFO_DATA the call fills.
        if unsafe { SetupDiEnumDeviceInfo(set, index, &mut data) } == 0 {
            break;
        }
        let Some(name) = port_name(set, &data) else {
            continue;
        };
        let (Some(description), Some(hardware_id)) = (
            registry_string(set, &data, SPDRP_DEVICEDESC),
            registry_string(set, &data, SPDRP_HARDWAREID),
        ) else {
            continue;
        };
        devices.push(WindowsDevice {
            name,
            description: Some(description),
            hardware_id: Some(hardware_id),
            board: property_string(set, &data, &DEVPKEY_Device_BusReportedDeviceDesc),
        });
    }
    // SAFETY: `set` came from SetupDiGetClassDevsW and is destroyed once.
    unsafe { SetupDiDestroyDeviceInfoList(set) };
    devices
}

/// `GetDeviceName`: the `PortName` value of the device's registry key, `None` when the key or the
/// value cannot be read.
/// `// C#: Utilities/Win32DeviceMgnt.cs:582-610`
fn port_name(set: HDEVINFO, data: &SP_DEVINFO_DATA) -> Option<String> {
    // SAFETY: `set` and `data` describe a device of that set; the key returned is checked and
    // closed below.
    let key = unsafe {
        SetupDiOpenDevRegKey(set, data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE)
    };
    if key == INVALID_HANDLE_VALUE {
        return None;
    }
    let value: Vec<u16> = "PortName".encode_utf16().chain(std::iter::once(0)).collect();
    let mut buffer = [0u8; PORT_NAME_BYTES];
    let mut length = u32::try_from(buffer.len()).unwrap_or(0);
    let mut kind = 0u32;
    // SAFETY: `key` is open; `value` is a NUL-terminated UTF-16 name; `buffer` is writable for
    // `length` bytes, which the call updates.
    let result = unsafe {
        RegQueryValueExW(
            key,
            value.as_ptr(),
            std::ptr::null(),
            &mut kind,
            buffer.as_mut_ptr(),
            &mut length,
        )
    };
    // SAFETY: `key` came from SetupDiOpenDevRegKey and is closed once.
    unsafe { RegCloseKey(key) };
    if result != 0 {
        return None;
    }
    let read = usize::try_from(length).unwrap_or(0).min(buffer.len());
    Some(utf16_to_first_nul(buffer.get(..read)?))
}

/// `GetDeviceDescription`: a registry property of the device, read up to its first NUL.
/// `// C#: Utilities/Win32DeviceMgnt.cs:612-640`
fn registry_string(set: HDEVINFO, data: &SP_DEVINFO_DATA, property: u32) -> Option<String> {
    let mut buffer = [0u8; PROPERTY_BYTES];
    let mut kind = 0u32;
    let mut required = 0u32;
    // SAFETY: `set` and `data` describe a device of that set; `buffer` is writable for its
    // length.
    let ok = unsafe {
        SetupDiGetDeviceRegistryPropertyW(
            set,
            data,
            property,
            &mut kind,
            buffer.as_mut_ptr(),
            u32::try_from(buffer.len()).unwrap_or(0),
            &mut required,
        )
    };
    (ok != 0).then(|| utf16_to_first_nul(&buffer))
}

/// A device property by its key, read as `Marshal.PtrToStringAuto` reads the buffer.
/// `// C#: Utilities/Win32DeviceMgnt.cs:535-556`
fn property_string(set: HDEVINFO, data: &SP_DEVINFO_DATA, key: &DEVPROPKEY) -> Option<String> {
    let mut buffer = [0u8; PROPERTY_BYTES];
    let mut kind = 0u32;
    let mut required = 0u32;
    // SAFETY: `set` and `data` describe a device of that set; `key` points at a DEVPROPKEY for
    // the call's length; `buffer` is writable for its length.
    let ok = unsafe {
        SetupDiGetDevicePropertyW(
            set,
            data,
            key,
            &mut kind,
            buffer.as_mut_ptr(),
            u32::try_from(buffer.len()).unwrap_or(0),
            &mut required,
            0,
        )
    };
    (ok != 0).then(|| utf16_to_first_nul(&buffer))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// On whatever machine runs it: every port Windows lists has a name, a description and a
    /// hardware id, and a USB one's id carries its VID and PID - the VM with the bench CubeOrange
    /// passed through lists COM3 and COM4 with `USB\VID_2DAE&PID_1016&REV_0200&MI_0x` and board
    /// "CubeOrange".
    #[test]
    fn every_port_windows_lists_has_its_name_description_and_hardware_id() {
        for device in com_ports() {
            eprintln!("{device:?}");
            assert!(!device.name.is_empty(), "{device:?}");
            assert!(device.description.as_deref().is_some_and(|d| !d.is_empty()), "{device:?}");
            let id = device.hardware_id.as_deref().unwrap_or_default();
            assert!(!id.is_empty(), "{device:?}");
            if id.starts_with("USB\\") {
                assert!(id.contains("VID_") && id.contains("&PID_"), "{device:?}");
            }
        }
    }
}
