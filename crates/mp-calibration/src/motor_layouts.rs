//! `APMotorLayout.json`: the motor layouts `ConfigMotorTest` labels its buttons from, as a table.
//!
//! Mission Planner ships the file beside its executable and reads it on every `Activate`, keeping
//! the layout whose `Class` and `Type` are the vehicle's frame, and only when the file's `Version`
//! is `AP_Motors library test ver 1.2`
//! (`// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:236-261`). The file is ArduPilot's - the
//! output of its `AP_Motors` library test - copied into the C# tree.
//!
//! This is that file, generated from `APMotorLayout.json` at the root of the C# tree, with the three
//! fields of the C#'s `_motors` the page uses: `Number`, `TestOrder` and `Rotation`
//! (`// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:22-29, 61-77`). `Roll` and `Pitch` are
//! read into that struct and never used, and each layout's `ClassName` and `TypeName` are not read
//! at all; the names are kept as a comment on each layout. `motor.rs`'s tests hold the table to the
//! file when the C# tree is checked out.

/// The `Version` the C# requires of the file before it uses any layout in it.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:245`
pub const LAYOUT_VERSION: &str = "AP_Motors library test ver 1.2";

/// One motor of a layout: the C#'s `_motors`, less the two fields it never reads.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:22-29`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Motor {
    /// `Number`: the motor's output number, as ArduPilot's motor diagrams label it.
    pub number: i32,
    /// `TestOrder`: where it comes in the test sequence, 1 first. The page's button `a` - lettered
    /// `A` for 1 - is labelled with the motor whose `TestOrder` is `a`.
    pub test_order: i32,
    /// `Rotation`: `CW`, `CCW`, or `?` where the layout does not say.
    pub rotation: &'static str,
}

/// One frame's layout: the C#'s `_layouts`.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:30-35`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    /// `Class`: the `FRAME_CLASS` (or `Q_FRAME_CLASS`) value it is for.
    pub class: i32,
    /// `Type`: the `FRAME_TYPE` (or `Q_FRAME_TYPE`) value it is for.
    pub frame_type: i32,
    /// `motors`, in the file's order.
    pub motors: &'static [Motor],
}

const fn motor(number: i32, test_order: i32, rotation: &'static str) -> Motor {
    Motor {
        number,
        test_order,
        rotation,
    }
}

const fn layout(class: i32, frame_type: i32, motors: &'static [Motor]) -> Layout {
    Layout {
        class,
        frame_type,
        motors,
    }
}

/// `layouts`, in the file's order; the C# takes the first whose class and type match.
/// `// C#: APMotorLayout.json; GCSViews/ConfigurationView/ConfigMotorTest.cs:247-254`
pub const LAYOUTS: &[Layout] = &[
    // QUAD PLUS
    layout(
        1,
        0,
        &[
            motor(1, 2, "CCW"),
            motor(2, 4, "CCW"),
            motor(3, 1, "CW"),
            motor(4, 3, "CW"),
        ],
    ),
    // QUAD X
    layout(
        1,
        1,
        &[
            motor(1, 1, "CCW"),
            motor(2, 3, "CCW"),
            motor(3, 4, "CW"),
            motor(4, 2, "CW"),
        ],
    ),
    // QUAD V
    layout(
        1,
        2,
        &[
            motor(1, 1, "CCW"),
            motor(2, 3, "CCW"),
            motor(3, 4, "CW"),
            motor(4, 2, "CW"),
        ],
    ),
    // QUAD H
    layout(
        1,
        3,
        &[
            motor(1, 1, "CW"),
            motor(2, 3, "CW"),
            motor(3, 4, "CCW"),
            motor(4, 2, "CCW"),
        ],
    ),
    // QUAD VTAIL
    layout(
        1,
        4,
        &[
            motor(1, 1, "?"),
            motor(2, 3, "CW"),
            motor(3, 4, "?"),
            motor(4, 2, "CCW"),
        ],
    ),
    // QUAD ATAIL
    layout(
        1,
        5,
        &[
            motor(1, 1, "?"),
            motor(2, 3, "CCW"),
            motor(3, 4, "?"),
            motor(4, 2, "CW"),
        ],
    ),
    // QUAD PLUSREV
    layout(
        1,
        6,
        &[
            motor(1, 2, "CW"),
            motor(2, 4, "CW"),
            motor(3, 1, "CCW"),
            motor(4, 3, "CCW"),
        ],
    ),
    // QUAD BF_X
    layout(
        1,
        12,
        &[
            motor(1, 2, "CW"),
            motor(2, 1, "CCW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
        ],
    ),
    // QUAD DJI_X
    layout(
        1,
        13,
        &[
            motor(1, 1, "CCW"),
            motor(2, 4, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 2, "CW"),
        ],
    ),
    // QUAD CW_X
    layout(
        1,
        14,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
        ],
    ),
    // QUAD NYT_PLUS
    layout(
        1,
        16,
        &[
            motor(1, 2, "?"),
            motor(2, 4, "?"),
            motor(3, 1, "?"),
            motor(4, 3, "?"),
        ],
    ),
    // QUAD NYT_X
    layout(
        1,
        17,
        &[
            motor(1, 1, "?"),
            motor(2, 3, "?"),
            motor(3, 4, "?"),
            motor(4, 2, "?"),
        ],
    ),
    // QUAD X_REV
    layout(
        1,
        18,
        &[
            motor(1, 2, "CCW"),
            motor(2, 1, "CW"),
            motor(3, 3, "CW"),
            motor(4, 4, "CCW"),
        ],
    ),
    // HEXA PLUS
    layout(
        2,
        0,
        &[
            motor(1, 1, "CW"),
            motor(2, 4, "CCW"),
            motor(3, 5, "CW"),
            motor(4, 2, "CCW"),
            motor(5, 6, "CCW"),
            motor(6, 3, "CW"),
        ],
    ),
    // HEXA X
    layout(
        2,
        1,
        &[
            motor(1, 2, "CW"),
            motor(2, 5, "CCW"),
            motor(3, 6, "CW"),
            motor(4, 3, "CCW"),
            motor(5, 1, "CCW"),
            motor(6, 4, "CW"),
        ],
    ),
    // HEXA H
    layout(
        2,
        3,
        &[
            motor(1, 2, "CW"),
            motor(2, 5, "CCW"),
            motor(3, 6, "CW"),
            motor(4, 3, "CCW"),
            motor(5, 1, "CCW"),
            motor(6, 4, "CW"),
        ],
    ),
    // HEXA DJI_X
    layout(
        2,
        13,
        &[
            motor(1, 1, "CCW"),
            motor(2, 6, "CW"),
            motor(3, 5, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 3, "CCW"),
            motor(6, 2, "CW"),
        ],
    ),
    // HEXA CW_X
    layout(
        2,
        14,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
        ],
    ),
    // OCTA PLUS
    layout(
        3,
        0,
        &[
            motor(1, 1, "CW"),
            motor(2, 5, "CW"),
            motor(3, 2, "CCW"),
            motor(4, 4, "CCW"),
            motor(5, 8, "CCW"),
            motor(6, 6, "CCW"),
            motor(7, 7, "CW"),
            motor(8, 3, "CW"),
        ],
    ),
    // OCTA X
    layout(
        3,
        1,
        &[
            motor(1, 1, "CW"),
            motor(2, 5, "CW"),
            motor(3, 2, "CCW"),
            motor(4, 4, "CCW"),
            motor(5, 8, "CCW"),
            motor(6, 6, "CCW"),
            motor(7, 7, "CW"),
            motor(8, 3, "CW"),
        ],
    ),
    // OCTA V
    layout(
        3,
        2,
        &[
            motor(1, 7, "CW"),
            motor(2, 3, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CCW"),
            motor(5, 8, "CCW"),
            motor(6, 2, "CCW"),
            motor(7, 1, "CW"),
            motor(8, 5, "CW"),
        ],
    ),
    // OCTA H
    layout(
        3,
        3,
        &[
            motor(1, 1, "CW"),
            motor(2, 5, "CW"),
            motor(3, 2, "CCW"),
            motor(4, 4, "CCW"),
            motor(5, 8, "CCW"),
            motor(6, 6, "CCW"),
            motor(7, 7, "CW"),
            motor(8, 3, "CW"),
        ],
    ),
    // OCTA DJI_X
    layout(
        3,
        13,
        &[
            motor(1, 1, "CCW"),
            motor(2, 8, "CW"),
            motor(3, 7, "CCW"),
            motor(4, 6, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 4, "CW"),
            motor(7, 3, "CCW"),
            motor(8, 2, "CW"),
        ],
    ),
    // OCTA CW_X
    layout(
        3,
        14,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CCW"),
            motor(8, 8, "CW"),
        ],
    ),
    // OCTA I
    layout(
        3,
        15,
        &[
            motor(1, 5, "CW"),
            motor(2, 1, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 8, "CCW"),
            motor(5, 4, "CCW"),
            motor(6, 2, "CCW"),
            motor(7, 3, "CW"),
            motor(8, 7, "CW"),
        ],
    ),
    // OCTAQUAD PLUS
    layout(
        4,
        0,
        &[
            motor(1, 1, "CCW"),
            motor(2, 7, "CW"),
            motor(3, 5, "CCW"),
            motor(4, 3, "CW"),
            motor(5, 8, "CCW"),
            motor(6, 2, "CW"),
            motor(7, 4, "CCW"),
            motor(8, 6, "CW"),
        ],
    ),
    // OCTAQUAD X
    layout(
        4,
        1,
        &[
            motor(1, 1, "CCW"),
            motor(2, 7, "CW"),
            motor(3, 5, "CCW"),
            motor(4, 3, "CW"),
            motor(5, 8, "CCW"),
            motor(6, 2, "CW"),
            motor(7, 4, "CCW"),
            motor(8, 6, "CW"),
        ],
    ),
    // OCTAQUAD V
    layout(
        4,
        2,
        &[
            motor(1, 1, "CCW"),
            motor(2, 7, "CW"),
            motor(3, 5, "CCW"),
            motor(4, 3, "CW"),
            motor(5, 8, "CCW"),
            motor(6, 2, "CW"),
            motor(7, 4, "CCW"),
            motor(8, 6, "CW"),
        ],
    ),
    // OCTAQUAD H
    layout(
        4,
        3,
        &[
            motor(1, 1, "CW"),
            motor(2, 7, "CCW"),
            motor(3, 5, "CW"),
            motor(4, 3, "CCW"),
            motor(5, 8, "CW"),
            motor(6, 2, "CCW"),
            motor(7, 4, "CW"),
            motor(8, 6, "CCW"),
        ],
    ),
    // OCTAQUAD BF_X
    layout(
        4,
        12,
        &[
            motor(1, 3, "CW"),
            motor(2, 1, "CCW"),
            motor(3, 5, "CCW"),
            motor(4, 7, "CW"),
            motor(5, 4, "CCW"),
            motor(6, 2, "CW"),
            motor(7, 6, "CW"),
            motor(8, 8, "CCW"),
        ],
    ),
    // OCTAQUAD CW_X
    layout(
        4,
        14,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CW"),
            motor(4, 4, "CCW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CW"),
            motor(8, 8, "CCW"),
        ],
    ),
    // OCTAQUAD X_REV
    layout(
        4,
        18,
        &[
            motor(1, 3, "CCW"),
            motor(2, 1, "CW"),
            motor(3, 5, "CW"),
            motor(4, 7, "CCW"),
            motor(5, 4, "CW"),
            motor(6, 2, "CCW"),
            motor(7, 6, "CCW"),
            motor(8, 8, "CW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        0,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        1,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        2,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        3,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        4,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        5,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        6,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        7,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        8,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        9,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 Y6B
    layout(
        5,
        10,
        &[
            motor(1, 1, "CW"),
            motor(2, 2, "CCW"),
            motor(3, 3, "CW"),
            motor(4, 4, "CCW"),
            motor(5, 5, "CW"),
            motor(6, 6, "CCW"),
        ],
    ),
    // Y6 Y6F
    layout(
        5,
        11,
        &[
            motor(1, 3, "CCW"),
            motor(2, 1, "CCW"),
            motor(3, 5, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 2, "CW"),
            motor(6, 6, "CW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        12,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        13,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        14,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        15,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        16,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        17,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // Y6 default
    layout(
        5,
        18,
        &[
            motor(1, 2, "CCW"),
            motor(2, 5, "CW"),
            motor(3, 6, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 1, "CW"),
            motor(6, 3, "CCW"),
        ],
    ),
    // TRI default
    layout(
        7,
        0,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        1,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        2,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        3,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        4,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        5,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI pitch-reversed
    layout(
        7,
        6,
        &[
            motor(1, 3, "?"),
            motor(2, 4, "?"),
            motor(4, 1, "?"),
            motor(7, 2, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        7,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        8,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        9,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        10,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        11,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        12,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        13,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        14,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        15,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        16,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        17,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // TRI default
    layout(
        7,
        18,
        &[
            motor(1, 1, "?"),
            motor(2, 4, "?"),
            motor(4, 2, "?"),
            motor(7, 3, "?"),
        ],
    ),
    // DODECAHEXA PLUS
    layout(
        12,
        0,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CW"),
            motor(4, 4, "CCW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CW"),
            motor(8, 8, "CCW"),
            motor(9, 9, "CCW"),
            motor(10, 10, "CW"),
            motor(11, 11, "CW"),
            motor(12, 12, "CCW"),
        ],
    ),
    // DODECAHEXA X
    layout(
        12,
        1,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CW"),
            motor(4, 4, "CCW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CW"),
            motor(8, 8, "CCW"),
            motor(9, 9, "CCW"),
            motor(10, 10, "CW"),
            motor(11, 11, "CW"),
            motor(12, 12, "CCW"),
        ],
    ),
    // DECA PLUS
    layout(
        14,
        0,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CCW"),
            motor(8, 8, "CW"),
            motor(9, 9, "CCW"),
            motor(10, 10, "CW"),
        ],
    ),
    // DECA X/CW_X
    layout(
        14,
        1,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CCW"),
            motor(8, 8, "CW"),
            motor(9, 9, "CCW"),
            motor(10, 10, "CW"),
        ],
    ),
    // DECA X/CW_X
    layout(
        14,
        14,
        &[
            motor(1, 1, "CCW"),
            motor(2, 2, "CW"),
            motor(3, 3, "CCW"),
            motor(4, 4, "CW"),
            motor(5, 5, "CCW"),
            motor(6, 6, "CW"),
            motor(7, 7, "CCW"),
            motor(8, 8, "CW"),
            motor(9, 9, "CCW"),
            motor(10, 10, "CW"),
        ],
    ),
];
