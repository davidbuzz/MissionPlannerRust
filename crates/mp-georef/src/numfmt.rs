//! The number formats the output files use beyond `mp_log::netfmt`'s general ones.

use mp_log::netfmt;

/// `Environment.NewLine`, which `StreamWriter.WriteLine` and the XML writers end lines with.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// A number as .NET's general format with `precision` digits has it: sign, digits (trailing zeros
/// dropped), and the power of ten of the first digit's place plus one (`0.d1d2... x 10^scale`).
fn digits15_at(value: f64, precision: i32) -> (bool, Vec<u8>, i32) {
    let text = netfmt::general(value, usize::try_from(precision).unwrap_or(15));
    let negative = text.starts_with('-');
    let body = text.trim_start_matches('-');
    let (mantissa, exponent) = match body.split_once('E') {
        Some((m, e)) => (m, e.parse::<i32>().unwrap_or(0)),
        None => (body, 0),
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let mut digits: Vec<u8> = whole
        .bytes()
        .chain(fraction.bytes())
        .map(|b| b - b'0')
        .collect();
    let mut scale = i32::try_from(whole.len()).unwrap_or(0) + exponent;
    while digits.first() == Some(&0) && digits.len() > 1 {
        digits.remove(0);
        scale -= 1;
    }
    if digits.iter().all(|&d| d == 0) {
        return (negative, Vec::new(), 0);
    }
    (negative, digits, scale)
}

/// `value.ToString("#0.##############")` in the invariant culture, what SharpKml's `KmlFormatter`
/// writes every `double` and `float` as: the value at 15 significant digits, then rounded half up
/// to 14 decimals, trailing zeros and a bare point dropped, and no minus sign on what rounds to
/// zero.
/// `// C#: ExtLibs/SharpKml/Base/KmlFormatter.cs:25-32`
#[must_use]
pub fn kml_double(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    let (negative, digits, scale) = digits15_at(value, 15);
    // Digits at places 10^(scale-1) down to 10^-14.
    let keep = usize::try_from((scale + 14).max(0)).unwrap_or(0);
    let mut kept: Vec<u8> = digits.iter().copied().take(keep).collect();
    let mut scale = scale;
    if digits.get(keep).is_some_and(|&d| d >= 5) {
        // Round up through the kept digits.
        let mut carry = true;
        for d in kept.iter_mut().rev() {
            if *d == 9 {
                *d = 0;
            } else {
                *d += 1;
                carry = false;
                break;
            }
        }
        if carry {
            kept.insert(0, 1);
            scale += 1;
        }
    }
    let mut integer = String::new();
    let mut fraction = String::new();
    for i in 0..usize::try_from(scale.max(0)).unwrap_or(0) {
        integer.push(char::from(b'0' + kept.get(i).copied().unwrap_or(0)));
    }
    if integer.is_empty() {
        integer.push('0');
    }
    let leading_zeros = usize::try_from((-scale).max(0)).unwrap_or(0);
    for _ in 0..leading_zeros.min(14) {
        fraction.push('0');
    }
    let from = usize::try_from(scale.max(0)).unwrap_or(0);
    for &d in kept.iter().skip(from) {
        fraction.push(char::from(b'0' + d));
    }
    let fraction = fraction.get(..fraction.len().min(14)).unwrap_or_default();
    let fraction = fraction.trim_end_matches('0');
    let zero = integer.bytes().all(|b| b == b'0') && fraction.is_empty();
    let mut out = String::new();
    if negative && !zero {
        out.push('-');
    }
    out.push_str(&integer);
    if !fraction.is_empty() {
        out.push('.');
        out.push_str(fraction);
    }
    out
}

/// `rgval64Power10`: 10^1 to 10^15, then 10^-1 to 10^-15, as normalised 64-bit mantissas - the
/// table `mp_log::netfmt` reads `double.Parse` with, read out of mono's `mscorlib.dll`.
const POWER10: [u64; 30] = [
    0xa000_0000_0000_0000,
    0xc800_0000_0000_0000,
    0xfa00_0000_0000_0000,
    0x9c40_0000_0000_0000,
    0xc350_0000_0000_0000,
    0xf424_0000_0000_0000,
    0x9896_8000_0000_0000,
    0xbebc_2000_0000_0000,
    0xee6b_2800_0000_0000,
    0x9502_f900_0000_0000,
    0xba43_b740_0000_0000,
    0xe8d4_a510_0000_0000,
    0x9184_e72a_0000_0000,
    0xb5e6_20f4_8000_0000,
    0xe35f_a931_a000_0000,
    0xcccc_cccc_cccc_cccd,
    0xa3d7_0a3d_70a3_d70b,
    0x8312_6e97_8d4f_df3c,
    0xd1b7_1758_e219_652e,
    0xa7c5_ac47_1b47_8425,
    0x8637_bd05_af6c_69b7,
    0xd6bf_94d5_e57a_42be,
    0xabcc_7711_8461_ceff,
    0x8970_5f41_36b4_a599,
    0xdbe6_fece_bded_d5c2,
    0xafeb_ff0b_cb24_ab02,
    0x8cbc_cc09_6f50_88cf,
    0xe12e_1342_4bb4_0e18,
    0xb424_dc35_095c_d813,
    0x901d_7cf7_3ab0_acdc,
];

/// `rgexp64Power10`.
const POWER10_EXPONENT: [i32; 15] = [4, 7, 10, 14, 17, 20, 24, 27, 30, 34, 37, 40, 44, 47, 50];

/// `rgval64Power10By16`: 10^16 to 10^336 in steps of 10^16, then their inverses.
const POWER10_BY16: [u64; 42] = [
    0x8e1b_c9bf_0400_0000,
    0x9dc5_ada8_2b70_b59e,
    0xaf29_8d05_0e43_95d6,
    0xc278_1f49_ffcf_a6d4,
    0xd7e7_7a8f_87da_f7fa,
    0xefb3_ab16_c59b_14a0,
    0x850f_adc0_9923_329c,
    0x93ba_47c9_80e9_8cde,
    0xa402_b9c5_a8d3_a6e6,
    0xb616_a12b_7fe6_17a8,
    0xca28_a291_859b_bf90,
    0xe070_f78d_3927_5566,
    0xf92e_0c35_3782_6140,
    0x8a52_96ff_e33c_c92c,
    0x9991_a6f3_d6bf_1762,
    0xaa7e_ebfb_9df9_de8a,
    0xbd49_d14a_a79d_bc7e,
    0xd226_fc19_5c6a_2f88,
    0xe950_df20_247c_83f8,
    0x8184_2f29_f2cc_e373,
    0x8fca_c257_558e_e4e2,
    0xe695_94be_c44d_e160,
    0xcfb1_1ead_4539_94c3,
    0xbb12_7c53_b17e_c165,
    0xa87f_ea27_a539_e9b3,
    0x97c5_60ba_6b09_19b5,
    0x88b4_02f7_fd75_53ab,
    0xf643_35bc_f065_d3a0,
    0xddd0_467c_64bc_e4c4,
    0xc7ca_ba6e_7c53_82ed,
    0xb3f4_e093_db73_a0b7,
    0xa217_27db_38cb_0053,
    0x91ff_8377_5423_cc29,
    0x8380_dea9_3da4_bc82,
    0xece5_3cec_4a31_4f00,
    0xd560_5fcd_cf32_e217,
    0xc031_4325_637a_1978,
    0xad1c_8eab_5ee4_3ba2,
    0x9bec_ce62_836a_c5b0,
    0x8c71_dcd9_ba0b_495c,
    0xfd00_b897_4782_3938,
    0xe3e2_7a44_4d8d_991a,
];

/// `rgexp64Power10By16`.
const POWER10_BY16_EXPONENT: [i32; 21] = [
    54, 107, 160, 213, 266, 319, 373, 426, 479, 532, 585, 638, 691, 745, 798, 851, 904, 957, 1010,
    1064, 1117,
];

/// `Mul64Lossy`.
fn mul64_lossy(a: u64, b: u64, exponent: &mut i32) -> u64 {
    let mul = |x: u64, y: u64| (x & 0xffff_ffff) * (y & 0xffff_ffff);
    let mut value = mul(a >> 32, b >> 32)
        .wrapping_add(mul(a >> 32, b) >> 32)
        .wrapping_add(mul(a, b >> 32) >> 32);
    if value & 0x8000_0000_0000_0000 == 0 {
        value <<= 1;
        *exponent -= 1;
    }
    value
}

/// .NET Framework's `NumberToDouble` over a `NUMBER` exactly as `DoubleToNumber` fills one: every
/// digit it was asked for, **trailing zeros included**. That is the difference from
/// `mp_log::netfmt`'s parse, whose `NUMBER` comes from `ParseNumber` and has them dropped: the
/// zeros change how many digits go into the 64-bit mantissa and which power of ten it is scaled
/// by, and so, for about one number in a thousand, the rounding.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp NumberToDouble; mp_log::netfmt::number_to_double`
fn number_to_double(digits: &[u8], scale: i64) -> f64 {
    let digits: Vec<u64> = digits
        .iter()
        .skip_while(|&&d| d == 0)
        .map(|&d| u64::from(d))
        .collect();
    let total = digits.len();
    if total == 0 {
        return 0.0;
    }
    let to_int = |part: &[u64]| part.iter().fold(0u64, |acc, &d| acc * 10 + d);
    let mut remaining = total;
    let count = remaining.min(9);
    remaining -= count;
    let mut value = to_int(digits.get(..count).unwrap_or_default());
    if remaining > 0 {
        let count = remaining.min(9);
        remaining -= count;
        let power = POWER10.get(count - 1).copied().unwrap_or(0);
        let shift = POWER10_EXPONENT.get(count - 1).copied().unwrap_or(0);
        let multiplier = power >> (64 - shift);
        value = value * multiplier + to_int(digits.get(9..9 + count).unwrap_or_default());
    }
    let consumed = i64::try_from(total - remaining).unwrap_or(0);
    let scale = scale.saturating_sub(consumed);
    let magnitude = scale.unsigned_abs();
    if magnitude >= 22 * 16 {
        return if scale > 0 { f64::INFINITY } else { 0.0 };
    }
    let magnitude = usize::try_from(magnitude).unwrap_or(0);
    let mut exponent: i32 = 64;
    for (mask, shift) in [
        (0xFFFF_FFFF_0000_0000u64, 32),
        (0xFFFF_0000_0000_0000, 16),
        (0xFF00_0000_0000_0000, 8),
        (0xF000_0000_0000_0000, 4),
        (0xC000_0000_0000_0000, 2),
        (0x8000_0000_0000_0000, 1),
    ] {
        if value & mask == 0 {
            value <<= shift;
            exponent -= shift;
        }
    }
    let inverse = scale < 0;
    let index = magnitude & 15;
    if index > 0 {
        let multiplier_exponent = POWER10_EXPONENT.get(index - 1).copied().unwrap_or(0);
        exponent += if inverse {
            -multiplier_exponent + 1
        } else {
            multiplier_exponent
        };
        let multiplier = POWER10
            .get(index + if inverse { 15 } else { 0 } - 1)
            .copied()
            .unwrap_or(0);
        value = mul64_lossy(value, multiplier, &mut exponent);
    }
    let index = magnitude >> 4;
    if index > 0 {
        let multiplier_exponent = POWER10_BY16_EXPONENT.get(index - 1).copied().unwrap_or(0);
        exponent += if inverse {
            -multiplier_exponent + 1
        } else {
            multiplier_exponent
        };
        let multiplier = POWER10_BY16
            .get(index + if inverse { 21 } else { 0 } - 1)
            .copied()
            .unwrap_or(0);
        value = mul64_lossy(value, multiplier, &mut exponent);
    }
    if value & (1 << 10) != 0 {
        let mut rounded = value
            .wrapping_add((1 << 10) - 1)
            .wrapping_add((value >> 11) & 1);
        if rounded < value {
            rounded = (rounded >> 1) | 0x8000_0000_0000_0000;
            exponent += 1;
        }
        value = rounded;
    }
    exponent += 0x3FE;
    let bits = if exponent <= 0 {
        if exponent == -52 && value >= 0x8000_0000_0000_0058 {
            1
        } else if exponent <= -52 {
            0
        } else {
            value >> (-exponent + 11)
        }
    } else if exponent >= 0x7FF {
        0x7FF0_0000_0000_0000
    } else {
        (u64::try_from(exponent).unwrap_or(0) << 52) + ((value >> 11) & 0x000F_FFFF_FFFF_FFFF)
    };
    f64::from_bits(bits)
}

/// `DoubleToNumber(value, precision)` read back through `NumberToDouble`: the magnitude of the
/// value's first `precision` significant digits, zeros kept.
fn reread(value: f64, precision: i32) -> f64 {
    let (_, mut digits, scale) = digits15_at(value, precision);
    digits.resize(usize::try_from(precision).unwrap_or(0).max(digits.len()), 0);
    number_to_double(&digits, i64::from(scale))
}

/// `double.ToString("R")` as .NET Framework (and mono, the oracle) writes it, which is what
/// `XmlConvert.ToString(double)` and so `XmlSerializer` write: 15 digits when `NumberToDouble` of
/// those 15 digits gives the value back, else 17 - and that test is not "does the text parse back",
/// because the digits it reads keep their trailing zeros (see `number_to_double`), so
/// `-27.4692539`, whose 15 digits end `…539000000`, comes out `-27.469253899999998`.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp FormatDouble, case 'R'`
#[must_use]
pub fn double_roundtrip(value: f64) -> String {
    if !value.is_finite() || value == 0.0 {
        return netfmt::general(value, 15);
    }
    if reread(value, 15) == value.abs() {
        netfmt::general(value, 15)
    } else {
        netfmt::general(value, 17)
    }
}

/// `float.ToString("R")`: 7 digits when `NumberToDouble` of those 7, narrowed, is the value, else 9.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp FormatSingle, case 'R'`
#[must_use]
pub fn single_roundtrip(value: f32) -> String {
    let wide = f64::from(value);
    if !value.is_finite() || value == 0.0 {
        return netfmt::general(wide, 7);
    }
    #[allow(clippy::cast_possible_truncation)]
    let back = reread(wide, 7) as f32;
    if back == value.abs() {
        netfmt::general(wide, 7)
    } else {
        netfmt::general(wide, 9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn what_mono_printed() {
        // What mono printed for `ToString("#0.##############", CultureInfo.InvariantCulture)`.
        assert_eq!(kml_double(-27.469_820_022_583_008), "-27.469820022583");
        assert_eq!(kml_double(153.025_123_456_789_12), "153.025123456789");
        assert_eq!(kml_double(0.000_000_000_000_001), "0");
        assert_eq!(kml_double(-0.000_000_000_000_001), "0");
        assert_eq!(kml_double(1e20), "100000000000000000000");
        assert_eq!(kml_double(0.5), "0.5");
        assert_eq!(kml_double(2.000_000_000_000_05), "2.00000000000005");
        assert_eq!(
            kml_double(netfmt::parse_double("12345.6789012345678").unwrap()),
            "12345.6789012346"
        );
        assert_eq!(kml_double(25.1), "25.1");
        assert_eq!(kml_double(-27.4698), "-27.4698");
        assert_eq!(kml_double(0.0), "0");
        assert_eq!(kml_double(-180.268_989_562_988_28), "-180.268989562988");
    }

    #[test]
    fn round_trip_keeps_the_trailing_zeros_it_reads() {
        // What mono printed for ToString("R") (and XmlConvert.ToString) of double.Parse(text).
        for (text, r) in [
            ("-27.4692539", "-27.469253899999998"),
            ("-27.4698", "-27.4698"),
            ("153.0251011", "153.0251011"),
            ("65.18", "65.18"),
            ("25.1", "25.1"),
            ("-13.1813034", "-13.181303399999999"),
            ("41.3394838", "41.339483799999996"),
        ] {
            let v = netfmt::parse_double(text).unwrap();
            assert_eq!(double_roundtrip(v), r, "{text}");
        }
        assert_eq!(double_roundtrip(0.0), "0");
        assert_eq!(single_roundtrip(0.073_479_59), "0.07347959");
    }
}
