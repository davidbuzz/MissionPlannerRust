//! A radio's settings: `TSetting` and its kin, the `ATI5?` and `ATI5` lines they are parsed
//! from, the options and ranges a setting can take, and the settings file the page's Save to
//! File and Load from File write and read.
//! `// C#: SikRadio/RFD900.cs:478-1031, 1302-1726`
//!
//! A radio describes its registers two ways. `ATI5` gives `S1:SERIAL_SPEED=57` - designator,
//! name, value. `ATI5?` (or, on newer firmware, `ATI10:n` one at a time) gives the same with the
//! range and the options: `S2:AIR_SPEED(L)[4..1000]=125{4,64,125,250,500,1000,}`.
//!
//! The C#'s `Dictionary<string, TBaseSetting>` is enumerated in the order the settings went in,
//! which is the order the page fills its controls and writes its commands; [`Settings`] keeps that
//! order.

use std::cmp::Ordering;

/// `TSetting.TRange`: the values a setting can take.
/// `// C#: SikRadio/RFD900.cs:1586-1713`
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Range {
    /// `TSimpleRange`: min to max by the increment, max always last.
    Simple {
        /// `Min`.
        min: i32,
        /// `Max`.
        max: i32,
        /// `Increment`.
        increment: i32,
    },
    /// `TMultiRange`: simple ranges one after another.
    Multi(Vec<(i32, i32, i32)>),
}

impl Range {
    /// `TSimpleRange.GetOptions`, `TMultiRange.GetOptions`.
    /// `// C#: SikRadio/RFD900.cs:1640-1680, 1691-1712`
    #[must_use]
    pub fn options(&self) -> Vec<i32> {
        match self {
            Self::Simple {
                min,
                max,
                increment,
            } => simple_options(*min, *max, *increment),
            Self::Multi(ranges) => ranges
                .iter()
                .flat_map(|(min, max, increment)| simple_options(*min, *max, *increment))
                .collect(),
        }
    }

    /// `GetOptionsIncludingValue`: the options, with `value` put in its place when it is not one.
    /// `// C#: SikRadio/RFD900.cs:1595-1625`
    #[must_use]
    pub fn options_including_value(&self, value: i32) -> Vec<i32> {
        let mut result = Vec::new();
        let mut got_value = false;
        for n in self.options() {
            if n == value {
                got_value = true;
            }
            if !got_value && n > value {
                result.push(value);
                got_value = true;
            }
            result.push(n);
        }
        if !got_value {
            result.push(value);
        }
        result
    }
}

/// `TSimpleRange.GetOptions`: a max below the min taken as the min (the firmware has sent one).
fn simple_options(min: i32, max: i32, increment: i32) -> Vec<i32> {
    let max = max.max(min);
    range(min, increment, max)
}

/// `Sikradio.Range(start, step, end)`: start to end by step, end always last.
/// `// C#: Radio/Sikradio.cs:1021-1073`
#[must_use]
pub fn range(start: i32, step: i32, end: i32) -> Vec<i32> {
    let mut list = Vec::new();
    let mut got_end = false;
    let step = i64::from(step.max(1));
    let mut a = i64::from(start);
    while a <= i64::from(end) {
        if a == i64::from(end) {
            got_end = true;
        }
        list.push(i32::try_from(a).unwrap_or(end));
        a += step;
    }
    if !got_end {
        list.push(end);
    }
    list
}

/// `TSetting.TOption`: a value and the name the radio gives it.
/// `// C#: SikRadio/RFD900.cs:1715-1725`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Choice {
    /// `Value`.
    pub value: i32,
    /// `OptionName`.
    pub name: String,
}

impl Choice {
    /// One.
    #[must_use]
    pub fn new(value: i32, name: impl Into<String>) -> Self {
        Self {
            value,
            name: name.into(),
        }
    }
}

/// What kind of `TBaseSetting` a setting is.
/// `// C#: SikRadio/RFD900.cs:1457-1584`
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Kind {
    /// `TShortSetting`: a value.
    Short {
        /// `Value`.
        value: i32,
    },
    /// `TSetting`: a value, with the range and options when the radio gave them.
    Full {
        /// `Value`.
        value: i32,
        /// `Range`, `None` when unknown.
        range: Option<Range>,
        /// `Options`, `None` when unknown.
        options: Option<Vec<Choice>>,
        /// `Increment`.
        increment: i32,
    },
    /// `TTextSetting`: text - the encryption key.
    Text {
        /// `Text`.
        text: String,
    },
}

/// `TBaseSetting`: a register's designator (`S2`, `&E`), its name and what it holds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Setting {
    /// `Designator`.
    pub designator: String,
    /// `Name`.
    pub name: String,
    /// What it is and holds.
    pub kind: Kind,
}

impl Setting {
    /// A `TSetting`.
    #[must_use]
    pub fn full(
        designator: impl Into<String>,
        name: impl Into<String>,
        range: Option<Range>,
        value: i32,
        options: Option<Vec<Choice>>,
        increment: i32,
    ) -> Self {
        Self {
            designator: designator.into(),
            name: name.into(),
            kind: Kind::Full {
                value,
                range,
                options,
                increment,
            },
        }
    }

    /// A `TShortSetting`.
    #[must_use]
    pub fn short(designator: impl Into<String>, name: impl Into<String>, value: i32) -> Self {
        Self {
            designator: designator.into(),
            name: name.into(),
            kind: Kind::Short { value },
        }
    }

    /// A `TTextSetting`.
    #[must_use]
    pub fn text(
        designator: impl Into<String>,
        name: impl Into<String>,
        text: impl Into<String>,
    ) -> Self {
        Self {
            designator: designator.into(),
            name: name.into(),
            kind: Kind::Text { text: text.into() },
        }
    }

    /// `is TSetting`.
    #[must_use]
    pub const fn is_full(&self) -> bool {
        matches!(self.kind, Kind::Full { .. })
    }

    /// `is TShortSetting` - a `TSetting` is one too.
    #[must_use]
    pub const fn is_short(&self) -> bool {
        matches!(self.kind, Kind::Short { .. } | Kind::Full { .. })
    }

    /// `is TTextSetting`.
    #[must_use]
    pub const fn is_text(&self) -> bool {
        matches!(self.kind, Kind::Text { .. })
    }

    /// The value of a `TShortSetting` or `TSetting`.
    #[must_use]
    pub const fn value(&self) -> Option<i32> {
        match self.kind {
            Kind::Short { value } | Kind::Full { value, .. } => Some(value),
            Kind::Text { .. } => None,
        }
    }

    /// `Value = value`; a text setting is left as it is.
    pub const fn set_value(&mut self, new: i32) {
        match &mut self.kind {
            Kind::Short { value } | Kind::Full { value, .. } => *value = new,
            Kind::Text { .. } => {}
        }
    }

    /// The text of a `TTextSetting`.
    #[must_use]
    pub fn text_value(&self) -> Option<&str> {
        match &self.kind {
            Kind::Text { text } => Some(text),
            _ => None,
        }
    }

    /// `Range`.
    #[must_use]
    pub const fn range(&self) -> Option<&Range> {
        match &self.kind {
            Kind::Full { range, .. } => range.as_ref(),
            _ => None,
        }
    }

    /// `Range = range`.
    pub fn set_range(&mut self, new: Option<Range>) {
        if let Kind::Full { range, .. } = &mut self.kind {
            *range = new;
        }
    }

    /// `Options`.
    #[must_use]
    pub fn options(&self) -> Option<&[Choice]> {
        match &self.kind {
            Kind::Full { options, .. } => options.as_deref(),
            _ => None,
        }
    }

    /// `Options = options`.
    pub fn set_options(&mut self, new: Option<Vec<Choice>>) {
        if let Kind::Full { options, .. } = &mut self.kind {
            *options = new;
        }
    }

    /// `GetValueAsString`.
    /// `// C#: SikRadio/RFD900.cs:1471-1474, 1509-1512`
    #[must_use]
    pub fn value_as_string(&self) -> String {
        match &self.kind {
            Kind::Short { value } | Kind::Full { value, .. } => value.to_string(),
            Kind::Text { text } => text.clone(),
        }
    }

    /// `SetValueFromString`: a number's text through `int.TryParse`, which leaves 0 when it is
    /// not one; a text setting's text as it is.
    /// `// C#: SikRadio/RFD900.cs:1476-1479, 1514-1517`
    pub fn set_value_from_string(&mut self, new: &str) {
        match &mut self.kind {
            Kind::Short { value } | Kind::Full { value, .. } => {
                *value = crate::try_parse_int(new).unwrap_or(0);
            }
            Kind::Text { text } => *text = new.to_owned(),
        }
    }

    /// `GetOptionNames`.
    /// `// C#: SikRadio/RFD900.cs:1549-1559`
    #[must_use]
    pub fn option_names(&self) -> Option<Vec<String>> {
        self.options()
            .map(|options| options.iter().map(|o| o.name.clone()).collect())
    }

    /// `GetOptionNameForValue`: the name of the option whose value's text is `value`.
    /// `// C#: SikRadio/RFD900.cs:1561-1575`
    #[must_use]
    pub fn option_name_for_value(&self, value: &str) -> Option<&str> {
        self.options()?
            .iter()
            .find(|o| o.value.to_string() == value)
            .map(|o| o.name.as_str())
    }

    /// `GetIsFlag`: a range of exactly 0 and 1.
    /// `// C#: SikRadio/RFD900.cs:1580-1584`
    #[must_use]
    pub fn is_flag(&self) -> bool {
        self.range().is_some_and(|range| range.options() == [0, 1])
    }
}

/// The settings by name, in the order they went in: the C#'s `Dictionary<string,
/// TBaseSetting>`, where setting a name already there keeps its place.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Settings {
    items: Vec<(String, Setting)>,
}

impl Settings {
    /// None.
    #[must_use]
    pub const fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// `this[name] = setting`.
    pub fn insert(&mut self, name: impl Into<String>, setting: Setting) {
        let name = name.into();
        if let Some(slot) = self.items.iter_mut().find(|(n, _)| *n == name) {
            slot.1 = setting;
        } else {
            self.items.push((name, setting));
        }
    }

    /// `this[name]`.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Setting> {
        self.items.iter().find(|(n, _)| n == name).map(|(_, s)| s)
    }

    /// `this[name]`, to change.
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Setting> {
        self.items
            .iter_mut()
            .find(|(n, _)| n == name)
            .map(|(_, s)| s)
    }

    /// `ContainsKey`.
    #[must_use]
    pub fn contains(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    /// The names and settings, in order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Setting)> {
        self.items.iter().map(|(n, s)| (n.as_str(), s))
    }

    /// `Keys`.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.items.iter().map(|(n, _)| n.as_str())
    }

    /// `Count`.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.items.len()
    }

    /// `Count == 0`.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// `TSettings.SaveToFile`'s text: `name = value` a line, the names sorted as `List.Sort`
    /// sorts strings (the culture's order), each line ended as `WriteLine` ends it.
    /// `// C#: SikRadio/RFD900.cs:1333-1355`
    #[must_use]
    pub fn to_file_text(&self) -> String {
        let mut names: Vec<&str> = self.names().collect();
        names.sort_by(|a, b| culture_compare(a, b));
        let newline = if cfg!(windows) { "\r\n" } else { "\n" };
        names
            .iter()
            .filter_map(|name| self.get(name).map(|s| (name, s)))
            .map(|(name, setting)| format!("{name} = {}{newline}", setting.value_as_string()))
            .collect()
    }

    /// `TSettings.LoadFromFile` over the file's text: each `name = value` line whose name is
    /// one of these settings sets it; `;` and `#` start a comment. The names and values taken,
    /// in the order the file gave them (a name given twice keeps its first place and its last
    /// value).
    /// `// C#: SikRadio/RFD900.cs:1357-1421`
    pub fn load_from_text(&mut self, text: &str) -> Vec<(String, String)> {
        let mut loaded: Vec<(String, String)> = Vec::new();
        for line in text.lines() {
            let Some((name, value)) = parse_ini_line(line) else {
                continue;
            };
            if let Some(setting) = self.get_mut(&name) {
                setting.set_value_from_string(&value);
                if let Some(slot) = loaded.iter_mut().find(|(n, _)| *n == name) {
                    slot.1 = value;
                } else {
                    loaded.push((name, value));
                }
            }
        }
        loaded
    }

    /// `CheckValid`: what is wrong with these settings, nothing when they are valid.
    /// `// C#: SikRadio/RFD900.cs:1423-1442`
    #[must_use]
    pub fn check_valid(&self) -> Vec<String> {
        let mut result = Vec::new();
        if let (Some(min), Some(max)) = (self.get("MIN_FREQ"), self.get("MAX_FREQ"))
            && let (Some(min), Some(max)) = (min.value(), max.value())
            && min > max
        {
            result.push("MIN_FREQ can't be more than MAX_FREQ".to_owned());
        }
        result
    }
}

/// `TSettings.ParseINILine`.
/// `// C#: SikRadio/RFD900.cs:1357-1382`
fn parse_ini_line(line: &str) -> Option<(String, String)> {
    let trimmed = line.trim();
    if trimmed.starts_with(';') || trimmed.starts_with('#') {
        return None;
    }
    let content = line.split([';', '#']).next()?;
    if !content.contains('=') {
        return None;
    }
    let parts: Vec<&str> = content.split('=').collect();
    match parts.as_slice() {
        [name, value] => Some((name.trim().to_owned(), value.trim().to_owned())),
        _ => None,
    }
}

/// `String.CompareTo` in the culture's order, as `List<string>.Sort` uses it, for the names a
/// radio gives: punctuation before digits before letters, letters without regard to case first.
fn culture_compare(a: &str, b: &str) -> Ordering {
    fn rank(c: char) -> (u8, u32) {
        if c.is_ascii_alphabetic() {
            (2, u32::from(c.to_ascii_lowercase()))
        } else if c.is_ascii_digit() {
            (1, u32::from(c))
        } else {
            // `_` before the other marks, as the culture's tables put it.
            (0, if c == '_' { 0 } else { u32::from(c) })
        }
    }
    a.chars()
        .map(rank)
        .cmp(b.chars().map(rank))
        .then_with(|| b.cmp(a))
}

// ---------------------------------------------------------------------------------------------
// The lines.
// ---------------------------------------------------------------------------------------------

/// `RFDLib.Text.CheckIsLetter`.
const fn is_letter(c: char) -> bool {
    c.is_ascii_alphabetic()
}

/// `RFDLib.Text.CheckIsNumeral`.
const fn is_numeral(c: char) -> bool {
    c.is_ascii_digit()
}

/// `SkipMultipointNodeNumber`: the first letter's place, or 0 with none.
/// `// C#: SikRadio/RFD900.cs:484-498`
fn skip_multipoint_node_number(line: &[char]) -> usize {
    line.iter().position(|c| is_letter(*c)).unwrap_or(0)
}

/// `ParseDesignator`: a letter and the digits before the `:`, leaving `n` at the `:`.
/// `// C#: SikRadio/RFD900.cs:505-534`
fn parse_designator(line: &[char], n: &mut usize) -> Option<String> {
    *n = skip_multipoint_node_number(line);
    let first = *line.get(*n)?;
    if !is_letter(first) {
        return None;
    }
    let mut designator = first.to_string();
    *n += 1;
    while *line.get(*n)? != ':' {
        let c = *line.get(*n)?;
        if !is_numeral(c) {
            return None;
        }
        designator.push(c);
        *n += 1;
    }
    Some(designator)
}

/// `ParseName`: up to a `(` or `=`.
/// `// C#: SikRadio/RFD900.cs:536-551`
fn parse_name(line: &[char], n: &mut usize) -> Option<String> {
    let mut name = String::new();
    loop {
        let c = *line.get(*n)?;
        if c == '(' || c == '=' {
            return Some(name);
        }
        name.push(c);
        *n += 1;
    }
}

/// `ParseType`: past the `)`.
/// `// C#: SikRadio/RFD900.cs:553-560`
fn parse_type(line: &[char], n: &mut usize) -> Option<()> {
    while *line.get(*n)? != ')' {
        *n += 1;
    }
    *n += 1;
    Some(())
}

/// `ParseIntUntil`: the digits from `n`, then the delimiter if there is one. `None` where the
/// C#'s `int.Parse` throws (no digits); `Some(None)` where it returns false (no delimiter).
/// `// C#: SikRadio/RFD900.cs:572-605`
fn parse_int_until(line: &[char], n: &mut usize, delimiter: Option<&str>) -> Option<Option<i32>> {
    let mut temp = String::new();
    while let Some(&c) = line.get(*n)
        && is_numeral(c)
    {
        temp.push(c);
        *n += 1;
    }
    let value: i32 = temp.parse().ok()?;
    let Some(delimiter) = delimiter else {
        return Some(Some(value));
    };
    let rest: String = line.get(*n..).unwrap_or_default().iter().collect();
    if *n < line.len() && rest.starts_with(delimiter) {
        *n += delimiter.chars().count();
        Some(Some(value))
    } else {
        Some(None)
    }
}

/// `ParseRange`: `[min..max]`, stepped by the name's increment.
/// `// C#: SikRadio/RFD900.cs:615-633`
fn parse_range(line: &[char], n: &mut usize, name: &str) -> Option<Option<Range>> {
    if *line.get(*n)? != '[' {
        return Some(None);
    }
    *n += 1;
    let Some(min) = parse_int_until(line, n, Some(".."))? else {
        return Some(None);
    };
    let Some(max) = parse_int_until(line, n, Some("]"))? else {
        return Some(None);
    };
    Some(Some(Range::Simple {
        min,
        max,
        increment: increment(name),
    }))
}

/// `ParseOptions`: `{a,b,}` - empty entries dropped. `None` where the C# runs off the end of the
/// line and throws.
/// `// C#: SikRadio/RFD900.cs:652-695`
fn parse_options(line: &[char], n: &mut usize) -> Option<Vec<String>> {
    if *line.get(*n)? != '{' {
        return Some(Vec::new());
    }
    *n += 1;
    let mut options = Vec::new();
    loop {
        let mut temp = String::new();
        loop {
            let c = *line.get(*n)?;
            if c == ',' || c == '}' {
                break;
            }
            temp.push(c);
            *n += 1;
        }
        if !temp.is_empty() {
            options.push(temp);
        }
        if *line.get(*n)? == '}' {
            break;
        }
        *n += 1;
    }
    Some(options)
}

/// `GetIncrement`: the frequencies by 500 kHz, the window by 20 ms, the rest by one.
/// `// C#: SikRadio/RFD900.cs:896-908`
#[must_use]
pub fn increment(name: &str) -> i32 {
    match name {
        "MIN_FREQ" | "MAX_FREQ" => 500,
        "MAX_WINDOW" => 20,
        _ => 1,
    }
}

/// `GetScaleFactor`: whether numeric options are in the setting's own units or a thousand times
/// them (a baud rate given as 57600 for a value of 57).
/// `// C#: SikRadio/RFD900.cs:711-746`
fn scale_factor(name: &str, ints: &[i32], value: i32) -> f32 {
    if name == "SERIAL_SPEED" {
        return if ints.iter().any(|i| *i < 1000) {
            1.0
        } else {
            0.001
        };
    }
    if value != 0 {
        if ints.contains(&value) {
            return 1.0;
        }
        if ints.iter().any(|i| i / 1000 == value) {
            return 0.001;
        }
    }
    1.0
}

/// `IsCapitalLetter`.
const fn is_capital(c: char) -> bool {
    c.is_ascii_uppercase()
}

/// `SplitOptionName`: the words of `HiLbtJAP` - `Hi`, `Lbt`, `JAP` - a word starting where lower
/// case turns upper, or one before where upper turns lower.
/// `// C#: SikRadio/RFD900.cs:752-785`
fn split_option_name(option: &str) -> Vec<String> {
    let chars: Vec<char> = option.chars().collect();
    let piece = |from: usize, to: usize| -> String {
        chars
            .get(from..to)
            .map(|word| word.iter().collect())
            .unwrap_or_default()
    };
    let mut last_start = 0;
    let mut result = Vec::new();
    for (prev_index, pair) in chars.windows(2).enumerate() {
        let &[prev, this] = pair else {
            continue;
        };
        let n = prev_index + 1;
        if is_capital(this) && !is_capital(prev) {
            result.push(piece(last_start, n));
            last_start = n;
        } else if is_capital(prev) && !is_capital(this) && last_start != prev_index {
            result.push(piece(last_start, prev_index));
            last_start = prev_index;
        }
    }
    if last_start < chars.len() {
        result.push(piece(last_start, chars.len()));
    }
    result
}

/// `CreateOptions`: the radio's option names given values. Numbers are their own values (scaled
/// by [`scale_factor`]); two names are the range's ends; as many names as the range has values
/// are those values; more names than values are a frequency band's, grouped by the country each
/// ends with, a group's n-th name the n-th value's - which counts only once some country has a
/// name for every value.
/// `// C#: SikRadio/RFD900.cs:787-894`
#[must_use]
pub fn create_options(
    range: Option<&Range>,
    options: Option<&[String]>,
    name: &str,
    value: i32,
) -> Option<Vec<Choice>> {
    let (range, options) = (range?, options?);
    let ints: Option<Vec<i32>> = options.iter().map(|o| crate::try_parse_int(o)).collect();
    if let Some(ints) = ints {
        let scale = scale_factor(name, &ints, value);
        return Some(
            ints.iter()
                .zip(options)
                .map(|(i, o)| {
                    // `(int)(SF * Ints[n])`: single precision, truncated.
                    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
                    let scaled = (scale * *i as f32) as i32;
                    Choice::new(scaled, o.clone())
                })
                .collect(),
        );
    }
    let values = range.options_including_value(value);
    if let [low, high] = options {
        // Matched up with the min and the max.
        let (first, last) = (values.first()?, values.last()?);
        return Some(vec![
            Choice::new(*first, low.clone()),
            Choice::new(*last, high.clone()),
        ]);
    }
    if values.len() == options.len() {
        return Some(
            values
                .iter()
                .zip(options)
                .map(|(v, o)| Choice::new(*v, o.clone()))
                .collect(),
        );
    }
    if values.len() > options.len() {
        return None;
    }
    // The frequency band: names grouped by country.
    let mut names: Vec<Option<String>> = vec![None; values.len()];
    let mut country: Option<String> = None;
    let mut index = 0usize;
    let mut got_to_max = false;
    for option in options {
        let words = split_option_name(option);
        let Some(code) = words.last() else {
            continue;
        };
        if country.as_ref() == Some(code) {
            index += 1;
            if index + 1 >= values.len() {
                got_to_max = true;
            }
        } else {
            index = 0;
            country = Some(code.clone());
        }
        if let Some(slot) = names.get_mut(index) {
            let slot = slot.get_or_insert_with(String::new);
            if !slot.is_empty() {
                slot.push('/');
            }
            slot.push_str(option);
        }
    }
    if !got_to_max {
        return None;
    }
    Some(
        values
            .iter()
            .zip(names)
            .map(|(v, n)| Choice::new(*v, n.unwrap_or_default()))
            .collect(),
    )
}

/// `ParseATI5QueryResponseLine`: a line of `ATI5?` - `S2:AIR_SPEED(L)[4..1000]=125{4,64,125,}` or
/// `S2:AIR_SPEED=64` - as a `TSetting`, or `None` when it is not one (a `RESERVED` place is not
/// one). Every throw of the C#'s inside its `try` is a `None` here.
/// `// C#: SikRadio/RFD900.cs:910-989`
#[must_use]
pub fn parse_query_line(line: &str) -> Option<Setting> {
    if line.is_empty() {
        return None;
    }
    let c: Vec<char> = line.chars().collect();
    let mut n = 0;
    let designator = parse_designator(&c, &mut n)?;
    n += 1;
    let name = parse_name(&c, &mut n)?;
    if name.is_empty() || name == "RESERVED" {
        return None;
    }
    let mut range = None;
    if *c.get(n)? == '(' {
        parse_type(&c, &mut n)?;
        range = Some(parse_range(&c, &mut n, &name)??);
    }
    if *c.get(n)? != '=' {
        return None;
    }
    n += 1;
    let value = parse_int_until(&c, &mut n, None)??;
    let mut options = None;
    if let Some(&next) = c.get(n) {
        match next {
            '\r' => {}
            '{' => options = Some(parse_options(&c, &mut n)?),
            _ => return None,
        }
    }
    let choices = create_options(range.as_ref(), options.as_deref(), &name, value);
    let step = increment(&name);
    Some(Setting::full(designator, name, range, value, choices, step))
}

/// `ParseATI5ResponseLine`: a line of `ATI5` - `S1:SERIAL_SPEED=57` - as a `TShortSetting`.
/// `// C#: SikRadio/RFD900.cs:996-1030`
#[must_use]
pub fn parse_line(line: &str) -> Option<Setting> {
    let c: Vec<char> = line.chars().collect();
    let mut n = 0;
    let designator = parse_designator(&c, &mut n)?;
    n += 1;
    let name = parse_name(&c, &mut n)?;
    if name.is_empty() || *c.get(n)? != '=' {
        return None;
    }
    n += 1;
    let value = parse_int_until(&c, &mut n, None)??;
    Some(Setting::short(designator, name, value))
}

/// `ParseATI5QueryResponse`: every line of an `ATI5?` answer that is a setting, by name.
/// `// C#: SikRadio/RFD900.cs:1061-1074`
pub fn parse_query_response(response: &str, into: &mut Settings) {
    for line in response.split(['\n', '\r']) {
        if let Some(setting) = parse_query_line(line) {
            into.insert(setting.name.clone(), setting);
        }
    }
}

/// `ParseATI5Response`: every line of an `ATI5` answer that is a setting, by name.
/// `// C#: SikRadio/RFD900.cs:1076-1089`
pub fn parse_response(response: &str, into: &mut Settings) {
    for line in response.split(['\n', '\r']) {
        if let Some(setting) = parse_line(line) {
            into.insert(setting.name.clone(), setting);
        }
    }
}

/// `GetBaudRateOptionsGivenRawBaudRates`: each rate's value in thousands, named by itself.
/// `// C#: SikRadio/RFD900.cs:1032-1040`
fn baud_options(rates: &[i32]) -> Vec<Choice> {
    rates
        .iter()
        .map(|rate| Choice::new(rate / 1000, rate.to_string()))
        .collect()
}

/// `GetDefaultBaudRateSettingForBoard`: the RFD900x's ten rates, every other board's eight.
/// `// C#: SikRadio/RFD900.cs:1042-1054`
#[must_use]
pub fn default_baud_options(rfd900x: bool) -> Vec<Choice> {
    if rfd900x {
        baud_options(&[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115_200, 230_400, 460_800,
        ])
    } else {
        baud_options(&[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115_200])
    }
}

/// `CopySettingsButUseDifferentRanges`: each value of `values` whose name `settings` has as a
/// `TSetting`, with that setting's ranges and options.
/// `// C#: SikRadio/RFD900.cs:1221-1238`
#[must_use]
pub fn copy_with_ranges(values: &Settings, settings: &Settings) -> Settings {
    let mut result = Settings::new();
    for (name, short) in values.iter() {
        if let Some(full) = settings.get(name).filter(|s| s.is_full())
            && let Some(value) = short.value()
        {
            let mut setting = full.clone();
            setting.set_value(value);
            result.insert(name, setting);
        }
    }
    result
}

/// The work-around `TSession` makes for firmware that leaves settings out of `RTI5?`: these four,
/// used with the value `ATI5` gives when the query has none.
/// `// C#: SikRadio/RFD900.cs:39-44`
pub const DEFAULT_SETTINGS: [&str; 4] = [
    "S21:GPO1_3STATLED(N)[0..1]=0{Off,On,}\r\n",
    "S22:GPO1_0TXEN485(N)[0..1]=0{Off,On,}\r\n",
    "S23:RATE/FREQBAND(N)[0..3]=0{LoAus,HiAus,StdNZ,LoUSA,HiUSA,StdEU,LbtEU,StdPRC,StdINS,HiLbtJAP,HiStdJAP,LoLbtJAP,LoStdJAP,}\r\n",
    "S20:ANT_MODE(N)[0..3]=0{Ant1&2,Ant1,Ant2,Ant1=TX;2=RX,}\r\n",
];

/// `GetSettings(ATI5QResponse, Board, ATI5Response, Ranges, out UseRanges)`: the settings of an
/// `ATI5?` answer, with a board's baud rates where the serial speed had no options; with none,
/// `ranges`' ranges over the `ATI5` values (`UseRanges`); the four defaults where only `ATI5` had
/// them; and a multipoint radio's node destination given the node ids' range and its own last
/// value.
///
/// The C# puts the session's own default objects in the result and sets their values there, so a
/// later call's value shows through an earlier call's result; each result here has its own.
/// `// C#: SikRadio/RFD900.cs:1126-1219`
#[must_use]
pub fn get_settings(
    query_response: &str,
    rfd900x: bool,
    response: &str,
    ranges: Option<&Settings>,
) -> (Settings, bool) {
    let mut use_ranges = false;
    let mut result = Settings::new();
    parse_query_response(query_response, &mut result);
    if result.is_empty() {
        use_ranges = true;
    } else if let Some(serial) = result.get_mut("SERIAL_SPEED")
        && serial.is_full()
        && serial.options().is_none()
    {
        serial.set_options(Some(default_baud_options(rfd900x)));
    }
    let mut shorts = Settings::new();
    parse_response(response, &mut shorts);
    if use_ranges && let Some(ranges) = ranges {
        result = copy_with_ranges(&shorts, ranges);
    }
    for line in DEFAULT_SETTINGS {
        let Some(mut default) = parse_query_line(line) else {
            continue;
        };
        if let Some(value) = shorts.get(&default.name).and_then(Setting::value)
            && !result.contains(&default.name)
        {
            default.set_value(value);
            result.insert(default.name.clone(), default);
        }
    }
    widen_node_destination(&mut result);
    (result, use_ranges)
}

/// The multipoint fix: a node destination whose last option is past the node ids' last takes the
/// node ids' range and then that value alone.
/// `// C#: SikRadio/RFD900.cs:1178-1216`
fn widen_node_destination(result: &mut Settings) {
    let (Some(node_id), Some(dest)) = (result.get("NODEID"), result.get("NODEDESTINATION")) else {
        return;
    };
    if !node_id.is_full() || !dest.is_full() {
        return;
    }
    let (Some(id_range), Some(dest_range)) = (node_id.range(), dest.range()) else {
        return;
    };
    let Range::Simple {
        min,
        max,
        increment,
    } = *id_range
    else {
        return;
    };
    let id_options = id_range.options_including_value(node_id.value().unwrap_or(0));
    let dest_options = dest_range.options_including_value(dest.value().unwrap_or(0));
    if let (Some(&last_dest), Some(&last_id)) = (dest_options.last(), id_options.last())
        && last_dest > last_id
        && let Some(dest) = result.get_mut("NODEDESTINATION")
    {
        dest.set_range(Some(Range::Multi(vec![
            (min, max, increment),
            (last_dest, last_dest, 1),
        ])));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The documented lines: a full one with options, a plain one, a multipoint one.
    #[test]
    fn query_lines_parse_with_their_range_and_options() {
        let s = parse_query_line("S2:AIR_SPEED(L)[4..1000]=125{4,64,125,250,500,1000,}\r").unwrap();
        assert_eq!(s.designator, "S2");
        assert_eq!(s.name, "AIR_SPEED");
        assert_eq!(s.value(), Some(125));
        assert_eq!(
            s.range(),
            Some(&Range::Simple {
                min: 4,
                max: 1000,
                increment: 1
            })
        );
        let names = s.option_names().unwrap();
        assert_eq!(names, ["4", "64", "125", "250", "500", "1000"]);
        assert_eq!(s.option_name_for_value("250"), Some("250"));

        let plain = parse_query_line("S1:SERIAL_SPEED=57").unwrap();
        assert!(plain.is_full());
        assert_eq!(plain.range(), None);
        assert_eq!(plain.options(), None);

        let node = parse_query_line("[1] S3:NETID(N)[0..499]=25").unwrap();
        assert_eq!(node.designator, "S3");
        assert_eq!(parse_query_line("S9:RESERVED=0"), None);
        assert_eq!(parse_query_line("ERROR"), None);
        assert_eq!(parse_query_line(""), None);
        assert_eq!(
            parse_query_line("S2:AIR_SPEED(L)[4..1000]=125{4,64"),
            None,
            "runs off"
        );
        assert_eq!(parse_query_line("S2:AIR_SPEED(L)[4..]=125"), None);
        assert_eq!(parse_query_line("S2:AIR_SPEED=x"), None);
        assert_eq!(
            parse_query_line("S2:AIR_SPEED=1 "),
            None,
            "neither \\r nor {{"
        );
        assert_eq!(parse_line("S1:SERIAL_SPEED=57").unwrap().value(), Some(57));
        assert_eq!(parse_line("S1:SERIAL_SPEED(N)[1..2]=57"), None);
    }

    /// The baud rates given raw are scaled to the setting's thousands, single precision and
    /// truncated; a flag is a range of 0 and 1.
    #[test]
    fn options_are_scaled_and_named() {
        let s = parse_query_line(
            "S1:SERIAL_SPEED(N)[1..115]=57{1200,2400,4800,9600,19200,38400,57600,115200,}",
        )
        .unwrap();
        let values: Vec<i32> = s.options().unwrap().iter().map(|o| o.value).collect();
        assert_eq!(values, [1, 2, 4, 9, 19, 38, 57, 115]);
        assert_eq!(s.option_name_for_value("57"), Some("57600"));
        let flag = parse_query_line("S5:ECC(N)[0..1]=0").unwrap();
        assert!(flag.is_flag());
        let level = parse_query_line("S16:ENCRYPTION_LEVEL(N)[0..1]=0{Off,128b,}").unwrap();
        assert_eq!(
            level.options().unwrap(),
            [Choice::new(0, "Off"), Choice::new(1, "128b")]
        );
        let mav = parse_query_line("S6:MAVLINK(N)[0..2]=1{RawData,Mavlink,LowLatency,}").unwrap();
        assert_eq!(mav.option_name_for_value("2"), Some("LowLatency"));
    }

    /// The frequency band's names grouped by the country each ends with.
    #[test]
    fn band_options_group_by_country() {
        let band = parse_query_line(DEFAULT_SETTINGS[2]).unwrap();
        let names = band.option_names().unwrap();
        assert_eq!(
            names,
            [
                "LoAus/StdNZ/LoUSA/StdEU/StdPRC/StdINS/HiLbtJAP",
                "HiAus/HiUSA/LbtEU/HiStdJAP",
                "LoLbtJAP",
                "LoStdJAP"
            ]
        );
        assert_eq!(split_option_name("HiLbtJAP"), ["Hi", "Lbt", "JAP"]);
        assert_eq!(split_option_name("LoAus"), ["Lo", "Aus"]);
        let ant = parse_query_line(DEFAULT_SETTINGS[3]).unwrap();
        assert_eq!(ant.option_names().unwrap()[3], "Ant1=TX;2=RX");
    }

    /// Ranges step to their end, the end always last, and a value outside put in its place.
    #[test]
    fn ranges_end_at_their_end() {
        assert_eq!(range(0, 3, 10), [0, 3, 6, 9, 10]);
        assert_eq!(range(5, 1, 5), [5]);
        assert_eq!(range(414_000, 50, 414_100), [414_000, 414_050, 414_100]);
        let r = Range::Simple {
            min: 20,
            max: 100,
            increment: 20,
        };
        assert_eq!(r.options(), [20, 40, 60, 80, 100]);
        assert_eq!(r.options_including_value(50), [20, 40, 50, 60, 80, 100]);
        assert_eq!(r.options_including_value(131), [20, 40, 60, 80, 100, 131]);
        let backwards = Range::Simple {
            min: 9,
            max: 3,
            increment: 1,
        };
        assert_eq!(backwards.options(), [9]);
        let multi = Range::Multi(vec![(0, 2, 1), (65535, 65535, 1)]);
        assert_eq!(multi.options(), [0, 1, 2, 65535]);
    }

    /// `GetSettings`: the query's settings, the board's baud rates where the radio gave none, the
    /// defaults from `ATI5` where the query lacks them, the ranges of another radio's settings
    /// when the query gave nothing, and the node destination widened.
    #[test]
    fn get_settings_fills_in_what_the_query_left_out() {
        let query = "S1:SERIAL_SPEED(N)[1..115]=57\r\nS2:AIR_SPEED(N)[2..250]=64\r\n";
        let plain = "S1:SERIAL_SPEED=57\r\nS2:AIR_SPEED=64\r\nS23:RATE/FREQBAND=2\r\n";
        let (local, used) = get_settings(query, true, plain, None);
        assert!(!used);
        assert_eq!(
            local.names().collect::<Vec<_>>(),
            ["SERIAL_SPEED", "AIR_SPEED", "RATE/FREQBAND"]
        );
        assert_eq!(
            local.get("SERIAL_SPEED").unwrap().options().unwrap().len(),
            10
        );
        assert_eq!(local.get("RATE/FREQBAND").unwrap().value(), Some(2));

        let (remote, used) = get_settings("ERROR\r\n", false, "S2:AIR_SPEED=128\r\n", Some(&local));
        assert!(used);
        let air = remote.get("AIR_SPEED").unwrap();
        assert_eq!(air.value(), Some(128));
        assert!(air.range().is_some(), "the local radio's range");

        let multipoint = "S10:NODEID(N)[0..29]=1\r\nS11:NODEDESTINATION(N)[0..65535]=65535\r\n";
        let (settings, _) = get_settings(multipoint, false, "", None);
        let dest = settings.get("NODEDESTINATION").unwrap();
        assert_eq!(
            dest.range(),
            Some(&Range::Multi(vec![(0, 29, 1), (65535, 65535, 1)]))
        );
    }

    /// The settings file: sorted as the culture sorts, and read back by name.
    #[test]
    fn the_file_writes_sorted_and_reads_by_name() {
        let mut s = Settings::new();
        s.insert(
            "SERIAL_SPEED",
            Setting::full("S1", "SERIAL_SPEED", None, 57, None, 1),
        );
        s.insert(
            "SER_BRK_DETMS",
            Setting::full("S30", "SER_BRK_DETMS", None, 5, None, 1),
        );
        s.insert("AESKEY", Setting::text("&E", "AESKEY", "0123"));
        s.insert(
            "AIR_SPEED",
            Setting::full("S2", "AIR_SPEED", None, 64, None, 1),
        );
        let text = s.to_file_text();
        let names: Vec<&str> = text
            .lines()
            .map(|l| l.split(" = ").next().unwrap())
            .collect();
        assert_eq!(
            names,
            ["AESKEY", "AIR_SPEED", "SER_BRK_DETMS", "SERIAL_SPEED"]
        );
        assert!(text.contains("AESKEY = 0123"));

        let loaded = s.load_from_text(
            "; a comment\nAIR_SPEED = 128 ; fast\n# another\nNOT_THERE = 1\nSERIAL_SPEED = x\nAESKEY=ABCD\n",
        );
        assert_eq!(
            loaded,
            [
                ("AIR_SPEED".to_owned(), "128".to_owned()),
                ("SERIAL_SPEED".to_owned(), "x".to_owned()),
                ("AESKEY".to_owned(), "ABCD".to_owned())
            ]
        );
        assert_eq!(s.get("AIR_SPEED").unwrap().value(), Some(128));
        assert_eq!(
            s.get("SERIAL_SPEED").unwrap().value(),
            Some(0),
            "TryParse's 0"
        );
        assert_eq!(s.get("AESKEY").unwrap().text_value(), Some("ABCD"));
    }

    /// `CheckValid`: the minimum frequency above the maximum.
    #[test]
    fn a_minimum_above_the_maximum_is_invalid() {
        let mut s = Settings::new();
        s.insert(
            "MIN_FREQ",
            Setting::full("S8", "MIN_FREQ", None, 928_000, None, 500),
        );
        s.insert(
            "MAX_FREQ",
            Setting::full("S9", "MAX_FREQ", None, 915_000, None, 500),
        );
        assert_eq!(s.check_valid(), ["MIN_FREQ can't be more than MAX_FREQ"]);
        s.get_mut("MIN_FREQ").unwrap().set_value(902_000);
        assert!(s.check_valid().is_empty());
    }
}
