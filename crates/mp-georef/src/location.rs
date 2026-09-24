//! What the log and the photos are turned into: `SingleLocation`, its two subclasses, and the
//! dictionary order the C#'s `Dictionary<,>` gives them.

use std::collections::HashMap;
use std::hash::Hash;

use crate::time::DateTime;

/// `SingleLocation` (and `VehicleLocation`, which adds nothing): a position and attitude at a
/// time. The field types are the C#'s - altitudes `double`, angles `float` - because they decide
/// how each value prints.
/// `// C#: ExtLibs/Utilities/SingleLocation.cs:8-101, VehicleLocation.cs:7-10`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Location {
    /// `Time`.
    pub time: DateTime,
    /// `Lat`.
    pub lat: f64,
    /// `Lon`.
    pub lon: f64,
    /// `AltAMSL`.
    pub alt_amsl: f64,
    /// `RelAlt`.
    pub rel_alt: f64,
    /// `GPSAlt`.
    pub gps_alt: f64,
    /// `SAlt`: the rangefinder's or `CTUN`'s sonar altitude.
    pub s_alt: f64,
    /// `Roll`.
    pub roll: f32,
    /// `Pitch`.
    pub pitch: f32,
    /// `Yaw`.
    pub yaw: f32,
}

impl Location {
    /// `getAltitude`: the GPS altitude if asked for, else AMSL or relative.
    /// `// C#: ExtLibs/Utilities/SingleLocation.cs:90-95`
    #[must_use]
    pub const fn get_altitude(&self, amsl: bool, gpsalt: bool) -> f64 {
        if gpsalt {
            self.gps_alt
        } else if amsl {
            self.alt_amsl
        } else {
            self.rel_alt
        }
    }
}

/// `PictureInformation`: a photo, where and when it was taken.
/// `// C#: ExtLibs/Utilities/PictureInformation.cs:8-47`
#[derive(Debug, Clone, PartialEq)]
pub struct PictureInformation {
    /// The position and attitude matched to the photo.
    pub location: Location,
    /// `Path`: the photo's path as `Directory.GetFiles` gave it.
    pub path: String,
    /// `ShotTimeReportedByCamera`: its EXIF time.
    pub shot_time_reported_by_camera: DateTime,
    /// `Width`, 3200 unless set - and nothing sets it.
    pub width: i32,
    /// `Height`, 2400 unless set.
    pub height: i32,
}

impl Default for PictureInformation {
    /// `new PictureInformation()`. `// C#: ExtLibs/Utilities/PictureInformation.cs:42-46`
    fn default() -> Self {
        Self {
            location: Location::default(),
            path: String::new(),
            shot_time_reported_by_camera: DateTime::MIN,
            width: 3200,
            height: 2400,
        }
    }
}

/// A `Dictionary<TKey, TValue>` as `GeoRefImageBase` uses one: enumerated in insertion order, a
/// key set again keeping its first place.
///
/// .NET's dictionary enumerates its entry array, so that is the order - until an entry is removed
/// and a later insert reuses its slot. `GeoRefImageBase` never inserts into a dictionary it has
/// removed from (`camLocations` is filtered and then only read, `GeoRefImageBase.cs:786-792`), so
/// removing here closes the gap instead of keeping a free list.
#[derive(Debug, Clone)]
pub struct OrderedMap<K, V> {
    entries: Vec<(K, V)>,
    index: HashMap<K, usize>,
}

impl<K, V> Default for OrderedMap<K, V> {
    fn default() -> Self {
        Self {
            entries: Vec::new(),
            index: HashMap::new(),
        }
    }
}

impl<K: Clone + Eq + Hash, V> OrderedMap<K, V> {
    /// An empty dictionary.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// `Count`.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// `Count == 0`.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// `ContainsKey`.
    #[must_use]
    pub fn contains_key(&self, key: &K) -> bool {
        self.index.contains_key(key)
    }

    /// `this[key]` read, or `None` where the C# throws `KeyNotFoundException`.
    #[must_use]
    pub fn get(&self, key: &K) -> Option<&V> {
        let at = *self.index.get(key)?;
        self.entries.get(at).map(|(_, v)| v)
    }

    /// `this[key] = value`: replaced in place, or added at the end.
    pub fn set(&mut self, key: K, value: V) {
        if let Some(&at) = self.index.get(&key) {
            if let Some(slot) = self.entries.get_mut(at) {
                slot.1 = value;
            }
        } else {
            self.index.insert(key.clone(), self.entries.len());
            self.entries.push((key, value));
        }
    }

    /// `Remove(key)`.
    pub fn remove(&mut self, key: &K) -> bool {
        let Some(at) = self.index.remove(key) else {
            return false;
        };
        self.entries.remove(at);
        for slot in self.index.values_mut() {
            if *slot > at {
                *slot -= 1;
            }
        }
        true
    }

    /// `Clear()`.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.index.clear();
    }

    /// The entries in enumeration order.
    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.entries.iter().map(|(k, v)| (k, v))
    }

    /// `Keys`.
    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.entries.iter().map(|(k, _)| k)
    }

    /// `Values`.
    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.entries.iter().map(|(_, v)| v)
    }
}

impl<K: Clone + Eq + Hash, V> FromIterator<(K, V)> for OrderedMap<K, V> {
    /// `ToDictionary`, which throws on a repeated key; the sources here never repeat one.
    fn from_iter<T: IntoIterator<Item = (K, V)>>(iter: T) -> Self {
        let mut map = Self::new();
        for (k, v) in iter {
            map.set(k, v);
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_key_set_again_keeps_its_first_place() {
        let mut map = OrderedMap::new();
        map.set(3_i64, "a");
        map.set(1, "b");
        map.set(3, "c");
        assert_eq!(map.iter().collect::<Vec<_>>(), [(&3, &"c"), (&1, &"b")]);
        assert!(map.remove(&3));
        map.set(7, "d");
        assert_eq!(map.keys().copied().collect::<Vec<_>>(), [1, 7]);
        assert_eq!(map.get(&7), Some(&"d"));
    }

    #[test]
    fn altitude_picks_gps_first() {
        let l = Location {
            alt_amsl: 1.0,
            rel_alt: 2.0,
            gps_alt: 3.0,
            ..Location::default()
        };
        assert_eq!(l.get_altitude(true, true), 3.0);
        assert_eq!(l.get_altitude(true, false), 1.0);
        assert_eq!(l.get_altitude(false, false), 2.0);
    }
}
