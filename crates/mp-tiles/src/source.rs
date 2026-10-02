//! Tile providers: where tiles come from and what must be said about them.
//!
//! The providers are Mission Planner's, in Mission Planner's order, followed by the two this crate
//! had before the port began. The C#'s map-type box is `GMapProviders.List` handed over whole
//! (`GCSViews/FlightPlanner.cs:176-178`), and that list is every `GMapProvider` field of the
//! `GMapProviders` class in declaration order, with Mission Planner's own providers appended at
//! startup (`GMap.NET.MapProviders/GMapProvider.cs:24-36`, `Program.cs:328-351`). [`CSHARP_LIST`]
//! is that list by name; [`SOURCES`] is the part of it ported so far, in the same order.

use mp_units::TileId;

use crate::gmap;
use crate::versions::{self, Correction};

/// A raster tile provider.
///
/// Deliberately data rather than a trait. Every provider here is "substitute the tile into a URL",
/// and a trait would invite an implementation that does something else - which is how a map ends
/// up with one provider that ignores the cache or blocks the render thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TileSource {
    /// Short identifier, used in settings and `MP_TILE_SOURCE`. Stable: changing it forgets the
    /// operator's choice.
    pub id: &'static str,
    /// The directory this provider's tiles are cached under, which is the C# provider's `Name`.
    ///
    /// Not `id`. The cache is laid out as the C# application lays out its own, which files tiles
    /// under `GMapProviders.TryGetProvider(type).Name` - so this must be that string exactly, or a
    /// copy of the C#'s cache is a second cache of the same imagery that nothing here finds.
    /// Stable for the same reason `id` is, and more so: changing it orphans gigabytes.
    /// It is also the name `config.xml`'s `MapType` holds.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:72; GCSViews/FlightPlanner.cs:2237`
    pub cache_name: &'static str,
    /// What to call it on screen. For the C#'s providers, the `Name` again: the C#'s box shows
    /// `ToString()`, which is `Name` (`GMapProvider.cs:482-485`).
    pub label: &'static str,
    /// The URL, with placeholders in braces:
    ///
    /// - `{z}`, `{x}`, `{y}`: the tile.
    /// - `{s}`: one of [`Self::subdomains`], chosen by `(x + y)` - this crate's own rotation.
    /// - `{n}`: GMap.NET's `GetServerNum(pos, servers)`, `(x + 2y) % servers` ([`gmap::server_num`]).
    /// - `{v}`: the provider's version, as corrected at runtime ([`versions::current`]).
    /// - `{hl}`: the language, `"en"` ([`gmap::LANGUAGE`]).
    /// - `{q}`: the Bing quadkey ([`gmap::quad_key`]).
    /// - `{sec1}`, `{sec2}`: Google's secure words ([`gmap::secure_words`]).
    ///
    /// For a C# provider this is its `UrlFormat` with the constant arguments written in and each
    /// per-tile argument named, so the two can be read side by side.
    pub url: &'static str,
    /// Subdomains for `{s}`, or empty if the provider has none.
    ///
    /// Rotating spreads a screenful of tiles over several hostnames, which is what the providers
    /// that offer subdomains expect. A provider that does not offer them must not be given any:
    /// requesting `a.tile.example.com` when only `tile.example.com` exists fails every tile.
    pub subdomains: &'static [&'static str],
    /// The `max` GMap.NET's `GetServerNum` is called with for `{n}`, or zero if the URL has none.
    pub servers: u8,
    /// The version `{v}` stands for until a check finds a newer one: the C# provider's hard-coded
    /// `Version`. Empty for a provider with none.
    pub version: &'static str,
    /// Which version check the provider runs the first time it is shown.
    pub correction: Correction,
    /// The `Referer` sent with every request, `GMapProvider.RefererUrl`, or empty for none.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:341, 405-406`
    pub referer: &'static str,
    /// Deepest zoom the provider serves. Asking for more returns errors or blank tiles.
    pub max_zoom: u8,
    /// Attribution text, shown on the map. For the C#'s providers, its `Copyright`, where `{0}` is
    /// the year - use [`Self::attribution_text`] to show it.
    ///
    /// Not optional: every provider here requires it, and a map that shows someone else's data
    /// without saying whose is not one worth shipping.
    pub attribution: &'static str,
}

impl TileSource {
    /// Whether the provider fetches at all: `Custom` has no URL and serves its cache alone.
    #[must_use]
    pub const fn fetches(&self) -> bool {
        !self.url.is_empty()
    }

    /// The URL for one tile, or `None` if the provider does not serve that zoom - or does not
    /// fetch at all.
    #[must_use]
    pub fn url_for(&self, tile: TileId) -> Option<String> {
        if tile.z > self.max_zoom || !self.fetches() {
            return None;
        }
        let mut out = String::with_capacity(self.url.len() + 32);
        let mut rest = self.url;
        loop {
            let Some((before, after)) = rest.split_once('{') else {
                out.push_str(rest);
                break;
            };
            out.push_str(before);
            let Some((name, tail)) = after.split_once('}') else {
                out.push('{');
                out.push_str(after);
                break;
            };
            self.substitute(name, tile, &mut out);
            rest = tail;
        }
        Some(out)
    }

    /// Writes one placeholder's value, or the placeholder itself if it has none.
    fn substitute(&self, name: &str, tile: TileId, out: &mut String) {
        use std::fmt::Write as _;
        let _ = match name {
            "z" => write!(out, "{}", tile.z),
            "x" => write!(out, "{}", tile.x),
            "y" => write!(out, "{}", tile.y),
            "n" => write!(out, "{}", gmap::server_num(tile.x, tile.y, self.servers)),
            "v" => write!(out, "{}", versions::current(self)),
            "hl" => write!(out, "{}", gmap::LANGUAGE),
            "q" => write!(out, "{}", gmap::quad_key(tile.x, tile.y, tile.z)),
            "sec1" => write!(out, "{}", gmap::secure_words(tile.x, tile.y).0),
            "sec2" => write!(out, "{}", gmap::secure_words(tile.x, tile.y).1),
            "s" if !self.subdomains.is_empty() => {
                write!(out, "{}", self.subdomain_for(tile).unwrap_or_default())
            }
            other => write!(out, "{{{other}}}"),
        };
    }

    /// Which subdomain a tile should come from.
    ///
    /// Chosen from the tile's own coordinates rather than a counter, so the same tile always goes
    /// to the same host. A counter would send a tile to a different host on every pan, defeating
    /// every HTTP cache between here and the provider.
    fn subdomain_for(&self, tile: TileId) -> Option<&'static str> {
        if self.subdomains.is_empty() {
            return None;
        }
        // Summed as u64 and reduced with a u64 modulus, so nothing truncates on a 32-bit target.
        let index = (u64::from(tile.x) + u64::from(tile.y)) % self.subdomains.len() as u64;
        self.subdomains.get(usize::try_from(index).ok()?).copied()
    }

    /// The attribution as it is shown, with the year written in.
    ///
    /// The C#'s providers format their `Copyright` with `DateTime.Today.Year` once, when the
    /// provider is constructed; this does it once per process too, and keeps the result for the
    /// life of it. The handful of strings this can make are held for good, which is what lets the
    /// map borrow one without owning it.
    #[must_use]
    pub fn attribution_text(&self) -> &'static str {
        use chrono::Datelike as _;
        use std::collections::BTreeMap;
        use std::sync::Mutex;

        static FORMATTED: Mutex<BTreeMap<&'static str, &'static str>> = Mutex::new(BTreeMap::new());

        if !self.attribution.contains("{0}") {
            return self.attribution;
        }
        let Ok(mut formatted) = FORMATTED.lock() else {
            return self.attribution;
        };
        formatted.entry(self.attribution).or_insert_with(|| {
            let year = chrono::Local::now().year().to_string();
            Box::leak(self.attribution.replace("{0}", &year).into_boxed_str())
        })
    }
}

/// The deepest zoom of the tile grid, which is what a provider gets when the C# sets no
/// `MaxZoom`: the C# then leaves the limit to the map control, which allows 24
/// (`GCSViews/FlightPlanner.cs:188`), and the grid here stops before that.
const UNLIMITED: u8 = mp_units::tiles::MAX_ZOOM;

/// OpenStreetMap's standard raster tiles.
///
/// Their usage policy requires a real identifying User-Agent, forbids bulk downloading, and asks
/// that clients not send more than a couple of requests at a time. All three are honoured in
/// `fetch.rs`; this is a ground station showing the operator their own surroundings, which is the
/// use the policy contemplates.
///
/// The C# sets no `MaxZoom` for it (`OpenStreetMapProvider.cs:16`). 19 is kept here from before
/// the port, because 19 is where OpenStreetMap's servers stop and a deeper request is an error.
/// The attribution is also this crate's own rather than the C#'s `Copyright`
/// (`OpenStreetMapProvider.cs:18`); both are for the owner to settle, and neither is changed here.
pub const OPENSTREETMAP: TileSource = TileSource {
    id: "osm",
    // `readonly string name = "OpenStreetMap"` - the directory the C# has been filing these under.
    // C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:508
    cache_name: "OpenStreetMap",
    label: "OpenStreetMap",
    // C#: OpenStreetMapProvider.cs:547 - "https://tile.openstreetmap.org/{1}/{2}/{3}.png"
    url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    // OSM retired its a/b/c subdomains; using them now is a redirect at best.
    subdomains: &[],
    servers: 0,
    version: "",
    correction: Correction::None,
    // C#: OpenStreetMapProvider.cs:17
    referer: "https://www.openstreetmap.org/",
    max_zoom: 19,
    attribution: "© OpenStreetMap contributors",
};

/// The Google constructor's shared settings: no `MaxZoom`, Google Maps as the referer, and the
/// copyright line.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:22-27`
const GOOGLE_REFERER: &str = "https://www.google.com/maps/preview";
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:26`
const GOOGLE_COPYRIGHT: &str = "©{0} Google - Map data ©{0} Tele Atlas, Imagery ©{0} TerraMetrics";

/// Google's road map, `GoogleMap`.
///
/// The URL is `UrlFormat` = `https://{0}{1}.{10}/{2}/lyrs={3}&hl={4}&x={5}{6}&y={7}&z={8}&s={9}`
/// with `{0}` = `"mts"`, `{1}` = `GetServerNum(pos, 4)`, `{10}` = `Server`, `{2}` = `"vt"`, `{3}` =
/// `Version`, `{4}` = the language, `{5}`/`{7}`/`{8}` = x, y and zoom, and `{6}`/`{9}` = the
/// secure words. `Server` is stored encrypted (`Stuff.GString("gosr2U13BoS+bXaIxt6XWg==")`); it
/// decrypts - SHA-1 of GMap.NET's passphrase as a two-key 3DES key, ECB, PKCS7 - to `google.com`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:2084-2139, 32;
/// GMap.NET.Internals/Stuff.cs:148-178`
pub const GOOGLE_MAP: TileSource = TileSource {
    id: "google-map",
    // `Resources.Strings.GoogleMap`, whose English value is "GoogleMap". The C# names providers
    // in the interface language, so a Chinese-language installation files these elsewhere.
    // C#: GoogleMapProvider.cs:2110; ExtLibs/GMap.NET.Core/Resources/Strings.resx:150-151
    cache_name: "GoogleMap",
    label: "GoogleMap",
    url: "https://mts{n}.google.com/vt/lyrs={v}&hl={hl}&x={x}{sec1}&y={y}&z={z}&s={sec2}",
    subdomains: &[],
    servers: 4,
    // C#: GoogleMapProvider.cs:2097
    version: "m@354000000",
    correction: Correction::Google,
    referer: GOOGLE_REFERER,
    max_zoom: UNLIMITED,
    attribution: GOOGLE_COPYRIGHT,
};

/// Google's imagery, `GoogleSatelliteMap`: Mission Planner's default map.
///
/// `UrlFormat` = `https://{0}{1}.{10}/{2}/v={3}&hl={4}&x={5}{6}&y={7}&z={8}&s={9}`, with `{0}` =
/// `"khms"` and `{2}` = `"kh"`; the other arguments are as for [`GOOGLE_MAP`]. The version is a bare
/// number, 955 until the loader script says otherwise - which on this machine, on 2026-09-20, it
/// said was 1015 ([`versions::google_versions`]).
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleSatelliteMapProvider.cs:22-64`
pub const GOOGLE_SATELLITE_MAP: TileSource = TileSource {
    id: "google-satellite",
    // C#: GoogleSatelliteMapProvider.cs:35; Resources/Strings.resx:153-154
    cache_name: "GoogleSatelliteMap",
    label: "GoogleSatelliteMap",
    url: "https://khms{n}.google.com/kh/v={v}&hl={hl}&x={x}{sec1}&y={y}&z={z}&s={sec2}",
    subdomains: &[],
    servers: 4,
    // C#: GoogleSatelliteMapProvider.cs:22
    version: "955",
    correction: Correction::Google,
    referer: GOOGLE_REFERER,
    max_zoom: UNLIMITED,
    attribution: GOOGLE_COPYRIGHT,
};

/// Google's terrain map, `GoogleTerrainMap`.
///
/// `UrlFormat` = `https://{0}{1}.{10}/maps/{2}/lyrs={3}&hl={4}&x={5}{6}&y={7}&z={8}&s={9}`, with
/// `{0}` = `"mts"` and `{2}` = `"vt"` - the road map's, with `/maps` in front.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleTerrainMapProvider.cs:22-64`
pub const GOOGLE_TERRAIN_MAP: TileSource = TileSource {
    id: "google-terrain",
    // C#: GoogleTerrainMapProvider.cs:35; Resources/Strings.resx:156-157
    cache_name: "GoogleTerrainMap",
    label: "GoogleTerrainMap",
    url: "https://mts{n}.google.com/maps/vt/lyrs={v}&hl={hl}&x={x}{sec1}&y={y}&z={z}&s={sec2}",
    subdomains: &[],
    servers: 4,
    // C#: GoogleTerrainMapProvider.cs:22
    version: "t@354,r@354000000",
    correction: Correction::Google,
    referer: GOOGLE_REFERER,
    max_zoom: UNLIMITED,
    attribution: GOOGLE_COPYRIGHT,
};

/// The Bing constructor's shared settings.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:19-26`
const BING_REFERER: &str = "http://www.bing.com/maps/";
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:23`
const BING_COPYRIGHT: &str = "©{0} Microsoft Corporation, ©{0} NAVTEQ, ©{0} Image courtesy of NASA";
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:26`
const BING_VERSION: &str = "4810";

/// Bing's road map, `BingMap`.
///
/// `UrlFormat` = `http://ecn.t{0}.tiles.virtualearth.net/tiles/r{1}?g={2}&mkt={3}&lbl=l1&stl=h&shading=hill&n=z{4}`,
/// with `{0}` = `GetServerNum(pos, 4)`, `{1}` = the quadkey, `{2}` = `Version`, `{3}` = the
/// language, and `{4}` = `"&key=" + ClientKey` when there is a session key and empty when there is
/// not. There is none here ([`versions::correct_bing`] says why), so `{4}` is always empty - the URL
/// the C# sends when its own key request fails. Plain `http`, as the C# has it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:578-629`
pub const BING_MAP: TileSource = TileSource {
    id: "bing-map",
    // C#: BingMapProvider.cs:602; Resources/Strings.resx:129-130
    cache_name: "BingMap",
    label: "BingMap",
    url: "http://ecn.t{n}.tiles.virtualearth.net/tiles/r{q}?g={v}&mkt={hl}&lbl=l1&stl=h&shading=hill&n=z",
    subdomains: &[],
    servers: 4,
    version: BING_VERSION,
    correction: Correction::Bing,
    referer: BING_REFERER,
    max_zoom: UNLIMITED,
    attribution: BING_COPYRIGHT,
};

/// Bing's imagery, `BingSatelliteMap`.
///
/// `UrlFormat` = `http://ecn.t{0}.tiles.virtualearth.net/tiles/a{1}.jpeg?g={2}&mkt={3}&n=z{4}`,
/// arguments as for [`BING_MAP`].
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingSatelliteMapProvider.cs:51-59`
pub const BING_SATELLITE_MAP: TileSource = TileSource {
    id: "bing-satellite",
    // C#: BingSatelliteMapProvider.cs:33; Resources/Strings.resx:132-133
    cache_name: "BingSatelliteMap",
    label: "BingSatelliteMap",
    url: "http://ecn.t{n}.tiles.virtualearth.net/tiles/a{q}.jpeg?g={v}&mkt={hl}&n=z",
    ..BING_MAP
};

/// Bing's imagery with labels, `BingHybridMap`.
///
/// One layer, unlike Google's hybrid: Bing composites the labels into the tile, and the C#'s
/// `Overlays` for it is the provider alone (`BingMapProvider.cs:126-137`).
/// `UrlFormat` = `http://ecn.t{0}.tiles.virtualearth.net/tiles/h{1}.jpeg?g={2}&mkt={3}&n=z{4}`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingHybridMapProvider.cs:51-59`
pub const BING_HYBRID_MAP: TileSource = TileSource {
    id: "bing-hybrid",
    // C#: BingHybridMapProvider.cs:33; Resources/Strings.resx:126-127
    cache_name: "BingHybridMap",
    label: "BingHybridMap",
    url: "http://ecn.t{n}.tiles.virtualearth.net/tiles/h{q}.jpeg?g={v}&mkt={hl}&n=z",
    ..BING_MAP
};

/// `Custom`: the provider Inject Custom Map fills - no URL, `GetTileImage` returns null, so a
/// tile is what the cache holds and nothing else; `MaxZoom = 22`.
/// `// C#: ExtLibs/Maps/Custom.cs:20-23, 45, 75-82`
pub const CUSTOM: TileSource = TileSource {
    id: "custom",
    cache_name: "Custom",
    label: "Custom",
    url: "",
    subdomains: &[],
    servers: 0,
    version: "",
    correction: Correction::None,
    referer: "",
    max_zoom: 22,
    attribution: "",
};

/// OpenTopoMap: contour lines and hillshading, which is what a pilot wants over terrain.
///
/// **Not a Mission Planner provider.** Nothing in the C# fetches from opentopomap.org, so there is
/// no C# name to file its tiles under and no existing cache to share; the directory name is ours.
/// Whether it stays is a question for the owner under the not-in-the-C# rule, not one to settle
/// while porting the cache - which is also why it comes after every provider the C# has.
pub const OPENTOPOMAP: TileSource = TileSource {
    id: "opentopo",
    cache_name: "OpenTopoMap",
    label: "OpenTopoMap",
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    subdomains: &["a", "b", "c"],
    servers: 0,
    version: "",
    correction: Correction::None,
    referer: "",
    max_zoom: 17,
    attribution: "© OpenStreetMap contributors, SRTM | © OpenTopoMap (CC-BY-SA)",
};

/// Esri's World Imagery: aerial and satellite photography.
///
/// Planning over a paddock needs imagery, not a street map - a survey grid over farmland is drawn
/// against field boundaries and tree lines that a road map does not have, and a landing site is
/// chosen by looking at the ground.
///
/// **The URL is `{z}/{y}/{x}`, not `{z}/{x}/{y}`.** ArcGIS orders it row-then-column where every
/// other provider here orders it column-then-row, and the C# says so plainly:
/// `string.Format(UrlFormat, zoom, pos.Y, pos.X)`. Getting it backwards does not fail - it
/// returns a real tile, of somewhere else, and the map looks like imagery that does not match the
/// roads under it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/ArcGIS/ArcGIS_Imagery_World_2D_MapProvider.cs:55`
///
/// **The endpoint is not the one Mission Planner ships.** Its provider points at
/// `ESRI_Imagery_World_2D`, which Esri retired - it answers 301 today. `World_Imagery` is the
/// current service and returns tiles. PLAN.md §9.1 predicted this ("audit the dead endpoints");
/// this is one of them, found by asking the server rather than by reading the C#.
///
/// **Not a Mission Planner provider either**, for that reason: the C#'s is
/// `ArcGIS_Imagery_World_2D_Map`, at the retired address, and this directory name is ours and
/// shares nothing. Mission Planner's default imagery is [`GOOGLE_SATELLITE_MAP`]. Like
/// [`OPENTOPOMAP`], it comes after every provider the C# has, pending the owner's decision.
pub const ESRI_WORLD_IMAGERY: TileSource = TileSource {
    id: "esri-imagery",
    cache_name: "EsriWorldImagery",
    label: "Satellite (Esri)",
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    subdomains: &[],
    servers: 0,
    version: "",
    correction: Correction::None,
    referer: "",
    // Esri serves 23 levels, but coverage above 17 is patchy outside cities and a missing tile
    // costs a request and a blank square. 19 is where the imagery generally stops being useful.
    max_zoom: 19,
    attribution: "Imagery © Esri, Maxar, Earthstar Geographics, and the GIS User Community",
};

/// Every provider offered: the C#'s, in [`CSHARP_LIST`]'s order, then this crate's own two.
pub const SOURCES: &[TileSource] = &[
    OPENSTREETMAP,
    BING_MAP,
    BING_SATELLITE_MAP,
    BING_HYBRID_MAP,
    GOOGLE_MAP,
    GOOGLE_SATELLITE_MAP,
    GOOGLE_TERRAIN_MAP,
    CUSTOM,
    OPENTOPOMAP,
    ESRI_WORLD_IMAGERY,
];

/// The map Mission Planner shows when `config.xml` names none: `GoogleSatelliteMap`.
///
/// Unless the interface language is simplified Chinese, when it is Google's China imagery - a
/// provider that shifts coordinates onto China's GCJ-02 datum, which is not ported.
/// `// C#: GCSViews/FlightPlanner.cs:7247-7295` (the default itself at `:7282`)
pub const DEFAULT_PROVIDER: &str = "GoogleSatelliteMap";

/// Every provider the C# application offers, by `Name`, in the order its map-type box lists them.
///
/// Built as the C# builds it: each `GMapProvider` field of `GMapProviders`, in declaration order -
/// which is the order `Type.GetFields()` returns them in under both .NET and mono, and the order a
/// Mission Planner `GMap.NET.Core.dll` gave when asked (`tests/fixtures/gmap-oracle.tsv`) - then
/// the providers `Program.cs` adds. Two providers declared under `#if` (`OpenStreetOsm`, the
/// `OpenStreetMapSurfer` pair) are not compiled in and so are not here, and a GDAL provider is
/// added last only when a `gdal` directory sits beside the executable (`Program.cs:379-387`).
///
/// Names are the English ones. The Google, Bing and AMap providers take theirs from
/// `Resources/Strings.resx`, which has a simplified Chinese translation.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:55-156; Program.cs:328-351`
pub const CSHARP_LIST: &[&str] = &[
    // GMapProvider.cs:55 - EmptyProvider, named at :515.
    "None",
    // :57-69
    "OpenStreetMap",
    "OpenStreet4UMap",
    "OpenCycleMap",
    "OpenCycleLandscapeMap",
    "OpenCycleTransportMap",
    "OpenStreetMapQuest",
    "OpenStreetMapQuestSattelite",
    "OpenStreetMapQuestHybrid",
    "OpenSeaMapHybrid",
    // :79-86
    "WikiMapiaMap",
    "BingMap",
    "BingSatelliteMap",
    "BingHybridMap",
    "AMap",
    "AMapSatellite",
    // :88-100
    "GoogleMap",
    "GoogleSatelliteMap",
    "GoogleHybridMap",
    "GoogleTerrainMap",
    "GoogleChinaMap",
    "GoogleChinaSatelliteMap",
    "GoogleChinaHybridMap",
    "GoogleChinaTerrainMap",
    "GoogleKoreaMap",
    "GoogleKoreaSatelliteMap",
    "GoogleKoreaHybridMap",
    // :102-113
    "NearMap",
    "NearSatelliteMap",
    "NearHybridMap",
    "OviMap",
    "OviSatelliteMap",
    "OviHybridMap",
    "OviTerrainMap",
    "YandexMap",
    "YandexSatelliteMap",
    "YandexHybridMap",
    // :115-131
    "LithuaniaMap",
    "Lithuania 2.5d Map",
    "LithuaniaOrtoFotoMap",
    "LithuaniaOrtoFotoMapOld",
    "LithuaniaHybridMap",
    "LithuaniaHybridMapOld",
    "LithuaniaTOP50",
    "LatviaMap",
    "MapBender, WMS demo",
    "TurkeyMap",
    "CloudMade, Demo",
    "SpainMap",
    // :133-142
    "CzechMap",
    "CzechSatelliteMap",
    "CzechHybridMap",
    "CzechTuristMap",
    "CzechTuristWinterMap",
    "CzechHistoryMap",
    "CzechGeographicMap",
    // :145-156
    "ArcGIS_Imagery_World_2D_Map",
    "ArcGIS_ShadedRelief_World_2D_Map",
    "ArcGIS_StreetMap_World_2D_Map",
    "ArcGIS_Topo_US_2D_Map",
    "ArcGIS_World_Physical_Map",
    "ArcGIS_World_Shaded_Relief_Map",
    "ArcGIS_World_Street_Map",
    "ArcGIS_World_Terrain_Base_Map",
    "ArcGIS_World_Topo_Map",
    "ArcGIS_DarbAE_Q2_2011_NAVTQ_Eng_V5_MapProvider",
    // Program.cs:328-351, each named in its own file under ExtLibs/Maps.
    "WMS Custom",
    "WMTS Custom",
    "Custom",
    "NoMap",
    "Earthbuilder Custom",
    "Statkart_Topo2",
    "Eniro_Topo",
    "MapBox Satellite",
    "MapboxNoFly",
    "MapBox User",
    "Japan",
    "Japan_Lake",
    "Japan_1974",
    "Japan_1979",
    "Japan_1984",
    "Japan_1988",
    "Japan_Relief",
    "Japan_Slopezone",
    "Japan_Sea",
    "GIBS Arctic",
    "GIBS Antarctic",
    "ArcGIS Arctic Bathymetry",
    "Esri Arctic Imagery",
    "Esri Arctic Ocean Base",
];

/// Finds a provider by its identifier.
#[must_use]
pub fn source_by_id(id: &str) -> Option<&'static TileSource> {
    SOURCES.iter().find(|source| source.id == id)
}

/// Finds a provider by the C#'s `Name` - what `config.xml`'s `MapType` holds.
///
/// `GMapProviders.List.FindIndex(x => x.Name == mapType)`, over the providers ported.
/// `// C#: GCSViews/FlightPlanner.cs:7252`
#[must_use]
pub fn source_by_name(name: &str) -> Option<&'static TileSource> {
    SOURCES.iter().find(|source| source.cache_name == name)
}

/// The provider shown when nothing has been chosen: [`DEFAULT_PROVIDER`].
#[must_use]
pub fn default_source() -> &'static TileSource {
    source_by_name(DEFAULT_PROVIDER).unwrap_or(&GOOGLE_SATELLITE_MAP)
}

#[cfg(test)]
mod tests {
    use super::*;

    // The URL tests below spell out what the C#'s `string.Format` produces from its `UrlFormat`
    // and arguments, worked by hand. `tests/providers.rs` checks every one of these providers
    // against URLs a Mission Planner GMap.NET.Core.dll produced itself.

    fn tile(z: u8, x: i64, y: i64) -> TileId {
        TileId::new(z, x, y).expect("a valid tile")
    }

    /// `GoogleSatelliteMap` at the SITL field, zoom 16: (59922, 39658).
    ///
    /// `{1}` = (59922 + 2 * 39658) % 4 = 139238 % 4 = 2; `{6}` = "&s=" because 10000 <= 39658 <
    /// 100000; `{9}` = the first (3 * 59922 + 39658) % 8 = 219424 % 8 = 0 letters of "Galileo".
    /// `// C#: GoogleSatelliteMapProvider.cs:53-64; GoogleMapProvider.cs:240-252; GMapProvider.cs:463-466`
    #[test]
    fn a_google_satellite_url_is_the_csharps() {
        assert_eq!(
            GOOGLE_SATELLITE_MAP
                .url_for(tile(16, 59_922, 39_658))
                .as_deref(),
            Some("https://khms2.google.com/kh/v=955&hl=en&x=59922&s=&y=39658&z=16&s=")
        );
        // (15089 + 19628) % 4 = 1; 9814 < 10000 so no sec1; (45267 + 9814) % 8 = 1: "G".
        assert_eq!(
            GOOGLE_SATELLITE_MAP
                .url_for(tile(14, 15_089, 9_814))
                .as_deref(),
            Some("https://khms1.google.com/kh/v=955&hl=en&x=15089&y=9814&z=14&s=G")
        );
        // (2 + 2) % 4 = 0; (6 + 1) % 8 = 7: the whole word.
        assert_eq!(
            GOOGLE_SATELLITE_MAP.url_for(tile(2, 2, 1)).as_deref(),
            Some("https://khms0.google.com/kh/v=955&hl=en&x=2&y=1&z=2&s=Galileo")
        );
    }

    /// `GoogleMap` and `GoogleTerrainMap` at (1, 2, 5): `{1}` = (1 + 4) % 4 = 1; `{9}` = the first
    /// (3 + 2) % 8 = 5 letters, "Galil".
    /// `// C#: GoogleMapProvider.cs:2097, 2128-2139; GoogleTerrainMapProvider.cs:22, 53-64`
    #[test]
    fn the_google_road_and_terrain_urls_are_the_csharps() {
        assert_eq!(
            GOOGLE_MAP.url_for(tile(5, 1, 2)).as_deref(),
            Some("https://mts1.google.com/vt/lyrs=m@354000000&hl=en&x=1&y=2&z=5&s=Galil")
        );
        assert_eq!(
            GOOGLE_TERRAIN_MAP.url_for(tile(5, 1, 2)).as_deref(),
            Some(
                "https://mts1.google.com/maps/vt/lyrs=t@354,r@354000000&hl=en&x=1&y=2&z=5&s=Galil"
            )
        );
    }

    /// The Bing providers at (3, 1, 2): `{0}` = (3 + 2) % 4 = 1; the quadkey is "13" (level 2:
    /// x's bit, "1"; level 1: both bits, "3"); `{4}` is empty for want of a session key.
    /// `// C#: BingMapProvider.cs:42-61, 620-628; BingSatelliteMapProvider.cs:51-59;
    /// BingHybridMapProvider.cs:51-59`
    #[test]
    fn the_bing_urls_are_the_csharps() {
        assert_eq!(
            BING_MAP.url_for(tile(2, 3, 1)).as_deref(),
            Some(
                "http://ecn.t1.tiles.virtualearth.net/tiles/r13?g=4810&mkt=en&lbl=l1&stl=h&shading=hill&n=z"
            )
        );
        assert_eq!(
            BING_SATELLITE_MAP.url_for(tile(2, 3, 1)).as_deref(),
            Some("http://ecn.t1.tiles.virtualearth.net/tiles/a13.jpeg?g=4810&mkt=en&n=z")
        );
        assert_eq!(
            BING_HYBRID_MAP.url_for(tile(2, 3, 1)).as_deref(),
            Some("http://ecn.t1.tiles.virtualearth.net/tiles/h13.jpeg?g=4810&mkt=en&n=z")
        );
        // Zoom 0 has an empty quadkey, and the C# asks for exactly that.
        assert_eq!(
            BING_SATELLITE_MAP.url_for(tile(0, 0, 0)).as_deref(),
            Some("http://ecn.t0.tiles.virtualearth.net/tiles/a.jpeg?g=4810&mkt=en&n=z")
        );
    }

    #[test]
    fn the_default_is_the_csharps() {
        // C#: GCSViews/FlightPlanner.cs:7282
        assert_eq!(DEFAULT_PROVIDER, "GoogleSatelliteMap");
        assert_eq!(default_source(), &GOOGLE_SATELLITE_MAP);
        assert_eq!(default_source().cache_name, DEFAULT_PROVIDER);
        assert!(CSHARP_LIST.contains(&DEFAULT_PROVIDER));
    }

    /// The C#'s providers come first and in the C#'s order; this crate's own come after them.
    #[test]
    fn the_providers_are_in_the_csharps_order_with_ours_last() {
        let positions: Vec<Option<usize>> = SOURCES
            .iter()
            .map(|source| {
                CSHARP_LIST
                    .iter()
                    .position(|name| *name == source.cache_name)
            })
            .collect();
        let ported: Vec<usize> = positions.iter().map_while(|found| *found).collect();
        assert!(
            ported.windows(2).all(|pair| pair[0] < pair[1]),
            "not in the C#'s order: {positions:?}"
        );
        assert!(
            positions.iter().skip(ported.len()).all(Option::is_none),
            "a provider of ours sits among the C#'s: {positions:?}"
        );
        assert_eq!(
            SOURCES
                .iter()
                .skip(ported.len())
                .map(|source| source.id)
                .collect::<Vec<_>>(),
            vec!["opentopo", "esri-imagery"]
        );
        assert_eq!(ported.len(), 8);
    }

    #[test]
    fn a_provider_is_found_by_the_csharps_name() {
        assert_eq!(
            source_by_name("GoogleSatelliteMap"),
            Some(&GOOGLE_SATELLITE_MAP)
        );
        assert_eq!(source_by_name("BingHybridMap"), Some(&BING_HYBRID_MAP));
        // Not ported: two layers.
        assert_eq!(source_by_name("GoogleHybridMap"), None);
        // Ours, which the C# has never heard of - but its directory is still its name.
        assert_eq!(source_by_name("OpenTopoMap"), Some(&OPENTOPOMAP));
    }

    #[test]
    fn the_copyright_carries_this_years_date() {
        use chrono::Datelike as _;
        let year = chrono::Local::now().year();
        assert_eq!(
            GOOGLE_SATELLITE_MAP.attribution_text(),
            format!("©{year} Google - Map data ©{year} Tele Atlas, Imagery ©{year} TerraMetrics")
        );
        assert_eq!(
            BING_MAP.attribution_text(),
            format!(
                "©{year} Microsoft Corporation, ©{year} NAVTEQ, ©{year} Image courtesy of NASA"
            )
        );
        // Formatted once and kept: the same string, not merely an equal one.
        assert!(std::ptr::eq(
            GOOGLE_MAP.attribution_text(),
            GOOGLE_SATELLITE_MAP.attribution_text()
        ));
        // A provider with no year is shown as it is.
        assert_eq!(OPENSTREETMAP.attribution_text(), OPENSTREETMAP.attribution);
    }

    #[test]
    fn the_csharps_providers_send_the_csharps_referer() {
        // GoogleMapProvider.cs:25, BingMapProvider.cs:22, OpenStreetMapProvider.cs:17.
        for source in [&GOOGLE_MAP, &GOOGLE_SATELLITE_MAP, &GOOGLE_TERRAIN_MAP] {
            assert_eq!(source.referer, "https://www.google.com/maps/preview");
        }
        for source in [&BING_MAP, &BING_SATELLITE_MAP, &BING_HYBRID_MAP] {
            assert_eq!(source.referer, "http://www.bing.com/maps/");
        }
        assert_eq!(OPENSTREETMAP.referer, "https://www.openstreetmap.org/");
    }

    /// ArcGIS orders its path row-then-column, and every other provider here does the opposite.
    ///
    /// Getting it backwards does not fail. It returns a real tile of somewhere else, so the map
    /// shows imagery that quietly does not match the roads drawn over it - which is worse than a
    /// blank square, because a blank square is obviously wrong.
    #[test]
    fn the_imagery_url_is_zoom_then_row_then_column() {
        // The SITL home field outside Canberra at zoom 16.
        let tile = TileId {
            z: 16,
            x: 59_922,
            y: 39_658,
        };
        let url = ESRI_WORLD_IMAGERY.url_for(tile).expect("zoom 16 is served");
        assert!(
            url.ends_with("/tile/16/39658/59922"),
            "the path must be zoom/y/x, got {url}"
        );
        assert!(
            !url.ends_with("/tile/16/59922/39658"),
            "x and y are transposed: {url}"
        );
    }

    /// The endpoint is the live one, not the retired one the C# still points at.
    #[test]
    fn the_imagery_endpoint_is_not_the_retired_one() {
        assert!(
            ESRI_WORLD_IMAGERY.url.contains("World_Imagery"),
            "{}",
            ESRI_WORLD_IMAGERY.url
        );
        assert!(
            !ESRI_WORLD_IMAGERY.url.contains("ESRI_Imagery_World_2D"),
            "that service answers 301 - see the note on this provider"
        );
        assert!(ESRI_WORLD_IMAGERY.url.starts_with("https://"));
    }

    /// Esri's terms require attribution by name, not merely some attribution - which
    /// `every_provider_carries_attribution` already covers for all of them.
    #[test]
    fn the_imagery_attribution_names_esri() {
        assert!(ESRI_WORLD_IMAGERY.attribution.contains("Esri"));
    }

    /// Cache names are directories in a tree laid out as the C# application's is, so a collision
    /// mixes two providers' tiles - and one that differs from the C# name by a character keeps
    /// a second copy of everything.
    #[test]
    fn cache_names_are_unique() {
        let mut names: Vec<&str> = SOURCES.iter().map(|source| source.cache_name).collect();
        let count = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), count, "two providers share a cache directory");
    }

    /// The providers Mission Planner also ships must file their tiles where Mission Planner does.
    #[test]
    fn the_csharps_providers_are_cached_under_the_csharps_names() {
        // C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:508
        assert_eq!(OPENSTREETMAP.cache_name, "OpenStreetMap");
        // C#: ExtLibs/GMap.NET.Core/Resources/Strings.resx:127, 130, 133, 151, 154, 157
        assert_eq!(BING_HYBRID_MAP.cache_name, "BingHybridMap");
        assert_eq!(BING_MAP.cache_name, "BingMap");
        assert_eq!(BING_SATELLITE_MAP.cache_name, "BingSatelliteMap");
        assert_eq!(GOOGLE_MAP.cache_name, "GoogleMap");
        assert_eq!(GOOGLE_SATELLITE_MAP.cache_name, "GoogleSatelliteMap");
        assert_eq!(GOOGLE_TERRAIN_MAP.cache_name, "GoogleTerrainMap");
        for source in SOURCES
            .iter()
            .filter(|source| CSHARP_LIST.contains(&source.cache_name))
        {
            // The C#'s box shows the provider's Name, so ours does too.
            assert_eq!(source.label, source.cache_name, "{}", source.id);
        }
    }

    /// A cache name is a path component, so it must not contain anything a path would interpret.
    #[test]
    fn cache_names_are_plain_directory_names() {
        for source in SOURCES {
            assert!(!source.cache_name.is_empty(), "{}", source.id);
            assert!(
                source
                    .cache_name
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '_'),
                "{} has a cache name a path would mangle: {}",
                source.id,
                source.cache_name
            );
        }
    }

    /// There is a satellite option at all, which is the point of the item.
    #[test]
    fn imagery_is_among_the_providers_offered() {
        assert!(source_by_id("google-satellite").is_some());
        assert!(source_by_id("bing-satellite").is_some());
        assert!(source_by_id("esri-imagery").is_some());
    }

    #[test]
    fn a_url_substitutes_the_coordinates() {
        let url = OPENSTREETMAP
            .url_for(tile(14, 15_089, 9_814))
            .expect("a url");
        assert_eq!(url, "https://tile.openstreetmap.org/14/15089/9814.png");
    }

    #[test]
    fn a_zoom_the_provider_does_not_serve_has_no_url() {
        // Better than requesting it and getting a blank tile that then gets cached as if it were
        // real data.
        assert!(OPENSTREETMAP.url_for(tile(20, 0, 0)).is_none());
        assert!(OPENSTREETMAP.url_for(tile(19, 0, 0)).is_some());
        // Google sets no limit, so the whole grid is asked for.
        assert!(
            GOOGLE_SATELLITE_MAP
                .url_for(tile(mp_units::tiles::MAX_ZOOM, 0, 0))
                .is_some()
        );
    }

    #[test]
    fn a_subdomain_is_chosen_from_the_tile_not_from_a_counter() {
        // The same tile must always go to the same host, or every pan sends it somewhere new and
        // defeats every HTTP cache in between.
        let subject = tile(10, 511, 340);
        let first = OPENTOPOMAP.url_for(subject).expect("a url");
        let second = OPENTOPOMAP.url_for(subject).expect("a url");
        assert_eq!(first, second);
        assert!(first.starts_with("https://"), "{first}");
        assert!(
            !first.contains("{s}"),
            "the subdomain was not substituted: {first}"
        );
    }

    #[test]
    fn subdomains_are_actually_spread_across_hosts() {
        let hosts: std::collections::BTreeSet<String> = (0..12)
            .map(|x| {
                let url = OPENTOPOMAP.url_for(tile(10, x, 0)).expect("a url");
                url.split('/').nth(2).unwrap_or_default().to_owned()
            })
            .collect();
        assert_eq!(hosts.len(), OPENTOPOMAP.subdomains.len(), "{hosts:?}");
    }

    #[test]
    fn numbered_servers_are_spread_across_all_four_hosts() {
        let hosts: std::collections::BTreeSet<String> = (0..8)
            .map(|x| {
                let url = GOOGLE_SATELLITE_MAP.url_for(tile(10, x, 0)).expect("a url");
                url.split('/').nth(2).unwrap_or_default().to_owned()
            })
            .collect();
        assert_eq!(
            hosts.into_iter().collect::<Vec<_>>(),
            vec![
                "khms0.google.com",
                "khms1.google.com",
                "khms2.google.com",
                "khms3.google.com"
            ]
        );
    }

    #[test]
    fn a_provider_without_subdomains_gets_none() {
        // Requesting a.tile.openstreetmap.org when only tile.openstreetmap.org exists would fail
        // every tile.
        let url = OPENSTREETMAP.url_for(tile(5, 1, 2)).expect("a url");
        assert!(url.starts_with("https://tile.openstreetmap.org/"), "{url}");
    }

    #[test]
    fn every_url_is_fully_substituted() {
        // A placeholder left in a URL is a request for a tile named "{v}" - which a server answers
        // with an error that then counts against the tile.
        for source in SOURCES {
            for subject in [tile(0, 0, 0), tile(10, 511, 340), tile(16, 59_922, 39_658)] {
                if let Some(url) = source.url_for(subject) {
                    assert!(
                        !url.contains('{') && !url.contains('}'),
                        "{}: {url}",
                        source.id
                    );
                }
            }
        }
    }

    #[test]
    fn every_provider_carries_attribution() {
        // A map that shows someone else's data without saying whose is not one worth shipping,
        // and for these providers it is also a licence breach. Custom fetches nothing: its tiles
        // are the operator's own, put there by Inject Custom Map, and carry no one's notice.
        for source in SOURCES {
            assert!(
                !source.attribution.is_empty() || !source.fetches(),
                "{} has no attribution",
                source.id
            );
            assert!(!source.id.is_empty());
            // A quadkey carries all three.
            let tiled = source.url.contains("{z}")
                && source.url.contains("{x}")
                && source.url.contains("{y}");
            assert!(
                !source.fetches() || tiled || source.url.contains("{q}"),
                "{}",
                source.id
            );
        }
    }

    #[test]
    fn provider_ids_are_unique_because_the_cache_is_keyed_on_them() {
        let mut ids: Vec<&str> = SOURCES.iter().map(|source| source.id).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count, "duplicate provider ids");
    }

    #[test]
    fn a_provider_can_be_found_by_id_and_an_unknown_one_cannot() {
        assert_eq!(source_by_id("osm"), Some(&OPENSTREETMAP));
        assert_eq!(
            source_by_id("google-satellite"),
            Some(&GOOGLE_SATELLITE_MAP)
        );
        assert!(source_by_id("nothing-here").is_none());
    }

    #[test]
    fn declaring_subdomains_servers_or_a_version_requires_the_url_to_use_them() {
        // A provider with subdomains and no {s} would send every tile to one host while looking
        // like it was spreading them; the same goes for numbered servers and {n}, and a version
        // with no {v} is a check that changes nothing.
        for source in SOURCES {
            assert_eq!(
                source.subdomains.is_empty(),
                !source.url.contains("{s}"),
                "{} disagrees with itself about subdomains",
                source.id
            );
            assert_eq!(
                source.servers == 0,
                !source.url.contains("{n}"),
                "{} disagrees with itself about servers",
                source.id
            );
            assert_eq!(
                source.version.is_empty(),
                !source.url.contains("{v}"),
                "{} disagrees with itself about its version",
                source.id
            );
            assert_eq!(
                source.correction == Correction::None,
                source.version.is_empty(),
                "{} checks a version it does not have",
                source.id
            );
        }
    }
}
