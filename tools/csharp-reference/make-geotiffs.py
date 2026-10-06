#!/usr/bin/env python3
# Copyright (C) 2026 David "Buzz" Bussenschutt
#
# This file is part of MissionPlannerRust, a Rust implementation derived from
# Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
# https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
#
# MissionPlannerRust is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by the
# Free Software Foundation, version 3 of the License.
#
# MissionPlannerRust is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-3.0-only

# Not taken from Mission Planner: this file is MissionPlannerRust's own. It writes the GeoTIFFs
# that tools/csharp-reference/GeoTiffOracle.cs runs Mission Planner's GeoTiff.cs over, and that
# crates/mp-terrain/tests/geotiff.rs runs the port over:
#
#   tools/csharp-reference/make-geotiffs.py [testdata/geotiff]
#
# Every file is small (tens of samples a side), uncompressed, one image, one sample per pixel, and
# written here byte by byte - TIFF 6.0's header and IFD, GeoTIFF 1.0's three tags and key
# directory - so nothing between this script and the two readers decides what they see. Each case
# is a branch of GeoTiff.cs: a projection case of LoadFile (geographic EPSG 4326, WGS 84 UTM north
# and south, GDA94 MGA, ETRS89 UTM and TM, projected and geographic systems named by an ESRI
# string in PCSCitation and a citation that is not one, a user-defined polar stereographic,
# geographic codes DotSpatial knows - with and without a datum shift - and one it does not, no
# projection keys at all, projected codes the EPSG case cannot use), a sample type of
# ProcessScanLine (16-bit signed and unsigned, 32-bit unsigned, signed and float, 64-bit float,
# 8-bit, no SampleFormat tag), a way of storing the samples (one row per strip, several, square
# and non-square tiles both ways, big-endian), and the values getAltitude tests (no-data blocks,
# the -1000 threshold, a NaN). Two files adjoin, so a point on their shared edge asks the index's
# order.
#
# Deterministic: the same bytes every run. numpy only for the sample arrays.

import os
import struct
import sys

import numpy as np

# TIFF field types.
ASCII, SHORT, LONG, DOUBLE = 2, 3, 4, 12
TYPE_SIZE = {ASCII: 1, SHORT: 2, LONG: 4, DOUBLE: 8}
TYPE_CODE = {SHORT: "H", LONG: "I", DOUBLE: "d"}

# GeoKey IDs (GeoTIFF 1.0, section 6.2), as GeoTiff.cs's GKID names them.
GT_MODEL_TYPE = 1024
GT_RASTER_TYPE = 1025
GEOGRAPHIC_TYPE = 2048
GEOG_CITATION = 2049
GEOG_ANGULAR_UNITS = 2054
GEOG_SEMI_MAJOR_AXIS = 2057
GEOG_INV_FLATTENING = 2059
PROJECTED_CS_TYPE = 3072
PCS_CITATION = 3073
PROJ_COORD_TRANS = 3075
PROJ_LINEAR_UNITS = 3076
PROJ_ORIGIN_LAT = 3081
PROJ_STRAIGHT_VERT_POLE_LONG = 3095

PIXEL_IS_AREA = 1
PIXEL_IS_POINT = 2

WKT_UTM55S = (
    'PROJCS["WGS_1984_UTM_Zone_55S",GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",'
    'SPHEROID["WGS_1984",6378137.0,298.257223563]],PRIMEM["Greenwich",0.0],'
    'UNIT["Degree",0.0174532925199433]],PROJECTION["Transverse_Mercator"],'
    'PARAMETER["False_Easting",500000.0],PARAMETER["False_Northing",10000000.0],'
    'PARAMETER["Central_Meridian",147.0],PARAMETER["Scale_Factor",0.9996],'
    'PARAMETER["Latitude_Of_Origin",0.0],UNIT["Meter",1.0]]'
)

WKT_GEOGCS_WGS84 = (
    'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],'
    'PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]'
)


def geokeys(keys):
    """The GeoKeyDirectory, GeoDoubleParams and GeoAsciiParams for a list of (key, value): an
    int is stored in the directory (location 0), a float in the doubles, a str in the ASCII
    params with its '|' terminator, as GeoTIFF 1.0 section 2.4 lays them out."""
    directory = [1, 1, 0, len(keys)]
    doubles = []
    ascii_params = ""
    for key, value in sorted(keys, key=lambda kv: kv[0]):
        if isinstance(value, str):
            directory += [key, 34737, len(value) + 1, len(ascii_params)]
            ascii_params += value + "|"
        elif isinstance(value, float):
            directory += [key, 34736, 1, len(doubles)]
            doubles.append(value)
        else:
            directory += [key, 0, 1, value]
    return directory, doubles, ascii_params


def write_tiff(path, data, *, big_endian=False, sample_format=1, rows_per_strip=1, tile=None,
               scale, tie, keys, pad=0):
    """One uncompressed single-sample TIFF of `data` (rows x columns, numpy), its samples in
    strips of `rows_per_strip` rows or in `tile` = (width, length) tiles padded with `pad`."""
    order = ">" if big_endian else "<"
    height, width = data.shape
    bits = data.dtype.itemsize * 8
    raw = data.astype(data.dtype.newbyteorder(order))

    chunks = []
    if tile is None:
        for start in range(0, height, rows_per_strip):
            chunks.append(raw[start:start + rows_per_strip].tobytes())
    else:
        tile_width, tile_length = tile
        for top in range(0, height, tile_length):
            for left in range(0, width, tile_width):
                block = np.full((tile_length, tile_width), pad, dtype=raw.dtype)
                part = raw[top:top + tile_length, left:left + tile_width]
                block[:part.shape[0], :part.shape[1]] = part
                chunks.append(block.tobytes())

    directory, doubles, ascii_params = geokeys(keys)

    entries = [
        (256, LONG, [width]),
        (257, LONG, [height]),
        (258, SHORT, [bits]),
        (259, SHORT, [1]),
        (262, SHORT, [1]),
        (277, SHORT, [1]),
        (284, SHORT, [1]),
        (33550, DOUBLE, list(scale) + [0.0]),
        (33922, DOUBLE, [0.0, 0.0, 0.0, tie[0], tie[1], 0.0]),
        (34735, SHORT, directory),
    ]
    if sample_format is not None:
        entries.append((339, SHORT, [sample_format]))
    if doubles:
        entries.append((34736, DOUBLE, doubles))
    if ascii_params:
        entries.append((34737, ASCII, ascii_params.encode("ascii") + b"\0"))
    # The chunk offsets are filled in once the layout is known.
    if tile is None:
        entries += [(273, LONG, None), (278, LONG, [rows_per_strip]), (279, LONG, None)]
    else:
        entries += [(322, LONG, [tile[0]]), (323, LONG, [tile[1]]), (324, LONG, None),
                    (325, LONG, None)]
    entries.sort(key=lambda e: e[0])
    offsets_tag, counts_tag = (273, 279) if tile is None else (324, 325)

    # Layout: header, IFD, then each out-of-line value, then the samples.
    ifd_size = 2 + 12 * len(entries) + 4
    cursor = 8 + ifd_size

    def size_of(kind, values, n_chunks):
        count = n_chunks if values is None else len(values)
        return count, count * TYPE_SIZE[kind]

    placed = {}
    for tag, kind, values in entries:
        count, size = size_of(kind, values, len(chunks))
        if size > 4:
            cursor += cursor % 2  # word-aligned, as TIFF 6.0 asks
            placed[tag] = cursor
            cursor += size
    data_start = cursor + cursor % 2
    chunk_offsets = []
    at = data_start
    for chunk in chunks:
        chunk_offsets.append(at)
        at += len(chunk)

    def pack_values(kind, values):
        if kind == ASCII:
            return bytes(values)
        return struct.pack(order + TYPE_CODE[kind] * len(values), *values)

    out = bytearray(b"MM\0*" if big_endian else b"II*\0")
    out += struct.pack(order + "I", 8)
    out += struct.pack(order + "H", len(entries))
    blobs = []
    for tag, kind, values in entries:
        if tag == offsets_tag:
            values = chunk_offsets
        elif tag == counts_tag:
            values = [len(c) for c in chunks]
        count = len(values)
        packed = pack_values(kind, values)
        out += struct.pack(order + "HHI", tag, kind, count)
        if len(packed) <= 4:
            out += packed + b"\0" * (4 - len(packed))
        else:
            out += struct.pack(order + "I", placed[tag])
            blobs.append((placed[tag], packed))
    out += struct.pack(order + "I", 0)
    for offset, packed in blobs:
        out += b"\0" * (offset - len(out))
        out += packed
    out += b"\0" * (data_start - len(out))
    for chunk in chunks:
        out += chunk
    with open(path, "wb") as f:
        f.write(bytes(out))


def grid(rows, cols, fn, dtype):
    r, c = np.mgrid[0:rows, 0:cols]
    return fn(r, c).astype(dtype)


def geographic(code=4326, raster=PIXEL_IS_AREA):
    return [(GT_MODEL_TYPE, 2), (GT_RASTER_TYPE, raster), (GEOGRAPHIC_TYPE, code),
            (GEOG_ANGULAR_UNITS, 9102)]


def projected(code, raster=PIXEL_IS_AREA, citation=None):
    keys = [(GT_MODEL_TYPE, 1), (GT_RASTER_TYPE, raster), (PROJECTED_CS_TYPE, code),
            (PROJ_LINEAR_UNITS, 9001)]
    if citation is not None:
        keys.append((PCS_CITATION, citation))
    return keys


def main(out_dir):
    os.makedirs(out_dir, exist_ok=True)

    def path(name):
        return os.path.join(out_dir, name)

    # 4326, int16, one row per strip, PixelIsArea: a slope with a no-data block and the four
    # cells either side of getAltitude's -1000 threshold. Its east edge is geo_east_point's west.
    west = grid(16, 20, lambda r, c: 100 + 3 * c + 7 * r, np.int16)
    west[10:12, 4:7] = -32768
    west[2, 15], west[2, 16], west[3, 15], west[3, 16] = -1000, -1001, -999, -1000
    write_tiff(path("geo_west_area.tif"), west, scale=(0.01, 0.01), tie=(149.0, -35.0),
               keys=geographic() + [(GEOG_CITATION, "WGS 84"), (GEOG_SEMI_MAJOR_AXIS, 6378137.0),
                                    (GEOG_INV_FLATTENING, 298.257223563)])

    # 4326, int16, PixelIsPoint, placed so its west edge is geo_west_area's east edge.
    east = grid(16, 20, lambda r, c: 500 - 2 * c + 5 * r, np.int16)
    write_tiff(path("geo_east_point.tif"), east, scale=(0.01, 0.01), tie=(149.205, -35.005),
               keys=geographic(raster=PIXEL_IS_POINT))

    # WGS 84 / UTM 55S, float32, four rows per strip (the last strip short), no-data -9999.
    utm = grid(22, 24, lambda r, c: 600.5 + 1.25 * c - 0.75 * r + 0.0625 * (c % 3), np.float32)
    utm[12:15, 3:6] = -9999.0
    write_tiff(path("utm55s_f32.tif"), utm, sample_format=3, rows_per_strip=4,
               scale=(30.0, 30.0), tie=(690000.0, 6090600.0),
               keys=projected(32755, citation="WGS 84 / UTM zone 55S"))

    # GDA94 / MGA zone 55, int16, three rows per strip over 17 rows, PixelIsPoint.
    mga = grid(17, 22, lambda r, c: 700 + c - 2 * r, np.int16)
    write_tiff(path("mga55_int16.tif"), mga, sample_format=2, rows_per_strip=3,
               scale=(25.0, 25.0), tie=(700000.0, 6100000.0),
               keys=projected(28355, raster=PIXEL_IS_POINT))

    # 4326, int16, 16x16 tiles over 40x36, the right and bottom tiles padded.
    tiled = grid(36, 40, lambda r, c: 200 + c + 3 * r, np.int16)
    write_tiff(path("tiled_int16.tif"), tiled, sample_format=2, tile=(16, 16), pad=12345,
               scale=(0.001, 0.001), tie=(150.0, -34.0), keys=geographic())

    # 4326, int16, 16-wide 32-long tiles: ExtractScanLineFromTile steps a tile's rows by its
    # length, so the C# reads the wrong samples here. Each sample is 10 * row + column.
    nonsquare = grid(36, 40, lambda r, c: 10 * r + c, np.int16)
    write_tiff(path("tiled_nonsquare.tif"), nonsquare, sample_format=2, tile=(16, 32), pad=7777,
               scale=(0.001, 0.001), tie=(150.1, -34.0), keys=geographic())

    # 32-wide 16-long tiles: the other way round, which the C# reads without throwing - wrongly.
    write_tiff(path("tiled_wide.tif"), nonsquare, sample_format=2, tile=(32, 16), pad=7777,
               scale=(0.001, 0.001), tie=(150.2, -34.0), keys=geographic())

    # Big-endian ("MM"), 4326, int16, two rows per strip, negative values.
    be = grid(14, 18, lambda r, c: 300 + 40 * c - 25 * r, np.int16)
    write_tiff(path("bigendian_int16.tif"), be, big_endian=True, sample_format=2,
               rows_per_strip=2, scale=(0.005, 0.005), tie=(151.0, -33.0), keys=geographic())

    # The sample types: uint32 (one past int.MaxValue), int32, float64 (a NaN), uint16 (above
    # short.MaxValue, which the C# reads signed), no SampleFormat tag, and 8-bit.
    u32 = grid(8, 10, lambda r, c: 1000 + 100 * c + 10 * r, np.uint32)
    u32[4, 5] = 4000000000
    write_tiff(path("uint32.tif"), u32, sample_format=1, scale=(0.01, 0.01),
               tie=(152.0, -32.0), keys=geographic())

    i32 = grid(8, 10, lambda r, c: -50 + 7 * c - 9 * r, np.int32)
    i32[4, 5] = -2000000
    write_tiff(path("int32.tif"), i32, sample_format=2, scale=(0.01, 0.01),
               tie=(153.0, -32.0), keys=geographic())

    f64 = grid(8, 10, lambda r, c: 123.456 + 0.1 * c - 0.3 * r, np.float64)
    f64[4, 5] = np.nan
    write_tiff(path("float64.tif"), f64, sample_format=3, scale=(0.01, 0.01),
               tie=(154.0, -32.0), keys=geographic())

    u16 = grid(8, 10, lambda r, c: 30000 + 1000 * c, np.uint16)
    write_tiff(path("uint16.tif"), u16, sample_format=1, scale=(0.01, 0.01),
               tie=(152.0, -31.0), keys=geographic())

    nosf = grid(8, 10, lambda r, c: 10 + c + r, np.int16)
    write_tiff(path("nosampleformat.tif"), nosf, sample_format=None, scale=(0.01, 0.01),
               tie=(153.0, -31.0), keys=geographic())

    i8 = grid(8, 10, lambda r, c: 10 + c + r, np.uint8)
    write_tiff(path("int8.tif"), i8, sample_format=1, scale=(0.01, 0.01),
               tie=(154.0, -31.0), keys=geographic())

    # User-defined projected systems named only by PCSCitationGeoKey: an ESRI WKT string, and
    # GDAL's kind of citation, which is a name and not WKT.
    wkt = grid(10, 12, lambda r, c: 300 + 2 * c + r, np.float32)
    write_tiff(path("esri_wkt.tif"), wkt, sample_format=3, scale=(100.0, 100.0),
               tie=(600000.0, 6000000.0),
               keys=projected(32767, citation=WKT_UTM55S) + [(PROJ_COORD_TRANS, 1)])
    write_tiff(path("esri_name.tif"), wkt, sample_format=3, scale=(100.0, 100.0),
               tie=(610000.0, 6010000.0),
               keys=projected(32767, citation="WGS 84 / UTM zone 55S"))

    # A geographic system named by an ESRI string alone, ProjectedCSTypeGeoKey absent.
    write_tiff(path("esri_geogcs.tif"), wkt, sample_format=3, scale=(0.01, 0.01),
               tie=(153.0, -30.0),
               keys=[(GT_MODEL_TYPE, 2), (GT_RASTER_TYPE, PIXEL_IS_AREA),
                     (PCS_CITATION, WKT_GEOGCS_WGS84)])

    # Geographic files: a user-defined code (32767), no projection keys at all, GDA94 (a datum
    # DotSpatial shifts through geocentric coordinates) and GRS 1980 with no datum (one it does not).
    geo = grid(10, 12, lambda r, c: 50 + 5 * c + 3 * r, np.int16)
    write_tiff(path("geo_userdefined.tif"), geo, sample_format=2, scale=(0.01, 0.01),
               tie=(150.0, -30.0), keys=geographic(32767))
    write_tiff(path("geo_nokeys.tif"), geo, sample_format=2, scale=(0.01, 0.01),
               tie=(151.0, -30.0), keys=[(GT_MODEL_TYPE, 2), (GT_RASTER_TYPE, PIXEL_IS_POINT)])
    write_tiff(path("geo_gda94.tif"), geo, sample_format=2, scale=(0.01, 0.01),
               tie=(152.0, -30.0), keys=geographic(4283))
    write_tiff(path("geo_grs80.tif"), geo, sample_format=2, scale=(0.01, 0.01),
               tie=(154.0, -30.0), keys=geographic(4019))

    # Projected codes LoadFile's EPSG case cannot use: 32690 is in the WGS 84 UTM range but names
    # no zone, 32700 is the range's gap, and 2193 (NZTM) is one DotSpatial knows and the port does
    # not.
    write_tiff(path("utm_bogus_32690.tif"), geo, sample_format=2, scale=(50.0, 50.0),
               tie=(500000.0, 5000000.0), keys=projected(32690))
    write_tiff(path("utm_gap_32700.tif"), geo, sample_format=2, scale=(50.0, 50.0),
               tie=(500000.0, 5000000.0), keys=projected(32700))
    write_tiff(path("nztm_2193.tif"), geo, sample_format=2, scale=(50.0, 50.0),
               tie=(1750000.0, 5900000.0), keys=projected(2193))

    # ETRS89 / UTM 32N (25832), ETRS89 / UTM 32N (N-E) (3044), WGS 84 / UTM 33N (32633).
    eu = grid(12, 16, lambda r, c: 150 + 2 * c + 3 * r, np.int16)
    write_tiff(path("etrs89_utm32.tif"), eu, sample_format=2, scale=(50.0, 50.0),
               tie=(500000.0, 5500000.0), keys=projected(25832))
    write_tiff(path("etrs89_tm32.tif"), eu, sample_format=2, scale=(50.0, 50.0),
               tie=(520000.0, 5520000.0), keys=projected(3044))
    write_tiff(path("utm33n_int16.tif"), eu, sample_format=2, scale=(50.0, 50.0),
               tie=(400000.0, 5800000.0), keys=projected(32633))

    # User-defined polar stereographic (32767, ProjCoordTrans 15): the first case of LoadFile.
    polar = grid(10, 12, lambda r, c: 1000 + 10 * c - 10 * r, np.int16)
    write_tiff(path("polar_stereo.tif"), polar, sample_format=2, scale=(1000.0, 1000.0),
               tie=(150000.0, -2000000.0),
               keys=[(GT_MODEL_TYPE, 1), (GT_RASTER_TYPE, PIXEL_IS_AREA),
                     (PROJECTED_CS_TYPE, 32767), (PROJ_COORD_TRANS, 15),
                     (GEOG_CITATION, "WGS 84"), (PROJ_ORIGIN_LAT, 70.0),
                     (PROJ_STRAIGHT_VERT_POLE_LONG, -45.0), (PROJ_LINEAR_UNITS, 9001)])


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "..", "testdata", "geotiff"))
