#!/usr/bin/env python3
"""Lays out a map tile cache exactly as the C# Mission Planner writes one.

usage: tests/gui/seed-csharp-cache.py [--block N] [--hole] <gmapcache-dir> [lat] [lon]

The latitude and longitude default to -35.363262 149.165237, ArduPilot SITL's default home, which
is where a test with SITL on the link will be looking. Every zoom from 14 to 19 gets a 9x9 block
of tiles centred on the tile holding that point: wide enough that the map framing the vehicle at
any of those zooms lands inside it. `--block N` makes the block N x N instead, and `--hole` leaves
the centre tile of each block unwritten - a cache with a gap in it, for a test of what the map
does about the tiles it does not have while it shows the ones it does.

The point is that nothing about the layout comes from this port. A test that seeded its cache
through the Rust cache code would prove only that the code agrees with itself; this writes what
`ExtLibs/Maps/MyImageCache.cs` writes, so a map that shows these tiles reads a cache the C#
application filled.

Two things about the layout are not what the files suggest, and both are copied on purpose.

The name is `.jpg` and the bytes are PNG. `PutImageToCache` writes every tile, whatever format the
provider served, to `<x>.jpg` (MyImageCache.cs:72-74), and `GetImageFromCache` hands the bytes to
the image decoder without looking at the name (MyImageCache.cs:94-117). OpenStreetMap serves PNG,
so a real C# cache of it is PNG under `.jpg` - and a reader that trusted the extension would fail
on every tile of it.

The path is `<z>/<y>/<x>`, row before column: the C# builds it from `zoom`, `pos.Y`, `pos.X` in
that order (MyImageCache.cs:72-74). Almost every other slippy-map tool writes `z/x/y`, and a seed
written that way would lay out a cache neither application reads - the tiles would exist, in the
wrong places, and the test would say so only by failing to find them.

The provider directory is the C# provider's `Name`, `OpenStreetMap`
(GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:508), under
`TileDBv3/en`, which `MyImageCache.CacheLocation` appends to the gmapcache directory
(MyImageCache.cs:27-42).

Every tile holds the same bytes: a 256x256 PNG of one colour, made here with zlib and struct so the
script needs nothing beyond the standard library. The colour is OpenStreetMap's land colour, so a
screenshot of a test using it reads as an empty map rather than as a fault.
"""
import math
import os
import struct
import sys
import zlib

DEFAULT_LAT = -35.363262
DEFAULT_LON = 149.165237

PROVIDER = 'OpenStreetMap'
ZOOMS = range(14, 20)
BLOCK = 9
TILE_PX = 256
COLOUR = (0xF2, 0xEF, 0xE9)


def chunk(kind, data):
    """One PNG chunk: length, type, data, and a CRC over type and data."""
    return (
        struct.pack('>I', len(data))
        + kind
        + data
        + struct.pack('>I', zlib.crc32(kind + data) & 0xFFFFFFFF)
    )


def solid_png(size, colour):
    """A size x size 8-bit RGB PNG of one colour: signature, IHDR, one IDAT, IEND."""
    # Colour type 2 (RGB), bit depth 8, deflate, adaptive filtering, no interlace.
    header = struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0)
    # Each scanline starts with its filter type; 0 is "none", so the row is the pixels verbatim.
    row = b'\x00' + bytes(colour) * size
    pixels = zlib.compress(row * size, 9)
    return (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', header)
        + chunk(b'IDAT', pixels)
        + chunk(b'IEND', b'')
    )


def tile_for(lat, lon, zoom):
    """The slippy-map tile holding a point: Web Mercator, y growing southwards."""
    n = 2 ** zoom
    lat_rad = math.radians(lat)
    x = int(math.floor((lon + 180.0) / 360.0 * n))
    y = int(math.floor((1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n))
    # A point on the east edge or at the pole's limit lands one past the last tile.
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def main(argv):
    args = argv[1:]
    block, hole = BLOCK, False
    while args and args[0].startswith('--'):
        option = args.pop(0)
        if option == '--block' and args:
            block = int(args.pop(0))
        elif option == '--hole':
            hole = True
        else:
            print('usage: seed-csharp-cache.py [--block N] [--hole] <gmapcache-dir> [lat] [lon]',
                  file=sys.stderr)
            return 2
    if len(args) not in (1, 3) or block < 1:
        print('usage: seed-csharp-cache.py [--block N] [--hole] <gmapcache-dir> [lat] [lon]',
              file=sys.stderr)
        return 2
    root = args[0]
    lat, lon = (float(args[1]), float(args[2])) if len(args) == 3 else (DEFAULT_LAT, DEFAULT_LON)

    png = solid_png(TILE_PX, COLOUR)
    provider_dir = os.path.join(root, 'TileDBv3', 'en', PROVIDER)
    half = block // 2
    written = 0
    for zoom in ZOOMS:
        n = 2 ** zoom
        cx, cy = tile_for(lat, lon, zoom)
        for y in range(max(cy - half, 0), min(cy + half, n - 1) + 1):
            row_dir = os.path.join(provider_dir, str(zoom), str(y))
            os.makedirs(row_dir, exist_ok=True)
            for x in range(max(cx - half, 0), min(cx + half, n - 1) + 1):
                if hole and (x, y) == (cx, cy):
                    continue
                with open(os.path.join(row_dir, '%d.jpg' % x), 'wb') as tile:
                    tile.write(png)
                written += 1

    print('seeded %d tiles (%d bytes each, PNG under .jpg) around %.6f %.6f, zooms %d-%d, in %s'
          % (written, len(png), lat, lon, ZOOMS[0], ZOOMS[-1], provider_dir))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
