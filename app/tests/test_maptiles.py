"""Map tiles and the address search for the Mission tab (wbclient/maptiles.py): fetched once, kept on disk, served from there
without a network; nothing but well-formed tile numbers ever reaches a URL."""
import json

import pytest

from wbclient import maptiles


class Net:
    def __init__(self, body=b"\x89PNG tile", fail=False):
        self.calls = []
        self.body = body
        self.fail = fail

    def __call__(self, url, headers, timeout):
        self.calls.append((url, headers))
        if self.fail:
            raise OSError("offline")
        return self.body


def test_tile_fetched_once_then_from_disk(tmp_path):
    net = Net()
    t = maptiles.MapTiles(str(tmp_path), fetch=net, offline=False)
    body, ctype = t.tile("osm", 15, 17165, 11476)
    assert body == b"\x89PNG tile" and ctype == "image/png"
    assert net.calls[0][0] == "https://tile.openstreetmap.org/15/17165/11476.png"
    assert "Windburst" in net.calls[0][1]["User-Agent"], "OSM's tile policy: a User-Agent that names the program"
    again, _ = t.tile("osm", 15, 17165, 11476)
    assert again == body and len(net.calls) == 1, "the second time from the disk cache"
    body, ctype = t.tile("sat", 3, 4, 2)
    assert ctype == "image/jpeg" and net.calls[-1][0] == "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/3/2/4", "Esri World Imagery: z / y / x"
    # the names over the satellite pictures (Google's satellite view with its labels): see-through PNGs from Esri's reference maps
    for p, svc in (("road", "Reference/World_Transportation"), ("lbl", "Reference/World_Boundaries_and_Places")):
        body, ctype = t.tile(p, 17, 104076, 57701)
        assert ctype == "image/png" and net.calls[-1][0] == "https://server.arcgisonline.com/ArcGIS/rest/services/%s/MapServer/tile/17/57701/104076" % svc, p
        assert (tmp_path / p / "17" / "104076" / "57701.png").is_file(), "kept on disk like the pictures under them"


def test_offline_and_bad_numbers(tmp_path):
    t = maptiles.MapTiles(str(tmp_path), fetch=Net(fail=True), offline=False)
    assert t.tile("osm", 10, 1, 1) is None, "no network and nothing cached: no tile (Leaflet shows the empty square)"
    t2 = maptiles.MapTiles(str(tmp_path), fetch=Net(), offline=True)
    assert t2.tile("osm", 10, 1, 1) is None and t2.fetch.calls == [], "offline: the network is never asked"
    for bad in [("osm", 20, 0, 0), ("osm", 2, 4, 0), ("osm", 2, -1, 0), ("xyz", 1, 0, 0), ("osm", "1;", 0, 0)]:
        with pytest.raises(ValueError):
            t2.tile(*bad)


def test_geocode(tmp_path):
    reply = json.dumps([{"display_name": "Bến Thành, Quận 1, Hồ Chí Minh", "lat": "10.7725", "lon": "106.6980"}]).encode()
    net = Net(body=reply)
    t = maptiles.MapTiles(str(tmp_path), fetch=net, offline=False)
    hits = t.geocode("chợ Bến Thành")
    assert hits == [{"name": "Bến Thành, Quận 1, Hồ Chí Minh", "lat": 10.7725, "lon": 106.698}]
    url = net.calls[0][0]
    assert url.startswith("https://nominatim.openstreetmap.org/search?") and "format=json" in url and "limit=5" in url and "ch%E1%BB%A3" in url
    assert maptiles.MapTiles(str(tmp_path), fetch=Net(fail=True), offline=False).geocode("x") == [], "no network: no hits, no error"
    assert t.geocode("   ") == []


def test_blank_tile_is_a_png():
    import struct
    import zlib
    b = maptiles.BLANK_PNG
    assert b[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]) and b[12:16] == b"IHDR" and struct.unpack(">II", b[16:24]) == (1, 1), "a 1 x 1 PNG"
    assert b[24:26] == bytes([8, 6]), "8 bit RGBA"
    idat = b.index(b"IDAT")
    n = struct.unpack(">I", b[idat - 4:idat])[0]
    assert zlib.decompress(b[idat + 4:idat + 4 + n]) == bytes([0, 0, 0, 0, 0]), "one transparent pixel"
