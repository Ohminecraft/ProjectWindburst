"""Map tiles and the address search for the Mission tab.

The page asks the local server (/api/tile, /api/geocode); the server fetches from the providers with a User-Agent that names
the program (OpenStreetMap's tile policy asks for one, and for caching) and keeps every tile on disk, so a place looked at once
is there again without a network (in the field). Offline (WBCLIENT_TILES_OFFLINE=1, the tests): the network is never asked.
"""
import json
import os
import threading
import urllib.parse
import urllib.request

USER_AGENT = "WindburstConfigurator/0.37 (flight controller configurator; desktop app)"
PROVIDERS = {
    # name: (URL template with {z} {x} {y}, file extension, content type)
    "osm": ("https://tile.openstreetmap.org/{z}/{x}/{y}.png", "png", "image/png"),
    "sat": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "jpg", "image/jpeg"),
    # drawn over "sat" (see-through PNGs), as Google's satellite view has them: the roads with their names, then the places
    # (wards, districts, lakes, landmarks) - Vietnamese names keep their accents
    "road": ("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}", "png", "image/png"),
    "lbl": ("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}", "png", "image/png"),
}
MAX_ZOOM = 19


def _png_1x1_clear():
    import struct
    import zlib

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    return (bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]) + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes([0, 0, 0, 0, 0]))) + chunk(b"IEND", b""))


BLANK_PNG = _png_1x1_clear()               # what the page gets for a tile there is none of (offline): see-through, not an error
GEOCODE_URL = "https://nominatim.openstreetmap.org/search?"


def default_dir():
    return os.environ.get("WBCLIENT_TILE_DIR") or os.path.join(os.path.expanduser("~"), "Windburst", "tiles")


def _fetch(url, headers, timeout):
    req = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(req, timeout=timeout) as r:      # noqa: S310 - fixed https hosts above
        return r.read()


class MapTiles:
    def __init__(self, root=None, fetch=None, offline=None):
        self.root = os.path.abspath(root or default_dir())
        self.fetch = fetch or _fetch
        self.offline = (os.environ.get("WBCLIENT_TILES_OFFLINE") == "1") if offline is None else offline
        self._lock = threading.Lock()

    def tile(self, provider, z, x, y):
        """(bytes, content type) of one tile, or None when there is none (no network, nothing cached). ValueError: bad numbers."""
        if provider not in PROVIDERS:
            raise ValueError("unknown map: %r" % provider)
        try:
            z, x, y = int(z), int(x), int(y)
        except (TypeError, ValueError):
            raise ValueError("tile numbers must be integers")
        if not (0 <= z <= MAX_ZOOM) or not (0 <= x < (1 << z)) or not (0 <= y < (1 << z)):
            raise ValueError("no such tile: %d/%d/%d" % (z, x, y))
        url, ext, ctype = PROVIDERS[provider]
        path = os.path.join(self.root, provider, str(z), str(x), "%d.%s" % (y, ext))
        if os.path.isfile(path):
            with open(path, "rb") as f:
                return f.read(), ctype
        if self.offline:
            return None
        try:
            body = self.fetch(url.format(z=z, x=x, y=y), {"User-Agent": USER_AGENT}, 10)
        except Exception:                   # noqa: BLE001 - no network, a provider down: the page shows an empty square
            return None
        if not body:
            return None
        with self._lock:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".part"
            with open(tmp, "wb") as f:
                f.write(body)
            os.replace(tmp, path)
        return body, ctype

    def geocode(self, query, limit=5):
        """Places matching a free-text address (OpenStreetMap Nominatim): [{name, lat, lon}]; [] without a network."""
        q = (query or "").strip()
        if not q or self.offline:
            return []
        url = GEOCODE_URL + urllib.parse.urlencode({"format": "json", "limit": limit, "q": q})
        try:
            rows = json.loads(self.fetch(url, {"User-Agent": USER_AGENT, "Accept-Language": "vi,en"}, 10).decode("utf-8"))
        except Exception:                   # noqa: BLE001
            return []
        out = []
        for r in rows if isinstance(rows, list) else []:
            try:
                out.append({"name": str(r["display_name"]), "lat": float(r["lat"]), "lon": float(r["lon"])})
            except (KeyError, TypeError, ValueError):
                continue
        return out
