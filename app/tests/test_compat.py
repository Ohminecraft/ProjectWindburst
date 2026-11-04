"""0.43: an app that talks to older firmware. The app carries the schema of every released firmware (wbclient/schemas); on HELLO it
picks the one whose hash the firmware reports. A firmware newer than the app is refused with "update the app"."""
import pytest

from wbclient.schema import Schema, library, pick_schema
from wbclient.client import SchemaMismatch


def test_the_library_has_the_released_firmware():
    lib = library()
    versions = [e["fw"] for e in lib.entries()]
    for v in ("0.37.2", "0.41.0", "0.42.1"):
        assert v in versions, versions
    old = lib.by_hash(0xCAF05FD1)                         # 0.41.0 (the hash its firmware reports)
    assert old is not None and old.hash == 0xCAF05FD1 and old.computed_hash() == old.hash, "each stored schema is the one its firmware was built from"
    assert "loiter_speed" in [f["n"] for f in old.sections["nav"]["fields"]]
    assert "crashflip_rate" not in [f["n"] for f in old.sections["motors"]["fields"]], "0.41.0 had no crash flip"


def test_the_current_schema_is_in_the_library():
    cur = Schema()
    assert library().by_hash(cur.hash) is not None, "tools/gen_proto.py keeps the library up to date"


def test_pick_the_schema_of_the_firmware():
    cur = Schema()
    s, note = pick_schema(cur, protocol=cur.version, schema_hash=cur.hash, fw="9.9.9")
    assert s is cur and note is None
    s, note = pick_schema(cur, protocol=cur.version, schema_hash=0xCAF05FD1, fw="0.41.0")
    assert s.hash == 0xCAF05FD1 and "0.41.0" in note, "an older firmware: its own schema, and a note saying so"


def test_a_firmware_newer_than_the_app_wants_an_app_update():
    cur = Schema()
    with pytest.raises(SchemaMismatch) as e:
        pick_schema(cur, protocol=cur.version, schema_hash=0x12345678, fw="99.0.0")
    assert "update" in str(e.value).lower() and "99.0.0" in str(e.value)


def test_an_unknown_build_of_an_older_version_is_refused_plainly():
    cur = Schema()
    with pytest.raises(SchemaMismatch) as e:
        pick_schema(cur, protocol=cur.version, schema_hash=0x12345678, fw="0.20.0")
    assert "update" not in str(e.value).lower() and "0x12345678" in str(e.value)


def test_another_protocol_is_refused():
    cur = Schema()
    with pytest.raises(SchemaMismatch):
        pick_schema(cur, protocol=cur.version + 1, schema_hash=cur.hash, fw="0.43.0")
