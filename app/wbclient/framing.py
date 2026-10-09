"""WBP framing - the Python twin of src/proto/wbp_codec.h.

    | 0xA5 | 0x5A | len_lo | len_hi | type | seq | payload (len bytes) | crc_lo | crc_hi |

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over len, type, seq and payload; everything little-endian.
The parser re-synchronises after noise (boot messages share the serial port): a candidate frame that fails is re-scanned
from its second byte, so a false 0xA5 0x5A in front of a real frame never hides it.
"""
import binascii
import struct

SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 512

HELLO, DEVICE_INFO = 0x01, 0x02
SECTION_GET, SECTION, SECTION_SET, SETTINGS_SAVE, SETTINGS_DEFAULTS, REBOOT = 0x10, 0x11, 0x12, 0x13, 0x14, 0x15
STREAM_CFG, STREAM = 0x20, 0x21
COMMAND, CMD_RESULT = 0x30, 0x31
LOG = 0x40
BB_HEADER, BB_DATA, BB_BLOB = 0x60, 0x61, 0x62          # blackbox: log header text, log records, a piece of the log memory (download)
ACK = 0x7F


def crc16_reference(data, crc=0xFFFF):
    """The CRC spelled out bit by bit, exactly like the firmware's loop. Slow; kept to check crc16() against."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16(data, crc=0xFFFF):
    """Same result as crc16_reference, computed by the C routine of the standard library (megabytes of flight log per second)."""
    return binascii.crc_hqx(bytes(data), crc)


def encode(msg_type, seq, payload=b""):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long (%d > %d)" % (len(payload), MAX_PAYLOAD))
    body = struct.pack("<HBB", len(payload), msg_type, seq & 0xFF) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class Parser:
    """Feed it whatever arrives; it returns the complete frames as (type, seq, payload) tuples."""

    NOISE_MAX = 65536

    def __init__(self):
        self._buf = bytearray()
        self.frames = 0
        self.errors = 0
        self._noise = bytearray()                      # bytes that were not part of any frame (boot messages, a crash dump, a wrong baud rate)

    def take_noise(self):
        """The bytes dropped since the last call, as bytes. Text a firmware prints outside the protocol ends up here."""
        out = bytes(self._noise)
        del self._noise[:]
        return out

    def _drop(self, n):
        if n > 0:
            self._noise += self._buf[:n]
            if len(self._noise) > self.NOISE_MAX:
                del self._noise[:len(self._noise) - self.NOISE_MAX]
            del self._buf[:n]

    def feed(self, data):
        self._buf += data
        return self._scan()

    def idle(self):
        """Call after ~20 ms of silence: gives up a half-received (or false) frame and re-scans what it swallowed."""
        if len(self._buf) >= 2:
            self._drop(1)
            return self._scan()
        return []

    @property
    def mid_frame(self):
        return len(self._buf) >= 2

    def _scan(self):
        out = []
        buf = self._buf
        while True:
            i = buf.find(SYNC)
            if i < 0:                                   # keep a trailing 0xA5: its 0x5A may be the next byte
                keep = 1 if buf.endswith(b"\xA5") else 0
                self._drop(len(buf) - keep)
                break
            if i:
                self._drop(i)
            if len(buf) < 6:
                break
            length = buf[2] | (buf[3] << 8)
            if length > MAX_PAYLOAD:
                self.errors += 1
                self._drop(1)
                continue
            total = length + 8
            if len(buf) < total:
                break
            if crc16(buf[2:6 + length]) == (buf[6 + length] | (buf[7 + length] << 8)):
                out.append((buf[4], buf[5], bytes(buf[6:6 + length])))
                self.frames += 1
                del buf[:total]
            else:
                self.errors += 1
                self._drop(1)
        return out
