"""Streaming COIN-D6 decoder, based on supplied V1.0 manual and CSPC SDK.

The manual's checksum diagram and the supplied SDK disagree on the grouping
of the 3 sample bytes. Both are checked explicitly; no unchecked packet is used.
No undocumented distance-dependent angular correction is applied.
"""
from dataclasses import dataclass
import struct
import time

START = bytes.fromhex('AA 55 F0 0F')
STOP = bytes.fromhex('AA 55 F5 0A')

@dataclass(frozen=True)
class Point:
    angle_deg: float             # Sensor convention: clockwise from its zero mark
    distance_mm: int             # 0 retained as invalid, never treated as free space
    intensity: int
    high_reflection: bool

@dataclass(frozen=True)
class Packet:
    points: tuple[Point, ...]
    ring_start: bool
    reported_hz: float | None
    received_monotonic: float
    checksum_profile: str


def checksum(frame: bytes, profile: str) -> int:
    """frame contains 10-byte header and exactly N three-byte samples."""
    value = 0x55AA ^ (frame[2] | frame[3] << 8)
    value ^= int.from_bytes(frame[4:6], 'little')
    value ^= int.from_bytes(frame[6:8], 'little')
    for i in range(10, len(frame), 3):
        a, b, c = frame[i:i+3]
        if profile == 'manual':
            value ^= (a | b << 8) ^ c
        elif profile == 'sdk':
            value ^= a ^ (b | c << 8)
        else:
            raise ValueError('Unknown checksum profile')
    return value & 0xFFFF


class Decoder:
    def __init__(self, checksum_mode='auto'):
        if checksum_mode not in ('auto', 'manual', 'sdk'):
            raise ValueError('checksum_mode must be auto, manual or sdk')
        self.mode = checksum_mode
        self.buffer = bytearray()
        self.good_packets = self.bad_checksums = self.bad_headers = 0
        self.discarded_bytes = self.incomplete_timeouts = 0
        self.profile_counts = {'manual': 0, 'sdk': 0, 'both': 0}
        self._pending_since = None

    def feed(self, data: bytes, now=None) -> list[Packet]:
        now = time.monotonic() if now is None else now
        self.buffer.extend(data)
        result = []
        while True:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                # Retain a possible fragmented first header byte.
                keep = 1 if self.buffer.endswith(b'\xaa') else 0
                discard = len(self.buffer) - keep
                self.discarded_bytes += discard
                if discard:
                    del self.buffer[:discard]
                self._pending_since = None
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
                self._pending_since = None
            if len(self.buffer) < 10:
                if self._expired(now):
                    del self.buffer[0]
                    continue
                break
            _, ct, count, fsa, lsa, expected = struct.unpack_from('<HBBHHH', self.buffer)
            first, last = (fsa >> 1) / 64, (lsa >> 1) / 64
            if not count or not (fsa & 1 and lsa & 1) or first >= 360 or last >= 360:
                self.bad_headers += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                self._pending_since = None
                continue
            size = 10 + 3 * count
            if len(self.buffer) < size:
                if self._expired(now):
                    del self.buffer[0]
                    continue
                break
            frame = bytes(self.buffer[:size])
            modes = ('manual', 'sdk') if self.mode == 'auto' else (self.mode,)
            matches = [p for p in modes if checksum(frame, p) == expected]
            if not matches:
                self.bad_checksums += 1
                self.discarded_bytes += 1
                del self.buffer[0]  # Search again; never discard a subsequent good packet blindly.
                self._pending_since = None
                continue
            profile = 'both' if len(matches) == 2 else matches[0]
            self.profile_counts[profile] += 1
            span = (last - first) % 360
            points = []
            for i in range(count):
                a, b, c = frame[10+3*i:13+3*i]
                points.append(Point(
                    (first + (span*i/(count-1) if count > 1 else 0)) % 360,
                    c*64 + (b >> 2),
                    (b & 3)*64 + (a >> 2), bool(a & 1)))
            result.append(Packet(tuple(points), bool(ct & 1),
                                 (ct >> 1)/10 if ct & 1 and ct >> 1 else None,
                                 now, profile))
            self.good_packets += 1
            del self.buffer[:size]
            self._pending_since = None
        return result

    def _expired(self, now):
        if self._pending_since is None:
            self._pending_since = now
        if now - self._pending_since > 1.0:
            self.incomplete_timeouts += 1
            self.discarded_bytes += 1
            self._pending_since = None
            return True
        return False
