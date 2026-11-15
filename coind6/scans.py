"""Bounded revolutions in sensor frame; deliberately no accumulation or mapping."""
from dataclasses import dataclass
import math
from .protocol import Point, Packet

@dataclass(frozen=True)
class Scan:
    sequence: int
    points: tuple[Point, ...]
    start_monotonic: float
    end_monotonic: float
    boundary: str
    damaged: bool

    @property
    def hz(self):
        duration = self.end_monotonic - self.start_monotonic
        return 1/duration if duration > 0 else 0.0

    def record(self):
        return {'sequence': self.sequence, 'frame': 'lidar_raw_clockwise',
                'host_start_monotonic_s': self.start_monotonic,
                'host_end_monotonic_s': self.end_monotonic,
                'boundary': self.boundary, 'damaged': self.damaged,
                'angle_deg': [p.angle_deg for p in self.points],
                'distance_mm': [p.distance_mm for p in self.points],
                'intensity': [p.intensity for p in self.points],
                'high_reflection': [p.high_reflection for p in self.points]}

class ScanAssembler:
    def __init__(self):
        self.sequence = 0
        self.points = []
        self.started = None
        self.last_angle = None
        self.damage = False
        self.dropped = 0
        self.coalesced_boundaries = 0
        self._travel_deg = 0.0

    def feed(self, packet: Packet, damaged=False):
        out = []
        # Gap/CRC error belongs to the scan ending at this packet boundary.
        self.damage |= damaged
        for i, point in enumerate(packet.points):
            ring = packet.ring_start and i == 0
            wrap = self.last_angle is not None and self.last_angle > 270 and point.angle_deg < 90
            boundary = 'ring_start' if ring else 'angle_wrap'
            if self.started is not None and self.last_angle is not None:
                step = point.angle_deg - self.last_angle
                if step < -180:
                    step += 360
                # Small reverse jitter is not a revolution of forward travel.
                self._travel_deg += max(0.0, step)
            candidate = ring or wrap
            # Hardware ring markers may precede OR follow zero crossing. They
            # delimit the same revolution, even across packets and USB chunks.
            # Angular travel, not host timing, debounces the two events. Keep
            # the marker sample: it is a valid measurement, not a separate scan.
            if candidate and self.started is not None and self._travel_deg < 270:
                self.coalesced_boundaries += 1
                candidate = False
            if candidate:
                if self.started is not None and self.points:
                    self.sequence += 1
                    out.append(Scan(self.sequence, tuple(self.points), self.started,
                                    packet.received_monotonic, boundary, self.damage))
                self.points = []
                self.started = packet.received_monotonic
                self.damage = False
                self._travel_deg = 0.0
            if self.started is not None:
                self.points.append(point)
                if len(self.points) > 10000 or packet.received_monotonic - self.started > 2.0:
                    self.points = []
                    self.started = None
                    self.dropped += 1
            self.last_angle = point.angle_deg
        return out


def cartesian(points, offset_deg=0.0, clockwise=True, min_m=0.05, max_m=12.0):
    """Right-handed planar output: X forward at calibrated zero, Y left, metres."""
    coords = []
    for p in points:
        r = p.distance_mm / 1000
        if p.distance_mm == 0 or not min_m <= r <= max_m:
            continue
        theta = math.radians(offset_deg + (-p.angle_deg if clockwise else p.angle_deg))
        coords.append((r*math.cos(theta), r*math.sin(theta), p.intensity))
    return coords
