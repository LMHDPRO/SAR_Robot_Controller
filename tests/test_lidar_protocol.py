import struct
from pathlib import Path
import random
import tempfile
import time
import unittest
from coind6.protocol import Decoder, Point, Packet, START, STOP
from coind6.scans import ScanAssembler, cartesian
from coind6.acquisition import Acquisition, DemoSource, ReplaySource

MANUAL = bytes.fromhex('AA 55 00 02 01 00 81 00 8A B5 90 A1 0F 20 43 1F')
SDK = bytes.fromhex('AA 55 00 02 01 00 81 00 78 47 90 A1 0F 20 43 1F')

def encode(first, last, distances, ct=0):
    # Independent test encoder following manual's checksum diagram.
    fsa, lsa = round(first*128) | 1, round(last*128) | 1
    n = len(distances)
    words = [0x55aa, ct | n << 8, fsa, lsa]
    body = bytearray()
    for d in distances:
        a, b, c = 0x50, (d & 63) << 2, d >> 6
        body.extend([a,b,c]); words.extend([a | b << 8, c])
    check = 0
    for w in words: check ^= w
    return struct.pack('<HBBHHH', 0x55aa, ct, n, fsa, lsa, check) + body

class ProtocolTests(unittest.TestCase):
    def test_manual_golden_vector(self):
        p, = Decoder().feed(MANUAL)
        self.assertEqual(p.checksum_profile, 'manual')
        self.assertEqual([x.distance_mm for x in p.points], [1000,2000])
        self.assertEqual([x.intensity for x in p.points], [100,200])
        self.assertEqual([x.angle_deg for x in p.points], [0,1])
    def test_sdk_golden_vector_and_strict_modes(self):
        self.assertEqual(Decoder().feed(SDK)[0].checksum_profile, 'sdk')
        self.assertFalse(Decoder('manual').feed(SDK))
        self.assertFalse(Decoder('sdk').feed(MANUAL))
    def test_every_fragment_boundary(self):
        for split in range(len(MANUAL)+1):
            d=Decoder()
            self.assertEqual(len(d.feed(MANUAL[:split])+d.feed(MANUAL[split:])),1)
    def test_byte_by_byte(self):
        d=Decoder();result=[]
        for b in MANUAL:result.extend(d.feed(bytes([b])))
        self.assertEqual(len(result),1)
    def test_ack_descriptor_noise_and_header_fragment(self):
        d=Decoder()
        self.assertFalse(d.feed(bytes.fromhex('A5 5A 00 00 80 01 81 FE FF AA')))
        self.assertEqual(len(d.feed(MANUAL[1:])),1)
    def test_checksum_corruption_and_recovery(self):
        bad=bytearray(MANUAL);bad[-1]^=0x40;d=Decoder()
        self.assertEqual(len(d.feed(bad+MANUAL)),1)
        self.assertGreater(d.bad_checksums,0)
    def test_zero_count_and_angle_checkbit(self):
        bad=bytearray(MANUAL);bad[3]=0;d=Decoder()
        self.assertEqual(len(d.feed(bad+MANUAL)),1)
        bad=bytearray(MANUAL);bad[4]&=0xfe
        self.assertEqual(len(d.feed(bad+MANUAL)),1)
    def test_angles_over_360_rejected(self):
        bad=bytearray(MANUAL);struct.pack_into('<H',bad,4,360*128+1)
        self.assertEqual(len(Decoder().feed(bad+MANUAL)),1)
    def test_wrap_interpolation(self):
        p,=Decoder().feed(encode(350,10,[100,200,300]))
        self.assertEqual([x.angle_deg for x in p.points],[350,0,10])
    def test_single_start_packet_and_hz(self):
        p,=Decoder().feed(encode(0,0,[0],ct=201))
        self.assertTrue(p.ring_start);self.assertEqual(p.reported_hz,10)
        self.assertEqual(p.points[0].distance_mm,0)
    def test_reflection_and_zero_distance(self):
        frame=bytearray(encode(0,0,[0]));frame[10]|=1;frame[8]^=1
        p,=Decoder().feed(frame)
        self.assertTrue(p.points[0].high_reflection)
        self.assertEqual(cartesian(p.points),[])
    def test_max_count(self):
        self.assertEqual(len(Decoder().feed(encode(0,100,[100]*255))[0].points),255)
    def test_partial_timeout_recovers(self):
        d=Decoder();d.feed(MANUAL[:5],now=1);d.feed(b'',now=2.1)
        self.assertEqual(len(d.feed(MANUAL,now=2.2)),1)
        self.assertEqual(d.incomplete_timeouts,1)
    def test_random_chunks_multiple_packets(self):
        rng=random.Random(4410);data=MANUAL*300;d=Decoder();out=[]
        while data:
            n=rng.randint(1,73);out.extend(d.feed(data[:n]));data=data[n:]
        self.assertEqual(len(out),300);self.assertLessEqual(len(d.buffer),775)

class ScanTests(unittest.TestCase):
    def packet(self, angles, t, ring=False):
        return Packet(tuple(Point(a,1000,100,False) for a in angles),ring,None,t,'manual')
    def test_first_partial_is_discarded(self):
        a=ScanAssembler()
        self.assertFalse(a.feed(self.packet([180,270,359],0)))
        self.assertFalse(a.feed(self.packet([0,90,180,270,359],.1)))
        s,=a.feed(self.packet([0,90],.2))
        self.assertEqual(len(s.points),5);self.assertAlmostEqual(s.hz,10)
    def test_ring_and_wrap_do_not_duplicate(self):
        a=ScanAssembler();a.feed(self.packet([0,90,359],.1,True))
        s,=a.feed(self.packet([0,90],.2,True));self.assertEqual(len(s.points),3)
    def test_corrupt_scan_flagged(self):
        a=ScanAssembler();a.feed(self.packet([0,90,359],0,True))
        s,=a.feed(self.packet([0],.1,True),damaged=True)
        self.assertTrue(s.damaged)
    def test_bound_and_stalled_rotation(self):
        a=ScanAssembler();a.feed(self.packet([0],0,True));a.feed(self.packet([10],3))
        self.assertIsNone(a.started);self.assertEqual(a.dropped,1)
    def test_marker_before_zero_does_not_emit_one_point_scan(self):
        a=ScanAssembler()
        self.assertFalse(a.feed(self.packet([359.9],0,True)))
        self.assertFalse(a.feed(self.packet([0.8,90,180,270,359.1],.01)))
        s,=a.feed(self.packet([359.9],.1,True))
        self.assertEqual(len(s.points),6)
        self.assertEqual(a.coalesced_boundaries,1)
    def test_marker_after_wrap_is_coalesced(self):
        a=ScanAssembler();a.feed(self.packet([359],0))
        a.feed(self.packet([0,90,180,270,359],.01))
        s,=a.feed(self.packet([0],.1))
        self.assertEqual(len(s.points),5)
        self.assertFalse(a.feed(self.packet([.3],.101,True)))
        a.feed(self.packet([90,180,270,359],.15))
        s,=a.feed(self.packet([0],.2))
        self.assertEqual(len(s.points),6)
    def test_duplicate_boundary_at_same_usb_timestamp(self):
        a=ScanAssembler();a.feed(self.packet([359.9],1,True))
        self.assertFalse(a.feed(self.packet([0.8,90,180,270,359],1)))
        s,=a.feed(self.packet([359.9],1,True))
        self.assertEqual(len(s.points),6)
    def test_missing_marker_uses_wrap(self):
        a=ScanAssembler();a.feed(self.packet([359.9],0,True))
        a.feed(self.packet([.8,90,180,270,359],.01))
        s,=a.feed(self.packet([.8],.1))
        self.assertEqual(s.boundary,'angle_wrap')
        self.assertEqual(len(s.points),6)
    def test_small_backwards_jitter_is_not_full_rotation(self):
        a=ScanAssembler();a.feed(self.packet([.5,1],0,True))
        self.assertFalse(a.feed(self.packet([.9],.01,True)))
        self.assertLess(a._travel_deg,1)
    def test_coordinate_convention_offset_and_units(self):
        p=[Point(0,1000,1,False),Point(90,2000,1,False),Point(180,0,0,False)]
        c=cartesian(p);self.assertEqual(len(c),2)
        self.assertAlmostEqual(c[0][0],1);self.assertAlmostEqual(c[1][1],-2)
        self.assertAlmostEqual(cartesian(p,offset_deg=90)[0][1],1)

class AcquisitionTests(unittest.TestCase):
    def test_demo_record_replay_and_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Acquisition(DemoSource(),tmp);a.start();deadline=time.monotonic()+3
            while time.monotonic()<deadline and a.snapshot()[1]['scans']<3:time.sleep(.02)
            a.shutdown();a.join(2);self.assertFalse(a.is_alive())
            self.assertGreaterEqual(a.snapshot()[1]['scans'],3)
            raw,=Path(tmp).glob('*.bin');self.assertGreater(raw.stat().st_size,0)
            logs,=Path(tmp).glob('*.jsonl');self.assertIn('distance_mm',logs.read_text())
            b=Acquisition(ReplaySource(raw));b.start();b.join(4)
            self.assertFalse(b.is_alive());self.assertGreaterEqual(b.snapshot()[1]['scans'],3)
    def test_start_stop_commands_and_no_auto_start(self):
        class Fake:
            writable=True
            def __init__(self):self.sent=[]
            def open(self):pass
            def read(self):time.sleep(.005);return b''
            def write(self,b):self.sent.append(b)
            def close(self):pass
        source=Fake();a=Acquisition(source);a.start();time.sleep(.04)
        self.assertEqual(source.sent,[])
        a.command('start');time.sleep(.04);a.shutdown();a.join(2)
        self.assertEqual(source.sent,[START,STOP])
    def test_open_error_finishes(self):
        class Broken:
            writable=False
            def open(self):raise OSError('test unavailable')
            def close(self):pass
        a=Acquisition(Broken());a.start();a.join(1)
        self.assertFalse(a.is_alive());self.assertFalse(a.snapshot()[1]['connected'])

if __name__ == '__main__':unittest.main()
