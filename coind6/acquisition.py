"""I/O thread and bounded latest-scan handoff. Qt is never used in this module."""
import json
import math
from pathlib import Path
import queue
import socket
import struct
import threading
import time
from datetime import datetime, timezone
from .protocol import Decoder, START, STOP, checksum
from .scans import ScanAssembler

class TCPSource:
    """Transparent full-duplex TCP byte stream from the wireless bridge."""
    writable = True
    def __init__(self, host, port=9000):
        self.host, self.tcp_port = host.strip(), int(port)
        self.device = None
        self.baud = 230400  # Firmware setting, not a socket setting.
    def open(self):
        self.device = socket.create_connection((self.host, self.tcp_port), timeout=2.0)
        self.device.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.device.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.device.settimeout(0.02)
    def read(self):
        try:
            data = self.device.recv(4096)
        except socket.timeout:
            return b''
        if not data:
            raise ConnectionError('External LiDAR bridge closed TCP; check the network and reconnect.')
        return data
    def write(self, data):
        # Commands are four bytes. A send failure ends the session rather than
        # silently retrying a partial command and duplicating bytes.
        self.device.sendall(data)
    def close(self):
        if self.device is not None:
            try:
                self.device.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.device.close()
            self.device = None

class ReplaySource:
    writable = False
    def __init__(self, path, baud=230400):
        self.path, self.baud = path, baud
    def open(self):
        self.file = open(self.path, 'rb')
    def read(self):
        data = self.file.read(512)
        if not data:
            raise EOFError('End of capture')
        time.sleep(len(data)*10/self.baud)
        return data
    def write(self, data):
        pass
    def close(self):
        if hasattr(self, 'file'):
            self.file.close()

class DemoSource:
    """Synthetic 4x6-m room; feeds real decoder, never presented as hardware data."""
    writable = False
    def open(self):
        self.index = 0
    def read(self):
        time.sleep(0.025)
        n = 100
        first = (self.index % 4)*90.0
        last = first + 89.1
        ct = 0
        header = struct.pack('<HBBHHH', 0x55AA, ct, n,
                             round(first*128) | 1, round(last*128) | 1, 0)
        data = bytearray(header)
        for i in range(n):
            a = math.radians(first + i*0.9)
            r = min(3/max(abs(math.cos(a)), 1e-9), 2/max(abs(math.sin(a)), 1e-9))
            distance = round(r*1000)
            intensity = 80 + i
            data.extend(((intensity & 63) << 2,
                         ((distance & 63) << 2) | (intensity >> 6), distance >> 6))
        struct.pack_into('<H', data, 8, checksum(data, 'manual'))
        prefix = b''
        if self.index % 4 == 0:
            # Manual specifies a one-sample revolution-start packet.
            prefix = bytearray(struct.pack('<HBBHHH', 0x55AA, 201, 1, 1, 1, 0))
            prefix.extend(data[10:13])
            struct.pack_into('<H', prefix, 8, checksum(prefix, 'manual'))
            # The start sample replaces (does not duplicate) the first normal sample.
            data = bytearray(struct.pack('<HBBHHH', 0x55AA, 0, n-1,
                             round(0.9*128) | 1, round(last*128) | 1, 0)) + data[13:]
            struct.pack_into('<H', data, 8, checksum(data, 'manual'))
        self.index += 1
        return bytes(prefix) + bytes(data)
    def write(self, data):
        pass
    def close(self):
        pass

class Acquisition(threading.Thread):
    def __init__(self, source, output=None, checksum_mode='auto'):
        super().__init__(name='coin-d6-reader', daemon=True)
        self.source, self.output = source, Path(output) if output else None
        self.decoder = Decoder(checksum_mode)
        self.assembler = ScanAssembler()
        self.commands = queue.Queue(maxsize=16)
        self.messages = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.stats = {'connected': False, 'bytes': 0, 'packets': 0, 'bad_crc': 0,
                      'bad_headers': 0, 'discarded_bytes': 0, 'scans': 0,
                      'dropped_scans': 0, 'coalesced_boundaries': 0, 'profiles': {}, 'last_rx': None,
                      'recording': '', 'state': 'Connecting'}
        self._raw = self._jsonl = None
        self._capture_prefix = None

    def message(self, text):
        try:
            self.messages.put_nowait(text)
        except queue.Full:
            pass

    def command(self, action):
        try:
            self.commands.put_nowait(action)
        except queue.Full:
            self.message('Command queue full; wait and retry.')

    def snapshot(self):
        with self.lock:
            return self.latest, dict(self.stats)

    def shutdown(self):
        self.stop_event.set()

    def _recording_open(self):
        if not self.output:
            return
        self.output.mkdir(parents=True, exist_ok=True)
        prefix = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        if isinstance(self.source, DemoSource):
            prefix = 'DEMO_' + prefix
        self._capture_prefix = self.output / prefix
        self._raw = self._capture_prefix.with_suffix('.bin').open('wb')
        self._jsonl = self._capture_prefix.with_suffix('.jsonl').open('w', encoding='utf8')
        meta = {'application': 'COIN-D6 visualizer 0.2.0',
                'started_utc': datetime.now(timezone.utc).isoformat(),
                'host_monotonic_origin_s': time.monotonic(),
                'source': type(self.source).__name__,
                'port': getattr(self.source, 'port', None),
                'host': getattr(self.source, 'host', None),
                'tcp_port': getattr(self.source, 'tcp_port', None),
                'baud': getattr(self.source, 'baud', 230400),
                'units': 'raw mm, clockwise degrees from sensor zero',
                'checksum_mode': self.decoder.mode,
                'timestamp_note': 'Host reception times, not sensor timestamps; binary replay uses nominal UART pacing.',
                'slam': False}
        self._capture_prefix.with_suffix('.meta.json').write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding='utf8')
        self.stats['recording'] = str(self._capture_prefix)
        self.message('Recording bytes and scans to: ' + str(self._capture_prefix))

    def _recording_close(self):
        for f in (self._raw, self._jsonl):
            if f:
                f.close()
        if self._capture_prefix:
            self._capture_prefix.with_suffix('.stats.json').write_text(
                json.dumps(self.stats, indent=2, ensure_ascii=False), encoding='utf8')

    def _handle_commands(self):
        while True:
            try:
                cmd = self.commands.get_nowait()
            except queue.Empty:
                break
            if not self.source.writable:
                self.message('Demo/replay: no commands sent to LiDAR.')
                continue
            if cmd == 'start':
                self.decoder.buffer.clear()
                self.assembler = ScanAssembler()
                with self.lock:
                    self.latest = None
                self.source.write(START)
                self.stats['state'] = 'Start sent; waiting for valid packets'
                self.message('TX: AA 55 F0 0F - start. ACK not interpreted; verify packets.')
            elif cmd == 'stop':
                self.source.write(STOP)
                self.stats['state'] = 'Stop sent'
                self.message('TX: AA 55 F5 0A - stop. Verify physical stop.')

    def run(self):
        last_bad = 0
        try:
            self.source.open()
            self._recording_open()
            with self.lock:
                self.stats['connected'] = True
                self.stats['state'] = 'Connected; press Start LiDAR' if self.source.writable else 'DEMO / replay'
            self.message('TCP connected: '+self.source.host+':'+str(self.source.tcp_port) if isinstance(self.source, TCPSource) else
                         'Source connected.' if self.source.writable else
                         'Test source opened; not live hardware data.')
            while not self.stop_event.is_set():
                self._handle_commands()
                data = self.source.read()
                now = time.monotonic()
                if data:
                    if self._raw:
                        self._raw.write(data)
                    self.stats['bytes'] += len(data)
                    self.stats['last_rx'] = now
                packets = self.decoder.feed(data, now)
                faults = self.decoder.bad_checksums + self.decoder.bad_headers + self.decoder.incomplete_timeouts
                damaged = faults > last_bad
                for packet in packets:
                    for scan in self.assembler.feed(packet, damaged=damaged):
                        if self._jsonl:
                            self._jsonl.write(json.dumps(scan.record(), separators=(',', ':'))+'\n')
                            self._jsonl.flush()
                        with self.lock:
                            self.latest = scan
                    damaged = False
                # Carry faults until a valid packet is available to attribute them.
                if packets:
                    last_bad = faults
                    if self.source.writable:
                        self.stats['state'] = 'Receiving LiDAR data'
                with self.lock:
                    self.stats.update(packets=self.decoder.good_packets,
                        bad_crc=self.decoder.bad_checksums, bad_headers=self.decoder.bad_headers,
                        discarded_bytes=self.decoder.discarded_bytes,
                        scans=self.assembler.sequence, dropped_scans=self.assembler.dropped,
                        coalesced_boundaries=self.assembler.coalesced_boundaries,
                        profiles=dict(self.decoder.profile_counts))
        except EOFError:
            self.message('End of capture. Last scan remains visible as historical data.')
        except Exception as exc:
            self.message(f'ERROR: {type(exc).__name__}: {exc}')
        finally:
            if self.source.writable:
                try:
                    self.source.write(STOP)
                    self.message('Stop sent before closing the connection.')
                except Exception:
                    self.message('PC could not send STOP. Bridge attempts to stop on disconnect; verify the sensor.')
            try:
                self.source.close()
            except Exception as exc:
                self.message('Close error: ' + str(exc))
            with self.lock:
                self.stats['connected'] = False
                self.stats['state'] = 'Disconnected / end of file'
            try:
                self._recording_close()
            except Exception as exc:
                self.message('Recording close error: ' + str(exc))
