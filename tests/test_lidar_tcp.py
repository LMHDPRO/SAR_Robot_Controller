"""Loopback tests: real sockets, binary integrity and full acquisition pipeline."""
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from coind6.acquisition import TCPSource, Acquisition, DemoSource
from coind6.protocol import START, STOP

class FakeBridge:
    def __init__(self, handler):
        self.server=socket.socket();self.server.bind(('127.0.0.1',0));self.server.listen(1)
        self.server.settimeout(3);self.port=self.server.getsockname()[1]
        self.errors=[];self.handler=handler
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        try:
            with self.server:
                conn,_=self.server.accept()
                with conn:
                    conn.settimeout(3);self.handler(conn)
        except Exception as e:self.errors.append(e)
    def finish(self,test):
        self.thread.join(4);test.assertFalse(self.thread.is_alive());test.assertEqual(self.errors,[])

def receive(conn, n):
    b=b''
    while len(b)<n:
        part=conn.recv(n-len(b))
        if not part:raise EOFError('premature close')
        b+=part
    return b

class TCPTests(unittest.TestCase):
    def test_raw_bytes_fragmentation_and_commands(self):
        payload=bytes(range(256))*12;received=[]
        def server(conn):
            received.append(receive(conn,4))
            for i in range(0,len(payload),17):conn.sendall(payload[i:i+17])
            received.append(receive(conn,4))
        b=FakeBridge(server);s=TCPSource('127.0.0.1',b.port);s.open();s.write(START)
        raw=b'';end=time.monotonic()+3
        while len(raw)<len(payload) and time.monotonic()<end:raw+=s.read()
        self.assertEqual(raw,payload);s.write(STOP);s.close();b.finish(self)
        self.assertEqual(received,[START,STOP])
    def test_timeout_and_remote_close(self):
        def server(conn):time.sleep(.15)
        b=FakeBridge(server);s=TCPSource('127.0.0.1',b.port);s.open()
        self.assertEqual(s.read(),b'');time.sleep(.2)
        with self.assertRaises(ConnectionError):s.read()
        s.close();b.finish(self)
    def test_full_acquisition_recording_over_tcp(self):
        demo=DemoSource();demo.open();payload=b''.join(demo.read() for _ in range(24))
        commands=[]
        def server(conn):
            commands.append(receive(conn,4))
            for i in range(0,len(payload),251):
                conn.sendall(payload[i:i+251]);time.sleep(.001)
            commands.append(receive(conn,4))
        b=FakeBridge(server)
        with tempfile.TemporaryDirectory() as tmp:
            a=Acquisition(TCPSource('127.0.0.1',b.port),tmp);a.start();a.command('start')
            end=time.monotonic()+3
            while time.monotonic()<end and a.snapshot()[1]['bytes']<len(payload):time.sleep(.01)
            a.shutdown();a.join(2)
            self.assertFalse(a.is_alive())
            scan,stats=a.snapshot();self.assertGreaterEqual(stats['scans'],5)
            self.assertEqual(len(scan.points),400);self.assertEqual(stats['bad_crc'],0)
            raw,=Path(tmp).glob('*.bin');self.assertEqual(raw.read_bytes(),payload)
            meta,=Path(tmp).glob('*.meta.json');self.assertIn('TCPSource',meta.read_text())
        b.finish(self);self.assertEqual(commands,[START,STOP])
    def test_connection_refused(self):
        with socket.socket() as port_holder:
            port_holder.bind(('127.0.0.1',0));port=port_holder.getsockname()[1]
            s=TCPSource('127.0.0.1',port)
            with self.assertRaises(OSError):s.open()
            s.close()

if __name__=='__main__':unittest.main()
