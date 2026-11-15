import json
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import unittest
from unittest.mock import Mock, patch
from camera_connection import prepare_arm_stream


class CameraConnectionTests(unittest.TestCase):
    def setUp(self):
        self.data={'cameras':[{'id':7,'resolutions':[
            {'width':160,'height':120,'index':0},
            {'width':320,'height':240,'index':2}]}],'activated':[]}
        self.posts=[];self.paths=[];owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                owner.paths.append(self.path)
                body=json.dumps(owner.data).encode()
                self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def do_POST(self):
                body=self.rfile.read(int(self.headers['Content-Length']))
                owner.posts.append((self.path,self.headers['Content-Type'],body))
                self.send_response(204);self.end_headers()
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.address=f'127.0.0.1:{self.server.server_port}'
        self.event=Mock();self.event.is_set.return_value=False;self.event.wait.return_value=False

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join(2)

    def test_ip_discovery_compact_activation_and_preferred_mode(self):
        result=prepare_arm_stream(self.address,self.event)
        self.assertEqual(result,'http://'+self.address+'/api/stream/7')
        self.assertEqual(self.paths,['/api/cameras'])
        self.assertEqual(self.posts,[('/api/active','application/json',b'{"id":"7","resolution":{"width":320,"height":240,"index":2}}')])
        self.event.wait.assert_called_once_with(1)

    def test_active_camera_is_not_reactivated(self):
        self.data['activated']=[{'id':'7'}]
        prepare_arm_stream('http://'+self.address+'/',self.event)
        self.assertFalse(self.posts);self.event.wait.assert_not_called()

    def test_smallest_mode_when_preferred_mode_missing(self):
        self.data['cameras'][0]['resolutions'][1]['width']=640
        prepare_arm_stream(self.address,self.event)
        self.assertEqual(json.loads(self.posts[0][2])['resolution']['index'],0)

    def test_empty_and_invalid_modes_fail_without_activation(self):
        for cameras in ([],[{'id':7,'resolutions':[]}],[{'id':7,'resolutions':[{'width':-1,'height':3,'index':0}]}]):
            self.data['cameras']=cameras
            with self.assertRaises(RuntimeError):prepare_arm_stream(self.address,self.event)
        self.assertFalse(self.posts)

    def test_direct_legacy_stream_preserved_and_stop_prevents_requests(self):
        url='http://'+self.address+'/stream'
        self.assertEqual(prepare_arm_stream(url),url)
        self.assertFalse(self.paths)
        self.event.is_set.return_value=True
        with self.assertRaisesRegex(RuntimeError,'cancelled'):prepare_arm_stream(self.address,self.event)
        self.assertFalse(self.paths)

    def test_explicit_api_stream_checks_requested_camera(self):
        with self.assertRaises(ValueError):prepare_arm_stream('http://'+self.address+'/api/stream/missing',self.event)
        self.assertFalse(self.posts)


class CameraIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6 import QtWidgets
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_arm_and_agv_panel_modes_and_startup_watchdog(self):
        from cameras import CameraPanel
        from PySide6 import QtCore
        import time
        arm=CameraPanel('Arm',{},arm_camera_api=True);agv=CameraPanel('AGV',{})
        try:
            self.assertTrue(arm.config()['arm_camera_api'])
            self.assertFalse(agv.config()['arm_camera_api'])
            arm.last_frame=time.monotonic()-6
            fake=Mock();fake.state.return_value=QtCore.QProcess.ProcessState.Running
            with patch.object(arm,'proc',fake):
                arm.check_age();fake.kill.assert_not_called()
                arm.received_frame=True;arm.check_age();fake.kill.assert_called_once()
        finally:arm.shutdown();agv.shutdown()

    def test_arm_vision_worker_prepares_api_before_opening_video(self):
        from robotarm_v7 import VisionThread
        worker=VisionThread('192.168.3.17',arm_camera_api=True)
        cv=Mock();cv.VideoCapture.return_value.isOpened.return_value=True
        with patch('camera_connection.prepare_arm_stream',return_value='http://camera/api/stream/7') as prepare:
            self.assertTrue(worker._open_source(cv))
            prepare.assert_called_once()
            self.assertEqual(cv.VideoCapture.call_args.args[0],'http://camera/api/stream/7')
        worker._release_sources_in_worker()
