"""Arm camera discovery/activation protocol from the supplied CameraIP.py."""
import json
import threading
from urllib.parse import urlsplit, urlunsplit, quote, unquote
from urllib.request import Request, urlopen


def prepare_arm_stream(address, stop_event=None, status=None):
    """Accept an ESP32-S3 IP/base URL or an explicit legacy stream URL.

    Network work belongs in a capture worker, never the Qt UI thread.
    """
    stop_event = stop_event or threading.Event()
    status = status or (lambda text: None)
    value = str(address).strip()
    if not value:
        raise ValueError('Enter the arm camera ESP32-S3 IP or URL')
    if '://' not in value:
        value = 'http://' + value
    parsed = urlsplit(value)
    if parsed.scheme not in ('http', 'https', 'rtsp') or not parsed.hostname:
        raise ValueError('Use an ESP32-S3 IP or an HTTP(S)/RTSP camera URL')
    # Validate the port before making any request.
    parsed.port
    if parsed.scheme == 'rtsp' or (parsed.path not in ('', '/', '/api/cameras') and not parsed.path.startswith('/api/stream/')):
        return value  # Preserve explicitly configured older stream endpoints.
    base = urlunsplit((parsed.scheme, parsed.netloc, '', '', ''))
    requested_id = unquote(parsed.path[len('/api/stream/'):]) if parsed.path.startswith('/api/stream/') else None

    def check_cancelled():
        if stop_event.is_set():
            raise RuntimeError('Camera connection cancelled')

    def request(path, payload=None, timeout=5):
        check_cancelled()
        data = None if payload is None else json.dumps(payload, separators=(',', ':')).encode('utf8')
        headers = {} if data is None else {'Content-Type': 'application/json'}
        with urlopen(Request(base+path, data=data, headers=headers), timeout=timeout) as response:
            # Bound discovery JSON memory. Activation only needs a successful status.
            body = response.read(1_048_577) if payload is None else None
        check_cancelled()
        if body is not None:
            if len(body) > 1_048_576: raise ValueError('Camera discovery response is too large')
            return json.loads(body)

    status('Discovering arm camera: /api/cameras')
    data = request('/api/cameras')
    cameras = data.get('cameras') if isinstance(data, dict) else None
    if not isinstance(cameras, list) or not cameras:
        raise RuntimeError('No cameras reported by the ESP32-S3')
    camera = cameras[0] if requested_id is None else next((c for c in cameras if isinstance(c,dict) and str(c.get('id')) == requested_id), None)
    if not isinstance(camera, dict) or camera.get('id') is None:
        raise ValueError('Requested camera was not found or has no ID')
    camera_id = str(camera['id'])
    resolutions = camera.get('resolutions', [])
    modes = [r for r in resolutions if isinstance(r, dict)
             and all(type(r.get(k)) is int for k in ('width','height','index'))
             and r['width'] > 0 and r['height'] > 0 and r['index'] >= 0] if isinstance(resolutions, list) else []
    if not modes: raise RuntimeError('Camera offers no valid MJPEG modes')
    resolution = next((r for r in modes if r['width'] == 320 and r['height'] == 240),
                      min(modes, key=lambda r: r['width']*r['height']))
    activated = data.get('activated', [])
    if not isinstance(activated, list): raise ValueError('Invalid activated camera list')
    active = any(isinstance(c,dict) and str(c.get('id')) == camera_id for c in activated)
    if not active:
        status(f"Activating camera {camera_id}: {resolution['width']}x{resolution['height']}")
        request('/api/active', {'id':camera_id,'resolution':{k:resolution[k] for k in ('width','height','index')}}, timeout=10)
        if stop_event.wait(1): raise RuntimeError('Camera connection cancelled')
    check_cancelled()
    status('Opening arm camera MJPEG stream')
    return base+'/api/stream/'+quote(camera_id, safe='')
