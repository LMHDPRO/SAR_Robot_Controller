#pragma once

static constexpr char VIDEO_VIEWER_HTML[] = R"HTML(<!doctype html>
<html lang="es" dir="ltr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-S3-CAM</title>
  <style>
    :root { color-scheme: dark; font: 1rem system-ui; }
    body { margin: 1rem; display: grid; justify-items: center; gap: 0.75rem; }
    .controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.75rem; }
    #frame {
      width: min(640px, 100%, 128vh);
      aspect-ratio: 16 / 9;
      position: relative;
      overflow: hidden;
      background: #171717;
      --angle: -90deg;
    }
    #frame[data-shape="portrait"] { width: min(360px, 100%, 43vh); aspect-ratio: 9 / 16; }
    #frame[data-shape="square"] { width: min(480px, 100%, 76vh); aspect-ratio: 1; }
    #frame.clockwise { --angle: 90deg; }
    #video {
      position: absolute;
      top: 50%;
      left: 50%;
      width: 56.25%;
      height: 177.777778%;
      object-fit: cover;
      transform: translate(-50%, -50%) rotate(var(--angle));
    }
    #frame[data-shape="portrait"] #video { width: 177.777778%; height: 56.25%; }
    #frame[data-shape="square"] #video { width: 100%; height: 100%; }
    #frame.full #video { object-fit: contain; }
    #status:empty { display: none; }
  </style>
</head>
<body>
  <div class="controls">
    <button type="button" id="rotate">Giro: −90°</button>
    <label>Formato
      <select id="shape">
        <option value="landscape">Horizontal 16:9</option>
        <option value="portrait">Vertical 9:16</option>
        <option value="square">Cuadrado 1:1</option>
      </select>
    </label>
    <label><input type="checkbox" id="full"> Ver imagen completa</label>
    <a href="/panel">Ajustes de cámara</a>
  </div>
  <div id="frame" data-shape="landscape"><img id="video" alt="Video en vivo"></div>
  <div id="status" role="status"></div>
  <script>
    const frame = document.getElementById('frame');
    const video = document.getElementById('video');
    const rotate = document.getElementById('rotate');
    const shape = document.getElementById('shape');
    const full = document.getElementById('full');
    const status = document.getElementById('status');
    const stream = new URL(location.href);
    stream.port = '81';
    stream.pathname = '/stream';
    stream.search = '';
    stream.hash = '';

    rotate.addEventListener('click', () => {
      const clockwise = frame.classList.toggle('clockwise');
      rotate.textContent = clockwise ? 'Giro: +90°' : 'Giro: −90°';
    });
    function updateView() {
      frame.dataset.shape = shape.value;
      frame.classList.toggle('full', full.checked);
    }
    shape.addEventListener('change', updateView);
    full.addEventListener('change', updateView);
    updateView();

    let retry;
    function connect() {
      stream.searchParams.set('t', Date.now());
      video.src = stream.href;
    }
    video.addEventListener('error', () => {
      status.textContent = 'Reconectando…';
      clearTimeout(retry);
      retry = setTimeout(connect, 2000);
    });
    video.addEventListener('load', () => {
      clearTimeout(retry);
      status.textContent = '';
    });
    connect();
  </script>
</body>
</html>)HTML";
