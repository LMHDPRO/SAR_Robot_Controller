# SAR Robot Controller 1.6

Integración de **SAR 1.5 + RobotArmCodeV7(2).py**. Aplicación local Python/PySide6, firmware del AGV y firmware COCO. No requiere BAT. Ejecutar `app.py` desde VS Code. Este paquete contiene solamente Python, sketches `.ino` y este README.

## Qué está integrado

- **AGV:** conducción, odometría, calibraciones de ruedas/distancia, giros por IMU, rutinas y exportación CSV de la versión 1.5.
- **Programa por bloques:** combinar avance con encoder/IMU, giro preciso, espera, pose por servos, pose XYZ mediante IK y alineación ArUco 12. Los bloques se ejecutan en orden; el brazo espera confirmación PWM estimada y STOP cancela el programa. Guardar/cargar conserva la compatibilidad con rutinas anteriores. XYZ usa el modelo del brazo, sin detección de colisiones ni medición articular; se requiere verificar cada trayectoria física.
- **Arm V7:** separate 3D preview, FK/IK controls, gripper, speed, A/B points, obstacles and manual path planning. The Camera tab is preview-only; it cannot start autonomous search or grasping. ArUco detection remains in Cameras IP.
- **Control global:** IP del AGV y COCO, propietario PC/COCO, alimentación, STOP, E-STOP, desbloqueo, reset de odometría y estados I2C/PWM/telemetría visibles en todas las pestañas.
- **Dos cámaras ESP32-S3 IP:** conexiones independientes, preview MJPEG/RTSP según firmware de cámara, detección ArUco y distancia en coordenadas de cada cámara si está calibrada. Captura en procesos separados, sin bloquear el hilo de interfaz si la conexión falla.
- **LiDAR/SLAM:** external COIN-D6 over a dedicated PC-to-LiDAR TCP connection (no USB or LiDAR traffic through the AGV), offline replay, recording and experimental local 2D SLAM. Select Left/Right encoder + IMU prediction, configure footprint/sector exclusion, and save the finished map as NPZ + PNG. No loop closure or autonomous navigation.
- **Ground calibration:** 50 cm test with selectable Left/Right encoder and IMU heading hold, actual-distance entry and atomic persistence in `settings.json`.

## Instalación y ejecución

Se recomienda Python 3.11 o 3.12 de 64 bits. Desde la terminal de VS Code en esta carpeta:

```sh
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
python -m pip install "PySide6==6.8.3" "pyqtgraph==0.14.0" "numpy>=1.26,<3" matplotlib pyserial "opencv-contrib-python-headless>=4.10,<6"
python app.py
```

Select this interpreter in VS Code. Qt provides the window; OpenCV does not need its own windows. Avoid installing multiple OpenCV distributions in the same environment. No learned object detector or model weights are required.

Se fija Qt 6.8.3 para evitar el fallo de instalación por rutas largas encontrado con Qt 6.12 en esta carpeta de Windows. Para conexión, calibración y límites de SLAM, consultar [LIDAR_SLAM.md](LIDAR_SLAM.md).

## Archivos

| Archivo | Función |
|---|---|
| `app.py` | Ventana global, control AGV y motor de comunicaciones/seguridad |
| `core.py` | Odometría, geometría, calibración y primitivas AGV |
| `arm.py` | FK/IK compartida por SAR y COCO |
| `robotarm_v7.py` | Algoritmos y UI originales V7, con fuente IP configurable |
| `arm_workspace.py` | Adaptación de V7 al transporte SAR y paradas globales |
| `cameras.py` | Dos paneles de cámara y gestión de procesos |
| `camera_worker.py` | Lectura de stream, detección ArUco y estimación de pose |
| `camera_connection.py` | Arm ESP32-S3 camera discovery, MJPEG mode selection and activation |
| `lidar_workspace.py` | Pestaña LiDAR, calibración, captura, mapa y exportación |
| `slam.py` | SLAM local 2D por ICP, validación y mapa de puntos acotado |
| `lidar_filters.py` | Robot footprint and configurable angular exclusion |
| `map_export.py` | NPZ map archive, calibration metadata and PNG plot |
| `coind6/` | Decodificador, ensamblador y adquisición del ZIP Lidar del usuario |
| `defaults.py` | IP y calibraciones iniciales conservadas de la versión 1.5 |
| `firmware/SAR_AGV/SAR_AGV.ino` | AGV, BNO055, relés y PCA9685 del brazo |
| `firmware/COCO_Controller/COCO_Controller.ino` | Mando COCO |
| `tests/*.py` | Pruebas de lógica, interfaz, sockets simulados e integración |

## Red y conexiones

Se conserva como valor inicial la IP **192.168.3.11** del AGV y **192.168.137.115** de COCO. Son subredes distintas: el PC necesita acceso a ambas, o hay que ajustar las IP en pantalla para la red real. Los sketches reciben su IP por DHCP; la aplicación no asigna direcciones a las placas.

| Equipo | Comunicación | Configuración |
|---|---|---|
| PC → AGV | TCP 4211 | IP en barra global |
| PC → COCO | TCP 4210 | IP en barra global |
| PC → cámara AGV | URL del stream | Pestaña Cámaras IP |
| PC → arm camera | ESP32-S3 IP/base URL; CameraIP API | Cameras IP → Arm camera |
| PC → arm camera preview | Same ESP32-S3 IP/base URL; CameraIP API | Arm → Camera |
| AGV → PCA9685/BNO055/OLED | I2C | SDA 11 / SCL 12 |

Ejemplo de URL común en firmware CameraWebServer: `http://IP_DE_LA_ESP32:81/stream`. Usar el endpoint que publique realmente cada cámara; no todas usan ese puerto o ruta. Conservar el firmware de cámara que ya funciona: este paquete no inventa un pinout de cámara ESP32-S3, porque depende del modelo exacto. Los dos sketches incluidos son AGV y COCO.

### Arm camera: supplied CameraIP.py procedure

Enter the arm camera ESP32-S3 IP (for example `192.168.3.17`) or base URL (`http://192.168.3.17`, including a port if needed). The address remains user-configurable; connecting does not use a hardcoded device address. The same procedure is used by the arm camera panel and Arm/Camera:

1. GET `/api/cameras` with a 5-second timeout.
2. Select the first reported camera. Prefer 320x240 MJPEG; otherwise use the smallest advertised valid mode.
3. If its ID is not listed in `activated`, POST compact JSON to `/api/active` with `id` and `resolution` (`width`, `height`, `index`), using a 10-second timeout, then wait one second.
4. Open `/api/stream/{id}`. An already active camera is not reconfigured. Discovery/activation run in the capture worker, outside the UI thread.

An explicit `/api/stream/{id}` URL discovers that specific ID. Existing explicit direct stream URLs such as `/stream`, `:81/stream` or RTSP remain supported without discovery. Use the IP/base URL for the supplied firmware's automatic procedure. The arm camera panel allows up to 25 seconds for initial discovery/activation/opening, then uses the normal 4-second frame-loss watchdog. The AGV camera retains its existing direct-stream connection procedure.

Save panel addresses with the camera page's save button. Arm/Camera preloads `arm_camera_url` from `runtime.json` (initialized to the current arm-camera IP) and saves edits when the field loses focus; the two views retain separate address settings. Stop the unused view before switching if the camera firmware only supports one stream client. No firmware changes or new Python dependencies are required. Hardware connectivity and image quality still need testing with the actual ESP32-S3.

Las claves Wi-Fi se sustituyeron por `CONFIGURAR_CLAVE_WIFI` para no incluir contraseñas reales en un repositorio. Configurar SSID y contraseña en ambos sketches antes de cargar. No publicar los valores reales en GitHub.

### Pinout AGV conservado del sketch 1.5

| Función | GPIO / dirección |
|---|---|
| ENA / ENB | 9 / 18 |
| IN1 / IN2 / IN3 / IN4 | 10 / 17 / 21 / 38 |
| Encoder izquierdo A/B | 5 / 6 |
| Encoder derecho A/B | 7 / 8 |
| Relés 1/2, activos en bajo | 48 / 47 |
| I2C SDA/SCL | 11 / 12 |
| E-Stop físico NC hacia GND | GPIO 1; abierto activa emergencia |
| PCA9685 | 0x40 |
| BNO055 | búsqueda 0x28 y 0x29 |
| OLED | búsqueda 0x3C y 0x3D |

Este es el cableado del archivo adjunto, no un pinout genérico para cualquier driver. El PCA9685 comparte el bus del AGV en esta arquitectura; no se añadió un ESP32-C3 independiente. Alimentar los servos por su fuente adecuada y unir las referencias GND, sin alimentarlos desde 3V3 del ESP32.

### Canales PCA9685

| Canal | Función | Rango de comando |
|---|---|---|
| CH0 | Base | 0–180° |
| CH1 | Hombro | 0–180° |
| CH2 | Codo | 0–180° |
| CH3 | Giro de muñeca | 0–180°; preview PC no lo modela (inicia en 90°) |
| CH4 | Elevación muñeca | 0–180° |
| CH5 | Pinza | 10–75° |

V7 permitía 90° de apertura automática; el adaptador usa 75° para respetar los límites del firmware 1.5. La pose inicial se conserva: `[90, 126.5, 74.2, 106]` para las cuatro articulaciones activas. No se modificaron las longitudes DH, offsets ni signos de V7/SAR.

### Pinout COCO conservado

| Función | GPIO |
|---|---|
| Joystick brazo X/Y/SW | 2 / 1 / 5 |
| Joystick AGV X/Y/SW | 4 / 3 / 6 |
| Encoder CLK/DT/SW | 7 / 8 / 9 |
| Deadman | 10 |
| OLED SDA/SCL | 11 / 12 |

### Control del brazo desde COCO

- **A · Joint:** girar el encoder para seleccionar uno de los seis canales PCA9685; el eje vertical del joystick del brazo suma o resta grados al canal seleccionado. CH0–CH4 usan 0–180° y CH5 (pinza) 10–75°. El modo A no mueve el AGV.
- **B · Gamepad:** conserva el control legacy: el eje horizontal cambia el ángulo de base y el vertical acerca/aleja el objetivo radialmente; el IK CCD avanza en pasos limitados hacia el objetivo. El encoder cambia Z o pinza, según su botón.
- **C · TCP:** queda temporalmente igual que B y reutiliza el IK CCD actual; no es aún un controlador TCP independiente.
- El OLED de COCO muestra la XYZ estimada actual del brazo y el estado de conducción; el detalle de AGV sigue en la pantalla Debug / AGV INFO. Son estimaciones PWM/FK, no mediciones con encoders articulares.
- Los tres modos requieren COCO como propietario, AGV listo, deadman y joystick centrado antes de habilitar movimiento. La casilla global **Live Robot Arm movement** conserva el control continuo entre pestañas; al desactivarla deja de enviar poses. **Enviar pose (prueba)** en el panel Brazo manda una pose puntual y queda deshabilitada durante Live. Al cambiar de pestaña, el STOP global sigue actuando cuando Live está apagado.
- Al avanzar o retroceder con COCO o con los botones de PC, el rumbo del IMU se fija al comenzar y el control corrige la diferencia de ruedas. Al soltar, invertir o girar se libera esa referencia. Si falta el IMU o se pierde su continuidad, se detiene el avance y COCO requiere liberar el deadman antes de volver a habilitarlo. Los giros manuales siguen siendo libres. En las rutinas, los desplazamientos usan el encoder seleccionado en **Drive** y el IMU; los giros usan el modo preciso. Las rutas guardadas con los nombres anteriores se interpretan con esos modos.

Bibliotecas Arduino: WiFi/Wire/Preferences incluidas en ESP32; Adafruit BNO055, Unified Sensor, GFX, SSD1306, PWM Servo Driver y sus dependencias. El constructor correcto es `Adafruit_PWMServoDriver pca(PCA_ADDR, Wire);`. Usar la placa/PSRAM/flash correspondientes a cada módulo. El firmware conserva la API ESP32 Arduino 3.x del proyecto original; no se compiló aquí con tu placa.

## Secuencia de operación

1. Abrir `app.py`. La interfaz no conecta ni mueve automáticamente el robot.
2. Revisar IP, conectar AGV/COCO y comprobar telemetría reciente, BNO055, PCA9685 y relés.
3. Si hay emergencia, liberar el paro físico y pulsar **Desbloquear E-STOP**. Esperar confirmación; esto no activa los relés.
4. Seleccionar propietario **PC** o **COCO** y pulsar **POWER ON** cuando corresponda.
5. En **AGV**, utilizar los controles existentes. Se conservan los ajustes reales del ZIP: rueda 120 mm, track 200 mm, 900 ticks/rev, escalas 0.6463144897, signos de encoder -1, IMU -1 e intercambio de encoders activado. Verificar físicamente esos valores en lugar de asumir que ya están calibrados para otra mecánica.
6. Al cambiar de pestaña principal, se envía STOP salvo que **Live Robot Arm movement** esté activado; esa casilla mantiene el control del brazo activo hasta desactivarla o pulsar STOP. El control Live requiere conexión, telemetría, PCA9685 y propiedad PC.
7. En **Brazo**, sliders y trayectorias son simulados por defecto. **Enviar pose (prueba)** manda la pose actual una sola vez y se deshabilita durante Live. Las casillas de reproducción física requieren conexión, telemetría, PCA9685 y propiedad PC.
8. COCO sigue conectado por SAR, con sus modos y deadman existentes. La conexión USB/pendant propia de V7 no se expone para evitar dos controladores compitiendo por el brazo.
9. **STOP** cancela rutas, colas de visión, recuperación y envío live; detiene motores e interpolación de servos. **E-STOP** además enclava el paro y apaga relés. Los PWM de servo reportados son estimaciones, no realimentación de encoders. Reset odometría cambia la referencia estimada; no hace homing.
10. Una pérdida de enlace o rechazo del motor SAR cancela las fuentes físicas de V7. La reconexión no vuelve a marcar las casillas ni reanuda rutas. Cerrar la app detiene el robot y los procesos de cámara.

## Cámaras IP y ArUco

En **Cámara AGV**, el botón **Calibrar cámara AGV** abre una sesión con el tablero de 10 × 8 cuadros de 25 mm (9 × 7 esquinas interiores). Con el AGV detenido, mueve el tablero y pulsa **Capturar vista** en al menos 12 posiciones e inclinaciones diferentes; se recomiendan 20–30 y cubrir el centro y los bordes de la imagen. Cada captura debe mostrar el tablero completo. Pulsa **Calcular y guardar**: la aplicación guarda `camera_agv_calibration.npz` y recuerda su ruta en `cameras.json`. Comprueba las distancias con una regla. Si 10 × 8 representa esquinas interiores en vez de cuadros, o los 25 × 20 cm incluyen márgenes, esta configuración del tablero no corresponde y debe corregirse antes de calibrar.

Para aproximarse al ArUco 12, conecta cámara y AGV, coloca manualmente el brazo en la postura elegida y el marcador en la posición deseada a unos 20 cm de la lente; espera lecturas estables y pulsa **Guardar referencia ArUco 12** en configuración. La referencia se conserva entre sesiones. Con el camino despejado, selecciona control PC, activa POWER y pulsa **Iniciar secuencia ArUco**. El brazo recorre las dos posturas indicadas abajo; tras la confirmación PWM estimada, el AGV espera una nueva lectura válida del marcador y se acerca a una distancia estimada de 180 ± 5 mm de lectura directa. Antes del aviso de extracción comprueba que la orientación visual coincida con la referencia y que el marcador esté aproximadamente frontal a la cámara; si no, pide ajuste manual. A más de 240 mm usa tramos limitados por encoders e IMU y después espera imágenes; cerca del objetivo usa pulsos cortos. STOP cancela la secuencia. La recolección se inicia por separado tras verificar la distancia real. Si cambia el tamaño configurado del marcador, la referencia se ajusta proporcionalmente; verifica que sea la medida física correcta. La referencia se invalida si cambian cámara, calibración, diccionario o resolución. No detecta obstáculos; supervisa la prueba con acceso a STOP. Se requiere validación física de la dirección, distancia de frenado y postura de agarre. La distancia es una estimación visual, no una garantía de precisión física de 5 mm.

El botón **Colocar brazo en postura de aproximación** solicita primero la postura ArUco CH0–CH5 `[90, 126, 74, 90, 106, 75]` durante 3 segundos. Tras recibir telemetría de interpolación y su finalización, solicita `[108.5, 149.4, 180, 90, 6.8, 75]` durante otros 3 segundos. El protocolo redondea los ángulos a grados enteros. STOP cancela la secuencia; si no se confirma la primera etapa en 5 segundos se cancela. La telemetría es una estimación PWM, no una medición física. Requiere AGV conectado, control PC, POWER y movimiento de base detenido.

Tras verificar la distancia real y el recorrido del brazo, **Iniciar Approach → Pick → regreso** ejecuta Approach `[110.9,112.5,160.5,106]` con pinza 75°, Pick `[110.9,71.1,142.8,84.7]`, espera 1 segundo, cierra la pinza a 10°, espera otros 2 segundos y vuelve a Approach y finalmente a `[108.5,149.4,180,6.8]` con la pinza cerrada. **Dejar pieza → abrir → regresar** exige telemetría estimada de esa pose con pinza a 10°, va a Approach y a Pick con la pinza cerrada, espera 1 segundo, abre a 75°, espera 2 segundos y regresa a Approach y a la postura segura con pinza abierta. CH3 se ordena a 90°. Cada etapa dura 3 segundos y exige confirmación de la interpolación estimada. STOP cancela; no hay detección física de agarre, encoders articulares ni detección de colisiones.

Normalmente basta una calibración intrínseca por cámara mientras no cambien la óptica, el enfoque o el recorte de la imagen. Si se mueve la cámara respecto al chasis hay que volver a medir su montaje respecto a la base; este botón no calibra esa relación ni activa el movimiento automático del AGV.

Configurar cada URL, diccionario, rotación del preview y lado exterior del marcador en milímetros. La rotación ofrece 0°, 90° horario, 90° antihorario y 180°; solo cambia la imagen mostrada, no los píxeles usados por ArUco ni la calibración. Los 35 mm iniciales son un valor editable: deben coincidir con el marcador impreso. Pulsar Conectar en cada panel y guardar la configuración. Si el stream se pierde, el worker reintenta con espera progresiva; Detener cancela esos reintentos.

Sin calibración se muestran imagen e IDs; las columnas métricas quedan vacías. Para habilitarlas, elegir un `.npz` calibrado para **esa cámara, óptica y resolución/recorte** con:

- `camera_matrix`: matriz intrínseca 3×3.
- `dist_coeffs`: coeficientes de distorsión OpenCV (4, 5, 8, 12 o 14).
- `image_size`: `[ancho, alto]` de la calibración.

Se escala la matriz si cambia proporcionalmente la resolución. Se rechazan relaciones de aspecto distintas; un recorte diferente requiere recalibración. La pose usa `solvePnP` con IPPE_SQUARE; se descartan poses con error de reproyección superior a 3 px. El preview conserva proporción y no se amplía por encima de la resolución de la imagen. Esto no elimina toda la incertidumbre óptica ni la ambigüedad de un marcador plano.

X cámara apunta a la derecha de la imagen, Y hacia abajo y Z hacia delante. Distancia es la norma del vector de traslación cámara–marcador. **No es la distancia desde la base del robot.** No se promedian las posiciones de ambas cámaras ni se mandan automáticamente como objetivos IK.

La conversión futura requiere `T_base_camera` para una cámara fija. En la cámara montada en muñeca, requiere `T_base_tool(q) @ T_tool_camera` y sincronización temporal con la pose. También falta asociar el ID al objeto de rescate y definir agarre, altura de aproximación y destino. Esas calibraciones no se pueden inferir de las IP.

## Camera tests and future marker-guided SAR procedure

**Arm > Camera** tests the camera connection and shows a preview without inference or motion. **Cameras IP** detects ArUco IDs and displays calibrated camera-frame positions. The old object-search, class/confidence, model selection and automatic pickup pipeline have been removed. Manual arm control and path planning remain available. Preview resizing preserves the camera aspect ratio.

The camera panel supports a supervised approach to a taught ArUco 12 reference as described above. Automatic object localization for the arm and grasping are not implemented; the marker ID alone does not specify grasp coordinates.

See [OpenCV ArUco detection and pose estimation](https://docs.opencv.org/4.10.0/d5/dae/tutorial_aruco_detection.html) for the camera calibration and physical marker-size requirements. Camera-frame measurements must still be transformed into the relevant robot frame.

No abrir el mismo stream simultáneamente en V7 y en Cámaras IP si el firmware ESP32 admite un único cliente. Detener una vista antes de iniciar la otra. Las dos cámaras distintas sí pueden verse simultáneamente.

## Persistencia y repositorio

Al usar la app se crean archivos locales JSON para ajustes, URLs y parámetros de visión. No se incluyen archivos de configuración generados por las pruebas. `defaults.py` contiene los valores originales para el primer inicio. Conviene excluir del repositorio `.venv/`, `__pycache__/`, `*.pyc`, los JSON locales, calibraciones personales, grabaciones. No se incluyeron BAT, informes antiguos ni hashes desactualizados.

## Validación y límites

Ejecutar desde esta carpeta:

```sh
python -m unittest discover -s tests -v
```

Las pruebas incluyen el controlador existente, sockets con robot simulado, exclusión visual AGV/brazo, cancelación del brazo, correspondencia FK entre SAR y V7, bloqueo live sin conexión y ArUco sintético con/sin calibración. El test de mantener relés durante pérdida de telemetría selecciona explícitamente esa política; el valor inicial conservado del usuario es apagar relés.

Resultado en este entorno: 135 pruebas automatizadas aprobadas. También se verificaron dos procesos simultáneos contra streams MJPEG locales con un marcador ArUco sintético. La interfaz y la lógica se verifican en software. Falta la validación con el AGV, servos, cámaras ESP32-S3 y placas reales, y la compilación Arduino local. La comprobación de obstáculos de V7 es un modelo geométrico; no equivale a detección física de colisión. Esta versión no incorpora homing con sensores, SLAM, calibración extrínseca ni rescate autónomo por ArUco.
