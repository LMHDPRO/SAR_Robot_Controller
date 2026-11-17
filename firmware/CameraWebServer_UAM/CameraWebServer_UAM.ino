#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

#include "board_config.h"

// =====================================================
// CONFIGURACIÓN WIFI
// =====================================================
const char *WIFI_SSID = "No_Disponible";
const char *WIFI_PASSWORD = "Mapache2003";

// =====================================================
// CONFIGURACIÓN DE STREAMING
// =====================================================

// VGA = 640 x 480. Si necesitas JPEG cuadrado, usa FRAMESIZE_320X320.
// FRAMESIZE_HD da JPEG nativo 16:9 (1280 x 720) y requiere más ancho de banda.
// Para una prueba de menor ancho de banda, usa FRAMESIZE_QVGA (320 x 240).
constexpr framesize_t STREAM_RESOLUTION = FRAMESIZE_VGA;
// Número mayor = más compresión y menos datos por cuadro.
constexpr int JPEG_QUALITY = 22;
constexpr bool DIRECT_WIFI_TEST = false; // true: conectar PC a ESP32-CAM-TEST

void startCameraServer();
void setupLedFlash();

framesize_t streamFrameLimit() {
  return psramFound() ? STREAM_RESOLUTION : FRAMESIZE_QVGA;
}

// =====================================================
// CONEXIÓN WIFI
// =====================================================
bool conectarWiFi() {
  if (DIRECT_WIFI_TEST) {
    WiFi.mode(WIFI_AP);
    const bool ok = WiFi.softAP("ESP32-CAM-TEST", "camtest123", 6, false, 1);
    WiFi.setSleep(false);
    Serial.println("Prueba directa: conectar PC a ESP32-CAM-TEST, clave camtest123");
    return ok;
  }
  Serial.println();
  Serial.print("Conectando a la red WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  // Reconexión administrada por el sistema
  WiFi.setAutoReconnect(true);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Evita pausas causadas por ahorro de energía Wi-Fi.
  WiFi.setSleep(false);

  const unsigned long tiempoMaximo = 15000;
  const unsigned long inicio = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - inicio < tiempoMaximo) {
    Serial.print(".");
    delay(250);
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No fue posible conectarse al WiFi.");
    Serial.println("Revisa SSID, password y que la red sea 2.4 GHz.");
    return false;
  }

  Serial.println("WiFi conectado correctamente.");

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  Serial.print("IP de la camara: ");
  Serial.println(WiFi.localIP());

  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  return true;
}

// =====================================================
// CONFIGURACIÓN DE CÁMARA
// =====================================================
bool iniciarCamara() {
  camera_config_t config = {};

  // Temporizador para reloj XCLK de la cámara
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // Pines de datos
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  // Pines de sincronización
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  // Comunicación SCCB/I2C con el sensor
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  // Control del sensor
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Configuración principal
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  const bool hasPsram = psramFound();
  config.frame_size = streamFrameLimit();
  config.jpeg_quality = JPEG_QUALITY;

  // ===================================================
  // CONFIGURACIÓN SEGÚN DISPONIBILIDAD DE PSRAM
  // ===================================================
  if (hasPsram) {
    Serial.printf("PSRAM: %lu bytes\n", static_cast<unsigned long>(ESP.getPsramSize()));

    // Permite capturar un cuadro mientras se transmite otro
    config.fb_count = 2;

    // Guarda los framebuffers en PSRAM
    config.fb_location = CAMERA_FB_IN_PSRAM;

    // Entrega el cuadro más reciente y descarta cuadros viejos
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM no detectada.");
    Serial.println("Usando QVGA y un buffer. Si el modulo es N16R8, habilita OPI PSRAM.");

    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // ===================================================
  // INICIALIZAR CÁMARA
  // ===================================================
  const esp_err_t error = esp_camera_init(&config);

  if (error != ESP_OK) {
    Serial.printf(
      "Error iniciando la camara. Codigo: 0x%x\n",
      error
    );

    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();

  if (sensor == nullptr) {
    Serial.println("No fue posible obtener el sensor.");
    return false;
  }

  // ===================================================
  // AJUSTES PARA OV3660
  // ===================================================
  if (sensor->id.PID == OV3660_PID) {
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, 1);
    sensor->set_saturation(sensor, -2);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || \
    defined(CAMERA_MODEL_M5STACK_ESP32CAM)

  sensor->set_vflip(sensor, 1);
  sensor->set_hmirror(sensor, 1);

#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)

  sensor->set_vflip(sensor, 1);

#endif

#if defined(LED_GPIO_NUM)

  setupLedFlash();

#endif

  Serial.printf("Camara inicializada: %ux%u\n",
                static_cast<unsigned>(resolution[config.frame_size].width),
                static_cast<unsigned>(resolution[config.frame_size].height));

  return true;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("CAM V4 - VGA JPEG directo - diagnostico STA/AP");

  // Desactiva mensajes internos continuos para reducir carga
  Serial.setDebugOutput(false);

  delay(500);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP32-S3-CAM | OV3660");
  Serial.println("==============================");

  // Primero se inicializa la cámara
  if (!iniciarCamara()) {
    Serial.println("No se pudo iniciar la camara.");

    while (true) {
      delay(1000);
    }
  }

  // Después se conecta al WiFi
  if (!conectarWiFi()) {
    Serial.println("No se iniciara el servidor.");

    while (true) {
      delay(1000);
    }
  }

  // Iniciar servidor web de la cámara
  startCameraServer();

  const IPAddress ip = DIRECT_WIFI_TEST ? WiFi.softAPIP() : WiFi.localIP();

  Serial.println();
  Serial.println("==============================");
  Serial.println(" CAMARA LISTA");
  Serial.println("==============================");

  Serial.print("Visor 16:9 con giro -90: http://");
  Serial.println(ip);

  Serial.print("Panel de control: http://");
  Serial.print(ip);
  Serial.println("/panel");

  Serial.print("Stream directo: http://");
  Serial.print(ip);
  Serial.println(":81/stream");

}

// =====================================================
// LOOP
// =====================================================
void loop() {
  /*
   * El servidor de cámara funciona en otra tarea.
   * No colocamos ciclos, impresiones ni procesos pesados aquí.
   *
   * WiFi.setAutoReconnect(true) administra la reconexión.
   */

  delay(1000);
}
