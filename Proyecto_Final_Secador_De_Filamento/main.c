#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <WiFi.h>           // Añadido para conexión WiFi
#include <FirebaseESP32.h>  // Añadido para Firebase

U8G2_SSD1306_128X64_NONAME_1_HW_I2C myScreen(U8G2_R0, U8X8_PIN_NONE);
Adafruit_BME280 bme;

// Credenciales WiFi
const char* WIFI_SSID = "INFINITUM5D9B_2.4";
const char* WIFI_PASSWORD = "CVXmG6gWNF";

// Configuración Firebase
const char* FIREBASE_HOST = "https://esp32-dex-dryer-default-rtdb.firebaseio.com/";
const char* FIREBASE_AUTH = "AIzaSyBj7Xppjmqawd1Lho2xcyhRX_j7Et3qDXU";

FirebaseData fbdo;      // Objeto para datos Firebase
FirebaseAuth auth;      // Objeto para autenticación
FirebaseConfig config;  // Configuración de Firebase

// Pines del hardware
const uint8_t relay_pin = 25;  // Relé para ventilador
const uint8_t ssr_pin   = 17;  // SSR para calefacción

const uint8_t encoder_clk = 19;  // Canal A del encoder
const uint8_t encoder_dt  = 18;  // Canal B del encoder
const uint8_t button     = 23;   // Botón pulsador

// Configuración PWM para SSR
const int SSR_PWM_CHANNEL = 0;
const int SSR_PWM_FREQ    = 100;  // 100 Hz
const int SSR_PWM_RES     = 8;    // 8 bits (0-255)
const int SSR_MAX_DUTY    = 255;

// Variables del sistema
float setTemp = 45.0;    // Temperatura objetivo
float currentTemp = 0;   // Temperatura actual leída
float currentHum  = 0;   // Humedad actual leída

SemaphoreHandle_t dataMutex;  // Mutex para proteger datos compartidos

// Parámetros del controlador PID
float kp = 20.0, ki = 1.25, kd = 0.2;
float pidIntegral = 0;  // Acumulador del término integral
float pidLastErr  = 0;  // Último error para derivada

// Variables del encoder (volatile para ISR)
volatile int8_t encoderDirection = 0;  // Dirección: -1, 0, +1
volatile int8_t stepAccumulator  = 0;  // Acumulador de pasos
volatile uint8_t lastEncoderState = 0; // Estado anterior del encoder

float lastTemp = 0;      // Temperatura anterior para calcular tasa
bool fanState = false;   // Estado actual del ventilador

// Intervalo para actualizar Firebase (milisegundos)
const unsigned long FB_UPDATE_INTERVAL = 5000;  // Enviar cada 5 segundos
unsigned long lastFirebaseUpdate = 0;

// Prototipos de funciones para nuevas tareas
void TaskFirebaseUpdate(void *pvParameters);
void TaskWiFiManager(void *pvParameters);

// Rutina de servicio de interrupción para encoder
void IRAM_ATTR readEncoder() {
  // Lee estado actual del encoder (valor de 2 bits)
  uint8_t state = (digitalRead(encoder_clk) << 1) | digitalRead(encoder_dt);
  uint8_t transition = (lastEncoderState << 2) | state;  // Código de transición de 4 bits

  // Determina dirección basada en secuencia de código Gray
  switch (transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: 
      stepAccumulator++; break;  // Sentido horario
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: 
      stepAccumulator--; break;  // Sentido antihorario
  }

  lastEncoderState = state;  // Guarda estado actual

  // Verifica si se completó un detente (4 pasos por detente)
  if (stepAccumulator >= 4) {
    encoderDirection = +1;  // Detente horario completado
    stepAccumulator = 0;
  } else if (stepAccumulator <= -4) {
    encoderDirection = -1;  // Detente antihorario completado
    stepAccumulator = 0;
  }
}

// Cálculo del controlador PID
uint8_t computePID(float temp) {
  float error = setTemp - temp;  // Error = objetivo - actual

  pidIntegral += error;  // Acumula error para término integral
  pidIntegral = constrain(pidIntegral, -50, 50);  // Anti-windup

  // Salida PID: P + I + D
  float output = kp * error + ki * pidIntegral + kd * (error - pidLastErr);
  pidLastErr = error;  // Guarda error para siguiente derivada

  return constrain(output, 0, 100);  // Limita a 0-100%
}

// Controla SSR con PWM basado en salida PID
void driveSSR(uint8_t pidPercent) {
  // Mapea 0-100% a duty cycle 255-0 (invertido porque SSR es activo-bajo)
  uint8_t duty = map(pidPercent, 0, 100, 255, 255 - SSR_MAX_DUTY);
  ledcWrite(SSR_PWM_CHANNEL, duty);
}

// Lógica difusa para control del ventilador
bool fuzzyFan(float temp) {
  float error = temp - setTemp;  // Positivo = por encima del objetivo
  float dTemp = temp - lastTemp; // Tasa de cambio de temperatura

  // Reglas difusas:
  if (error > 2.0 && dTemp > 0.05) return false;  // Caliente y aumentando
  if (error > 1.0 && dTemp > 0.0)  return false;  // Encima del objetivo y subiendo
  if (error < -1.5)                return true;   // Muy por debajo del objetivo
  if (abs(error) < 0.5 && abs(dTemp) < 0.03) return true;  // Estable cerca del objetivo

  return fanState;  // Mantiene estado actual
}

// Controla relé del ventilador
void driveFan(bool on) {
  fanState = on;
  // LOW = ENCENDIDO (relé activo-bajo)
  digitalWrite(relay_pin, on ? LOW : HIGH);
}

// Tarea 1: Maneja entrada del encoder
void TaskEncoder(void *pvParameters) {
  for (;;) {
    if (encoderDirection != 0) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      setTemp += encoderDirection * 0.5;  // Cambia temp en 0.5°C por detente
      setTemp = constrain(setTemp, 30, 70);  // Limita rango 30-70°C
      encoderDirection = 0;  // Reinicia dirección
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // Verifica cada 20ms
  }
}

// Tarea 2: Lee sensores y controla sistema
void TaskSensorControl(void *pvParameters) {
  for (;;) {
    // Lee datos del sensor
    float t = bme.readTemperature();
    float h = bme.readHumidity();

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    currentTemp = t;
    currentHum  = h;

    // Calcula y aplica control PID
    uint8_t pid = computePID(t);
    driveSSR(pid);

    // Controla ventilador con lógica difusa
    bool fanCmd = fuzzyFan(t);
    driveFan(fanCmd);

    lastTemp = t;  // Guarda para siguiente cálculo de tasa
    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(250));  // Actualiza cada 250ms
  }
}

// Tarea 3: Actualiza pantalla OLED
void TaskDisplay(void *pvParameters) {
  char b1[8], b2[8], b3[8];  // Buffers para cadenas formateadas

  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    // Convierte floats a strings
    dtostrf(currentTemp, 4, 1, b1);
    dtostrf(currentHum,  4, 1, b2);
    dtostrf(setTemp,     4, 1, b3);
    xSemaphoreGive(dataMutex);

    // Muestra en OLED
    myScreen.firstPage();
    do {
      myScreen.drawStr(0, 15, "Temp:");
      myScreen.drawStr(50, 15, b1);
      myScreen.drawStr(90, 15, "C");

      myScreen.drawStr(0, 32, "Hum:");
      myScreen.drawStr(50, 32, b2);
      myScreen.drawStr(90, 32, "%");

      myScreen.drawStr(0, 50, "Set:");
      myScreen.drawStr(50, 50, b3);
      myScreen.drawStr(90, 50, "C");
    } while (myScreen.nextPage());

    vTaskDelay(pdMS_TO_TICKS(500));  // Actualiza pantalla cada 500ms
  }
}

// NUEVA Tarea 4: Envía datos a Firebase
void TaskFirebaseUpdate(void *pvParameters) {
  for (;;) {
    unsigned long currentTime = millis();
    
    // Verifica si es momento de actualizar Firebase
    if (currentTime - lastFirebaseUpdate >= FB_UPDATE_INTERVAL) {
      float temp, hum, target;
      
      // Obtiene datos protegidos
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      temp = currentTemp;
      hum = currentHum;
      target = setTemp;
      xSemaphoreGive(dataMutex);
      
      // Prepara datos JSON
      String path = "/sensor_data";
      FirebaseJson json;
      
      json.set("temperature", temp);
      json.set("humidity", hum);
      json.set("target_temperature", target);
      json.set("timestamp", millis() / 1000);  // Marca de tiempo Unix
      json.set("fan_state", fanState ? "ON" : "OFF");
      
      // Envía a Firebase
      if (Firebase.updateNode(fbdo, path, json)) {
        Serial.println("Firebase: Actualización exitosa");
      } else {
        Serial.println("Firebase: Error - " + fbdo.errorReason());
      }
      
      lastFirebaseUpdate = currentTime;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Verifica cada segundo
  }
}

// NUEVA Tarea 5: Maneja conexión WiFi
void TaskWiFiManager(void *pvParameters) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Conectando a WiFi...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        attempts++;
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n¡WiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nError conectando WiFi");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10000));  // Verifica conexión cada 10 segundos
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializa WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Configura Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);  // Reintenta si WiFi se desconecta
  
  // Inicializa pines de hardware
  pinMode(encoder_clk, INPUT_PULLUP);
  pinMode(encoder_dt,  INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);
  
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, HIGH);  // Inicia con ventilador APAGADO
  
  // Configura PWM para SSR
  ledcSetup(SSR_PWM_CHANNEL, SSR_PWM_FREQ, SSR_PWM_RES);
  ledcAttachPin(ssr_pin, SSR_PWM_CHANNEL);
  ledcWrite(SSR_PWM_CHANNEL, 255);  // Inicia con SSR APAGADO
  
  // Inicializa encoder
  lastEncoderState = (digitalRead(encoder_clk) << 1) | digitalRead(encoder_dt);
  attachInterrupt(digitalPinToInterrupt(encoder_clk), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_dt),  readEncoder, CHANGE);
  
  // Inicializa OLED
  myScreen.begin();
  myScreen.setFont(u8g2_font_10x20_tf);
  myScreen.setContrast(255);
  
  // Inicializa sensor BME280
  if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
    Serial.println("¡BME280 no encontrado!");
    while (1);
  }
  
  lastTemp = bme.readTemperature();  // Lectura inicial de temperatura
  
  // Crea mutex para protección de datos
  dataMutex = xSemaphoreCreateMutex();
  
  // Crea tareas RTOS con prioridades y núcleos asignados
  xTaskCreatePinnedToCore(TaskWiFiManager, "WiFi", 4096, NULL, 1, NULL, 0);    // Prioridad baja
  xTaskCreatePinnedToCore(TaskEncoder, "Encoder", 2048, NULL, 3, NULL, 1);     // Prioridad alta
  xTaskCreatePinnedToCore(TaskSensorControl, "Control", 4096, NULL, 4, NULL, 1); // Prioridad más alta
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 4096, NULL, 2, NULL, 0);     // Prioridad media
  xTaskCreatePinnedToCore(TaskFirebaseUpdate, "Firebase", 4096, NULL, 1, NULL, 0); // Prioridad baja
  
  // Nota: Las tareas se asignan a diferentes núcleos:
  // Núcleo 0: WiFi, Display, Firebase (menos críticas en tiempo)
  // Núcleo 1: Encoder, Control (tareas críticas en tiempo)
}

void loop() {
  // Vacío - toda la funcionalidad está en tareas RTOS
  vTaskDelay(portMAX_DELAY);
}
