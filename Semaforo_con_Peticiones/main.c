// Configuración de núcleos del procesador para FreeRTOS
#if CONFIG_FREERTOS_UNICORE
  // Modo de un solo núcleo: utilizar núcleo 0
  static const BaseType_t app_cpu = 0;
#else
  // Modo dual-core: utilizar núcleo 1 para las tareas de la aplicación
  static const BaseType_t app_cpu = 1;
#endif

// Pines de LEDs para Semáforo 1 (intersección norte/sur)
const int SEMAFORO_1_ROJO = 19;
const int SEMAFORO_1_VERDE = 5;

// Pines de LEDs para Semáforo 2 (intersección este/oeste)
const int SEMAFORO_2_ROJO = 15;
const int SEMAFORO_2_VERDE = 4;

// Pines de sensores/entradas para detección de vehículos
const int SENSOR_ESPERA_SEMAFORO_1 = 0;   // Vehículo esperando en semáforo 1
const int SENSOR_ESPERA_SEMAFORO_2 = 2;   // Vehículo esperando en semáforo 2
const int SENSOR_SALIDA_SEMAFORO_1 = 18;  // Vehículo ha salido del semáforo 1
const int SENSOR_SALIDA_SEMAFORO_2 = 21;  // Vehículo ha salido del semáforo 2

// Mutex para control de acceso a la sección crítica (carretera compartida)
static SemaphoreHandle_t mutexCarretera;

// Tarea: Control del semáforo 1 (intersección norte/sur)
void tareaControlSemaforo1(void *parametros) {
  while (1) {
    // Detectar si hay un vehículo esperando en el semáforo 1
    if (digitalRead(SENSOR_ESPERA_SEMAFORO_1) == LOW) {
      // Intentar adquirir acceso exclusivo a la carretera
      if (xSemaphoreTake(mutexCarretera, portMAX_DELAY) == pdTRUE) {
        // Conceder paso: cambiar semáforo 1 a verde
        digitalWrite(SEMAFORO_1_ROJO, LOW);
        digitalWrite(SEMAFORO_1_VERDE, HIGH);
        
        // Esperar a que el vehículo complete el cruce
        // (detectar que el sensor de salida se active)
        while (digitalRead(SENSOR_SALIDA_SEMAFORO_1) == HIGH) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        // Restaurar semáforo a rojo
        digitalWrite(SEMAFORO_1_VERDE, LOW);
        digitalWrite(SEMAFORO_1_ROJO, HIGH);
        
        // Liberar acceso a la carretera
        xSemaphoreGive(mutexCarretera);
      }
    }
    
    // Pequeña pausa para evitar sobrecarga de la CPU
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Tarea: Control del semáforo 2 (intersección este/oeste)
void tareaControlSemaforo2(void *parametros) {
  while (1) {
    // Detectar si hay un vehículo esperando en el semáforo 2
    if (digitalRead(SENSOR_ESPERA_SEMAFORO_2) == LOW) {
      // Intentar adquirir acceso exclusivo a la carretera
      if (xSemaphoreTake(mutexCarretera, portMAX_DELAY) == pdTRUE) {
        // Conceder paso: cambiar semáforo 2 a verde
        digitalWrite(SEMAFORO_2_ROJO, LOW);
        digitalWrite(SEMAFORO_2_VERDE, HIGH);
        
        // Esperar a que el vehículo complete el cruce
        // (detectar que el sensor de salida se active)
        while (digitalRead(SENSOR_SALIDA_SEMAFORO_2) == HIGH) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        // Restaurar semáforo a rojo
        digitalWrite(SEMAFORO_2_VERDE, LOW);
        digitalWrite(SEMAFORO_2_ROJO, HIGH);
        
        // Liberar acceso a la carretera
        xSemaphoreGive(mutexCarretera);
      }
    }
    
    // Pequeña pausa para evitar sobrecarga de la CPU
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
};

void setup() {
  // Inicializar comunicación serial para monitoreo
  Serial.begin(115200);
  
  // Configurar pines de LEDs como salidas
  pinMode(SEMAFORO_1_ROJO, OUTPUT);
  pinMode(SEMAFORO_1_VERDE, OUTPUT);
  pinMode(SEMAFORO_2_ROJO, OUTPUT);
  pinMode(SEMAFORO_2_VERDE, OUTPUT);
  
  // Configurar pines de sensores como entradas con pull-up interno
  pinMode(SENSOR_ESPERA_SEMAFORO_1, INPUT_PULLUP);
  pinMode(SENSOR_ESPERA_SEMAFORO_2, INPUT_PULLUP);
  pinMode(SENSOR_SALIDA_SEMAFORO_1, INPUT_PULLUP);
  pinMode(SENSOR_SALIDA_SEMAFORO_2, INPUT_PULLUP);
  
  // Establecer estado inicial de los semáforos (todos en rojo)
  digitalWrite(SEMAFORO_1_ROJO, HIGH);
  digitalWrite(SEMAFORO_1_VERDE, LOW);
  digitalWrite(SEMAFORO_2_ROJO, HIGH);
  digitalWrite(SEMAFORO_2_VERDE, LOW);
  
  // Crear mutex para control de acceso a la carretera compartida
  mutexCarretera = xSemaphoreCreateMutex();
  
  // Crear tareas para control de semáforos
  xTaskCreatePinnedToCore(tareaControlSemaforo1, "Control_Semaforo1", 
                          2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(tareaControlSemaforo2, "Control_Semaforo2", 
                          2048, NULL, 1, NULL, app_cpu);
}

void loop() {
  // No se utiliza en FreeRTOS - el control se realiza mediante tareas
}
