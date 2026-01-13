// Configuración de núcleos del procesador para FreeRTOS
#ifdef CONFIG_FREERTOS_UNICORE
  // Modo de un solo núcleo: utilizar núcleo 0
  static const BaseType_t app_cpu = 0;
#else
  // Modo dual-core: utilizar núcleo 1 para las tareas de la aplicación
  static const BaseType_t app_cpu = 1;
#endif

// Pines de salida para actuadores del sistema
static const int PIN_HARINA = 4;
static const int PIN_LECHE = 16;
static const int PIN_AGUA = 18;
static const int PIN_SERVIR = 21;
static const int PIN_BATIDORA = 2;
static const int PIN_DESCARGAR = 15;

// Variable para control de estado (actualmente no utilizada)
int estado = 0;

// Pines de entrada para sensores/botones
static const int SENSOR_HARINA = 5;    // Sensor para activar dosificación de harina
static const int SENSOR_LECHE = 17;    // Sensor para activar dosificación de leche
static const int SENSOR_NIVEL = 19;    // Sensor de nivel de líquido

// Tarea: Control de dosificación de harina
void tareaHarina(void *parameter) {
  while (1) {
    bool activarHarina = (digitalRead(SENSOR_HARINA) == LOW);
    
    if (activarHarina == HIGH) {
      digitalWrite(PIN_HARINA, HIGH);   // Activar dosificador de harina
      vTaskDelay(800 / portTICK_PERIOD_MS);
      digitalWrite(PIN_HARINA, LOW);    // Desactivar dosificador
    }
  }
}

// Tarea: Control de dosificación de leche y agua
void tareaLeche(void *parameter) {
  while (1) {
    bool activarLeche = (digitalRead(SENSOR_LECHE) == LOW);
    
    if (activarLeche == HIGH) {
      // Dosificar leche
      digitalWrite(PIN_LECHE, HIGH);
      vTaskDelay(800 / portTICK_PERIOD_MS);
      digitalWrite(PIN_LECHE, LOW);
      
      // Activar sistema de servicio
      digitalWrite(PIN_SERVIR, HIGH);
      vTaskDelay(800 / portTICK_PERIOD_MS);
      digitalWrite(PIN_SERVIR, LOW);
      
      // Iniciar dosificación de agua
      digitalWrite(PIN_AGUA, HIGH);
    }
  }
}

// Tarea: Control de mezcla y descarga
void tareaMezcla(void *parameter) {
  while (1) {
    bool nivelAlcanzado = (digitalRead(SENSOR_NIVEL) == LOW);
    
    if (nivelAlcanzado == HIGH) {
      // Detener dosificación de agua
      digitalWrite(PIN_AGUA, LOW);
      
      // Activar batidora
      digitalWrite(PIN_BATIDORA, HIGH);
      vTaskDelay(800 / portTICK_PERIOD_MS);
      digitalWrite(PIN_BATIDORA, LOW);
      
      // Descargar producto terminado
      digitalWrite(PIN_DESCARGAR, HIGH);
      vTaskDelay(800 / portTICK_PERIOD_MS);
      digitalWrite(PIN_DESCARGAR, LOW);
    }
  }
}

void setup() {
  // Configurar pines de salida
  pinMode(PIN_DESCARGAR, OUTPUT);
  pinMode(PIN_AGUA, OUTPUT);
  pinMode(PIN_LECHE, OUTPUT);
  pinMode(PIN_HARINA, OUTPUT);
  pinMode(PIN_SERVIR, OUTPUT);
  pinMode(PIN_BATIDORA, OUTPUT);
  
  // Configurar pines de entrada con resistencia pull-up interna
  pinMode(SENSOR_HARINA, INPUT_PULLUP);
  pinMode(SENSOR_LECHE, INPUT_PULLUP);
  pinMode(SENSOR_NIVEL, INPUT_PULLUP);

  // Crear tarea para control de harina
  xTaskCreatePinnedToCore(
    tareaHarina,          // Función de la tarea
    "Control_Harina",     // Nombre descriptivo
    2048,                 // Tamaño de pila (bytes)
    NULL,                 // Parámetros (ninguno)
    1,                    // Prioridad
    NULL,                 // Manejador de tarea
    app_cpu               // Núcleo de ejecución
  );
  
  // Crear tarea para control de leche
  xTaskCreatePinnedToCore(
    tareaLeche,           // Función de la tarea
    "Control_Leche",      // Nombre descriptivo
    4096,                 // Tamaño de pila (bytes)
    NULL,                 // Parámetros (ninguno)
    1,                    // Prioridad
    NULL,                 // Manejador de tarea
    app_cpu               // Núcleo de ejecución
  );
  
  // Crear tarea para control de mezcla
  xTaskCreatePinnedToCore(
    tareaMezcla,          // Función de la tarea
    "Control_Mezcla",     // Nombre descriptivo
    4096,                 // Tamaño de pila (bytes)
    NULL,                 // Parámetros (ninguno)
    1,                    // Prioridad
    NULL,                 // Manejador de tarea
    app_cpu               // Núcleo de ejecución
  );
}

void loop() {
  // No se requiere código aquí, el sistema funciona con tareas FreeRTOS
}
