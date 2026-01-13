// Configuración de núcleos del procesador para FreeRTOS
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;  // Modo un núcleo: usar núcleo 0
#else
  static const BaseType_t app_cpu = 1;  // Modo dual-core: usar núcleo 1 para aplicación
#endif

// Configuración de colas (buffers) para comunicación entre tareas
static const uint8_t LONGITUD_MAXIMA_COLA = 5;  // Capacidad máxima de cada cola

// Colas para comunicación entre tareas (productor-consumidor)
static QueueHandle_t colaProductorConsumidor;   // Cola entre Productor y Tarea A
static QueueHandle_t colaConsumidorFinal;       // Cola entre Tarea A y Tarea B

// Tarea: Productor de números (genera secuencia numérica)
void tareaProductor(void *parametros) {
  int numeroActual = 0;
  
  while (1) {
    // Intentar enviar número a la cola (sin bloqueo)
    if (xQueueSend(colaProductorConsumidor, &numeroActual, 0) != pdTRUE) {
      Serial.println("ADVERTENCIA: Cola productor-consumidor llena");
    } else {
      Serial.print("Productor -> Enviado: ");
      Serial.println(numeroActual);
    }
    
    numeroActual++;  // Incrementar contador para siguiente envío
    
    // Esperar 500ms antes de generar siguiente número
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Tarea: Consumidor intermedio (procesa y reenvía datos)
void tareaConsumidorIntermedio(void *parametros) {
  int datoRecibido;
  
  while (1) {
    // Recibir dato de la cola productor-consumidor (espera indefinida)
    if (xQueueReceive(colaProductorConsumidor, &datoRecibido, portMAX_DELAY) == pdTRUE) {
      Serial.print("Tarea A -> Recibido: ");
      Serial.println(datoRecibido);
      
      // Reenviar dato procesado a la siguiente cola
      xQueueSend(colaConsumidorFinal, &datoRecibido, portMAX_DELAY);
    }
  }
}

// Tarea: Consumidor final (procesa datos finales)
void tareaConsumidorFinal(void *parametros) {
  int datoProcesado;
  
  while (1) {
    // Recibir dato de la cola consumidor-final (espera indefinida)
    if (xQueueReceive(colaConsumidorFinal, &datoProcesado, portMAX_DELAY) == pdTRUE) {
      Serial.print("Tarea B -> Recibido: ");
      Serial.println(datoProcesado);
    }
  }
}

void setup() {
  // Inicializar comunicación serial para monitoreo
  Serial.begin(115200);
  
  // Pausa inicial para estabilización
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  // Crear colas (buffers) para comunicación entre tareas
  colaProductorConsumidor = xQueueCreate(LONGITUD_MAXIMA_COLA, sizeof(int));
  colaConsumidorFinal = xQueueCreate(LONGITUD_MAXIMA_COLA, sizeof(int));
  
  // Crear tarea Productor (genera números)
  xTaskCreatePinnedToCore(
    tareaProductor,               // Función de la tarea
    "Productor_Numeros",          // Nombre descriptivo
    1024,                         // Tamaño de pila (bytes)
    NULL,                         // Parámetros (ninguno)
    1,                            // Prioridad
    NULL,                         // Manejador de tarea
    app_cpu                       // Núcleo de ejecución
  );
  
  // Crear tarea Consumidor Intermedio (Tarea A)
  xTaskCreatePinnedToCore(
    tareaConsumidorIntermedio,    // Función de la tarea
    "Consumidor_Intermedio",      // Nombre descriptivo
    1024,                         // Tamaño de pila (bytes)
    NULL,                         // Parámetros (ninguno)
    1,                            // Prioridad
    NULL,                         // Manejador de tarea
    app_cpu                       // Núcleo de ejecución
  );
  
  // Crear tarea Consumidor Final (Tarea B)
  xTaskCreatePinnedToCore(
    tareaConsumidorFinal,         // Función de la tarea
    "Consumidor_Final",           // Nombre descriptivo
    1024,                         // Tamaño de pila (bytes)
    NULL,                         // Parámetros (ninguno)
    1,                            // Prioridad
    NULL,                         // Manejador de tarea
    app_cpu                       // Núcleo de ejecución
  );
}

void loop() {
  // No se requiere código aquí, el sistema funciona con tareas FreeRTOS
  // y comunicación mediante colas entre tareas
}
