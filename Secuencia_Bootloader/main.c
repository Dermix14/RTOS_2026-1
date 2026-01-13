// Configuración de núcleos del procesador para FreeRTOS
#ifdef CONFIG_FREERTOS_UNICORE
  // Modo de un solo núcleo: utilizar núcleo 0
  static const BaseType_t app_cpu = 0;
#else
  // Modo dual-core: utilizar núcleo 1 para las tareas de la aplicación
  static const BaseType_t app_cpu = 1;
#endif

// Pines de LEDs para indicación visual
static const int LED_INDICADOR_1 = 4;  // LED con parpadeo fijo de 500ms
static const int LED_INDICADOR_2 = 2;  // LED con parpadeo fijo de 323ms
static const int LED_SECUENCIA = 16;   // LED que se activa al completar secuencia

// Pines de botones para entrada de usuario
static const int BOTON_1 = 5;    // Botón 1 para secuencia
static const int BOTON_2 = 17;   // Botón 2 para secuencia

// Variable de estado para la máquina de estados de secuencia
int estadoSecuencia = 0;

// Tarea: Control de LED indicador 1 (parpadeo periódico)
void tareaParpadeoLED1(void *parametro) {
  while (1) {
    digitalWrite(LED_INDICADOR_1, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Encendido por 500ms
    digitalWrite(LED_INDICADOR_1, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Apagado por 500ms
  }
}

// Tarea: Control de LED indicador 2 (parpadeo con tiempo diferente)
void tareaParpadeoLED2(void *parametro) {
  while (1) {
    digitalWrite(LED_INDICADOR_2, HIGH);
    vTaskDelay(323 / portTICK_PERIOD_MS);  // Encendido por 323ms
    digitalWrite(LED_INDICADOR_2, LOW);
    vTaskDelay(323 / portTICK_PERIOD_MS);  // Apagado por 323ms
  }
}

// Tarea: Control de secuencia de botones con máquina de estados
void tareaControlSecuencia(void *parametro) {
  while (1) {
    // Leer estado actual de los botones (LOW = presionado debido a INPUT_PULLUP)
    bool boton1Presionado = (digitalRead(BOTON_1) == LOW);
    bool boton2Presionado = (digitalRead(BOTON_2) == LOW);
    
    // Máquina de estados para validar secuencia de botones
    switch (estadoSecuencia) {
      case 0: // Estado inicial - esperando inicio de secuencia
        if (boton1Presionado && boton2Presionado) {
          estadoSecuencia = 1;  // Ambos botones presionados simultáneamente
        }
        break;
        
      case 1: // Estado 1 - secuencia iniciada, esperando liberación
        if ((!boton1Presionado && boton2Presionado) || 
            (!boton2Presionado && boton1Presionado)) {
          estadoSecuencia = 2;  // Solo un botón sigue presionado
        }
        break;
        
      case 2: // Estado 2 - esperando liberación total
        if (!boton1Presionado && !boton2Presionado) {
          estadoSecuencia = 3;  // Ambos botones liberados
        }
        break;
        
      case 3: // Estado 3 - esperando secuencia final
        if ((!boton1Presionado && boton2Presionado) || 
            (boton1Presionado && !boton2Presionado)) {
          estadoSecuencia = 4;  // Solo un botón presionado nuevamente
        }
        break;
        
      case 4: // Estado 4 - validación final de secuencia
        // Secuencia completada correctamente: ambos botones presionados
        if (boton1Presionado && boton2Presionado) {
          digitalWrite(LED_SECUENCIA, HIGH);     // Encender LED de confirmación
          vTaskDelay(2000 / portTICK_PERIOD_MS); // Mantener encendido por 2 segundos
          digitalWrite(LED_SECUENCIA, LOW);      // Apagar LED
          estadoSecuencia = 0;                    // Reiniciar secuencia
        }
        break;
    }
    
    // Pequeña pausa para anti-rebote y reducir uso de CPU
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Configurar pines de LEDs como salidas
  pinMode(LED_INDICADOR_1, OUTPUT);
  pinMode(LED_INDICADOR_2, OUTPUT);
  pinMode(LED_SECUENCIA, OUTPUT);
  
  // Configurar pines de botones como entradas con pull-up interno
  pinMode(BOTON_1, INPUT_PULLUP);
  pinMode(BOTON_2, INPUT_PULLUP);
  
  // Crear tarea para LED indicador 1
  xTaskCreatePinnedToCore(
    tareaParpadeoLED1,    // Función de la tarea
    "Parpadeo_LED1",      // Nombre descriptivo
    1024,                 // Tamaño de pila (bytes)
    NULL,                 // Parámetros (ninguno)
    1,                    // Prioridad
    NULL,                 // Manejador de tarea
    app_cpu               // Núcleo de ejecución
  );
  
  // Crear tarea para LED indicador 2
  xTaskCreatePinnedToCore(
    tareaParpadeoLED2,    // Función de la tarea
    "Parpadeo_LED2",      // Nombre descriptivo
    1024,                 // Tamaño de pila (bytes)
    NULL,                 // Parámetros (ninguno)
    1,                    // Prioridad
    NULL,                 // Manejador de tarea
    app_cpu               // Núcleo de ejecución
  );
  
  // Crear tarea para control de secuencia de botones
  xTaskCreatePinnedToCore(
    tareaControlSecuencia, // Función de la tarea
    "Control_Secuencia",   // Nombre descriptivo
    2048,                  // Tamaño de pila más grande por la lógica de estados
    NULL,                  // Parámetros (ninguno)
    2,                     // Prioridad más alta para mejor respuesta
    NULL,                  // Manejador de tarea
    app_cpu                // Núcleo de ejecución
  );
}

void loop() {
  // No se requiere código aquí, el sistema funciona con tareas FreeRTOS
}
