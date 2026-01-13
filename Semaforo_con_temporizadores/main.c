#include <Arduino.h>

// Configuración para sistemas de un solo núcleo vs doble núcleo
#if CONFIG_FRERTOS_UNICORE  // Nota: hay un typo, debería ser CONFIG_FREERTOS_UNICORE
	static const BaseType_t app_cpu = 0;  // Usar núcleo 0 si es unicore
#else
	static const BaseType_t app_cpu = 1;  // Usar núcleo 1 si es dual-core
#endif

// Declaración de timers de FreeRTOS (auto-reload)
static TimerHandle_t auto_reload_timer1 = NULL;  // Timer 1
static TimerHandle_t auto_reload_timer2 = NULL;  // Timer 2
static TimerHandle_t auto_reload_timer3 = NULL;  // Timer 3
static TimerHandle_t auto_reload_timer4 = NULL;  // Timer 4

// Definición de pines para LEDs
const uint8_t l1 = 23;  // LED 1
const uint8_t l2 = 22;  // LED 2
const uint8_t l3 = 21;  // LED 3

// Variables de estado de los LEDs
bool state1 = true;   // LED 1 inicialmente ENCENDIDO
bool state2 = false;  // LED 2 inicialmente APAGADO
bool state3 = false;  // LED 3 inicialmente APAGADO

uint8_t count = 0;  // Contador para el parpadeo del LED 2

// Función callback que se ejecuta cuando expira un timer
void myTimerCallback1(TimerHandle_t xTimer){
	// Obtiene el ID del timer que expiró
	uint32_t timerID = (uint32_t) pvTimerGetTimerID(xTimer);

  switch(timerID){
    case 1:  // Timer 1 expiró (después de 2 segundos)
      state1 = false;  // Apaga LED 1
      state2 = true;   // Enciende LED 2
      state3 = false;  // Apaga LED 3
      xTimerStop(auto_reload_timer1, portMAX_DELAY);  // Detiene timer 1
      xTimerStart(auto_reload_timer2, portMAX_DELAY); // Inicia timer 2
    break;

    case 2:  // Timer 2 expiró (cada 400ms para parpadeo)
      if(count <= 3){  // Parpadea 4 veces (0,1,2,3)
        state2 = !state2;  // Alterna estado del LED 2
        count++;
      }
      else{  // Después de 4 parpadeos
        count = 0;  // Reinicia contador
        state1 = false;  // LED 1 apagado
        state2 = false;  // LED 2 apagado
        state3 = true;   // LED 3 encendido
        xTimerStop(auto_reload_timer2, portMAX_DELAY);  // Detiene timer 2
        xTimerStart(auto_reload_timer3, portMAX_DELAY); // Inicia timer 3
      }
    break;

    case 3:  // Timer 3 expiró (después de 2 segundos)
      state1 = true;   // Enciende LED 1
      state2 = false;  // Apaga LED 2
      state3 = false;  // Apaga LED 3
      xTimerStop(auto_reload_timer3, portMAX_DELAY);  // Detiene timer 3
      // No inicia otro timer aquí - vuelve al estado inicial
    break;

    case 4:  // Timer 4 expiró (después de 6 segundos)
      xTimerStart(auto_reload_timer1, portMAX_DELAY);  // Inicia secuencia nuevamente
    break;
  }
}

void setup(){
	Serial.begin(115200);  // Inicia comunicación serial
  
  // Configura pines como salida
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(l3, OUTPUT);

	/* Crear Timers de FreeRTOS */
	
	// Timer 1: Dura 2 segundos
	auto_reload_timer1 = xTimerCreate(
	  "Timer Auto-reload",		// Nombre del timer
		2000 / portTICK_PERIOD_MS,	// Periodo: 2000ms = 2 segundos
		pdTRUE,				// Auto-reload = se reinicia automáticamente
		(void *)1,			// ID del timer = 1
	  myTimerCallback1);		// Función a ejecutar
  // Nota: NO se inicia aquí inmediatamente

  // Timer 2: Dura 400ms (para parpadeo)
  auto_reload_timer2 = xTimerCreate(
	  "Timer Auto-reload",		// Nombre del timer
		400 / portTICK_PERIOD_MS,	// Periodo: 400ms = 0.4 segundos
		pdTRUE,				// Auto-reload
		(void *)2,			// ID del timer = 2
	  myTimerCallback1);		// Función a ejecutar

  // Timer 3: Dura 2 segundos
  auto_reload_timer3 = xTimerCreate(
	  "Timer Auto-reload",		// Nombre del timer
		2000 / portTICK_PERIOD_MS,	// Periodo: 2000ms = 2 segundos
		pdTRUE,				// Auto-reload
		(void *)3,			// ID del timer = 3
	  myTimerCallback1);		// Función a ejecutar

  // Timer 4: Dura 6 segundos (timer maestro)
  auto_reload_timer4 = xTimerCreate(
	  "Timer Auto-reload",		// Nombre del timer
		6000 / portTICK_PERIOD_MS,	// Periodo: 6000ms = 6 segundos
		pdTRUE,				// Auto-reload
		(void *)4,			// ID del timer = 4
	  myTimerCallback1);		// Función a ejecutar
  xTimerStart(auto_reload_timer4, portMAX_DELAY);  // Inicia timer 4 inmediatamente
}

void loop(){
  // Actualiza el estado de los LEDs continuamente
  digitalWrite(l1, state1);
  digitalWrite(l2, state2);
  digitalWrite(l3, state3);
  // Nota: No hay delays en loop, todo el timing se maneja con timers
}
