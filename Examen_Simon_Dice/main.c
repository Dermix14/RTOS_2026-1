/*
 * SIMON DICE - Versión para ESP32 con FreeRTOS
 * Hardware: ESP32 + Pantalla OLED 128x64 + Teclado matricial 4x4
 * Autor: [Tu Nombre]
 * Descripción: Juego clásico Simon Dice con interfaz gráfica y múltiples tareas
 */

// ============================================
// SECCIÓN 1: INCLUSIÓN DE LIBRERÍAS
// ============================================
#include <U8g2lib.h>    // Librería para control de pantallas OLED con múltiples controladores
#include <Wire.h>       // Librería para comunicación I2C (usada por la pantalla OLED)
#include <Keypad.h>     // Librería para manejo de teclados matriciales

// ============================================
// SECCIÓN 2: CONFIGURACIÓN FREERTOS (ESP32)
// ============================================
/*
 * Configuración para sistemas multi-núcleo del ESP32.
 * Si está configurado como unicore, usa el núcleo 0.
 * Si es dual-core, usa el núcleo 1 para las tareas de la aplicación.
 */
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;  // Usar núcleo 0 en sistemas unicore
#else
  static const BaseType_t app_cpu = 1;  // Usar núcleo 1 en sistemas dual-core
#endif

// ============================================
// SECCIÓN 3: CONFIGURACIÓN PANTALLA OLED
// ============================================
/*
 * Configuración del objeto para controlar la pantalla OLED SSD1306 de 128x64 píxeles
 * - U8G2_R0: Rotación 0 grados (orientación normal)
 * - HW_I2C: Usa comunicación I2C por hardware
 */
U8G2_SSD1306_128X64_NONAME_1_HW_I2C myScreen(U8G2_R0);

// ============================================
// SECCIÓN 4: CONFIGURACIÓN TECLADO MATRICIAL
// ============================================
/*
 * Teclado 4x4 con la siguiente disposición:
 * [1] [2] [3] [A]
 * [4] [5] [6] [B]
 * [7] [8] [9] [C]
 * [*] [0] [#] [D]
 */
const byte ROWS = 4;  // Número de filas del teclado
const byte COLS = 4;  // Número de columnas del teclado

// Mapa de caracteres del teclado
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},  // Fila 1
  {'4', '5', '6', 'B'},  // Fila 2
  {'7', '8', '9', 'C'},  // Fila 3
  {'*', '0', '#', 'D'}   // Fila 4
};

// Pines del ESP32 conectados a las filas del teclado
byte rowPins[ROWS] = {32, 33, 25, 26};

// Pines del ESP32 conectados a las columnas del teclado
byte colPins[COLS] = {19, 18, 5, 17};

// Creación del objeto Keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ============================================
// SECCIÓN 5: VARIABLES GLOBALES FREERTOS
// ============================================
QueueHandle_t keyQueue;        // Cola para enviar teclas entre tareas (tamaño: 5 elementos)
SemaphoreHandle_t screenMutex; // Mutex para acceso exclusivo a la pantalla

// ============================================
// SECCIÓN 6: FUNCIÓN PARA DIBUJAR TEXTO CENTRADO
// ============================================
/*
 * Dibuja texto centrado horizontalmente en la pantalla OLED
 * @param text: Texto a mostrar
 * @param newFont: Fuente a utilizar
 * @param y: Posición vertical (por defecto 64 = parte inferior)
 */
void drawCentered(const char *text, const uint8_t *newFont, uint8_t y = 64) {
  // Intenta tomar el mutex con timeout de 50ms para acceso seguro a pantalla
  if (xSemaphoreTake(screenMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    myScreen.setFont(newFont);       // Establece la fuente especificada
    myScreen.firstPage();            // Inicia el buffer de la pantalla
    do {
      // Calcula el ancho del texto para centrarlo
      uint8_t w = myScreen.getStrWidth(text);
      uint8_t x = (128 - w) / 2;     // Centra horizontalmente (128px de ancho)
      myScreen.drawStr(x, y, text);  // Dibuja el texto en posición calculada
    } while (myScreen.nextPage());   // Renderiza la página completa
    xSemaphoreGive(screenMutex);     // Libera el mutex
  }
}

// ============================================
// SECCIÓN 7: TAREA 1 - MANEJO DEL TECLADO
// ============================================
/*
 * Tarea que se ejecuta continuamente para leer el teclado
 * y enviar las teclas pulsadas a la cola compartida
 */
void manageKeypad(void *parameter) {
  while(1) {
    char key = keypad.getKey();  // Lee tecla pulsada (NO_KEY si no hay)
    
    if (key != NO_KEY) {
      Serial.print("Key: ");     // Depuración por monitor serial
      Serial.println(key);
      xQueueSend(keyQueue, &key, 0);  // Envía tecla a la cola (sin espera)
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));  // Espera 20ms para no saturar CPU
  }
}

// ============================================
// SECCIÓN 8: TAREA 2 - LÓGICA PRINCIPAL DEL JUEGO
// ============================================
/*
 * Tarea que implementa el juego Simon Dice completo
 * - Genera secuencias aleatorias
 * - Muestra secuencias al jugador
 * - Verifica respuestas del jugador
 * - Controla niveles y estado del juego
 */
void SimonGame(void *parameter) {
  const uint8_t MAX_SEQ = 20;    // Longitud máxima de secuencia (20 niveles)
  char sequence[MAX_SEQ];        // Array para almacenar la secuencia
  uint8_t level = 1;             // Nivel actual (empieza en 1)

  randomSeed(analogRead(A0));    // Inicializa generador aleatorio

  while(1) {
    // --------------------------------------------------
    // GENERACIÓN DE SECUENCIA ALEATORIA
    // --------------------------------------------------
    // Genera una secuencia completa de 20 teclas aleatorias
    for (uint8_t i = 0; i < MAX_SEQ; i++) {
      // Selecciona una tecla aleatoria del teclado
      sequence[i] = keys[random(0, ROWS)][random(0, COLS)];
    }

    // Mensaje inicial "Listo?"
    drawCentered("Listo?", u8g2_font_logisoso24_tr, 42);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1 segundo

    bool lost = false;  // Flag para controlar si el jugador perdió

    // --------------------------------------------------
    // BUCLE PRINCIPAL DEL NIVEL
    // --------------------------------------------------
    while (!lost) {
      // --------------------------------------------------
      // FASE 1: MOSTRAR SECUENCIA AL JUGADOR
      // --------------------------------------------------
      for (uint8_t i = 0; i < level; i++) {
        // Convierte char a string para dibujar
        char str[2] = {sequence[i], '\0'};
        
        // Muestra la tecla de la secuencia
        drawCentered(str, u8g2_font_logisoso58_tr);
        vTaskDelay(pdMS_TO_TICKS(600));  // Muestra por 600ms
        
        // Borra la tecla (espacio en blanco)
        drawCentered(" ", u8g2_font_logisoso58_tr);
        vTaskDelay(pdMS_TO_TICKS(200));  // Pausa entre teclas 200ms
      }

      // --------------------------------------------------
      // FASE 2: ESPERAR RESPUESTA DEL JUGADOR
      // --------------------------------------------------
      for (uint8_t i = 0; i < level; i++) {
        char received;  // Variable para almacenar tecla recibida
        
        // Espera tecla del jugador (timeout de 5 segundos)
        if (xQueueReceive(keyQueue, &received, pdMS_TO_TICKS(5000)) == pdTRUE) {
          // Muestra la tecla presionada
          char str[2] = {received, '\0'};
          drawCentered(str, u8g2_font_logisoso58_tr);
          vTaskDelay(pdMS_TO_TICKS(300));  // Muestra por 300ms
          drawCentered(" ", u8g2_font_logisoso58_tr);  // Borra

          // Verifica si la tecla es correcta
          if (received != sequence[i]) {
            lost = true;  // Tecla incorrecta - juego perdido
            break;
          }
        } else {
          lost = true;  // Timeout - no presionó tecla a tiempo
          break;
        }
      }

      // --------------------------------------------------
      // FASE 3: EVALUAR RESULTADO DEL NIVEL
      // --------------------------------------------------
      if (!lost) {
        // NIVEL COMPLETADO CORRECTAMENTE
        drawCentered("¡Bien!", u8g2_font_logisoso24_tr, 46);
        level++;  // Incrementa dificultad
        
        // Verifica si ganó el juego (completó todos los niveles)
        if (level > MAX_SEQ) {
          drawCentered("¡Ganaste!", u8g2_font_logisoso24_tr, 46);
          vTaskDelay(pdMS_TO_TICKS(2000));  // Muestra mensaje 2 segundos
          break;  // Sale del bucle para reiniciar juego
        }
        
        vTaskDelay(pdMS_TO_TICKS(800));  // Pausa antes del siguiente nivel
      } else {
        // JUGADOR PERDIÓ
        drawCentered("¡Perdiste!", u8g2_font_logisoso24_tr, 46);
        vTaskDelay(pdMS_TO_TICKS(1500));  // Muestra mensaje 1.5 segundos
        level = 1;  // Reinicia nivel a 1
      }
    }

    // --------------------------------------------------
    // REINICIO DEL JUEGO
    // --------------------------------------------------
    drawCentered("Reiniciando...", u8g2_font_logisoso24_tr, 46);
    vTaskDelay(pdMS_TO_TICKS(1500));  // Espera 1.5 segundos antes de reiniciar
  }
}

// ============================================
// SECCIÓN 9: TAREA 3 - MANEJO DE PANTALLA
// ============================================
/*
 * Tarea para manejo de la interfaz gráfica
 * Actualmente solo muestra pantalla inicial
 * Podría extenderse para animaciones o menús
 */
void manageScreen(void *parameter) {
  // Muestra título inicial
  drawCentered("Simon Dice", u8g2_font_logisoso24_tr, 40);
  vTaskDelay(pdMS_TO_TICKS(1000));  // Muestra por 1 segundo
  
  // Loop principal (actualmente sin funcionalidad adicional)
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(1000));  // Evita que la tarea termine
  }
}

// ============================================
// SECCIÓN 10: SETUP (INICIALIZACIÓN)
// ============================================
void setup() {
  // Inicialización del puerto serial para depuración
  Serial.begin(115200);
  
  // Inicialización de la pantalla OLED
  myScreen.begin();
  myScreen.setFont(u8g2_font_logisoso24_tr);  // Fuente por defecto

  // Creación de objetos FreeRTOS
  keyQueue = xQueueCreate(5, sizeof(char));      // Cola para 5 teclas
  screenMutex = xSemaphoreCreateMutex();         // Mutex para pantalla

  // --------------------------------------------------
  // CREACIÓN DE TAREAS FREERTOS
  // --------------------------------------------------
  // Tarea 1: Manejo de teclado (prioridad 2 - alta)
  xTaskCreatePinnedToCore(
    manageKeypad,       // Función de la tarea
    "Manage Keypad",    // Nombre para depuración
    2048,               // Tamaño de pila (bytes)
    NULL,               // Parámetros
    2,                  // Prioridad (0-24, mayor = más prioridad)
    NULL,               // Manejador de tarea
    app_cpu             // Núcleo donde ejecutar
  );

  // Tarea 2: Lógica del juego (prioridad 1 - media)
  xTaskCreatePinnedToCore(
    SimonGame,          // Función de la tarea
    "Simon Game",       // Nombre para depuración
    4096,               // Tamaño de pila mayor (más compleja)
    NULL,               // Parámetros
    1,                  // Prioridad
    NULL,               // Manejador de tarea
    app_cpu             // Núcleo donde ejecutar
  );

  // Tarea 3: Manejo de pantalla (prioridad 1 - media)
  xTaskCreatePinnedToCore(
    manageScreen,       // Función de la tarea
    "Manage Screen",    // Nombre para depuración
    2048,               // Tamaño de pila
    NULL,               // Parámetros
    1,                  // Prioridad
    NULL,               // Manejador de tarea
    app_cpu             // Núcleo donde ejecutar
  );
}

// ============================================
// SECCIÓN 11: LOOP PRINCIPAL
// ============================================
/*
 * Loop vacío porque toda la lógica se ejecuta en tareas FreeRTOS
 * Se mantiene por compatibilidad con Arduino
 */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // Pequeño delay para no saturar
}
