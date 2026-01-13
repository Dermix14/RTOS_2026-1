#include "drv8871a.h"  // Librería para controlar motor driver

// Handles para FreeRTOS
TaskHandle_t controlTaskHandle = NULL;      // Para la tarea de control
TaskHandle_t serialTaskHandle = NULL;       // Para la tarea serial
QueueHandle_t commandQueue = NULL;          // Cola para comandos
SemaphoreHandle_t encoderMutex = NULL;      // Mutex para proteger encoder

// Constantes de configuración
const int dt_us = 2000;                     // Tiempo de muestreo: 2000 microsegundos
const float dt = dt_us * 0.000001f;         // Convertido a segundos
const int counts_per_motor_rev = 12;        // Encoder da 12 pulsos por rev del motor
const float gear_ratio = 30.0f;            // Relación de engranajes
const float R = 3.0f;                      // Grados por pulso del encoder

// Pines del hardware
const uint8_t A_encoder = 19;              // Canal A del encoder
const uint8_t B_encoder = 18;              // Canal B del encoder
const uint8_t motorPins[] = {22, 23};      // Pines para el motor

// Variables compartidas (protegidas con mutex)
volatile long Np = 0;                      // Contador del encoder
float th_des = 0.0f;                       // Posición deseada (ángulo)
float kp = 0.2f, kd = 0.01f, ki = 1.5f;   // Parámetros PID

// Estructura para estado del controlador
typedef struct {
    float th;      // Posición actual
    float thp;     // Posición anterior
    float dth_d;   // Velocidad derivada
    float dth_f;   // Velocidad filtrada
    float e;       // Error
    float de;      // Derivada del error
    float inte;    // Integral del error
    float u;       // Señal de control
    float usat;    // Señal saturada
    float PWM;     // Valor PWM final
    unsigned long k; // Contador de iteraciones
} control_state_t;

// Estructura para comandos seriales
typedef struct {
    char type;     // Tipo: 'c'=comando, 'p'=kp, 'i'=ki, 'd'=kd
    float value;   // Valor del comando
} command_t;

// Objeto para controlar el motor
drv8871 motor(motorPins);

// Interrupciones para el encoder
void IRAM_ATTR CH_A() {
    // Lectura segura en ISR con mutex
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(xSemaphoreTakeFromISR(encoderMutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        (digitalRead(A_encoder) == digitalRead(B_encoder)) ? Np++ : Np--;
        xSemaphoreGiveFromISR(encoderMutex, &xHigherPriorityTaskWoken);
    }
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR CH_B() {
    // Similar para canal B
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(xSemaphoreTakeFromISR(encoderMutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        (digitalRead(A_encoder) != digitalRead(B_encoder)) ? Np++ : Np--;
        xSemaphoreGiveFromISR(encoderMutex, &xHigherPriorityTaskWoken);
    }
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Tarea de control - corre cada dt_us microsegundos
void controlTask(void *parameter) {
    control_state_t state = {0};  // Inicializa estado
    const float alpha = 0.05f;    // Factor de filtro para velocidad
    
    while(1) {
        // Lectura protegida del encoder
        long currentNp;
        if(xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
            currentNp = Np;
            xSemaphoreGive(encoderMutex);
        }
        
        // Lectura protegida de parámetros PID
        float current_kp, current_kd, current_ki, current_th_des;
        if(xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
            current_kp = kp;
            current_kd = kd;
            current_ki = ki;
            current_th_des = th_des;
            xSemaphoreGive(encoderMutex);
        }
        
        // Algoritmo de control PID
        state.th = R * currentNp;  // Convierte pulsos a grados
        state.dth_d = (state.th - state.thp) / dt;  // Velocidad derivada
        state.dth_f = alpha * state.dth_d + (1 - alpha) * state.dth_f;  // Filtrada
        
        state.e = current_th_des - state.th;  // Error
        state.de = -state.dth_f;              // Derivada del error
        state.inte = state.inte + state.e * dt;  // Integral
        
        // Cálculo PID
        state.u = current_kp * state.e + current_kd * state.de + current_ki * state.inte;
        state.usat = constrain(state.u, -12, 12);  // Saturación a ±12V
        state.PWM = state.usat * 21.25f;          // Convertir a PWM
        
        // Aplicar al motor
        motor.writePWM((int16_t)state.PWM);
        
        state.k++;
        state.thp = state.th;  // Guarda posición para siguiente ciclo
        
        vTaskDelay(pdMS_TO_TICKS(dt_us / 1000));  // Espera hasta siguiente ciclo
    }
}

// Tarea serial - maneja comandos y comunicación
void serialTask(void *parameter) {
    command_t cmd;
    
    while(1) {
        // Leer comandos del puerto serial
        if(Serial.available() > 0) {
            String consigna = Serial.readStringUntil('\n');
            
            // Parsear diferentes tipos de comandos
            if(consigna.startsWith("c")) {
                cmd.type = 'c';
                cmd.value = consigna.substring(1).toFloat();
                xQueueSend(commandQueue, &cmd, 0);  // Enviar a cola
            }
            else if(consigna.startsWith("kp")) {
                cmd.type = 'p';
                cmd.value = consigna.substring(2).toFloat();
                xQueueSend(commandQueue, &cmd, 0);
            }
            else if(consigna.startsWith("ki")) {
                cmd.type = 'i';
                cmd.value = consigna.substring(2).toFloat();
                xQueueSend(commandQueue, &cmd, 0);
            }
            else if(consigna.startsWith("kd")) {
                cmd.type = 'd';
                cmd.value = consigna.substring(2).toFloat();
                xQueueSend(commandQueue, &cmd, 0);
            }
        }
        
        // Procesar comandos de la cola
        if(xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
            if(xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
                // Actualizar parámetros según tipo de comando
                switch(cmd.type) {
                    case 'c': th_des = cmd.value; break;  // Cambiar posición deseada
                    case 'p': kp = cmd.value; break;      // Cambiar Kp
                    case 'i': ki = cmd.value; break;      // Cambiar Ki
                    case 'd': kd = cmd.value; break;      // Cambiar Kd
                }
                xSemaphoreGive(encoderMutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Pequeña pausa
    }
}

void setup() {
    Serial.begin(115200);  // Iniciar comunicación serial
    
    // Crear objetos FreeRTOS
    encoderMutex = xSemaphoreCreateMutex();  // Mutex para encoder
    commandQueue = xQueueCreate(10, sizeof(command_t));  // Cola para 10 comandos
    
    // Configurar hardware
    attachInterrupt(digitalPinToInterrupt(A_encoder), CH_A, CHANGE);  // Interrupción encoder A
    attachInterrupt(digitalPinToInterrupt(B_encoder), CH_B, CHANGE);  // Interrupción encoder B
    motor.init();  // Inicializar motor
    
    // Crear tareas
    xTaskCreatePinnedToCore(
        controlTask,    // Función de control PID
        "Control",      // Nombre
        4096,           // Memoria
        NULL,           
        3,              // Alta prioridad (crítico en tiempo)
        &controlTaskHandle,
        1               // Núcleo 1 (para tareas críticas)
    );
    
    xTaskCreatePinnedToCore(
        serialTask,     // Función para comunicación
        "Serial",       // Nombre
        4096,           // Memoria
        NULL,
        1,              // Prioridad media
        &serialTaskHandle,
        0               // Núcleo 0
    );
    
    // Eliminar tarea loop por defecto (usamos FreeRTOS)
    vTaskDelete(NULL);
}

void loop() {
    // Vacío - todo se maneja con FreeRTOS
    vTaskDelete(NULL);
}
