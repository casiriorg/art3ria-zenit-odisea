/**
 * @file zenit-odisea-device.ino
 * @brief Adquiere datos de un IMU BNO085 (I2C), un sensor GSR (analógico)
 *     y un sensor PPG MAX30102 (I2C, pulso y SpO2), los transmite por
 *     serial en formato "TAG:valor,...", y controla un LED RGB.
 *
 * Proyecto: zenit-odisea-device
 *
 * Formato de salida:
 *     IMU:qw,qx,qy,qz,roll,pitch,yaw
 *     GSR:raw,filtrado,variacion
 *     PPG:ir,red,bpm,bpmValido,spo2,spo2Valido
 *
 * Cada sensor se muestrea de forma independiente y no bloqueante (sin
 * `delay()` dentro de `loop()`), para que la lectura de uno no retrase
 * ni altere la del otro y así preservar la integridad temporal del dato.
 *
 * El LED RGB corre en paralelo a los sensores, sin afectar su
 * muestreo. Hoy ejecuta una demostración fija (`TEST`, sección "LED
 * RGB"): R, luego G, luego B, luego todos encendidos, luego ninguno,
 * 1 segundo cada paso, de forma no bloqueante y aunque ningún sensor
 * haya sido detectado. Esta demostración es temporal y se eliminará
 * cuando el LED pase a reaccionar a los datos de los sensores y, más
 * adelante, al resultado de un modelo de ML alimentado por ellos; por
 * eso la escritura de los pines vive en una función reusable
 * (`setLedColor`) separada de la rutina `TEST` que la usa hoy.
 *
 * Con `DEBUG_LEDS` en `true` (sección "LED RGB"), cada cambio de
 * color se imprime por serial ("# LED: R=.. G=.. B=.."); por defecto
 * está en `false` para no mezclar esas líneas con el log de los
 * sensores (`IMU:`, `GSR:`, `PPG:`).
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <MAX30105.h>
#include "spo2_algorithm.h"

// ---------------------------------------------------------------------
// IMU (BNO085)
// ---------------------------------------------------------------------

/** Dirección I2C del BNO085. */
const uint8_t IMU_I2C_ADDR = 0x4B;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t imuEvent;

/**
 * Indica si el IMU fue detectado e inicializado correctamente. Si es
 * `false`, `readIMU()` no hace nada, pero el resto del programa (GSR,
 * PPG, LEDs) sigue funcionando con normalidad.
 */
bool imuAvailable = false;

// ---------------------------------------------------------------------
// GSR
// ---------------------------------------------------------------------

/** Pin analógico donde está conectada la señal GSR (SIG). */
const uint8_t GSR_PIN = A1;

/** Intervalo de muestreo del GSR, en milisegundos (100 Hz). */
const unsigned long GSR_SAMPLE_INTERVAL_MS = 10;

/** Factor de suavizado del filtro paso-bajo de la señal (0-1). */
const float GSR_FILTER_ALPHA = 0.05;

/** Factor de suavizado, más lento, para estimar la línea base (0-1). */
const float GSR_BASELINE_ALPHA = 0.01;

unsigned long lastGsrSampleTime = 0;
float filteredGSR = 0;
float baselineGSR = 0;

// ---------------------------------------------------------------------
// PPG (MAX30102) — pulso (IR) y SpO2 (IR + Red)
// ---------------------------------------------------------------------

MAX30105 particleSensor;

/**
 * Tamaño de la ventana de muestras IR/Red usada por el algoritmo de
 * SpO2 (`maxim_heart_rate_and_oxygen_saturation`). El algoritmo de
 * Maxim exige una ventana fija para estimar la razón AC/DC entre
 * ambos canales; 100 muestras es el tamaño de referencia de la
 * librería SparkFun. Esto solo afecta al cálculo de SpO2, no al de
 * pulso (que es muestra a muestra, ver más abajo).
 */
const uint8_t PPG_SPO2_WINDOW = 100;

uint32_t ppgIrBuffer[PPG_SPO2_WINDOW];
uint32_t ppgRedBuffer[PPG_SPO2_WINDOW];

/** Índice de escritura actual dentro de la ventana circular de SpO2. */
uint8_t ppgWindowIndex = 0;

/** Cuántas muestras nuevas se han acumulado desde el último cálculo. */
uint8_t ppgSamplesSinceCalc = 0;

/**
 * Cada cuántas muestras nuevas se recalcula SpO2, una vez la ventana
 * está llena por primera vez. Recalcular con más frecuencia que esto
 * no aporta precisión (la ventana cambia poco) y sí consume CPU.
 */
const uint8_t PPG_SPO2_RECALC_EVERY = 25;

bool ppgWindowFull = false;

int32_t spo2Value = 0;
int8_t spo2Valid = 0;
int32_t hrFromWindow = 0;
int8_t hrFromWindowValid = 0;

/**
 * Indica si el MAX30102 fue detectado e inicializado correctamente.
 * Si es `false`, `readPPG()` no hace nada, pero el resto del programa
 * (IMU, GSR, LEDs) sigue funcionando con normalidad.
 */
bool ppgAvailable = false;

/** Última lectura cruda del canal IR, usada para imprimir el dato. */
uint32_t lastIrReading = 0;

/** Última lectura cruda del canal Red, usada para imprimir el dato. */
uint32_t lastRedReading = 0;

// --- Estimación de pulso (BPM) muestra a muestra, sin bloquear ---
//
// El algoritmo de Maxim entrega BPM solo junto con SpO2, cada
// `PPG_SPO2_RECALC_EVERY` muestras (~cada 250 ms a 100 Hz). Para un
// BPM más responsivo se añade una detección de picos en tiempo real
// sobre el IR filtrado, con el mismo enfoque de filtro paso-bajo +
// línea base que ya se usa para el GSR.

/** Factor de suavizado del filtro paso-bajo de la señal IR (0-1). */
const float PPG_IR_FILTER_ALPHA = 0.3;

/** Factor de suavizado, más lento, para la línea base del IR (0-1). */
const float PPG_IR_BASELINE_ALPHA = 0.01;

/** Umbral mínimo, sobre la línea base, para considerar un pico como latido. */
const float PPG_BEAT_THRESHOLD = 500.0;

/** Tiempo mínimo entre latidos válidos, evita contar dobles picos (150 BPM máx). */
const unsigned long PPG_MIN_BEAT_INTERVAL_MS = 400;

/** Valor mínimo de IR para considerar que hay un dedo sobre el sensor. */
const uint32_t PPG_FINGER_PRESENT_THRESHOLD = 50000;

float filteredIR = 0;
float baselineIR = 0;
bool risingEdge = false;
unsigned long lastBeatTime = 0;
float instantBpm = 0;
bool instantBpmValid = false;

// ---------------------------------------------------------------------
// LED RGB
// ---------------------------------------------------------------------
//
// Funciones reusables de bajo nivel para escribir en los pines del
// LED RGB. Hoy solo las usa la demostración `TEST` de abajo, pero
// están pensadas para que más adelante otra rutina (una animación
// basada en los sensores, y luego una controlada por un modelo de ML)
// las reutilice sin tener que tocar esta capa.

/**
 * Activa o desactiva la impresión por serial de la configuración de
 * LEDs que se está mostrando en cada momento.
 *
 * En `true`, cada llamada a `setLedColor()` (venga de la demostración
 * `TEST` de hoy, de una animación basada en sensores más adelante, o
 * de un modelo de ML después) imprime una línea con el color
 * aplicado. En `false` (valor por defecto), no imprime nada, para no
 * contaminar el log de los sensores (`IMU:`, `GSR:`, `PPG:`) con
 * líneas de LEDs durante uso normal.
 */
const bool DEBUG_LEDS = false;

/**
 * Pin del canal verde (G) del LED RGB.
 */
const uint8_t LED_PIN_G = 9;

/**
 * Pin del canal rojo (R) del LED RGB.
 * Fisicamente conectado a D8, pero el core cambia el mapeo a D10.
 * TODO: Track https://github.com/FastLED/FastLED/issues/2061
 */
const uint8_t LED_PIN_R = 10;

/**
 * Pin del canal azul (B) del LED RGB.
 */
const uint8_t LED_PIN_B = 0;

/**
 * Configura como salida los tres pines del LED RGB y los deja
 * apagados.
 *
 * Los MOSFET de este circuito están cableados a tierra: llevar el
 * pin a HIGH activa el gate, drena el LED a tierra y lo enciende;
 * LOW lo apaga. `setLedColor()` asume esta misma convención.
 *
 * Antes de declarar cada pin como salida, se fuerza brevemente un
 * pull-down interno (`INPUT_PULLDOWN`). Esto evita que, mientras el
 * pin todavía es una entrada (el instante justo en que el firmware
 * toma control, antes de esta llamada), el gate del MOSFET quede en
 * un nivel indefinido/flotante que active el LED parcialmente; con
 * el pull-down, ese nivel queda anclado a LOW. No usa `delay()`: el
 * cambio de modo es instantáneo, no hay que esperar nada.
 */
void setupLeds()
{
    pinMode(LED_PIN_R, INPUT_PULLDOWN);
    pinMode(LED_PIN_G, INPUT_PULLDOWN);
    pinMode(LED_PIN_B, INPUT_PULLDOWN);

    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
}

/**
 * Enciende o apaga, de forma independiente, cada canal del LED RGB.
 *
 * Función reusable de bajo nivel: cualquier rutina que quiera
 * mostrar un color en el LED (la demostración `TEST` de hoy, una
 * animación basada en sensores más adelante, o una controlada por
 * ML después) pasa por aquí en vez de escribir los pines
 * directamente. Al ser el único punto de escritura, también es el
 * único lugar donde hace falta chequear `DEBUG_LEDS` para imprimir
 * el estado por serial: cualquier rutina que llame a esta función
 * obtiene ese log gratis, sin tener que implementarlo ella misma.
 *
 * Los MOSFET son a tierra, así que HIGH = encendido (activa el
 * MOSFET, drena el LED a tierra) y LOW = apagado.
 *
 * Args:
 *     red: `true` para encender el canal rojo, `false` para apagarlo.
 *     green: `true` para encender el canal verde, `false` para apagarlo.
 *     blue: `true` para encender el canal azul, `false` para apagarlo.
 */
void setLedColor(bool red, bool green, bool blue)
{
    digitalWrite(LED_PIN_R, red ? HIGH : LOW);
    digitalWrite(LED_PIN_G, green ? HIGH : LOW);
    digitalWrite(LED_PIN_B, blue ? HIGH : LOW);

    if (DEBUG_LEDS) {
        Serial.print("# LED: R=");
        Serial.print(red ? 1 : 0);
        Serial.print(" G=");
        Serial.print(green ? 1 : 0);
        Serial.print(" B=");
        Serial.println(blue ? 1 : 0);
    }
}

// --- TEST: demostración R -> G -> B -> Todos -> Ninguno ---
//
// Rutina temporal, solo para verificar el cableado del LED RGB. Se
// ejecuta siempre, incluso si ningún sensor fue detectado. Más
// adelante se elimina y se reemplaza por una animación basada en los
// datos de los sensores. No bloqueante: se turna con el resto del
// `loop()` usando `millis()`, igual que el resto del programa.

/** Duración de cada paso de la demostración (cada color), en ms. */
const unsigned long LED_TEST_STEP_MS = 1000;

/** Pasos de la demostración: R, luego G, luego B, luego todos, luego ninguno. */
enum LedTestStep {
    LED_TEST_STEP_RED = 0,
    LED_TEST_STEP_GREEN,
    LED_TEST_STEP_BLUE,
    LED_TEST_STEP_ALL,
    LED_TEST_STEP_NONE
};

LedTestStep ledTestStep = LED_TEST_STEP_RED;
unsigned long ledTestStepStart = 0;

// Prototipo explícito: el generador automático de prototipos del IDE
// de Arduino puede fallar al inferir el tipo `LedTestStep` (un enum
// definido en este mismo archivo) si se declara justo antes de su
// primer uso como parámetro. Declararlo a mano aquí evita el error
// "was not declared in this scope" al compilar.
void applyLedTestStep(LedTestStep step);

/**
 * Aplica el color del paso actual de la demostración.
 *
 * Solo se llama una vez por paso (ver `updateLedTest()`), no en cada
 * vuelta de `loop()`. La impresión por serial del color aplicado (si
 * `DEBUG_LEDS` está activo) ocurre dentro de `setLedColor()`, no
 * aquí, para que cualquier otra rutina que la use más adelante
 * (animación por sensores, luego por ML) obtenga el mismo log sin
 * duplicar esta lógica.
 *
 * Args:
 *     step: Paso de la demostración a aplicar.
 */
void applyLedTestStep(LedTestStep step)
{
    switch (step) {
        case LED_TEST_STEP_RED:
            setLedColor(true, false, false);
            break;

        case LED_TEST_STEP_GREEN:
            setLedColor(false, true, false);
            break;

        case LED_TEST_STEP_BLUE:
            setLedColor(false, false, true);
            break;

        case LED_TEST_STEP_ALL:
            setLedColor(true, true, true);
            break;

        case LED_TEST_STEP_NONE:
            setLedColor(false, false, false);
            break;
    }
}

/**
 * Avanza la demostración `TEST` del LED RGB sin bloquear el resto del
 * programa: R, luego G, luego B, luego todos encendidos, luego
 * ninguno, cada uno durante `LED_TEST_STEP_MS` (1 s), y al terminar
 * repite el ciclo desde R.
 *
 * Se ejecuta siempre desde `loop()`, sin depender de que algún sensor
 * haya sido detectado.
 */
void updateLedTest()
{
    unsigned long now = millis();

    // Al arrancar (o justo al cambiar de paso) el color aún no se ha
    // aplicado ni impreso; se hace aquí, una sola vez por paso.
    static bool stepApplied = false;
    if (!stepApplied) {
        applyLedTestStep(ledTestStep);
        stepApplied = true;
    }

    unsigned long elapsed = now - ledTestStepStart;
    if (elapsed < LED_TEST_STEP_MS) return;

    ledTestStepStart = now;
    stepApplied = false;

    switch (ledTestStep) {
        case LED_TEST_STEP_RED:   ledTestStep = LED_TEST_STEP_GREEN; break;
        case LED_TEST_STEP_GREEN: ledTestStep = LED_TEST_STEP_BLUE;  break;
        case LED_TEST_STEP_BLUE:  ledTestStep = LED_TEST_STEP_ALL;   break;
        case LED_TEST_STEP_ALL:   ledTestStep = LED_TEST_STEP_NONE;  break;
        case LED_TEST_STEP_NONE:  ledTestStep = LED_TEST_STEP_RED;   break;
    }
}

/**
 * Imprime por serial una línea de datos de sensor etiquetada.
 *
 * Emite "TAG:valor1,valor2,...,valorN\n" con precisión decimal
 * configurable por valor. Compartida por todos los sensores, de modo
 * que uno nuevo solo necesita armar sus arreglos de valores/decimales
 * y llamar a esta función.
 *
 * Args:
 *     tag: Etiqueta corta que identifica al sensor (p. ej. "IMU", "GSR").
 *     values: Arreglo de lecturas a imprimir, en el orden de salida.
 *     decimals: Arreglo con la cantidad de decimales por cada valor
 *         en `values`.
 *     count: Número de elementos en `values` y `decimals`.
 */
void printSensorLine(const char *tag, const float *values, const uint8_t *decimals, uint8_t count)
{
    Serial.print(tag);
    Serial.print(':');

    for (uint8_t i = 0; i < count; i++) {
        Serial.print(values[i], decimals[i]);
        if (i < count - 1) Serial.print(',');
    }

    Serial.println();
}

/**
 * Convierte un cuaternión unitario a ángulos de Euler (roll, pitch, yaw).
 *
 * Args:
 *     qr: Componente real (escalar) del cuaternión.
 *     qi: Componente X del cuaternión.
 *     qj: Componente Y del cuaternión.
 *     qk: Componente Z del cuaternión.
 *     roll: Referencia de salida donde se guarda el ángulo roll (grados).
 *     pitch: Referencia de salida donde se guarda el ángulo pitch (grados).
 *     yaw: Referencia de salida donde se guarda el ángulo yaw (grados).
 */
void quaternionToEuler(
    float qr,
    float qi,
    float qj,
    float qk,
    float &roll,
    float &pitch,
    float &yaw)
{
    float sinr_cosp = 2.0 * (qr * qi + qj * qk);
    float cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj);
    roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0 * (qr * qj - qk * qi);

    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    float siny_cosp = 2.0 * (qr * qk + qi * qj);
    float cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
    yaw = atan2(siny_cosp, cosy_cosp);

    roll *= 180.0 / PI;
    pitch *= 180.0 / PI;
    yaw *= 180.0 / PI;
}

/**
 * Inicializa el IMU BNO085 por I2C y habilita el vector de rotación.
 *
 * Si el sensor no es detectado o el reporte no puede habilitarse, se
 * reporta el fallo por serial y `imuAvailable` queda en `false`: el
 * programa continúa (no se congela), para que el resto de sensores y
 * la demostración de LEDs sigan funcionando aunque este sensor no
 * esté conectado.
 */
void setupIMU()
{
    if (!bno08x.begin_I2C(IMU_I2C_ADDR)) {
        Serial.println("# BNO085 no detectado");
        return;
    }

    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("# Rotation Vector fallo");
        return;
    }

    imuAvailable = true;
}

/**
 * Lee un nuevo evento del IMU, si está disponible, y lo transmite por
 * serial en formato "IMU:qw,qx,qy,qz,roll,pitch,yaw".
 *
 * No bloqueante: si aún no hay un evento nuevo, retorna de inmediato
 * sin afectar el muestreo del GSR ni del PPG.
 */
void readIMU()
{
    if (!imuAvailable) return;
    if (!bno08x.getSensorEvent(&imuEvent)) return;
    if (imuEvent.sensorId != SH2_ROTATION_VECTOR) return;

    float qw = imuEvent.un.rotationVector.real;
    float qx = imuEvent.un.rotationVector.i;
    float qy = imuEvent.un.rotationVector.j;
    float qz = imuEvent.un.rotationVector.k;

    float roll, pitch, yaw;
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);

    float imuValues[7]     = {qw, qx, qy, qz, roll, pitch, yaw};
    uint8_t imuDecimals[7] = {6,   6,  6,  6,     2,     2,   2};

    printSensorLine("IMU", imuValues, imuDecimals, 7);
}

/**
 * Configura el pin analógico y el estado inicial del filtro y la
 * línea base del GSR.
 */
void setupGSR()
{
    analogReadResolution(12);
    pinMode(GSR_PIN, INPUT);

    baselineGSR = analogRead(GSR_PIN);
    filteredGSR = baselineGSR;
}

/**
 * Muestrea el GSR a intervalos fijos y no bloqueantes, aplica un
 * filtro paso-bajo exponencial, actualiza la línea base y transmite
 * el resultado por serial en formato "GSR:raw,filtrado,variacion".
 *
 * El muestreo se controla con `millis()` en vez de `delay()`, de modo
 * que no retrasa ni es afectado por la lectura del IMU ni del PPG.
 */
void readGSR()
{
    unsigned long now = millis();
    if (now - lastGsrSampleTime < GSR_SAMPLE_INTERVAL_MS) return;
    lastGsrSampleTime = now;

    int rawGSR = analogRead(GSR_PIN);

    filteredGSR += GSR_FILTER_ALPHA * (rawGSR - filteredGSR);
    baselineGSR += GSR_BASELINE_ALPHA * (filteredGSR - baselineGSR);

    float variationGSR = filteredGSR - baselineGSR;

    float gsrValues[3]     = {(float)rawGSR, filteredGSR, variationGSR};
    uint8_t gsrDecimals[3] = {0, 2, 2};

    printSensorLine("GSR", gsrValues, gsrDecimals, 3);
}

/**
 * Inicializa el MAX30102 por I2C con la configuración recomendada por
 * el fabricante para lectura de SpO2/pulso (alta tasa de muestreo,
 * promediado en hardware para reducir ruido antes del filtrado en
 * software) y deja los LEDs encendidos en espera de un dedo.
 *
 * Si el sensor no es detectado, se reporta el fallo por serial y
 * `ppgAvailable` queda en `false`: el programa continúa (no se
 * congela), para que el resto de sensores y la demostración de LEDs
 * sigan funcionando aunque este sensor no esté conectado.
 */
void setupPPG()
{
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("# MAX30102 no detectado");
        return;
    }

    byte ledBrightness = 60;   // 0-255
    byte sampleAverage = 4;    // Promediado en hardware (reduce ruido de red)
    byte ledMode = 2;          // 2 = Red + IR (requerido para SpO2)
    int sampleRate = 400;      // Hz. Alta tasa para no perder picos de pulso
    int pulseWidth = 411;      // us. Máxima resolución del ADC (18 bits)
    int adcRange = 4096;

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

    ppgAvailable = true;
}

/**
 * Actualiza la detección de latido muestra a muestra sobre el canal
 * IR, aplicando el mismo esquema de filtro paso-bajo + línea base
 * lenta que ya se usa para el GSR, y marca un latido cuando la señal
 * filtrada cruza la línea base hacia arriba superando un umbral
 * mínimo, respetando un intervalo refractario para no contar dobles
 * picos.
 *
 * Este cálculo es independiente del de SpO2: entrega un BPM
 * actualizado en cada muestra en vez de esperar a llenar una ventana
 * completa, priorizando la respuesta rápida del pulso.
 *
 * Args:
 *     ir: Lectura cruda del canal infrarrojo para esta muestra.
 */
void updateInstantBpm(uint32_t ir)
{
    filteredIR += PPG_IR_FILTER_ALPHA * ((float)ir - filteredIR);
    baselineIR += PPG_IR_BASELINE_ALPHA * (filteredIR - baselineIR);

    float aboveBaseline = filteredIR - baselineIR;

    if (!risingEdge && aboveBaseline > PPG_BEAT_THRESHOLD) {
        risingEdge = true;

        unsigned long now = millis();
        unsigned long interval = now - lastBeatTime;

        if (lastBeatTime != 0 && interval >= PPG_MIN_BEAT_INTERVAL_MS) {
            instantBpm = 60000.0 / (float)interval;
            instantBpmValid = true;
        }

        lastBeatTime = now;
    } else if (risingEdge && aboveBaseline < PPG_BEAT_THRESHOLD * 0.5) {
        risingEdge = false;
    }
}

/**
 * Recalcula SpO2 (y el BPM promediado que entrega el mismo algoritmo)
 * a partir de la ventana circular de muestras IR/Red acumuladas,
 * usando el algoritmo de Maxim de la librería SparkFun.
 *
 * Se ejecuta solo una vez cada `PPG_SPO2_RECALC_EVERY` muestras
 * nuevas, y solo cuando la ventana ya está llena, para mantener el
 * costo de CPU bajo sin bloquear el resto del `loop()`.
 */
void recalculateSpo2()
{
    maxim_heart_rate_and_oxygen_saturation(
        ppgIrBuffer, PPG_SPO2_WINDOW, ppgRedBuffer,
        &spo2Value, &spo2Valid, &hrFromWindow, &hrFromWindowValid);
}

/**
 * Lee una nueva muestra del MAX30102, si está disponible, actualiza
 * la detección de pulso instantáneo, alimenta la ventana circular de
 * SpO2, y transmite el resultado por serial en formato
 * "PPG:ir,red,bpm,bpmValido,spo2,spo2Valido".
 *
 * El BPM emitido es el instantáneo (muestra a muestra); si aún no hay
 * un latido detectado se usa el BPM del algoritmo de ventana, siempre
 * que sea válido, como respaldo. No bloqueante: si no hay dato nuevo
 * en el FIFO del sensor, retorna de inmediato sin afectar la lectura
 * del IMU ni del GSR.
 */
void readPPG()
{
    if (!ppgAvailable) return;
    if (!particleSensor.safeCheck(0)) return;

    while (particleSensor.available()) {
        uint32_t ir = particleSensor.getFIFOIR();
        uint32_t red = particleSensor.getFIFORed();
        particleSensor.nextSample();

        lastIrReading = ir;
        lastRedReading = red;

        updateInstantBpm(ir);

        ppgIrBuffer[ppgWindowIndex] = ir;
        ppgRedBuffer[ppgWindowIndex] = red;
        ppgWindowIndex++;

        if (ppgWindowIndex >= PPG_SPO2_WINDOW) {
            ppgWindowIndex = 0;
            ppgWindowFull = true;
        }

        ppgSamplesSinceCalc++;
        if (ppgWindowFull && ppgSamplesSinceCalc >= PPG_SPO2_RECALC_EVERY) {
            ppgSamplesSinceCalc = 0;
            recalculateSpo2();
        }
    }

    bool fingerPresent = lastIrReading > PPG_FINGER_PRESENT_THRESHOLD;

    float bpmToReport = 0;
    bool bpmValidToReport = false;

    if (fingerPresent && instantBpmValid) {
        bpmToReport = instantBpm;
        bpmValidToReport = true;
    } else if (fingerPresent && hrFromWindowValid) {
        bpmToReport = (float)hrFromWindow;
        bpmValidToReport = true;
    }

    float spo2ToReport = (fingerPresent && spo2Valid) ? (float)spo2Value : 0;

    float ppgValues[6] = {
        (float)lastIrReading,
        (float)lastRedReading,
        bpmToReport,
        bpmValidToReport ? 1.0f : 0.0f,
        spo2ToReport,
        (fingerPresent && spo2Valid) ? 1.0f : 0.0f
    };
    uint8_t ppgDecimals[6] = {0, 0, 1, 0, 1, 0};

    printSensorLine("PPG", ppgValues, ppgDecimals, 6);
}

/**
 * Inicializa la comunicación serial, los tres sensores (IMU, GSR y
 * PPG) y el LED RGB.
 *
 * Un sensor no detectado no impide que el resto del `setup()` ni el
 * `loop()` continúen: `setupIMU()` y `setupPPG()` solo reportan el
 * fallo por serial y marcan el sensor como no disponible.
 */
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire.begin();
    setupIMU();
    setupGSR();
    setupPPG();
    setupLeds();

    ledTestStepStart = millis();
}

/**
 * Bucle principal: revisa los tres sensores y actualiza la
 * demostración del LED RGB en cada iteración, todo de forma
 * independiente y no bloqueante, preservando la integridad temporal
 * de cada lectura. La demostración de LEDs corre siempre, incluso si
 * ningún sensor fue detectado.
 */
void loop()
{
    readIMU();
    readGSR();
    readPPG();

    updateLedTest();  // TEST: eliminar cuando se reemplace por la animación real
}