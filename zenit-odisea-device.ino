/**
 * @file zenit-odisea-device.ino
 * @brief Adquiere datos de un IMU BNO085 (I2C), un sensor GSR (analógico)
 *     y un sensor PPG MAX30102 (I2C, pulso y SpO2), y los transmite por
 *     serial en formato "TAG:valor,...".
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
 * Detiene la ejecución si el sensor no es detectado o el reporte no
 * puede habilitarse, para evitar operar sin datos válidos de
 * orientación.
 */
void setupIMU()
{
    if (!bno08x.begin_I2C(IMU_I2C_ADDR)) {
        Serial.println("# BNO085 no detectado");
        while (1);
    }

    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("# Rotation Vector fallo");
        while (1);
    }
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
 * Detiene la ejecución si el sensor no es detectado, para evitar
 * operar sin datos válidos de pulso/oxigenación.
 */
void setupPPG()
{
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("# MAX30102 no detectado");
        while (1);
    }

    byte ledBrightness = 60;   // 0-255
    byte sampleAverage = 4;    // Promediado en hardware (reduce ruido de red)
    byte ledMode = 2;          // 2 = Red + IR (requerido para SpO2)
    int sampleRate = 400;      // Hz. Alta tasa para no perder picos de pulso
    int pulseWidth = 411;      // us. Máxima resolución del ADC (18 bits)
    int adcRange = 4096;

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
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
 * Inicializa la comunicación serial y los tres sensores (IMU, GSR y
 * PPG).
 */
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire.begin();
    setupIMU();
    setupGSR();
    setupPPG();
}

/**
 * Bucle principal: revisa los tres sensores en cada iteración de
 * forma independiente y no bloqueante, preservando la integridad
 * temporal de cada lectura.
 */
void loop()
{
    readIMU();
    readGSR();
    readPPG();
}
