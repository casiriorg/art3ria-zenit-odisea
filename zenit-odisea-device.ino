/**
 * @file zenit-odisea-device.ino
 * @brief Adquiere datos de un IMU BNO085 (I2C) y un sensor GSR (analógico)
 *     y los transmite por serial en formato "TAG:valor,...".
 *
 * Proyecto: zenit-odisea-device
 *
 * Formato de salida:
 *     IMU:qw,qx,qy,qz,roll,pitch,yaw
 *     GSR:raw,filtrado,variacion
 *
 * Cada sensor se muestrea de forma independiente y no bloqueante (sin
 * `delay()` dentro de `loop()`), para que la lectura de uno no retrase
 * ni altere la del otro y así preservar la integridad temporal del dato.
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

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
 * sin afectar el muestreo del GSR.
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
 * que no retrasa ni es afectado por la lectura del IMU.
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
 * Inicializa la comunicación serial y ambos sensores (IMU y GSR).
 */
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire.begin();
    setupIMU();
    setupGSR();
}

/**
 * Bucle principal: revisa ambos sensores en cada iteración de forma
 * independiente y no bloqueante, preservando la integridad temporal
 * de cada lectura.
 */
void loop()
{
    readIMU();
    readGSR();
}