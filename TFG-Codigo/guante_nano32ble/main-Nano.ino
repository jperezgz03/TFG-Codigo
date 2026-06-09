#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <Wire.h>
#include <NanoBLEFlashPrefs.h>

// LIBRERÍA DEL SENSOR CAPACITIVO (Bend Labs / Nitto)
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

// ===================== CONFIGURACIÓN =====================
const int ADS_RST_PIN = 3; // nRST del sensor Bend Labs/Nitto conectado a D3

const float EMA_ALPHA_FLEX = 0.20f; // Suavizado base estándar
const uint32_t LOOP_MS = 20;        // Frecuencia de muestreo (50 Hz)
const uint32_t WEB_NOTIFY_MS = 250;  // Web: 4 actualizaciones/segundo, suficiente y no satura BLE
const uint32_t COMMAND_QUIET_MS = 350; // Pausa de telemetría tras un comando para que el ACK salga limpio

// --- PROTECCIÓN DE LA SEÑAL DE FLEXIÓN ---
// El rechazo de picos se aplica siempre, porque un pico espurio del sensor no
// depende del perfil médico. La limitación de pendiente se aplica a la consigna
// de avance enviada al receptor, especialmente útil cuando el rango calibrado
// del usuario es reducido.
const float FLEX_MIN_VALID_DEG = -230.0f;       // Límite físico razonable inferior
const float FLEX_MAX_VALID_DEG = 50.0f;       // Límite físico razonable superior
const float FLEX_SPIKE_MIN_DEG = 6.0f;         // Umbral mínimo de salto sospechoso
const float FLEX_SPIKE_MAX_DEG = 12.0f;        // Umbral máximo de salto sospechoso
const uint8_t FLEX_SPIKE_CONFIRM_SAMPLES = 4;  // Muestras necesarias para aceptar un salto grande real

// Limitador de pendiente de la consigna normalizada de avance.
// Se permite frenar más rápido que acelerar por seguridad.
const float FLEX_PCT_INC_SMALL_RANGE = 2.0f;   // % por ciclo si el rango calibrado es pequeño
const float FLEX_PCT_INC_MED_RANGE   = 4.0f;   // % por ciclo si el rango calibrado es medio
const float FLEX_PCT_INC_BIG_RANGE   = 6.0f;   // % por ciclo si el rango calibrado es amplio
const float FLEX_PCT_MAX_DEC         = 15.0f;  // % por ciclo para reducir velocidad

// --- PERFIL TOURETTE: VALIDACIÓN TEMPORAL DE ESPASMOS ---
// Giro: solo entra como candidato si la diferencia respecto a la salida filtrada
// supera este umbral. Se ha aumentado para evitar que giros voluntarios amplios
// entren demasiado pronto en modo candidato.
const float TOUR_ROLL_SPIKE_DELTA_DEG = 40.0f;
const uint32_t TOUR_ROLL_CONFIRM_MS   = 1000;
const float TOUR_ROLL_RETURN_DEG      = 10.0f;
const float TOUR_ROLL_NORMAL_ALPHA    = 0.60f;
const float TOUR_ROLL_ACCEPT_ALPHA    = 0.85f;

// Flexión: se filtran subidas bruscas de velocidad para evitar arranques por tic.
// Las bajadas de velocidad/parada NO se retrasan por seguridad.
const float TOUR_FLEX_INC_SPIKE_PCT = 35.0f;
const uint32_t TOUR_FLEX_CONFIRM_MS = 1000;
const float TOUR_FLEX_RETURN_PCT    = 10.0f;

// --- PERFILES MÉDICOS ---
enum PathologyType { NONE = 0, PARKINSON = 1, TOURETTE = 2 };

// --- ESTRUCTURA PARA GUARDAR DATOS EN MEMORIA ---
// V4: estructura con cabecera, versión y CRC para no aceptar basura antigua
// ni datos corruptos tras un apagado.
struct CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  float minFlex;
  float maxFlex;
  float rotZero;
  float rotMin;
  float rotMax;
  float mountRef;

  uint8_t haveRef;
  uint8_t pathology; // Guarda el perfil médico seleccionado
  uint8_t reserved1;
  uint8_t reserved2;

  uint32_t crc;
};

const uint32_t CAL_MAGIC = 0x47564C34; // "GVL4"
const uint16_t CAL_VERSION = 4;

CalibrationData calData;
NanoBLEFlashPrefs flashPrefs;

// Sensor capacitivo Bend Labs / Nitto One Axis
ADS flexSensor;
bool flexOK = false;
float lastFlexAngle = 0.0f;

// Variables de estado
bool systemActive = false;
bool calibrationDirty = false;
bool pendingCalNotify = false;
bool pendingRcalNotify = false;
uint32_t lastCommandMs = 0;
bool bleReady = false;

// Variables de filtrado
float emaAngleFlex = 0.0f;
float emaRollRel = 0.0f;

// Estado del rechazo de picos del sensor de flexión
float flexClean = 0.0f;
float flexCandidate = 0.0f;
uint8_t flexCandidateCount = 0;
bool flexCleanInit = false;

// Estado del limitador de pendiente de la consigna de avance
float pctFlexCommandLimited = 0.0f;
bool pctFlexCommandInit = false;

Madgwick ahrs;
bool imuOK = false;

// ===================== BLE UUIDs =====================
const char* SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9F";
const char* RAW_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* NORM_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const char* ANG_UUID = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E";
const char* CMD_UUID = "6E400005-B5A3-F393-E0A9-E50E24DCCA9E";
const char* CAL_UUID = "6E400006-B5A3-F393-E0A9-E50E24DCCA9E";
const char* ROT_UUID = "6E400007-B5A3-F393-E0A9-E50E24DCCA9E";
const char* ROTN_UUID = "6E400008-B5A3-F393-E0A9-E50E24DCCA9E";
const char* RCAL_UUID = "6E400009-B5A3-F393-E0A9-E50E24DCCA9E";
const char* CTRL_UUID = "6E40000A-B5A3-F393-E0A9-E50E24DCCA9E";
const char* ACK_UUID  = "6E40000B-B5A3-F393-E0A9-E50E24DCCA9E";

BLEService gloveService(SVC_UUID);
BLECharacteristic rawChar(RAW_UUID, BLERead | BLENotify, 2);
BLECharacteristic normChar(NORM_UUID, BLERead | BLENotify, 1);
BLECharacteristic angChar(ANG_UUID, BLERead | BLENotify, 4);
BLECharacteristic cmdChar(CMD_UUID, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic calChar(CAL_UUID, BLERead | BLENotify, 8);
BLECharacteristic rcalChar(RCAL_UUID, BLERead | BLENotify, 6);
BLECharacteristic rotChar(ROT_UUID, BLERead | BLENotify, 4);
BLECharacteristic rotnChar(ROTN_UUID, BLERead | BLENotify, 1);
BLECharacteristic ctrlChar(CTRL_UUID, BLERead | BLENotify, 4);
BLECharacteristic ackChar(ACK_UUID, BLERead | BLENotify, 20);

// ===================== FUNCIONES AUXILIARES =====================
static inline void u16_to_be(uint16_t v, uint8_t out[2]) { out[0] = v >> 8; out[1] = v & 0xFF; }
static inline void i16_to_be(int16_t v, uint8_t out[2]) { out[0] = (v >> 8) & 0xFF; out[1] = v & 0xFF; }
static inline void f32_to_le(float f, uint8_t out[4]) {
  union { float f; uint8_t b[4]; } u;
  u.f = f; out[0] = u.b[0]; out[1] = u.b[1]; out[2] = u.b[2]; out[3] = u.b[3];
}
int clampi(int v, int mn, int mx) { return (v < mn) ? mn : ((v > mx) ? mx : v); }
float clampf(float v, float mn, float mx) { return (v < mn) ? mn : ((v > mx) ? mx : v); }
float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

float clampRoll180(float a) {
  if (!isfinite(a)) return 0.0f;
  if (a > 180.0f) return 180.0f;
  if (a < -180.0f) return -180.0f;
  return a;
}


float applyRollDynamicStability(float targetRoll, float currentRoll) {
  // Filtro dinámico común para estabilizar el Roll en todos los perfiles.
  // No sustituye al filtro específico de cada patología, sino que actúa como
  // etapa final para evitar que pequeñas oscilaciones con la mano quieta se
  // conviertan en variaciones continuas de dirección.
  //
  // La ganancia alpha NO es fija en la zona intermedia: aumenta de forma
  // progresiva con la diferencia angular. Así, una diferencia de 3 grados se
  // suaviza mucho más que una diferencia de 10 grados.

  targetRoll = clampRoll180(targetRoll);
  currentRoll = clampRoll180(currentRoll);

  float rollDiff = targetRoll - currentRoll;
  float diff = fabsf(rollDiff);

  // Zona de reposo: ignora microvariaciones alrededor de la posición actual.
  // Esto evita que el valor baile cuando la mano está prácticamente quieta.
  if (diff < 2.0f) {
    return currentRoll;
  }

  // Zona dinámica: alpha aumenta linealmente entre 2 y 12 grados.
  // diff = 3º  -> alpha bajo, mucha estabilidad.
  // diff = 10º -> alpha más alto, mayor seguimiento.
  if (diff < 12.0f) {
    const float alphaMin = 0.08f;
    const float alphaMax = 0.45f;
    const float diffMin = 2.0f;
    const float diffMax = 12.0f;

    float alpha = alphaMin + ((diff - diffMin) * (alphaMax - alphaMin) / (diffMax - diffMin));
    alpha = clampf(alpha, alphaMin, alphaMax);

    return clampRoll180(currentRoll + alpha * rollDiff);
  }

  // Zona de movimiento voluntario claro: respuesta rápida, pero sin abrir el
  // filtro tanto como antes con alpha = 0.90, que hacía el Roll muy nervioso.
  const float alphaFast = 0.65f;
  return clampRoll180(currentRoll + alphaFast * rollDiff);
}

float calibratedFlexRangeDeg() {
  return fabsf(calData.maxFlex - calData.minFlex);
}

float adaptiveFlexSpikeDeg() {
  // Si el rango calibrado es pequeño, un salto de pocos grados puede suponer
  // pasar de 0% a 100%. Por eso el umbral se hace adaptativo.
  float range = calibratedFlexRangeDeg();
  float th = 0.45f * range;
  th = clampf(th, FLEX_SPIKE_MIN_DEG, FLEX_SPIKE_MAX_DEG);
  return th;
}

float flexMaxIncreasePctPerCycle() {
  float range = calibratedFlexRangeDeg();

  // Con rangos pequeños el sistema es muy sensible: aceleramos más despacio.
  if (range < 15.0f) return FLEX_PCT_INC_SMALL_RANGE;
  if (range < 35.0f) return FLEX_PCT_INC_MED_RANGE;
  return FLEX_PCT_INC_BIG_RANGE;
}

void resetFlexSignalProtection(float currentFlexDeg) {
  flexClean = currentFlexDeg;
  flexCandidate = currentFlexDeg;
  flexCandidateCount = 0;
  flexCleanInit = true;

  // La consigna al receptor arranca desde cero por seguridad.
  pctFlexCommandLimited = 0.0f;
  pctFlexCommandInit = true;
}

float rejectFlexSpike(float x) {
  // Rechazo de lecturas imposibles o no numéricas.
  if (!isfinite(x) || x < FLEX_MIN_VALID_DEG || x > FLEX_MAX_VALID_DEG) {
    return flexCleanInit ? flexClean : lastFlexAngle;
  }

  // Inicialización con la primera lectura válida.
  if (!flexCleanInit) {
    resetFlexSignalProtection(x);
    return x;
  }

  float diff = x - flexClean;
  float spikeThreshold = adaptiveFlexSpikeDeg();

  // Si el salto es compatible con un gesto normal, se acepta.
  if (fabsf(diff) <= spikeThreshold) {
    flexClean = x;
    flexCandidate = x;
    flexCandidateCount = 0;
    return x;
  }

  // Si el salto es excesivo, se considera sospechoso. Solo se acepta si el
  // nuevo valor se mantiene durante varias muestras consecutivas.
  float candidateWindow = max(1.5f, spikeThreshold * 0.25f);
  if (fabsf(x - flexCandidate) <= candidateWindow) {
    flexCandidateCount++;
  } else {
    flexCandidate = x;
    flexCandidateCount = 1;
  }

  if (flexCandidateCount >= FLEX_SPIKE_CONFIRM_SAMPLES) {
    flexClean = x;
    flexCandidate = x;
    flexCandidateCount = 0;
    return x;
  }

  // Pico aislado: se mantiene la última muestra considerada válida.
  return flexClean;
}

float limitFlexCommandPercent(float targetPct) {
  targetPct = clampf(targetPct, 0.0f, 100.0f);

  if (!pctFlexCommandInit) {
    pctFlexCommandLimited = targetPct;
    pctFlexCommandInit = true;
    return pctFlexCommandLimited;
  }

  float delta = targetPct - pctFlexCommandLimited;
  float maxInc = flexMaxIncreasePctPerCycle();

  if (delta > maxInc) {
    delta = maxInc;
  } else if (delta < -FLEX_PCT_MAX_DEC) {
    delta = -FLEX_PCT_MAX_DEC;
  }

  pctFlexCommandLimited += delta;
  pctFlexCommandLimited = clampf(pctFlexCommandLimited, 0.0f, 100.0f);
  return pctFlexCommandLimited;
}

// ===================== PERFIL TOURETTE: FILTROS TEMPORALES =====================
// Esta lógica NO modifica la configuración de Madgwick ni la frecuencia interna.
// Solo se aplica cuando el perfil TOURETTE está seleccionado desde la HMI.
class TouretteRollTemporalGate {
public:
  bool candidateActive = false;
  float anchor = 0.0f;
  uint32_t candidateStartMs = 0;
  int8_t candidateSign = 0;

  void reset(float currentValue) {
    candidateActive = false;
    anchor = clampRoll180(currentValue);
    candidateStartMs = 0;
    candidateSign = 0;
  }

  float update(float raw, float currentOutput, uint32_t now) {
    // En este control el Roll NO se trata como una magnitud circular.
    // Si supera +180 o -180 se satura, no se envuelve al lado contrario.
    raw = clampRoll180(raw);
    currentOutput = clampRoll180(currentOutput);

    float diff = raw - currentOutput;
    float absDiff = fabsf(diff);

    // Movimiento normal o progresivo: respuesta rápida, sin congelar.
    if (!candidateActive && absDiff <= TOUR_ROLL_SPIKE_DELTA_DEG) {
      return clampRoll180(currentOutput + TOUR_ROLL_NORMAL_ALPHA * diff);
    }

    // Nuevo salto brusco: se considera candidato, no se acepta todavía.
    if (!candidateActive) {
      candidateActive = true;
      anchor = currentOutput;
      candidateStartMs = now;
      candidateSign = (diff >= 0.0f) ? 1 : -1;
      return clampRoll180(currentOutput);
    }

    // Candidato activo: evaluamos si el cambio se mantiene o desaparece.
    float candidateDiffFromAnchor = raw - anchor;
    float signedDistance = candidateSign * candidateDiffFromAnchor;

    // Si vuelve cerca del valor anterior, se interpreta como tic corto y se descarta.
    if (signedDistance <= TOUR_ROLL_RETURN_DEG) {
      candidateActive = false;
      candidateSign = 0;
      return clampRoll180(currentOutput);
    }

    // Si el cambio persiste durante la ventana temporal, se acepta rápido.
    if (now - candidateStartMs >= TOUR_ROLL_CONFIRM_MS) {
      candidateActive = false;
      candidateSign = 0;

      float acceptDiff = raw - currentOutput;
      return clampRoll180(currentOutput + TOUR_ROLL_ACCEPT_ALPHA * acceptDiff);
    }

    // Mientras no se confirme, se mantiene la salida anterior.
    return clampRoll180(currentOutput);
  }
};

class TouretteFlexTemporalGate {
public:
  bool candidateActive = false;
  float anchorPct = 0.0f;
  uint32_t candidateStartMs = 0;

  void reset(float currentPct) {
    candidateActive = false;
    anchorPct = clampf(currentPct, 0.0f, 100.0f);
    candidateStartMs = 0;
  }

  float update(float targetPct, float currentCommandPct, uint32_t now) {
    targetPct = clampf(targetPct, 0.0f, 100.0f);
    currentCommandPct = clampf(currentCommandPct, 0.0f, 100.0f);

    float delta = targetPct - currentCommandPct;

    // Seguridad: las bajadas de velocidad o parada se aceptan rápido.
    // No conviene retrasar una orden que reduce movimiento.
    if (delta <= 0.0f) {
      candidateActive = false;
      return targetPct;
    }

    // Subida pequeña/progresiva: se acepta.
    if (!candidateActive && delta <= TOUR_FLEX_INC_SPIKE_PCT) {
      return targetPct;
    }

    // Subida brusca: se guarda como candidata.
    if (!candidateActive) {
      candidateActive = true;
      anchorPct = currentCommandPct;
      candidateStartMs = now;
      return currentCommandPct;
    }

    // Si el valor vuelve cerca del punto inicial, se descarta como tic.
    if (targetPct <= anchorPct + TOUR_FLEX_RETURN_PCT) {
      candidateActive = false;
      return targetPct;
    }

    // Si la subida brusca persiste, se acepta.
    if (now - candidateStartMs >= TOUR_FLEX_CONFIRM_MS) {
      candidateActive = false;
      return targetPct;
    }

    // Mientras no se confirme, se mantiene la consigna anterior.
    return currentCommandPct;
  }
};

TouretteRollTemporalGate touretteRollGate;
TouretteFlexTemporalGate touretteFlexGate;

void resetTouretteGates() {
  touretteRollGate.reset(emaRollRel);
  touretteFlexGate.reset(pctFlexCommandLimited);
}

// ===================== FILTRO BUTTERWORTH PASO BAJO =====================
// Filtro digital IIR Butterworth de 2º orden.
// Coeficientes calculados para:
//   fs = 50 Hz  (LOOP_MS = 20 ms)
//   fc = 2 Hz   (perfil PARKINSON, rechazo de oscilaciones rápidas)
// Ecuación:
//   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
class ButterworthLPF2 {
public:
  const float b0 = 0.0133592000f;
  const float b1 = 0.0267184001f;
  const float b2 = 0.0133592000f;
  const float a1 = -1.6474599811f;
  const float a2 = 0.7008967812f;

  float x1 = 0.0f;
  float x2 = 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;

  float update(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    x2 = x1;
    x1 = x;

    y2 = y1;
    y1 = y;

    return y;
  }

  void reset(float value = 0.0f) {
    x1 = value;
    x2 = value;
    y1 = value;
    y2 = value;
  }
};

ButterworthLPF2 butterFlexPark;
ButterworthLPF2 butterRollPark;

// ===================== MEMORIA Y CALIBRACIÓN =====================
uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
  }
  return crc;
}

uint32_t calcCalibrationCrc(const CalibrationData& d) {
  const uint8_t* p = (const uint8_t*)&d;
  size_t n = offsetof(CalibrationData, crc);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < n; i++) crc = crc32_update(crc, p[i]);
  return ~crc;
}

bool finiteAndReasonable(float v, float lim = 360.0f) {
  return isfinite(v) && v > -lim && v < lim;
}

void setDefaultCalibration() {
  memset(&calData, 0, sizeof(calData));
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.size = sizeof(CalibrationData);

  // OJO: el sensor puede dar grados negativos. Guardar -22° es válido.
  // Estos son solo valores por defecto hasta que calibres.
  calData.minFlex = 0.0f;
  calData.maxFlex = 90.0f;
  calData.rotZero = 0.0f;
  calData.rotMin = 0.0f;
  calData.rotMax = 0.0f;
  calData.mountRef = 0.0f;
  calData.haveRef = 0;
  calData.pathology = NONE;
  calData.crc = calcCalibrationCrc(calData);
}

bool calibrationLooksValid(const CalibrationData& d) {
  if (d.magic != CAL_MAGIC) return false;
  if (d.version != CAL_VERSION) return false;
  if (d.size != sizeof(CalibrationData)) return false;
  if (d.crc != calcCalibrationCrc(d)) return false;

  if (!finiteAndReasonable(d.minFlex)) return false;
  if (!finiteAndReasonable(d.maxFlex)) return false;
  if (!finiteAndReasonable(d.rotZero)) return false;
  if (!finiteAndReasonable(d.rotMin)) return false;
  if (!finiteAndReasonable(d.rotMax)) return false;
  if (!finiteAndReasonable(d.mountRef)) return false;
  if (d.pathology > TOURETTE) return false;

  return true;
}

bool saveCalibrationNow(const char* reason) {
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.size = sizeof(CalibrationData);
  calData.crc = calcCalibrationCrc(calData);

  Serial.print("FLASH SAVE START: ");
  Serial.println(reason);

  // Silencio BLE alrededor de la escritura. Evita atascar Chrome al guardar flash.
  lastCommandMs = millis();
  if (bleReady) BLE.poll();

  int rc = flashPrefs.writePrefs(&calData, sizeof(calData));

  if (rc != 0) {
    Serial.print("FLASH SAVE ERROR 1: ");
    Serial.println(flashPrefs.errorString(rc));

    if (bleReady) BLE.poll();
    flashPrefs.garbageCollection();
    delay(80);
    if (bleReady) BLE.poll();
    rc = flashPrefs.writePrefs(&calData, sizeof(calData));
  }

  if (rc == 0) {
    calibrationDirty = false;
    Serial.print("FLASH SAVE OK: ");
    Serial.println(reason);

    // Lectura de verificación inmediata. Si esto falla, NO mandamos ACK.
    CalibrationData verify;
    int rr = flashPrefs.readPrefs(&verify, sizeof(verify));
    if (rr == 0 && calibrationLooksValid(verify)) {
      Serial.println("FLASH VERIFY OK");
      return true;
    }

    Serial.print("FLASH VERIFY ERROR: ");
    Serial.println(flashPrefs.errorString(rr));
    return false;
  }

  Serial.print("FLASH SAVE ERROR 2: ");
  Serial.println(flashPrefs.errorString(rc));
  return false;
}

void markCalibrationDirty() {
  calibrationDirty = true;
}

bool saveCalibrationIfDirty(const char* reason) {
  if (calibrationDirty) return saveCalibrationNow(reason);
  return true;
}

void sendAck(const String& seq, bool ok) {
  String msg = (ok ? "ACK|" : "ERR|") + seq;
  uint8_t buf[20] = {0};
  size_t n = min((size_t)20, msg.length());
  memcpy(buf, msg.c_str(), n);
  ackChar.writeValue(buf, n);
}

void loadCalibration() {
  CalibrationData temp;
  int rc = flashPrefs.readPrefs(&temp, sizeof(temp));

  if (rc == 0 && calibrationLooksValid(temp)) {
    calData = temp;
    Serial.println("FLASH LOAD OK V4");
  } else {
    Serial.print("FLASH LOAD DEFAULTS V4. rc=");
    Serial.print(rc);
    Serial.print(" ");
    Serial.println(flashPrefs.errorString(rc));

    // Forzamos valores limpios. Al cambiar a V4 se ignora memoria antigua V2/V3.
    setDefaultCalibration();
    calibrationDirty = true;
    saveCalibrationNow("defaults");
  }
}

// ===================== LECTURA SENSORES =====================
void resetFlexSensorHardware() {
  // El Arduino Nano 33 BLE trabaja a 3.3V, por eso HIGH es seguro en este caso.
  // nRST es activo en bajo: LOW = reset, HIGH = funcionamiento normal.
  pinMode(ADS_RST_PIN, OUTPUT);
  digitalWrite(ADS_RST_PIN, LOW);
  delay(100);
  digitalWrite(ADS_RST_PIN, HIGH);
  delay(700);
}

bool beginFlexSensor() {
  resetFlexSensorHardware();

  if (!flexSensor.begin(ADS_ONE_AXIS_ADDRESS, Wire)) {
    Serial.println("ERROR: Bend Labs/Nitto no detectado en 0x12");
    return false;
  }

  flexSensor.setSampleRate(ADS_100_HZ);
  flexSensor.run();
  delay(300);

  Serial.println("Bend Labs/Nitto iniciado en 0x12");
  Serial.print("Tipo/ejes: ");
  Serial.println(flexSensor.getDeviceType());
  Serial.print("Firmware: ");
  Serial.println(flexSensor.getFirmwareVersion());

  // Primera lectura válida para inicializar el filtro EMA
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if (flexSensor.available()) {
      lastFlexAngle = flexSensor.getX();
      return true;
    }
    delay(10);
  }

  Serial.println("Aviso: sensor detectado, pero sin muestra inicial");
  return true;
}

float readFlexSensor() {
  // La librería actualiza getX() únicamente cuando available() devuelve true.
  if (flexOK && flexSensor.available()) {
    lastFlexAngle = flexSensor.getX();
  }

  // Si no hay muestra nueva, mantenemos el último ángulo válido.
  return lastFlexAngle;
}

uint8_t flexAngleToPercent(float val) {
  float mn = min(calData.minFlex, calData.maxFlex);
  float mx = max(calData.minFlex, calData.maxFlex);
  val = clampf(val, mn, mx);
  float rng = max(1.0f, mx - mn);
  
  float pct = (calData.maxFlex > calData.minFlex) ? ((val - mn) * 100.0f / rng) : ((mx - val) * 100.0f / rng);
  return (uint8_t)clampi(roundf(pct), 0, 100);
}

uint8_t angleToPercent(float currentAngle) {
  if (currentAngle >= calData.rotZero) {
    float rango = max(1.0f, calData.rotMin - calData.rotZero);
    float pct = 50.0f + 50.0f * (currentAngle - calData.rotZero) / rango;
    return (uint8_t)clampi(roundf(pct), 50, 100);
  } else {
    float rango = max(1.0f, calData.rotZero - calData.rotMax);
    float pct = 50.0f * (currentAngle - calData.rotMax) / rango;
    return (uint8_t)clampi(roundf(pct), 0, 50);
  }
}

// ===================== NOTIFICACIONES BLE =====================
void notifyCAL() {
  uint8_t buf[8];
  f32_to_le(calData.minFlex, buf + 0);
  f32_to_le(calData.maxFlex, buf + 4);
  calChar.writeValue(buf, 8);
}

void notifyRCAL() {
  int16_t z = (int16_t)roundf(calData.rotZero * 10.0f);
  int16_t mn = (int16_t)roundf(calData.rotMin * 10.0f);
  int16_t mx = (int16_t)roundf(calData.rotMax * 10.0f);
  uint8_t b[6];
  i16_to_be(z, b + 0); i16_to_be(mn, b + 2); i16_to_be(mx, b + 4);
  rcalChar.writeValue(b, 6);
}

void notifyCALIfSubscribed() {
  if (!calChar.subscribed()) return;
  notifyCAL();
}

void notifyRCALIfSubscribed() {
  if (!rcalChar.subscribed()) return;
  notifyRCAL();
}

// ===================== COMANDOS DESDE LA WEB =====================
// V4: cada botón de calibración guarda inmediatamente en Flash.
// El ACK solo se manda cuando el dato se ha procesado, escrito y verificado.
bool handleCommand(const String& cmd, bool& mustSave, bool& mustNotifyCal, bool& mustNotifyRcal) {
  mustSave = false;
  mustNotifyCal = false;
  mustNotifyRcal = false;

  if (cmd == "SET_OPEN") {
    calData.minFlex = emaAngleFlex;
    mustSave = true;
    mustNotifyCal = true;
  }
  else if (cmd == "SET_CLOSED") {
    calData.maxFlex = emaAngleFlex;
    mustSave = true;
    mustNotifyCal = true;
  }
  else if (cmd == "RESET_CAL") {
    calData.minFlex = 0.0f;
    calData.maxFlex = 90.0f;
    systemActive = false;
    resetFlexSignalProtection(emaAngleFlex);
    resetTouretteGates();
    mustSave = true;
    mustNotifyCal = true;
  }
  else if (cmd == "ROT_ZERO") {
    calData.rotZero = emaRollRel;
    mustSave = true;
    mustNotifyRcal = true;
  }
  else if (cmd == "ROT_MIN") {
    calData.rotMin = emaRollRel;
    mustSave = true;
    mustNotifyRcal = true;
  }
  else if (cmd == "ROT_MAX") {
    calData.rotMax = emaRollRel;
    mustSave = true;
    mustNotifyRcal = true;
  }
  else if (cmd == "ROT_RESET") {
    calData.rotZero = 0.0f;
    calData.rotMin = 0.0f;
    calData.rotMax = 0.0f;
    systemActive = false;
    resetTouretteGates();
    mustSave = true;
    mustNotifyRcal = true;
  }
  else if (cmd == "MOUNT_SET") {
    calData.mountRef = ahrs.getRoll();
    calData.haveRef = 1;
    mustSave = true;
  }
  else if (cmd == "PAT_NONE") {
    calData.pathology = NONE;
    // Reinicio de estados para evitar transitorios al cambiar de perfil.
    butterFlexPark.reset(emaAngleFlex);
    butterRollPark.reset(emaRollRel);
    resetTouretteGates();
    mustSave = true;
  }
  else if (cmd == "PAT_PARK") {
    calData.pathology = PARKINSON;
    // Inicializamos el Butterworth con el valor actual filtrado para evitar saltos.
    butterFlexPark.reset(emaAngleFlex);
    butterRollPark.reset(emaRollRel);
    resetTouretteGates();
    mustSave = true;
  }
  else if (cmd == "PAT_TOUR") {
    calData.pathology = TOURETTE;
    // Reinicio de estados para evitar transitorios al cambiar de perfil.
    butterFlexPark.reset(emaAngleFlex);
    butterRollPark.reset(emaRollRel);
    resetTouretteGates();
    mustSave = true;
  }
  else if (cmd == "GET_CAL") {
    mustNotifyCal = true;
    mustNotifyRcal = true;
  }
  else if (cmd == "CAL_DONE") {
    // Se activa la salida al robot, pero NO se persiste el estado activo por seguridad.
    // La consigna de avance se inicia desde cero y sube con rampa, aunque la mano
    // ya esté cerrada en el momento de empezar a conducir.
    pctFlexCommandLimited = 0.0f;
    pctFlexCommandInit = true;
    resetTouretteGates();
    systemActive = true;
  }
  else {
    return false;
  }

  if (mustSave) markCalibrationDirty();
  return true;
}

void processCommandPayload() {
  lastCommandMs = millis();

  int len = cmdChar.valueLength();
  char buf[21] = { 0 };
  len = min(len, 20);
  cmdChar.readValue((uint8_t*)buf, len);

  String payload = String(buf);
  payload.trim();

  String cmd = payload;
  String seq = "0";

  int sep = payload.indexOf('|');
  if (sep >= 0) {
    cmd = payload.substring(0, sep);
    seq = payload.substring(sep + 1);
    cmd.trim();
    seq.trim();
  }

  Serial.print("CMD RX: ");
  Serial.println(cmd);

  bool mustSave = false;
  bool mustNotifyCal = false;
  bool mustNotifyRcal = false;
  bool ok = handleCommand(cmd, mustSave, mustNotifyCal, mustNotifyRcal);

  bool saved = true;
  if (ok && mustSave) {
    saved = saveCalibrationIfDirty(cmd.c_str());
  }

  // Actualizamos inmediatamente el valor de las características para que readValue()
  // al reconectar muestre lo último guardado, incluso si la notificación no llega.
  if (ok && mustNotifyCal) {
    notifyCAL();
    pendingCalNotify = false;
  }
  if (ok && mustNotifyRcal) {
    notifyRCAL();
    pendingRcalNotify = false;
  }
  if (ok && cmd == "GET_CAL") {
    notifyCAL();
    notifyRCAL();
  }

  BLE.poll();
  sendAck(seq, ok && saved);
  BLE.poll();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(100000); // estable para el Bend Labs/Nitto
  pinMode(LED_BUILTIN, OUTPUT);

  // Inicializar sensor capacitivo Bend Labs / Nitto mediante librería SparkFun
  flexOK = beginFlexSensor();

  loadCalibration();
  
  // Inicializar IMU
  imuOK = IMU.begin();
  ahrs.begin(200.0f);
  ahrs.beta = 10.0f;

  if (!BLE.begin()) {
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
  }

  bleReady = true;

  BLE.setLocalName("Guante-Cal-V4");
  BLE.setDeviceName("Guante-Cal-V4");
  BLE.setAdvertisedService(gloveService);

  gloveService.addCharacteristic(rawChar);
  gloveService.addCharacteristic(normChar);
  gloveService.addCharacteristic(angChar);
  gloveService.addCharacteristic(cmdChar);
  gloveService.addCharacteristic(calChar);
  gloveService.addCharacteristic(rotChar);
  gloveService.addCharacteristic(rotnChar);
  gloveService.addCharacteristic(rcalChar);
  gloveService.addCharacteristic(ctrlChar);
  gloveService.addCharacteristic(ackChar);

  BLE.addService(gloveService);
  
  notifyCAL();
  notifyRCAL();
  
  emaAngleFlex = readFlexSensor();
  resetFlexSignalProtection(emaAngleFlex);
  butterFlexPark.reset(emaAngleFlex);

  uint8_t initFlexBuf[4];
  f32_to_le(emaAngleFlex, initFlexBuf);
  angChar.writeValue(initFlexBuf, 4);
  uint8_t initPctFlex = flexAngleToPercent(emaAngleFlex);
  normChar.writeValue(&initPctFlex, 1);

  if (imuOK) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    ahrs.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az);
    emaRollRel = ahrs.getRoll();
    if (calData.rotZero == 0.0f && calData.rotMin == 0.0f) {
      calData.rotZero = emaRollRel;
      calData.rotMin = emaRollRel;
      calData.rotMax = emaRollRel;
    }
  }

  butterRollPark.reset(emaRollRel);
  resetTouretteGates();

  uint8_t initRotBuf[4];
  f32_to_le(emaRollRel, initRotBuf);
  rotChar.writeValue(initRotBuf, 4);
  uint8_t initRotPct = angleToPercent(emaRollRel);
  rotnChar.writeValue(&initRotPct, 1);

  BLE.advertise();
}

// ===================== BUCLE PRINCIPAL =====================
void loop() {
  BLEDevice central = BLE.central();
  static uint32_t lastMs = millis();

  if (central) {
    while (central.connected()) {
      BLE.poll();

      if (cmdChar.written()) {
        processCommandPayload();
      }

      uint32_t now = millis();
      if (now - lastMs >= LOOP_MS) {
        lastMs = now;
        
        // =========================================================
        // 1. LECTURAS CRUDAS BASE
        // =========================================================
        float rawFlexSensor = readFlexSensor();
        float rawFlex = rejectFlexSpike(rawFlexSensor);
        float rawRoll = emaRollRel; 
        
        if (imuOK && IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            float ax, ay, az, gx, gy, gz;
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);
            ahrs.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az);
            float rollAbs = ahrs.getRoll();
            rawRoll = calData.haveRef ? wrap180(rollAbs - calData.mountRef) : rollAbs;
        }

        // =========================================================
        // 2. PROCESAMIENTO BIOMÉDICO (Filtros según Patología)
        // =========================================================
        // Primero cada perfil calcula un Roll objetivo. Después, todos los
        // perfiles pasan por una etapa común de estabilización dinámica para
        // reducir oscilaciones cuando la mano está quieta cerca del centro.
        float rollTarget = clampRoll180(rawRoll);

        if (calData.pathology == PARKINSON) {
            // Perfil PARKINSON:
            // Filtro paso bajo Butterworth digital de 2º orden.
            // fs = 50 Hz, fc = 2 Hz. Atenúa oscilaciones rápidas
            // manteniendo la componente lenta del gesto voluntario.
            emaAngleFlex = butterFlexPark.update(rawFlex);
            rollTarget   = clampRoll180(butterRollPark.update(rawRoll));

        } else if (calData.pathology == TOURETTE) {
            // Perfil TOURETTE:
            // Se mantiene la dinámica de Madgwick que ya funcionaba bien.
            // No se aplica un filtro lento continuo: se validan temporalmente
            // los cambios bruscos extremos para evitar que un tic se convierta
            // en una orden de giro o en un arranque repentino.

            // Flexión: EMA base. La lógica específica de Tourette sobre la
            // velocidad se aplica después, ya en porcentaje normalizado.
            emaAngleFlex = (EMA_ALPHA_FLEX * rawFlex) + ((1.0f - EMA_ALPHA_FLEX) * emaAngleFlex);

            // Giro: candidato temporal. Solo entra en candidato si supera
            // TOUR_ROLL_SPIKE_DELTA_DEG. Si persiste TOUR_ROLL_CONFIRM_MS,
            // se acepta; si vuelve cerca del valor anterior, se descarta.
            rollTarget = clampRoll180(touretteRollGate.update(rawRoll, emaRollRel, now));

        } else {
            // Perfil ESTÁNDAR:
            // EMA base para la flexión. El Roll se estabiliza después con
            // el filtro dinámico común, igual que en el resto de perfiles.
            emaAngleFlex = (EMA_ALPHA_FLEX * rawFlex) + ((1.0f - EMA_ALPHA_FLEX) * emaAngleFlex);
            rollTarget = clampRoll180(rawRoll);
        }

        // Filtro dinámico común aplicado a TODOS los perfiles.
        // Mejora la estabilidad cuando la mano está quieta y evita que el
        // valor de Roll baile continuamente alrededor del centro.
        emaRollRel = applyRollDynamicStability(rollTarget, emaRollRel);

        // =========================================================
        // 3. ACTUALIZACIÓN BLE Y CÁLCULO DE MOTORES
        // =========================================================
        uint8_t pctFlexTarget = flexAngleToPercent(emaAngleFlex);

        float pctFlexForControl = (float)pctFlexTarget;
        if (calData.pathology == TOURETTE) {
          // Bloquea únicamente subidas bruscas de velocidad.
          // Las bajadas/paradas se aceptan rápido por seguridad.
          pctFlexForControl = touretteFlexGate.update(
            (float)pctFlexTarget,
            pctFlexCommandLimited,
            now
          );
        }

        uint8_t pctFlexCommand = (uint8_t)roundf(limitFlexCommandPercent(pctFlexForControl));
        uint8_t rp = angleToPercent(clampRoll180(emaRollRel));
        static uint32_t lastWebUpdate = 0;
        static uint32_t lastPendingNotify = 0;

        // Tras un comando dejamos un pequeño silencio de telemetría. Evita que Chrome
        // marque la característica como ocupada justo al pulsar varios botones.
        bool quietAfterCommand = (now - lastCommandMs) < COMMAND_QUIET_MS;

        if ((pendingCalNotify || pendingRcalNotify) && (now - lastPendingNotify >= 100)) {
          lastPendingNotify = now;
          if (pendingCalNotify) {
            notifyCALIfSubscribed();
            pendingCalNotify = false;
          }
          if (pendingRcalNotify) {
            notifyRCALIfSubscribed();
            pendingRcalNotify = false;
          }
        }

        if (!quietAfterCommand && now - lastWebUpdate >= WEB_NOTIFY_MS) {
          lastWebUpdate = now;
          // Enviar solo si la web está suscrita. Menos tráfico BLE = comandos más fiables.
          uint8_t f4[4]; f32_to_le(emaAngleFlex, f4);
          if (angChar.subscribed()) angChar.writeValue(f4, 4);

          int16_t flexCentideg = (int16_t)roundf(emaAngleFlex * 100.0f);
          uint8_t raw2[2]; i16_to_be(flexCentideg, raw2);
          if (rawChar.subscribed()) rawChar.writeValue(raw2, 2);

          if (normChar.subscribed()) normChar.writeValue(&pctFlexTarget, 1);

          float rollToSend = clampRoll180(emaRollRel);
          uint8_t r4[4]; f32_to_le(rollToSend, r4);
          if (rotChar.subscribed()) rotChar.writeValue(r4, 4);

          if (rotnChar.subscribed()) rotnChar.writeValue(&rp, 1);
        }
        // Lógica Progresiva y Zona Muerta
        int flexInput = pctFlexCommand;
        if (flexInput < 8) flexInput = 0; // Zona muerta (8%)
        
        int velocidadCalc = clampi(map(flexInput, 0, 100, 0, 255), 0, 255);
        
        uint8_t comandoMovimiento = (velocidadCalc > 0) ? 1 : 0;
        uint8_t comandoVelocidad = (uint8_t)velocidadCalc;
        uint8_t comandoGiro = rp;
        
        if (!systemActive) {
             comandoMovimiento = 0;
             comandoVelocidad = 0;
        }

        // Enviar trama al ESP32
        uint8_t statusCalibrado = systemActive ? 1 : 0;
        uint8_t controlPayload[4] = {comandoMovimiento, comandoVelocidad, comandoGiro, statusCalibrado};
        // Durante la calibración web no hace falta inundar BLE con CTRL a 50 Hz.
        // Si el receptor se suscribe después, sí recibirá las notificaciones.
        if (ctrlChar.subscribed()) ctrlChar.writeValue(controlPayload, 4);
        
        digitalWrite(LED_BUILTIN, (now / 500) % 2);
      }
      BLE.poll();
    }

    // Si el usuario sale sin pulsar CAL_DONE, guardamos al perder conexión.
    saveCalibrationIfDirty("disconnect");

  } else {
    BLE.poll();
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
  }
}