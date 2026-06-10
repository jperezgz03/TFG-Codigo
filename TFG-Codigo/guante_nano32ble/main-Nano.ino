#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <Wire.h>
#include <NanoBLEFlashPrefs.h>

// Librería del sensor capacitivo de flexión Bend Labs / Nitto.
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

// ===================== CONFIGURACIÓN GENERAL =====================
// Pin de reset hardware del sensor de flexión. Permite reiniciar el sensor
// al arrancar el guante para asegurar una lectura inicial estable.
const int ADS_RST_PIN = 3;

// Parámetros principales del lazo de control.
// El guante trabaja cada 20 ms, es decir, a 50 Hz.
const float EMA_ALPHA_FLEX = 0.20f;
const uint32_t LOOP_MS = 20;

// La web no necesita recibir datos a 50 Hz. Se limita la telemetría de la HMI
// para reducir tráfico BLE y dejar margen a los comandos de calibración.
const uint32_t WEB_NOTIFY_MS = 250;

// Pequeña pausa de telemetría después de recibir un comando. Evita conflictos
// entre notificaciones BLE y el envío del ACK de confirmación.
const uint32_t COMMAND_QUIET_MS = 350;

// ===================== PROTECCIÓN DE LA SEÑAL DE FLEXIÓN =====================
// Se descartan lecturas físicamente no razonables y picos aislados del sensor.
// Esto es importante porque una lectura errónea podría interpretarse como una
// orden brusca de avance.
const float FLEX_MIN_VALID_DEG = -230.0f;
const float FLEX_MAX_VALID_DEG = 50.0f;
const float FLEX_SPIKE_MIN_DEG = 6.0f;
const float FLEX_SPIKE_MAX_DEG = 12.0f;
const uint8_t FLEX_SPIKE_CONFIRM_SAMPLES = 4;

// Limitador de pendiente de la consigna de avance.
// La aceleración se limita para evitar arranques bruscos; la reducción de
// velocidad se permite más rápida por seguridad.
const float FLEX_PCT_INC_SMALL_RANGE = 2.0f;
const float FLEX_PCT_INC_MED_RANGE   = 4.0f;
const float FLEX_PCT_INC_BIG_RANGE   = 6.0f;
const float FLEX_PCT_MAX_DEC         = 15.0f;

// ===================== PERFIL TOURETTE =====================
// En este perfil se validan temporalmente cambios muy bruscos. La idea es no
// aceptar de inmediato un posible tic motor como si fuera una orden voluntaria.
const float TOUR_ROLL_SPIKE_DELTA_DEG = 40.0f;
const uint32_t TOUR_ROLL_CONFIRM_MS   = 1000;
const float TOUR_ROLL_RETURN_DEG      = 10.0f;
const float TOUR_ROLL_NORMAL_ALPHA    = 0.60f;
const float TOUR_ROLL_ACCEPT_ALPHA    = 0.85f;

// Protección equivalente para la flexión: se bloquean subidas bruscas de
// velocidad, pero no se retrasan las bajadas ni la parada.
const float TOUR_FLEX_INC_SPIKE_PCT = 35.0f;
const uint32_t TOUR_FLEX_CONFIRM_MS = 1000;
const float TOUR_FLEX_RETURN_PCT    = 10.0f;

// ===================== PERFILES MÉDICOS DISPONIBLES =====================
// NONE: funcionamiento estándar.
// PARKINSON: aplica filtrado paso bajo para atenuar temblores rápidos.
// TOURETTE: aplica validación temporal frente a tics o movimientos espasmódicos.
enum PathologyType { NONE = 0, PARKINSON = 1, TOURETTE = 2 };

// ===================== DATOS DE CALIBRACIÓN EN MEMORIA FLASH =====================
// Estructura persistente de calibración. Incluye cabecera, versión, tamaño y CRC
// para detectar datos antiguos o corruptos después de un apagado.
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
  uint8_t pathology;
  uint8_t reserved1;
  uint8_t reserved2;

  uint32_t crc;
};

const uint32_t CAL_MAGIC = 0x47564C34; // Identificador de la estructura: "GVL4".
const uint16_t CAL_VERSION = 4;

CalibrationData calData;
NanoBLEFlashPrefs flashPrefs;

// Sensor capacitivo Bend Labs / Nitto One Axis.
ADS flexSensor;
bool flexOK = false;
float lastFlexAngle = 0.0f;

// ===================== VARIABLES DE ESTADO =====================
// systemActive solo se activa al terminar la calibración. Mientras sea falso,
// el guante puede medir y calibrar, pero no ordena movimiento al receptor.
bool systemActive = false;
bool calibrationDirty = false;
bool pendingCalNotify = false;
bool pendingRcalNotify = false;
uint32_t lastCommandMs = 0;
bool bleReady = false;

// Variables filtradas principales: flexión de dedo y giro relativo de muñeca.
float emaAngleFlex = 0.0f;
float emaRollRel = 0.0f;

// Estado interno del rechazo de picos de flexión.
float flexClean = 0.0f;
float flexCandidate = 0.0f;
uint8_t flexCandidateCount = 0;
bool flexCleanInit = false;

// Estado interno del limitador de rampa de avance.
float pctFlexCommandLimited = 0.0f;
bool pctFlexCommandInit = false;

Madgwick ahrs;
bool imuOK = false;

// ===================== UUIDs BLE =====================
// Servicio BLE propio del guante. Cada característica separa una función:
// telemetría, calibración, control, comandos o confirmaciones.
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
// Conversión de datos a formato binario para enviarlos por BLE de forma compacta.
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
  // Filtro dinámico común aplicado al ángulo de giro.
  // Su objetivo es estabilizar la dirección cuando la mano está casi quieta,
  // sin hacer lenta la respuesta cuando el usuario realiza un giro claro.

  targetRoll = clampRoll180(targetRoll);
  currentRoll = clampRoll180(currentRoll);

  float rollDiff = targetRoll - currentRoll;
  float diff = fabsf(rollDiff);

  // Zona de reposo: pequeñas variaciones se consideran ruido o temblor leve.
  if (diff < 2.0f) {
    return currentRoll;
  }

  // Zona intermedia: el filtro se abre de forma progresiva según aumenta
  // la diferencia angular.
  if (diff < 12.0f) {
    const float alphaMin = 0.08f;
    const float alphaMax = 0.45f;
    const float diffMin = 2.0f;
    const float diffMax = 12.0f;

    float alpha = alphaMin + ((diff - diffMin) * (alphaMax - alphaMin) / (diffMax - diffMin));
    alpha = clampf(alpha, alphaMin, alphaMax);

    return clampRoll180(currentRoll + alpha * rollDiff);
  }

  // Movimiento voluntario claro: seguimiento rápido, pero todavía suavizado.
  const float alphaFast = 0.65f;
  return clampRoll180(currentRoll + alphaFast * rollDiff);
}

float calibratedFlexRangeDeg() {
  return fabsf(calData.maxFlex - calData.minFlex);
}

float adaptiveFlexSpikeDeg() {
  // El umbral de pico se adapta al rango de flexión calibrado. Si el usuario
  // tiene poco recorrido útil, pocos grados pueden representar un gran cambio.
  float range = calibratedFlexRangeDeg();
  float th = 0.45f * range;
  th = clampf(th, FLEX_SPIKE_MIN_DEG, FLEX_SPIKE_MAX_DEG);
  return th;
}

float flexMaxIncreasePctPerCycle() {
  float range = calibratedFlexRangeDeg();

  // Cuanto menor es el rango calibrado, más sensible es el sistema. Por eso
  // se reduce la velocidad máxima de subida de la consigna.
  if (range < 15.0f) return FLEX_PCT_INC_SMALL_RANGE;
  if (range < 35.0f) return FLEX_PCT_INC_MED_RANGE;
  return FLEX_PCT_INC_BIG_RANGE;
}

void resetFlexSignalProtection(float currentFlexDeg) {
  // Reinicia los estados internos de protección de flexión con el valor actual.
  flexClean = currentFlexDeg;
  flexCandidate = currentFlexDeg;
  flexCandidateCount = 0;
  flexCleanInit = true;

  // La velocidad enviada al receptor arranca siempre desde cero por seguridad.
  pctFlexCommandLimited = 0.0f;
  pctFlexCommandInit = true;
}

float rejectFlexSpike(float x) {
  // Lecturas no numéricas o fuera de rango se descartan.
  if (!isfinite(x) || x < FLEX_MIN_VALID_DEG || x > FLEX_MAX_VALID_DEG) {
    return flexCleanInit ? flexClean : lastFlexAngle;
  }

  // Primera lectura válida: inicializa la referencia limpia.
  if (!flexCleanInit) {
    resetFlexSignalProtection(x);
    return x;
  }

  float diff = x - flexClean;
  float spikeThreshold = adaptiveFlexSpikeDeg();

  // Cambio razonable: se acepta directamente como movimiento normal.
  if (fabsf(diff) <= spikeThreshold) {
    flexClean = x;
    flexCandidate = x;
    flexCandidateCount = 0;
    return x;
  }

  // Cambio demasiado brusco: se acepta solo si se repite durante varias muestras.
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

  // Pico aislado: se mantiene la última lectura aceptada.
  return flexClean;
}

float limitFlexCommandPercent(float targetPct) {
  // Limita la velocidad de cambio de la consigna de avance normalizada.
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
// Estos filtros no modifican el sensor ni Madgwick. Solo deciden si un cambio
// brusco debe aceptarse inmediatamente o esperar confirmación temporal.
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
    // El Roll se satura entre -180° y 180° para evitar saltos de envolvimiento.
    raw = clampRoll180(raw);
    currentOutput = clampRoll180(currentOutput);

    float diff = raw - currentOutput;
    float absDiff = fabsf(diff);

    // Movimiento normal: se sigue con una ganancia rápida.
    if (!candidateActive && absDiff <= TOUR_ROLL_SPIKE_DELTA_DEG) {
      return clampRoll180(currentOutput + TOUR_ROLL_NORMAL_ALPHA * diff);
    }

    // Salto brusco: se guarda como candidato y no se aplica todavía.
    if (!candidateActive) {
      candidateActive = true;
      anchor = currentOutput;
      candidateStartMs = now;
      candidateSign = (diff >= 0.0f) ? 1 : -1;
      return clampRoll180(currentOutput);
    }

    // Candidato activo: se comprueba si persiste o si vuelve al valor anterior.
    float candidateDiffFromAnchor = raw - anchor;
    float signedDistance = candidateSign * candidateDiffFromAnchor;

    // Si el gesto vuelve cerca de la referencia, se interpreta como tic breve.
    if (signedDistance <= TOUR_ROLL_RETURN_DEG) {
      candidateActive = false;
      candidateSign = 0;
      return clampRoll180(currentOutput);
    }

    // Si el cambio se mantiene el tiempo definido, se acepta como voluntario.
    if (now - candidateStartMs >= TOUR_ROLL_CONFIRM_MS) {
      candidateActive = false;
      candidateSign = 0;

      float acceptDiff = raw - currentOutput;
      return clampRoll180(currentOutput + TOUR_ROLL_ACCEPT_ALPHA * acceptDiff);
    }

    // Mientras no se confirme, la salida permanece congelada.
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

    // Seguridad: reducir velocidad o parar se acepta sin retardo.
    if (delta <= 0.0f) {
      candidateActive = false;
      return targetPct;
    }

    // Subida pequeña: se considera compatible con un gesto voluntario normal.
    if (!candidateActive && delta <= TOUR_FLEX_INC_SPIKE_PCT) {
      return targetPct;
    }

    // Subida brusca: queda pendiente de confirmación temporal.
    if (!candidateActive) {
      candidateActive = true;
      anchorPct = currentCommandPct;
      candidateStartMs = now;
      return currentCommandPct;
    }

    // Si vuelve cerca del punto inicial, se descarta como evento breve.
    if (targetPct <= anchorPct + TOUR_FLEX_RETURN_PCT) {
      candidateActive = false;
      return targetPct;
    }

    // Si persiste, se acepta como intención de avance real.
    if (now - candidateStartMs >= TOUR_FLEX_CONFIRM_MS) {
      candidateActive = false;
      return targetPct;
    }

    // Mientras no se confirme, se mantiene la consigna previa.
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
// Filtro IIR Butterworth de 2º orden usado en el perfil Parkinson.
// Con fs = 50 Hz y fc = 2 Hz, atenúa temblores rápidos y conserva movimientos
// voluntarios más lentos.
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
// CRC32 usado para comprobar que los datos guardados en Flash son válidos.
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

  // Valores iniciales seguros hasta que el usuario realice la calibración.
  // El sensor puede entregar ángulos negativos, por lo que se validan después.
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
  // Comprueba estructura, versión, tamaño, CRC y rangos básicos.
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
  // Guarda la calibración en Flash y verifica inmediatamente la lectura.
  // Solo se considera correcto si el dato queda escrito y validado.
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.size = sizeof(CalibrationData);
  calData.crc = calcCalibrationCrc(calData);

  Serial.print("FLASH SAVE START: ");
  Serial.println(reason);

  // Se reduce temporalmente la actividad BLE para no interferir con la escritura.
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

    // Verificación inmediata: evita confirmar a la web una calibración no guardada.
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
  // Respuesta a la web: ACK si el comando se procesó correctamente, ERR si falló.
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

    // Si no hay datos válidos, se cargan valores por defecto y se guardan.
    setDefaultCalibration();
    calibrationDirty = true;
    saveCalibrationNow("defaults");
  }
}

// ===================== LECTURA DE SENSORES =====================
void resetFlexSensorHardware() {
  // nRST es activo a nivel bajo. Se fuerza un reset físico del sensor antes
  // de iniciar la comunicación I2C.
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

  // Se espera una primera muestra válida para arrancar los filtros sin salto.
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
  // Si hay una muestra nueva, se actualiza. Si no, se conserva la última válida.
  if (flexOK && flexSensor.available()) {
    lastFlexAngle = flexSensor.getX();
  }

  return lastFlexAngle;
}

uint8_t flexAngleToPercent(float val) {
  // Convierte el ángulo calibrado de flexión en porcentaje de avance 0-100 %.
  float mn = min(calData.minFlex, calData.maxFlex);
  float mx = max(calData.minFlex, calData.maxFlex);
  val = clampf(val, mn, mx);
  float rng = max(1.0f, mx - mn);
  
  float pct = (calData.maxFlex > calData.minFlex) ? ((val - mn) * 100.0f / rng) : ((mx - val) * 100.0f / rng);
  return (uint8_t)clampi(roundf(pct), 0, 100);
}

uint8_t angleToPercent(float currentAngle) {
  // Convierte el Roll calibrado de la muñeca en dirección 0-100 %.
  // El 50 % representa avance recto.
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
  // Envía a la web los dos límites guardados de flexión.
  uint8_t buf[8];
  f32_to_le(calData.minFlex, buf + 0);
  f32_to_le(calData.maxFlex, buf + 4);
  calChar.writeValue(buf, 8);
}

void notifyRCAL() {
  // Envía a la web los tres puntos de calibración de giro en décimas de grado.
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

// ===================== COMANDOS RECIBIDOS DESDE LA WEB =====================
// Cada comando modifica calibración, perfil o estado. Si cambia un parámetro
// crítico, se guarda en Flash y se notifica a la web.
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
    // Al cambiar de perfil se reinician filtros para evitar transitorios.
    butterFlexPark.reset(emaAngleFlex);
    butterRollPark.reset(emaRollRel);
    resetTouretteGates();
    mustSave = true;
  }
  else if (cmd == "PAT_PARK") {
    calData.pathology = PARKINSON;
    // El Butterworth arranca desde el valor actual para no provocar saltos.
    butterFlexPark.reset(emaAngleFlex);
    butterRollPark.reset(emaRollRel);
    resetTouretteGates();
    mustSave = true;
  }
  else if (cmd == "PAT_TOUR") {
    calData.pathology = TOURETTE;
    // Se limpian estados internos de validación temporal.
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
    // A partir de este comando se permite enviar órdenes de movimiento.
    // La velocidad arranca desde cero, aunque la mano ya esté cerrada.
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
  // Lee el comando BLE, lo separa de su número de secuencia y responde con ACK.
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

  // Actualiza las características BLE para que la web lea siempre el último dato.
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
  Wire.setClock(100000);
  pinMode(LED_BUILTIN, OUTPUT);

  // Inicialización del sensor de flexión capacitivo.
  flexOK = beginFlexSensor();

  // Carga la calibración guardada o valores por defecto si no es válida.
  loadCalibration();
  
  // Inicialización de IMU y filtro Madgwick para estimar orientación.
  imuOK = IMU.begin();
  ahrs.begin(200.0f);
  ahrs.beta = 10.0f;

  // Si BLE no arranca, se deja el LED parpadeando como indicación de error.
  if (!BLE.begin()) {
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
  }

  bleReady = true;

  BLE.setLocalName("Guante-Cal-V4");
  BLE.setDeviceName("Guante-Cal-V4");
  BLE.setAdvertisedService(gloveService);

  // Registro de todas las características dentro del servicio BLE del guante.
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
  
  // Inicialización de la flexión filtrada y de las protecciones asociadas.
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

  // El guante queda anunciándose para que la web o el receptor puedan conectarse.
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
        // 1. Lectura de sensores
        // =========================================================
        // Se lee la flexión y se obtiene el Roll de la IMU. La flexión pasa
        // primero por una etapa de rechazo de picos aislados.
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
        // 2. Procesamiento según perfil seleccionado
        // =========================================================
        // Cada perfil modifica la señal de forma distinta. Después, todos pasan
        // por una estabilización común del giro para mejorar la conducción recta.
        float rollTarget = clampRoll180(rawRoll);

        if (calData.pathology == PARKINSON) {
            // Perfil Parkinson: filtro Butterworth paso bajo para atenuar
            // oscilaciones rápidas asociadas al temblor.
            emaAngleFlex = butterFlexPark.update(rawFlex);
            rollTarget   = clampRoll180(butterRollPark.update(rawRoll));

        } else if (calData.pathology == TOURETTE) {
            // Perfil Tourette: mantiene una respuesta rápida, pero valida
            // temporalmente cambios bruscos para evitar aceptar tics instantáneos.
            emaAngleFlex = (EMA_ALPHA_FLEX * rawFlex) + ((1.0f - EMA_ALPHA_FLEX) * emaAngleFlex);

            rollTarget = clampRoll180(touretteRollGate.update(rawRoll, emaRollRel, now));

        } else {
            // Perfil estándar: EMA para flexión y estabilización dinámica para giro.
            emaAngleFlex = (EMA_ALPHA_FLEX * rawFlex) + ((1.0f - EMA_ALPHA_FLEX) * emaAngleFlex);
            rollTarget = clampRoll180(rawRoll);
        }

        // Filtro común de giro. Reduce pequeñas oscilaciones alrededor del centro
        // sin penalizar movimientos voluntarios amplios.
        emaRollRel = applyRollDynamicStability(rollTarget, emaRollRel);

        // =========================================================
        // 3. Normalización, BLE y cálculo de consignas de control
        // =========================================================
        uint8_t pctFlexTarget = flexAngleToPercent(emaAngleFlex);

        float pctFlexForControl = (float)pctFlexTarget;
        if (calData.pathology == TOURETTE) {
          // En Tourette se filtran solo subidas bruscas de velocidad.
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

        // Pausa breve de telemetría tras comandos de la web para favorecer el ACK.
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
          // Envío de telemetría a la web solo si la característica está suscrita.
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

        // Zona muerta de avance: evita que pequeños porcentajes muevan el robot.
        int flexInput = pctFlexCommand;
        if (flexInput < 8) flexInput = 0;
        
        int velocidadCalc = clampi(map(flexInput, 0, 100, 0, 255), 0, 255);
        
        uint8_t comandoMovimiento = (velocidadCalc > 0) ? 1 : 0;
        uint8_t comandoVelocidad = (uint8_t)velocidadCalc;
        uint8_t comandoGiro = rp;
        
        // Mientras el sistema no esté activado, se fuerza orden de parada.
        if (!systemActive) {
             comandoMovimiento = 0;
             comandoVelocidad = 0;
        }

        // Trama de control enviada al receptor ESP32:
        // [movimiento, velocidad, giro, estado_calibrado]
        uint8_t statusCalibrado = systemActive ? 1 : 0;
        uint8_t controlPayload[4] = {comandoMovimiento, comandoVelocidad, comandoGiro, statusCalibrado};
        if (ctrlChar.subscribed()) ctrlChar.writeValue(controlPayload, 4);
        
        // LED de vida del sistema durante conexión BLE.
        digitalWrite(LED_BUILTIN, (now / 500) % 2);
      }
      BLE.poll();
    }

    // Si se pierde la conexión, se guarda cualquier calibración pendiente.
    saveCalibrationIfDirty("disconnect");

  } else {
    BLE.poll();
    // Parpadeo lento cuando no hay central BLE conectada.
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
  }
}
