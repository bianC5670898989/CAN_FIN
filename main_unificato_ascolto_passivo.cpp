#include <Arduino.h>
#include <math.h>

extern "C" {
  #include <driver/twai.h>
  #include <hal/twai_types.h>
}

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// =====================================================
// CODICE UNIFICATO PULITO PER PLATFORMIO / ARDUINO
// AtomS3R + M5Stack Mini CAN Unit + Hitec MDB961WP-CAN
//
// Mantiene:
// - Console registri Hitec OLD Normal Packet
// - Movimento servo in gradi
// - Lettura/scrittura registri raw
// - Help generale e spiegazione risposta
//
// Versione con ascolto passivo CAN non bloccante.
// Rimangono rimossi i comandi rapidi 1/2/3 e il report diagnostico TWAI del primo file.
// =====================================================

// =====================================================
// PIN ATOMS3R + MINI CAN UNIT
// =====================================================
// AtomS3R Grove: GND, 5V, G2, G1
// Mini CAN Unit:
//   CAN_TX Mini CAN -> GPIO2 AtomS3R
//   CAN_RX Mini CAN -> GPIO1 AtomS3R
//
// Se non ricevi nulla, controlla fisicamente CANH/CANL e alimentazione.
// Invertire TX/RX solo se il tuo cablaggio reale e diverso.
// =====================================================
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_2;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_1;

// =====================================================
// CONFIGURAZIONE CAN / SERVO
// =====================================================
static constexpr uint32_t CAN_BITRATE = 1000000;
static constexpr uint32_t RX_TIMEOUT_MS = 500;

// Default della console completa. Se il tuo servo usa SERVO_ID 0x01,
// scrivi da seriale: id 1
static constexpr uint8_t DEFAULT_SERVO_ID = 0x00;
static constexpr uint32_t DEFAULT_CAN_FRAME_ID = 0x000;

static uint8_t servoId = DEFAULT_SERVO_ID;
static uint32_t canFrameId = DEFAULT_CAN_FRAME_ID;
static bool canInstalled = false;

// Ascolto passivo CAN:
// - true  = stampa nel Serial Monitor ogni frame CAN ricevuto quando la console e libera
// - false = ascolto passivo spento, ma i comandi Hitec continuano a ricevere le risposte
static bool passiveListenEnabled = true;
static constexpr uint8_t PASSIVE_LISTEN_MAX_FRAMES_PER_LOOP = 8;
// =====================================================
// REGISTRI UTILI
// =====================================================
static constexpr uint8_t REG_POSITION_NOW = 0x0C;
static constexpr uint8_t REG_VOLTAGE = 0x12;
static constexpr uint8_t REG_POSITION_NEW = 0x1E;

static constexpr uint8_t REG_MOTOR_TEMP = 0xD0;
static constexpr uint8_t REG_TEMP = 0xD2;
static constexpr uint8_t REG_HUM = 0xD4;

// =====================================================
// CONVERSIONE POSIZIONE IN GRADI UTENTE
// =====================================================
// Scala servo Hitec:
//   4096 raw = 90 gradi
// Centro calibrato:
//   deg 0    -> raw 8425
//   deg 90   -> raw 12521
//   deg -90  -> raw 4329
// =====================================================
static constexpr float SERVO_RAW_PER_DEGREE = 4096.0f / 90.0f;
static constexpr float SERVO_RAW_PER_RADIAN = 8192.0f / PI;

static constexpr int32_t SERVO_RAW_MIN = 0;
static constexpr int32_t SERVO_RAW_MAX = 16383;
static constexpr int32_t SERVO_CENTER_RAW = 8425;

// =====================================================
// UTILITY PARSING / FORMATTAZIONE
// =====================================================
static bool parseNumber(const String &s, int32_t &out) {
  char *endPtr = nullptr;
  const char *cstr = s.c_str();

  long value = strtol(cstr, &endPtr, 0);

  if (endPtr == cstr || *endPtr != '\0') {
    return false;
  }

  out = static_cast<int32_t>(value);
  return true;
}

static bool parseFloatNumber(const String &s, float &out) {
  char *endPtr = nullptr;
  const char *cstr = s.c_str();

  float value = strtof(cstr, &endPtr);

  if (endPtr == cstr || *endPtr != '\0') {
    return false;
  }

  out = value;
  return true;
}

static float degreesToRadians(float degrees) {
  return degrees * PI / 180.0f;
}

static uint16_t userDegreesToServoRaw(float userDegrees) {
  float radians = degreesToRadians(userDegrees);

  int32_t raw = static_cast<int32_t>(
    lroundf(SERVO_CENTER_RAW + (radians * SERVO_RAW_PER_RADIAN))
  );

  if (raw < SERVO_RAW_MIN) {
    raw = SERVO_RAW_MIN;
  }

  if (raw > SERVO_RAW_MAX) {
    raw = SERVO_RAW_MAX;
  }

  return static_cast<uint16_t>(raw);
}

static float servoRawToUserDegrees(uint16_t raw) {
  return (static_cast<int32_t>(raw) - SERVO_CENTER_RAW) / SERVO_RAW_PER_DEGREE;
}

static float decodeMotorTempC(uint16_t raw) {
  float data = static_cast<float>(raw);
  float vt = 3.3f / 4096.0f * data;

  if (vt <= 0.0f || vt >= 3.3f) {
    return NAN;
  }

  float rt = (10.0f * vt) / (3.3f - vt);
  float t0 = 298.15f;

  return 1007747.0f /
         ((logf(rt) * t0) - (logf(10.0f) * t0) + 3380.0f)
         - 273.15f;
}

static float decodeInternalTempC(uint16_t raw) {
  return 175.72f * static_cast<float>(raw) / 65536.0f - 46.85f;
}

static float decodeHumidityPercent(uint16_t raw) {
  return 125.0f * static_cast<float>(raw) / 65536.0f - 6.0f;
}

static void printHex2(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

static void printFrame(const char *prefix, const twai_message_t &msg) {
  Serial.print(prefix);
  Serial.print(" CAN_ID=0x");
  Serial.print(msg.identifier, HEX);
  Serial.print(msg.extd ? " EXT" : " STD");
  Serial.print(msg.rtr ? " RTR" : " DATA");
  Serial.print(" DLC=");
  Serial.print(msg.data_length_code);
  Serial.print(" BYTES=");

  for (uint8_t i = 0; i < msg.data_length_code; i++) {
    printHex2(msg.data[i]);
    Serial.print(' ');
  }

  Serial.println();
}


// =====================================================
// ASCOLTO PASSIVO CAN NON BLOCCANTE
// =====================================================
static void printPassiveFrame(const twai_message_t &msg) {
  if (msg.rtr) {
    printFrame("RX_PASSIVE_RTR", msg);
    return;
  }

  if (msg.data_length_code > 0 && msg.data[0] == 0x69) {
    printFrame("RX_PASSIVE_HITEC_REPLY", msg);
    return;
  }

  if (msg.data_length_code > 0 && msg.data[0] == 0x96) {
    printFrame("RX_PASSIVE_HITEC_CMD", msg);
    return;
  }

  printFrame("RX_PASSIVE", msg);
}

static void servicePassiveCanListening() {
  if (!canInstalled || !passiveListenEnabled) {
    return;
  }

  twai_message_t rx = {};
  uint8_t framesRead = 0;

  while (framesRead < PASSIVE_LISTEN_MAX_FRAMES_PER_LOOP &&
         twai_receive(&rx, 0) == ESP_OK) {
    printPassiveFrame(rx);
    framesRead++;
  }
}


// =====================================================
// CHECKSUM HITEC OLD
// =====================================================
static uint8_t checksumSum(const uint8_t *data, uint8_t length) {
  uint16_t sum = 0;

  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }

  return static_cast<uint8_t>(sum & 0xFF);
}

static uint8_t calcTxChecksum(const uint8_t *data, uint8_t length) {
  return checksumSum(data, length);
}

// =====================================================
// PULIZIA CODA RX
// =====================================================
static void clearRxQueue(bool verbose) {
  if (!canInstalled) {
    return;
  }

  twai_message_t rx = {};

  while (twai_receive(&rx, 0) == ESP_OK) {
    if (verbose) {
      printFrame("RX_OLD", rx);
    }
  }
}

// =====================================================
// INIZIALIZZAZIONE CAN
// =====================================================
static bool setupCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  generalConfig.tx_queue_len = 10;
  generalConfig.rx_queue_len = 20;

  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(
    &generalConfig,
    &timingConfig,
    &filterConfig
  );

  if (err != ESP_OK) {
    Serial.print("twai_driver_install fallito: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  err = twai_start();

  if (err != ESP_OK) {
    Serial.print("twai_start fallito: ");
    Serial.println(esp_err_to_name(err));
    twai_driver_uninstall();
    return false;
  }

  canInstalled = true;

  Serial.println();
  Serial.println("CAN inizializzato");
  Serial.print("TX GPIO: ");
  Serial.println((int)CAN_TX_PIN);
  Serial.print("RX GPIO: ");
  Serial.println((int)CAN_RX_PIN);
  Serial.print("Bitrate: ");
  Serial.println(CAN_BITRATE);
  Serial.print("CAN frame ID: 0x");
  Serial.println(canFrameId, HEX);
  Serial.print("Servo ID target: ");
  Serial.println(servoId);
  Serial.print("Ascolto passivo: ");
  Serial.println(passiveListenEnabled ? "ON" : "OFF");
  Serial.println();

  return true;
}

// =====================================================
// INVIO CAN CON DEBUG
// =====================================================
static bool sendCanData(const uint8_t *data, uint8_t len) {
  if (!canInstalled) {
    Serial.println("CAN non inizializzato");
    return false;
  }

  if (len > 8) {
    Serial.println("Errore: frame CAN maggiore di 8 byte");
    return false;
  }

  twai_message_t msg = {};
  msg.identifier = canFrameId;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;

  for (uint8_t i = 0; i < len; i++) {
    msg.data[i] = data[i];
  }

  printFrame("TX", msg);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(200));

  if (err != ESP_OK) {
    Serial.print("Errore TX CAN: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  delay(20);
  return true;
}

// =====================================================
// RISPOSTA HITEC OLD NORMAL PACKET CON DEBUG
// =====================================================
static bool waitHitecResponse(uint8_t address, uint16_t &value, uint8_t &returnId) {
  const uint32_t start = millis();
  bool sawAnyFrame = false;

  while ((millis() - start) < RX_TIMEOUT_MS) {
    twai_message_t rx = {};

    esp_err_t err = twai_receive(&rx, pdMS_TO_TICKS(20));

    if (err != ESP_OK) {
      continue;
    }

    sawAnyFrame = true;
    printFrame("RX", rx);

    if (rx.rtr) {
      continue;
    }

    if (rx.data_length_code < 7) {
      continue;
    }

    if (rx.data[0] != 0x69) {
      continue;
    }

    if (rx.data[2] != address) {
      Serial.println("Frame ricevuto, ma indirizzo registro diverso");
      continue;
    }

    if (rx.data[3] != 0x02) {
      Serial.println("Frame ricevuto, ma length diversa da 0x02");
      continue;
    }

    uint8_t rxChecksum = checksumSum(&rx.data[1], 5);

    if (rxChecksum != rx.data[6]) {
      Serial.println("Checksum risposta non valido");
      Serial.print("Checksum calcolato: 0x");
      Serial.println(rxChecksum, HEX);
      Serial.print("Checksum ricevuto: 0x");
      Serial.println(rx.data[6], HEX);
      continue;
    }

    returnId = rx.data[1];

    value = static_cast<uint16_t>(rx.data[4]) |
            (static_cast<uint16_t>(rx.data[5]) << 8);

    Serial.println("Risposta valida: formato OLD");
    return true;
  }

  if (!sawAnyFrame) {
    Serial.println("Nessun frame CAN ricevuto nel timeout");
  } else {
    Serial.println("Frame CAN ricevuti, ma nessuna risposta Hitec valida");
  }

  return false;
}

// =====================================================
// LETTURA REGISTRO HITEC OLD CON DEBUG
// =====================================================
static bool hitecReadRegister(uint8_t address, uint16_t &value, uint8_t &returnId) {
  clearRxQueue(true);

  uint8_t length = 0x00;

  uint8_t checksumData[3] = {
    servoId,
    address,
    length
  };

  uint8_t packet[5] = {
    0x96,
    servoId,
    address,
    length,
    calcTxChecksum(checksumData, sizeof(checksumData))
  };

  if (!sendCanData(packet, sizeof(packet))) {
    return false;
  }

  return waitHitecResponse(address, value, returnId);
}

// =====================================================
// SCRITTURA REGISTRO HITEC OLD CON DEBUG
// =====================================================
static bool hitecWriteRegister(uint8_t address, uint16_t value) {
  clearRxQueue(true);

  uint8_t length = 0x02;
  uint8_t dataLow = static_cast<uint8_t>(value & 0xFF);
  uint8_t dataHigh = static_cast<uint8_t>((value >> 8) & 0xFF);

  uint8_t checksumData[5] = {
    servoId,
    address,
    length,
    dataLow,
    dataHigh
  };

  uint8_t packet[7] = {
    0x96,
    servoId,
    address,
    length,
    dataLow,
    dataHigh,
    calcTxChecksum(checksumData, sizeof(checksumData))
  };

  return sendCanData(packet, sizeof(packet));
}

// =====================================================
// LETTURA SILENZIOSA PER HELP LIVE
// =====================================================
static bool hitecReadRegisterQuiet(uint8_t address, uint16_t &value, uint8_t &returnId) {
  if (!canInstalled) {
    return false;
  }

  clearRxQueue(false);

  uint8_t length = 0x00;

  uint8_t checksumData[3] = {
    servoId,
    address,
    length
  };

  uint8_t packet[5] = {
    0x96,
    servoId,
    address,
    length,
    calcTxChecksum(checksumData, sizeof(checksumData))
  };

  twai_message_t tx = {};
  tx.identifier = canFrameId;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = sizeof(packet);

  for (uint8_t i = 0; i < sizeof(packet); i++) {
    tx.data[i] = packet[i];
  }

  esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(200));

  if (err != ESP_OK) {
    return false;
  }

  const uint32_t start = millis();

  while ((millis() - start) < RX_TIMEOUT_MS) {
    twai_message_t rx = {};

    err = twai_receive(&rx, pdMS_TO_TICKS(20));

    if (err != ESP_OK) {
      continue;
    }

    if (rx.rtr) {
      continue;
    }

    if (rx.data_length_code < 7) {
      continue;
    }

    if (rx.data[0] != 0x69) {
      continue;
    }

    if (rx.data[2] != address) {
      continue;
    }

    if (rx.data[3] != 0x02) {
      continue;
    }

    uint8_t rxChecksum = checksumSum(&rx.data[1], 5);

    if (rxChecksum != rx.data[6]) {
      continue;
    }

    returnId = rx.data[1];

    value = static_cast<uint16_t>(rx.data[4]) |
            (static_cast<uint16_t>(rx.data[5]) << 8);

    return true;
  }

  return false;
}

// =====================================================
// MOVIMENTO SERVO IN GRADI UTENTE
// =====================================================
static void moveServoDegrees(float degrees) {
  uint16_t rawTarget = userDegreesToServoRaw(degrees);

  Serial.println();
  Serial.println("COMANDO POSIZIONE IN GRADI");
  Serial.println("---------------------------");
  Serial.print("Gradi richiesti:        ");
  Serial.print(degrees, 2);
  Serial.println(" deg");

  Serial.print("Radianti calcolati:     ");
  Serial.println(degreesToRadians(degrees), 6);

  Serial.print("Raw target servo:       ");
  Serial.print(rawTarget);
  Serial.print(" / 0x");
  Serial.println(rawTarget, HEX);
  Serial.println("---------------------------");

  bool writeOk = hitecWriteRegister(REG_POSITION_NEW, rawTarget);

  if (!writeOk) {
    Serial.println("Errore durante invio comando posizione");
    Serial.println();
    return;
  }

  delay(100);

  uint16_t readbackTarget = 0;
  uint8_t returnId = 0;

  bool readTargetOk = hitecReadRegister(REG_POSITION_NEW, readbackTarget, returnId);

  if (readTargetOk) {
    Serial.println();
    Serial.println("---------------------------");
    Serial.println("Readback REG_POSITION_NEW:");
    Serial.print("Raw target servo:       ");
    Serial.print(readbackTarget);
    Serial.print(" / 0x");
    Serial.println(readbackTarget, HEX);
    Serial.print("Gradi richiesti:        ");
    Serial.print(servoRawToUserDegrees(readbackTarget), 2);
    Serial.println(" deg");
    Serial.println("---------------------------");
    Serial.println();
  } else {
    Serial.println("Readback REG_POSITION_NEW non ricevuto");
  }

  delay(100);

  uint16_t actualPosition = 0;
  bool readPositionOk = hitecReadRegister(REG_POSITION_NOW, actualPosition, returnId);

  if (readPositionOk) {
    Serial.println("---------------------------");
    Serial.println("REG_POSITION attuale:");
    Serial.print("Raw target servo:       ");
    Serial.print(actualPosition);
    Serial.print(" / 0x");
    Serial.println(actualPosition, HEX);
    Serial.print("Gradi letti:            ");
    Serial.print(servoRawToUserDegrees(actualPosition), 2);
    Serial.println(" deg");
  } else {
    Serial.println("REG_POSITION attuale non letto");
  }

  Serial.println("---------------------------");
  Serial.println();
}

// =====================================================
// INTERPRETAZIONE REGISTRI UTILI
// =====================================================
static void printDecodedHint(uint8_t address, uint16_t value) {
  Serial.println();

  switch (address) {
    case 0x0C:
      Serial.print("REG_POSITION: ");
      Serial.print(value);
      Serial.print(" raw = ");
      Serial.print(servoRawToUserDegrees(value), 2);
      Serial.println(" gradi utente, con 0 gradi = raw 8425");
      break;

    case 0x0E:
      Serial.print("REG_VELOCITY: ");
      Serial.print(value);
      Serial.println(" raw");
      break;

    case 0x10:
      Serial.print("REG_TORQUE: ");
      Serial.print((value * 100.0f) / 4095.0f, 1);
      Serial.println(" % duty circa");
      break;

    case 0x12:
      Serial.print("REG_VOLTAGE: ");
      Serial.print(value * 0.01f, 2);
      Serial.println(" V");
      break;

    case 0x14:
      Serial.print("REG_MCU_TEMPER: ");
      Serial.print((int16_t)value);
      Serial.println(" C");
      break;

    case 0x16:
      Serial.print("REG_CURRENT: ");
      Serial.print(value);
      Serial.println(" mA");
      break;

    case 0x18:
      Serial.print("REG_TURN_COUNT: ");
      Serial.print((int16_t)value);
      Serial.println(" giri");
      break;

    case 0x1E:
      Serial.print("REG_POSITION_NEW: ");
      Serial.print(value);
      Serial.print(" raw = ");
      Serial.print(servoRawToUserDegrees(value), 2);
      Serial.println(" gradi utente, con 0 gradi = raw 8425");
      break;

    case 0x32:
      Serial.print("REG_ID: ");
      Serial.println(value);
      break;

    case 0x38:
      Serial.print("REG_CAN_BAUDRATE: ");

      switch (value) {
        case 0: Serial.println("1000 kbps"); break;
        case 1: Serial.println("800 kbps"); break;
        case 2: Serial.println("750 kbps"); break;
        case 3: Serial.println("500 kbps"); break;
        case 4: Serial.println("400 kbps"); break;
        case 5: Serial.println("250 kbps"); break;
        case 6: Serial.println("200 kbps"); break;
        case 7: Serial.println("150 kbps"); break;
        case 8: Serial.println("125 kbps"); break;
        default: Serial.println("valore non standard"); break;
      }

      break;

    case 0x48:
      Serial.println("REG_EMERGENCY_STOP:");

      if (value == 0) {
        Serial.println("  Nessun errore");
      } else {
        if (value & (1 << 8))  Serial.println("  Errore posizione minima");
        if (value & (1 << 9))  Serial.println("  Errore posizione massima");
        if (value & (1 << 10)) Serial.println("  Temperatura MCU troppo bassa");
        if (value & (1 << 11)) Serial.println("  Temperatura MCU troppo alta");
        if (value & (1 << 13)) Serial.println("  Tensione troppo bassa");
        if (value & (1 << 14)) Serial.println("  Tensione troppo alta");
      }

      break;

    case 0x54:
      Serial.print("REG_VELOCITY_MAX: ");
      Serial.println(value);
      break;

    case 0x56:
      Serial.print("REG_TORQUE_MAX: ");
      Serial.print((value * 100.0f) / 4095.0f, 1);
      Serial.println(" % circa");
      break;

    case 0x58:
      Serial.print("REG_VOLTAGE_MAX: ");
      Serial.print(value * 0.01f, 2);
      Serial.println(" V");
      break;

    case 0x5A:
      Serial.print("REG_VOLTAGE_MIN: ");
      Serial.print(value * 0.01f, 2);
      Serial.println(" V");
      break;

    case 0x5C:
      Serial.print("REG_TEMPER_MAX: ");
      Serial.print(value);
      Serial.println(" C");
      break;

    case 0x6A:
      Serial.print("REG_CAN_MODE: ");

      if (value == 0) {
        Serial.println("CAN 2.0A");
      } else if (value == 1) {
        Serial.println("CAN 2.0B");
      } else if (value == 2) {
        Serial.println("DroneCAN");
      } else {
        Serial.println("valore sconosciuto");
      }

      break;

    case 0x74:
      Serial.print("REG_PRODUCT_NO: ");
      Serial.println(value);
      break;

    case 0xD0: {
      float motorTemp = decodeMotorTempC(value);

      Serial.print("REG_MOTOR_TEMP: ");
      Serial.print(value);
      Serial.print(" raw = ");

      if (isnan(motorTemp)) {
        Serial.println("conversione non valida");
      } else {
        Serial.print(motorTemp, 2);
        Serial.println(" C");
      }

      break;
    }

    case 0xD2: {
      float internalTemp = decodeInternalTempC(value);

      Serial.print("REG_TEMP: ");
      Serial.print(value);
      Serial.print(" raw = ");
      Serial.print(internalTemp, 2);
      Serial.println(" C temperatura interna servo");
      break;
    }

    case 0xD4: {
      float humidity = decodeHumidityPercent(value);

      Serial.print("REG_HUM: ");
      Serial.print(value);
      Serial.print(" raw = ");
      Serial.print(humidity, 2);
      Serial.println(" %RH umidita interna servo");
      break;
    }

    case 0xD8:
      Serial.print("REG_CURRENT_MAX: ");
      Serial.print(value);
      Serial.println(" mA");
      break;

    case 0xFC:
      Serial.print("REG_VERSION: ");
      Serial.println(value);
      break;

    default:
      break;
  }

  Serial.println();
}

// =====================================================
// HELP LIVE
// =====================================================
static const char *baudrateToText(uint16_t value) {
  switch (value) {
    case 0: return "1000 kbps";
    case 1: return "800 kbps";
    case 2: return "750 kbps";
    case 3: return "500 kbps";
    case 4: return "400 kbps";
    case 5: return "250 kbps";
    case 6: return "200 kbps";
    case 7: return "150 kbps";
    case 8: return "125 kbps";
    default: return "valore non standard";
  }
}

static const char *canModeToText(uint16_t value) {
  switch (value) {
    case 0: return "CAN 2.0A";
    case 1: return "CAN 2.0B";
    case 2: return "DroneCAN";
    default: return "valore sconosciuto";
  }
}

static const char *canFrameTypeToText(uint16_t canMode) {
  switch (canMode) {
    case 0: return "standard";
    case 1: return "extended";
    case 2: return "DroneCAN";
    default: return "sconosciuto";
  }
}

static void printHexFrameId(uint32_t frameId) {
  Serial.print("0x");

  if (frameId <= 0x7FF) {
    if (frameId < 0x100) Serial.print('0');
    if (frameId < 0x010) Serial.print('0');
    Serial.print(frameId, HEX);
  } else {
    if (frameId < 0x10000000) Serial.print('0');
    if (frameId < 0x01000000) Serial.print('0');
    if (frameId < 0x00100000) Serial.print('0');
    if (frameId < 0x00010000) Serial.print('0');
    if (frameId < 0x00001000) Serial.print('0');
    if (frameId < 0x00000100) Serial.print('0');
    if (frameId < 0x00000010) Serial.print('0');
    Serial.print(frameId, HEX);
  }
}

static void printServoCharacteristicsLive() {
  uint16_t productNo = 0;
  uint16_t firmwareVersion = 0;
  uint16_t idValue = 0;
  uint16_t baudValue = 0;
  uint16_t modeValue = 0;
  uint16_t canBusIdH = 0;
  uint16_t canBusIdL = 0;

  uint8_t returnId = 0;


  bool okVersion = hitecReadRegisterQuiet(0xFC, firmwareVersion, returnId);
  bool okProduct = hitecReadRegisterQuiet(0x74, productNo, returnId);
  bool okId = hitecReadRegisterQuiet(0x32, idValue, returnId);
  bool okBaud = hitecReadRegisterQuiet(0x38, baudValue, returnId);
  bool okMode = hitecReadRegisterQuiet(0x6A, modeValue, returnId);
  bool okCanIdH = hitecReadRegisterQuiet(0x3C, canBusIdH, returnId);
  bool okCanIdL = hitecReadRegisterQuiet(0x3E, canBusIdL, returnId);


  Serial.println("CARATTERISTICHE SERVO LETTE LIVE");
  Serial.println("-----------------------------------------------------");
  Serial.println("Modello presunto:        Hitec MDB961WP-CAN");

  Serial.print("Product number:         ");
  if (okProduct) {
    Serial.print(productNo);
    Serial.print(" / 0x");
    Serial.println(productNo, HEX);
  } else {
    Serial.println("non letto");
  }

  Serial.print("Firmware/version raw:   ");
  if (okVersion) {
    Serial.print(firmwareVersion);
    Serial.print(" / 0x");
    Serial.println(firmwareVersion, HEX);
  } else {
    Serial.println("non letto");
  }

  Serial.print("Servo ID letto:         ");
  if (okId) {
    Serial.println(idValue);
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN baudrate:           ");
  if (okBaud) {
    Serial.println(baudrateToText(baudValue));
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN baudrate raw:       ");
  if (okBaud) {
    Serial.println(baudValue);
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN mode:               ");
  if (okMode) {
    Serial.println(canModeToText(modeValue));
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN mode raw:           ");
  if (okMode) {
    Serial.println(modeValue);
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN frame ID servo:     ");
  if (okCanIdH && okCanIdL) {
    uint32_t frameId = (static_cast<uint32_t>(canBusIdH) << 16) |
                       static_cast<uint32_t>(canBusIdL);

    printHexFrameId(frameId);
    Serial.print(' ');

    if (okMode) {
      Serial.println(canFrameTypeToText(modeValue));
    } else {
      Serial.println("tipo non letto");
    }
  } else {
    Serial.println("non letto da REG_CAN_BUS_ID_H/L");
  }

  Serial.print("CAN Bus ID H raw:       ");
  if (okCanIdH) {
    Serial.print(canBusIdH);
    Serial.print(" / 0x");
    Serial.println(canBusIdH, HEX);
  } else {
    Serial.println("non letto");
  }

  Serial.print("CAN Bus ID L raw:       ");
  if (okCanIdL) {
    Serial.print(canBusIdL);
    Serial.print(" / 0x");
    Serial.println(canBusIdL, HEX);
  } else {
    Serial.println("non letto");
  }

  Serial.println("Protocollo risposta:    OLD Normal Packet");
  Serial.println("Header comando:         0x96");
  Serial.println("Header risposta:        0x69");
  Serial.println("Endian dati:            little-endian");
  Serial.println();
}

static void printHelpRisposta() {
  Serial.println();
  Serial.println("=====================================================");
  Serial.println("SPIEGAZIONE RISPOSTA HITEC OLD NORMAL PACKET");
  Serial.println("=====================================================");
  Serial.println();

  Serial.println("Formato risposta lettura registro:");
  Serial.println("  BYTES = 69 ID REG LEN DL DH CHECKSUM");
  Serial.println();

  Serial.println("Significato dei byte:");
  Serial.println("-----------------------------------------------------");
  Serial.println("  69       = header risposta Hitec OLD");
  Serial.println("  ID       = ID del servo che ha risposto");
  Serial.println("  REG      = indirizzo del registro letto");
  Serial.println("  LEN      = lunghezza dati, normalmente 0x02");
  Serial.println("  DL       = data low byte");
  Serial.println("  DH       = data high byte");
  Serial.println("  CHECKSUM = controllo somma");
  Serial.println();

  Serial.println("Esempio:");
  Serial.println("  RX CAN_ID=0x0 STD DATA DLC=7 BYTES=69 00 12 02 C7 04 DF");
  Serial.println();
  Serial.println("Interpretazione:");
  Serial.println("  69       = header risposta Hitec OLD");
  Serial.println("  00       = ID servo risposta");
  Serial.println("  12       = registro 0x12, REG_VOLTAGE");
  Serial.println("  02       = lunghezza dati, 2 byte");
  Serial.println("  C7 04    = valore little-endian = 0x04C7 = 1223 decimale");
  Serial.println("           = 12.23 V per REG_VOLTAGE");
  Serial.println("  DF       = checksum");
  Serial.println();

  Serial.println("Calcolo valore:");
  Serial.println("  valore = DL + (DH << 8)");
  Serial.println();

  Serial.println("Calcolo checksum risposta:");
  Serial.println("  checksum = ID + REG + LEN + DL + DH");
  Serial.println("  si tiene solo il byte basso del risultato");
  Serial.println();

  Serial.println("Esempi registri utili:");
  Serial.println("  0x0C = REG_POSITION");
  Serial.println("  0x12 = REG_VOLTAGE");
  Serial.println("  0x14 = REG_MCU_TEMPER");
  Serial.println("  0x16 = REG_CURRENT");
  Serial.println("  0x32 = REG_ID");
  Serial.println("  0x38 = REG_CAN_BAUDRATE");
  Serial.println("  0x6A = REG_CAN_MODE");
  Serial.println("  0x74 = REG_PRODUCT_NO");
  Serial.println("  0xD0 = REG_MOTOR_TEMP");
  Serial.println("  0xD2 = REG_TEMP");
  Serial.println("  0xD4 = REG_HUM");
  Serial.println("  0xFC = REG_VERSION");
  Serial.println();

  Serial.println("Conversioni frequenti:");
  Serial.println("  REG_VOLTAGE:      valore * 0.01 V");
  Serial.println("  REG_POSITION:     4096 raw = 90 gradi");
  Serial.println("  REG_MOTOR_TEMP:   formula NTC Hitec");
  Serial.println("  REG_TEMP:         175.72 * raw / 65536 - 46.85 C");
  Serial.println("  REG_HUM:          125 * raw / 65536 - 6 %RH");
  Serial.println();
  Serial.println("=====================================================");
  Serial.println();
}

static void printHelp(bool includeLiveRead) {
  Serial.println();
  Serial.println("=====================================================");
  Serial.println("Hitec MDB961WP-CAN Register Console");
  Serial.println("AtomS3R + M5Stack Mini CAN Unit");
  Serial.println("Protocollo: OLD Normal Packet");
  Serial.println("CAN: 1 Mbps, CAN 2.0A Standard");
  Serial.println("=====================================================");
  Serial.println();

  if (includeLiveRead) {
    printServoCharacteristicsLive();
  } else {
    Serial.println("Scrivi help per leggere live le caratteristiche del servo.");
    Serial.println();
  }

  Serial.println("COMANDI DISPONIBILI");
  Serial.println("-----------------------------------------------------");
  Serial.println("  <addr>                 Legge registro raw, es. 0x12");
  Serial.println("  <addr> <value>         Scrive registro raw e poi rilegge");
  Serial.println("  deg <gradi>            Muove il servo usando gradi utente");
  Serial.println("  id <0..255>            Imposta ID servo target");
  Serial.println("  canid <hex/dec>        Imposta CAN arbitration ID");
  Serial.println("  ascolto on/off         Attiva/disattiva ascolto passivo CAN");
  Serial.println("  listen on/off          Alias di ascolto on/off");
  Serial.println("  help                   Legge il servo e mostra aiuto generale");
  Serial.println("  help risposta          Spiega il significato dei BYTES ricevuti");
  Serial.println();

  Serial.println("ESEMPI LETTURA");
  Serial.println("-----------------------------------------------------");
  Serial.println("  0x12                   REG_VOLTAGE");
  Serial.println("  0x14                   REG_MCU_TEMPER");
  Serial.println("  0x0C                   REG_POSITION");
  Serial.println("  0x0E                   REG_VELOCITY");
  Serial.println("  0x10                   REG_TORQUE");
  Serial.println("  0x16                   REG_CURRENT");
  Serial.println("  0x18                   REG_TURN_COUNT");
  Serial.println("  0x1E                   REG_POSITION_NEW");
  Serial.println("  0x48                   REG_EMERGENCY_STOP");
  Serial.println("  0x74                   REG_PRODUCT_NO");
  Serial.println("  0xD0                   REG_MOTOR_TEMP");
  Serial.println("  0xD2                   REG_TEMP temperatura interna");
  Serial.println("  0xD4                   REG_HUM umidita interna");
  Serial.println("  0xFC                   REG_VERSION");
  Serial.println("  0x32                   REG_ID");
  Serial.println("  0x38                   REG_CAN_BAUDRATE");
  Serial.println("  0x6A                   REG_CAN_MODE");
  Serial.println();

  Serial.println("ESEMPI SCRITTURA RAW");
  Serial.println("-----------------------------------------------------");
  Serial.println("  0x1E 8425              REG_POSITION_NEW raw, centro calibrato");
  Serial.println("  0x1E 12521             REG_POSITION_NEW raw, circa +90 gradi");
  Serial.println("  0x1E 4329              REG_POSITION_NEW raw, circa -90 gradi");
  Serial.println("  0x54 1000              REG_VELOCITY_MAX");
  Serial.println("  0x56 2048              REG_TORQUE_MAX circa 50%");
  Serial.println("  0x58 1400              REG_VOLTAGE_MAX = 14.00 V");
  Serial.println("  0x5A 1000              REG_VOLTAGE_MIN = 10.00 V");
  Serial.println("  0x70 0xFFFF            Salva configurazione");
  Serial.println();

  Serial.println("ESEMPI POSIZIONE IN GRADI");
  Serial.println("-----------------------------------------------------");
  Serial.println("  deg 0                  Centro calibrato, raw 8425");
  Serial.println("  deg 90                 +90 gradi rispetto al centro, raw 12521");
  Serial.println("  deg -90                -90 gradi rispetto al centro, raw 4329");
  Serial.println("  deg 45.5               Posizione con decimali");
  Serial.println();

  Serial.println("CONFIGURAZIONE ATTUALE");
  Serial.println("-----------------------------------------------------");
  Serial.print("Servo ID target:        ");
  Serial.println(servoId);
  Serial.print("CAN frame ID:           0x");
  Serial.println(canFrameId, HEX);
  Serial.print("CAN bitrate:            ");
  Serial.println(CAN_BITRATE);
  Serial.print("CAN TX GPIO:            ");
  Serial.println((int)CAN_TX_PIN);
  Serial.print("CAN RX GPIO:            ");
  Serial.println((int)CAN_RX_PIN);
  Serial.print("Ascolto passivo:        ");
  Serial.println(passiveListenEnabled ? "ON" : "OFF");
  Serial.println();

  Serial.println("=====================================================");
  Serial.println();
}

// =====================================================
// TOKENIZER SERIALE
// =====================================================
static uint8_t tokenize(String line, String tokens[], uint8_t maxTokens) {
  line.trim();
  line.replace(",", " ");
  line.replace(";", " ");
  line.replace("\t", " ");

  uint8_t count = 0;

  while (line.length() > 0 && count < maxTokens) {
    line.trim();

    int spaceIndex = line.indexOf(' ');

    if (spaceIndex < 0) {
      tokens[count++] = line;
      break;
    }

    String token = line.substring(0, spaceIndex);
    token.trim();

    if (token.length() > 0) {
      tokens[count++] = token;
    }

    line = line.substring(spaceIndex + 1);
  }

  return count;
}

// =====================================================
// LETTURA REGISTRO CON STAMPA
// =====================================================
static void readAndPrintRegister(uint8_t address) {
  uint16_t value = 0;
  uint8_t returnId = 0;

  Serial.println();
  Serial.print("Richiesta lettura registro 0x");
  Serial.println(address, HEX);

  bool ok = hitecReadRegister(address, value, returnId);

  if (!ok) {
    Serial.println();
    Serial.println("ERRORE: nessuna risposta valida dal servo");
    Serial.println("Controlla alimentazione, CANH/CANL, terminazione, CAN ID e checksum.");
    Serial.println();
    return;
  }

  Serial.println();
  Serial.print("OK ID=");
  Serial.print(returnId);
  Serial.print(" REG=0x");
  Serial.print(address, HEX);
  Serial.print(" RAW=");
  Serial.print(value);
  Serial.print(" 00x");
  Serial.println(value, HEX);

  printDecodedHint(address, value);
}


// =====================================================
// GESTIONE COMANDI SERIALE
// =====================================================
static void handleSerialLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("#")) {
    return;
  }


  String tokens[4];
  uint8_t n = tokenize(line, tokens, 4);

  if (n == 0) {
    return;
  }

  String cmd = tokens[0];
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") {
    if (n >= 2) {
      String subcmd = tokens[1];
      subcmd.toLowerCase();

      if (subcmd == "risposta") {
        printHelpRisposta();
        return;
      }
    }

    printHelp(true);
    return;
  }

  if (cmd == "id") {
    if (n < 2) {
      Serial.println("Errore: usa id <0..255>");
      return;
    }

    int32_t v = 0;

    if (!parseNumber(tokens[1], v) || v < 0 || v > 255) {
      Serial.println("Errore: ID servo non valido");
      return;
    }

    servoId = static_cast<uint8_t>(v);

    Serial.print("Servo ID target = ");
    Serial.println(servoId);
    return;
  }

  if (cmd == "canid") {
    if (n < 2) {
      Serial.println("Errore: usa canid <valore>");
      return;
    }

    int32_t v = 0;

    if (!parseNumber(tokens[1], v) || v < 0) {
      Serial.println("Errore: CAN ID non valido");
      return;
    }

    canFrameId = static_cast<uint32_t>(v);

    Serial.print("CAN frame ID = 0x");
    Serial.println(canFrameId, HEX);
    return;
  }

  if (cmd == "ascolto" || cmd == "listen" || cmd == "passivo") {
    if (n < 2) {
      Serial.print("Ascolto passivo CAN = ");
      Serial.println(passiveListenEnabled ? "ON" : "OFF");
      Serial.println("Usa: ascolto on oppure ascolto off");
      return;
    }

    String mode = tokens[1];
    mode.toLowerCase();

    if (mode == "on" || mode == "1" || mode == "true") {
      passiveListenEnabled = true;
      Serial.println("Ascolto passivo CAN = ON");
      return;
    }

    if (mode == "off" || mode == "0" || mode == "false") {
      passiveListenEnabled = false;
      Serial.println("Ascolto passivo CAN = OFF");
      return;
    }

    Serial.println("Errore: usa ascolto on oppure ascolto off");
    return;
  }

  if (cmd == "deg" || cmd == "gradi" || cmd == "posdeg") {
    if (n < 2) {
      Serial.println("Errore: usa deg <gradi>");
      Serial.println("Esempi:");
      Serial.println("  deg 0");
      Serial.println("  deg 90");
      Serial.println("  deg -90");
      Serial.println("  deg 45.5");
      return;
    }

    float degrees = 0.0f;

    if (!parseFloatNumber(tokens[1], degrees)) {
      Serial.println("Errore: valore gradi non valido");
      return;
    }

    if (degrees < -180.0f || degrees > 180.0f) {
      Serial.println("Attenzione: valore fuori range consigliato -180..+180 gradi");
      Serial.println("Il codice limitera automaticamente il raw tra 0 e 16383");
    }

    moveServoDegrees(degrees);
    return;
  }

  // ===================================================
  // LETTURA/SCRITTURA REGISTRO RAW
  // ===================================================
  int32_t addrParsed = 0;

  if (!parseNumber(tokens[0], addrParsed) || addrParsed < 0 || addrParsed > 255) {
    Serial.println("Errore: indirizzo registro non valido");
    return;
  }

  uint8_t address = static_cast<uint8_t>(addrParsed);

  if ((address & 0x01) != 0) {
    Serial.println("Attenzione: molti registri Hitec hanno indirizzo pari.");
  }

  // LETTURA RAW
  if (n == 1) {
    readAndPrintRegister(address);
    return;
  }

  // SCRITTURA RAW
  if (n >= 2) {
    int32_t valueParsed = 0;

    if (!parseNumber(tokens[1], valueParsed)) {
      Serial.println("Errore: valore non valido");
      return;
    }

    if (valueParsed < -32768 || valueParsed > 65535) {
      Serial.println("Errore: valore fuori range. Usa -32768..65535");
      return;
    }

    uint16_t valueToWrite = static_cast<uint16_t>(valueParsed);

    Serial.println();
    Serial.print("Richiesta scrittura registro raw 0x");
    Serial.print(address, HEX);
    Serial.print(" = ");
    Serial.print(valueParsed);
    Serial.print(" / 0x");
    Serial.println(valueToWrite, HEX);

    bool writeOk = hitecWriteRegister(address, valueToWrite);

    if (!writeOk) {
      Serial.println("Errore durante invio scrittura");
      return;
    }

    delay(80);

    uint16_t readback = 0;
    uint8_t returnId = 0;

    bool readOk = hitecReadRegister(address, readback, returnId);

    if (!readOk) {
      Serial.println();
      Serial.println("Scrittura inviata, ma readback non ricevuto");
      Serial.println("Possibile registro write-only, protetto o non supportato.");
      Serial.println();
      return;
    }

    Serial.println();
    Serial.print("READBACK ID=");
    Serial.print(returnId);
    Serial.print(" REG=0x");
    Serial.print(address, HEX);
    Serial.print(" RAW=");
    Serial.print(readback);
    Serial.print(" HEX=0x");
    Serial.println(readback, HEX);

    printDecodedHint(address, readback);

    if (readback == valueToWrite) {
      Serial.println("Verifica OK");
    } else {
      Serial.println("Verifica diversa: il registro puo essere read-only, limitato o scalato dal firmware");
    }

    return;
  }
}

// =====================================================
// LETTURA SERIALE NON BLOCCANTE
// =====================================================
static void serviceSerialConsole() {
  static String lineBuffer;
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      String completed = lineBuffer;
      lineBuffer = "";
      handleSerialLine(completed);
      continue;
    }

    if (lineBuffer.length() < 96) {
      lineBuffer += c;
    } else {
      Serial.println("Errore: comando troppo lungo, buffer svuotato");
      lineBuffer = "";
    }
  }

}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    delay(10);
  }
  delay(500);

  Serial.println();
  Serial.println("=====================================================");
  Serial.println("ATOM S3R - Hitec CAN Console + Ascolto Passivo");
  Serial.println("=====================================================");

  if (!setupCan()) {
    Serial.println("Errore inizializzazione CAN");

    while (true) {
      delay(1000);
    }
  }

  printHelp(false);
}

void loop() {
  serviceSerialConsole();
  servicePassiveCanListening();
  delay(1);
}
