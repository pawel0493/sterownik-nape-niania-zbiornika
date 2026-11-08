#include <EEPROM.h>
#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

// ===== Gotowy pinout: ESP32 Wemos + MCP2515 =====
// MCP2515 (SPI): SCK=GPIO18, MISO=GPIO19, MOSI=GPIO23, CS=GPIO5, INT=GPIO4
// Sterownik: flow=GPIO27, valve=GPIO26, buzzer=GPIO12
const int flowSensorPin = 27;
const int valvePin = 26;
const int buzzerPin = 12;

const int canCsPin = 5;
const int canIntPin = 4;
const int canSckPin = 18;
const int canMisoPin = 19;
const int canMosiPin = 23;

// MCP2515 modules are commonly 8MHz; change to MCP_16MHZ when needed.
const byte canClock = MCP_8MHZ;
MCP_CAN CAN(canCsPin);

// EEPROM layout
const int addrImpPerL = 0;
const int addrTankSize = 4;
const int addrValvePol = 8;
const int addrTargetVol = 12;
const int addrDispensed = 16;
const int addrInProgress = 20;
const int addrStep = 24;
const int addrNoFlowTimeoutSec = 28;

volatile unsigned long pulseCount = 0;

float impulsesPerLiter = 450.0;
float tankCapacity = 100.0;
bool valvePolarity = true;
float targetStep = 5.0;

float targetVolume = 0.0;
float dispensedVolume = 0.0;
bool inDispenseMode = false;
bool valveOpen = false;
bool noFlowAlarm = false;

unsigned long lastPulseTime = 0;
unsigned long lastDispensePulseCount = 0;
unsigned long lastStatusTx = 0;
unsigned long lastConfigTx = 0;

unsigned long lastSessionSaveTime = 0;
float lastSavedDispensedVolume = -1.0;
float lastSavedTargetVolume = -1.0;

const unsigned long SESSION_SAVE_INTERVAL = 5000;
const float SESSION_SAVE_DELTA = 0.2;
unsigned long noFlowTimeoutMs = 10000;
const unsigned long STATUS_TX_MS = 300;
const unsigned long CONFIG_TX_MS = 1000;

const uint8_t ecuAddress = 0x80;

// Pre-ISOBUS test PGNs (proprietary)
const uint32_t PGN_CMD = 0x00EF10;
const uint32_t PGN_STATUS = 0x00EF11;
const uint32_t PGN_CONFIG = 0x00EF12;

enum CommandId : uint8_t {
  CMD_SET_TARGET_L_X10 = 0x01,      // payload[1..2]: target [0.1 L]
  CMD_START = 0x02,
  CMD_STOP = 0x03,
  CMD_RESET_COUNTER = 0x04,
  CMD_SET_STEP_L = 0x05,            // payload[1]: 5..100
  CMD_SET_VALVE_POL = 0x06,         // payload[1]: 0/1
  CMD_SET_IMP_PER_L = 0x07,         // payload[1..2]: 10..10000
  CMD_SET_TANK_CAP_L = 0x08,        // payload[1..2]: 1..10000
  CMD_SET_NO_FLOW_TIMEOUT_S = 0x09, // payload[1]: 1..120
  CMD_SAVE_CONFIG = 0x0A
};

enum StatusFlags : uint8_t {
  ST_VALVE_OPEN = 1 << 0,
  ST_IN_PROGRESS = 1 << 1,
  ST_NO_FLOW = 1 << 2
};

void IRAM_ATTR flowISR() {
  pulseCount++;
}

unsigned long getPulseCount() {
  noInterrupts();
  unsigned long value = pulseCount;
  interrupts();
  return value;
}

void setPulseCount(unsigned long value) {
  noInterrupts();
  pulseCount = value;
  interrupts();
}

void beep(unsigned int ms = 120) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
}

uint32_t buildJ1939Id(uint8_t priority, uint32_t pgn, uint8_t sourceAddr) {
  return ((uint32_t)(priority & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | sourceAddr;
}

void setValve(bool open) {
  valveOpen = open;
  if (open) {
    digitalWrite(valvePin, valvePolarity ? HIGH : LOW);
  } else {
    digitalWrite(valvePin, valvePolarity ? LOW : HIGH);
  }
}

void saveSettings() {
  EEPROM.put(addrImpPerL, impulsesPerLiter);
  EEPROM.put(addrTankSize, tankCapacity);
  EEPROM.put(addrValvePol, valvePolarity);
  EEPROM.put(addrStep, targetStep);
  uint8_t timeoutSec = (uint8_t)fminf(noFlowTimeoutMs / 1000.0f, 120.0f);
  EEPROM.put(addrNoFlowTimeoutSec, timeoutSec);
  EEPROM.commit();
}

void loadSettings() {
  EEPROM.get(addrImpPerL, impulsesPerLiter);
  EEPROM.get(addrTankSize, tankCapacity);
  EEPROM.get(addrValvePol, valvePolarity);
  EEPROM.get(addrStep, targetStep);

  uint8_t timeoutSec = 10;
  EEPROM.get(addrNoFlowTimeoutSec, timeoutSec);

  if (isnan(impulsesPerLiter) || impulsesPerLiter < 10 || impulsesPerLiter > 10000) impulsesPerLiter = 450.0;
  if (isnan(tankCapacity) || tankCapacity <= 0 || tankCapacity > 10000) tankCapacity = 100.0;
  if (isnan(targetStep) || targetStep < 5 || targetStep > 100) targetStep = 5.0;
  if (timeoutSec < 1 || timeoutSec > 120) timeoutSec = 10;
  noFlowTimeoutMs = (unsigned long)timeoutSec * 1000UL;
}

void saveSessionNow() {
  EEPROM.put(addrTargetVol, targetVolume);
  EEPROM.put(addrDispensed, dispensedVolume);
  bool inProgress = true;
  EEPROM.put(addrInProgress, inProgress);
  EEPROM.commit();

  lastSessionSaveTime = millis();
  lastSavedDispensedVolume = dispensedVolume;
  lastSavedTargetVolume = targetVolume;
}

void maybeSaveSession() {
  unsigned long now = millis();
  if (now - lastSessionSaveTime < SESSION_SAVE_INTERVAL) return;

  bool targetChanged = fabsf(targetVolume - lastSavedTargetVolume) >= 0.01f;
  bool dispensedChanged = fabsf(dispensedVolume - lastSavedDispensedVolume) >= SESSION_SAVE_DELTA;
  if (targetChanged || dispensedChanged) {
    saveSessionNow();
  }
}

void clearSession() {
  bool inProgress = false;
  EEPROM.put(addrInProgress, inProgress);
  EEPROM.commit();
}

void loadSession() {
  bool wasInProgress = false;
  EEPROM.get(addrInProgress, wasInProgress);
  if (!wasInProgress) return;

  EEPROM.get(addrTargetVol, targetVolume);
  EEPROM.get(addrDispensed, dispensedVolume);

  if (isnan(targetVolume) || targetVolume < 0 || targetVolume > 10000) targetVolume = 0;
  if (isnan(dispensedVolume) || dispensedVolume < 0 || dispensedVolume > 10000) dispensedVolume = 0;

  setPulseCount((unsigned long)(dispensedVolume * impulsesPerLiter));
}

void startDispense() {
  if (targetVolume <= 0.0f) return;
  inDispenseMode = true;
  noFlowAlarm = false;
  setValve(true);
  lastDispensePulseCount = getPulseCount();
  lastPulseTime = millis();
  saveSessionNow();
}

void stopDispense(bool keepSession) {
  setValve(false);
  inDispenseMode = false;
  if (keepSession) {
    saveSessionNow();
  } else {
    clearSession();
  }
}

void sendStatusFrame() {
  uint8_t data[8] = {0};
  uint16_t disp10 = (uint16_t)fminf(dispensedVolume * 10.0f, 65535.0f);
  uint16_t target10 = (uint16_t)fminf(targetVolume * 10.0f, 65535.0f);

  data[0] = lowByte(disp10);
  data[1] = highByte(disp10);
  data[2] = lowByte(target10);
  data[3] = highByte(target10);
  data[4] = 0;
  if (valveOpen) data[4] |= ST_VALVE_OPEN;
  if (inDispenseMode) data[4] |= ST_IN_PROGRESS;
  if (noFlowAlarm) data[4] |= ST_NO_FLOW;
  data[5] = (uint8_t)targetStep;
  data[6] = 0xFF;
  data[7] = 0xFF;

  uint32_t canId = buildJ1939Id(6, PGN_STATUS, ecuAddress);
  CAN.sendMsgBuf(canId, 1, 8, data);
}

void sendConfigFrame() {
  uint8_t data[8] = {0};
  uint16_t imp = (uint16_t)fminf(impulsesPerLiter, 65535.0f);
  uint16_t cap = (uint16_t)fminf(tankCapacity, 65535.0f);
  uint8_t timeoutSec = (uint8_t)fminf(noFlowTimeoutMs / 1000.0f, 255.0f);

  data[0] = lowByte(imp);
  data[1] = highByte(imp);
  data[2] = lowByte(cap);
  data[3] = highByte(cap);
  data[4] = timeoutSec;
  data[5] = valvePolarity ? 1 : 0;
  data[6] = (uint8_t)targetStep;
  data[7] = 0xFF;

  uint32_t canId = buildJ1939Id(6, PGN_CONFIG, ecuAddress);
  CAN.sendMsgBuf(canId, 1, 8, data);
}

void handleCanCommand(const uint8_t* data, uint8_t len) {
  if (len < 1) return;

  switch (data[0]) {
    case CMD_SET_TARGET_L_X10: {
      if (len < 3) return;
      uint16_t targetX10 = ((uint16_t)data[2] << 8) | data[1];
      targetVolume = targetX10 / 10.0f;
      if (targetVolume > tankCapacity) targetVolume = tankCapacity;
      break;
    }
    case CMD_START:
      if (!inDispenseMode && targetVolume > 0) startDispense();
      break;
    case CMD_STOP:
      if (inDispenseMode) stopDispense(true);
      break;
    case CMD_RESET_COUNTER:
      if (!inDispenseMode) {
        setPulseCount(0);
        dispensedVolume = 0;
        noFlowAlarm = false;
        clearSession();
      }
      break;
    case CMD_SET_STEP_L:
      if (len >= 2 && data[1] >= 5 && data[1] <= 100) {
        targetStep = data[1];
      }
      break;
    case CMD_SET_VALVE_POL:
      if (len >= 2) {
        valvePolarity = (data[1] != 0);
        setValve(false);
      }
      break;
    case CMD_SET_IMP_PER_L: {
      if (len < 3) return;
      uint16_t imp = ((uint16_t)data[2] << 8) | data[1];
      if (imp >= 10 && imp <= 10000) {
        impulsesPerLiter = imp;
      }
      break;
    }
    case CMD_SET_TANK_CAP_L: {
      if (len < 3) return;
      uint16_t cap = ((uint16_t)data[2] << 8) | data[1];
      if (cap >= 1 && cap <= 10000) {
        tankCapacity = cap;
        if (targetVolume > tankCapacity) targetVolume = tankCapacity;
      }
      break;
    }
    case CMD_SET_NO_FLOW_TIMEOUT_S:
      if (len >= 2 && data[1] >= 1 && data[1] <= 120) {
        noFlowTimeoutMs = (unsigned long)data[1] * 1000UL;
      }
      break;
    case CMD_SAVE_CONFIG:
      saveSettings();
      break;
    default:
      break;
  }
}

void pollCan() {
  if (digitalRead(canIntPin) != LOW) return;

  unsigned long id = 0;
  unsigned char len = 0;
  unsigned char data[8] = {0};

  if (CAN.readMsgBuf(&id, &len, data) != CAN_OK) return;

  if (id <= 0x7FF) return;
  uint32_t pgn = (id >> 8) & 0x3FFFF;
  if (pgn == PGN_CMD) {
    handleCanCommand(data, len);
  }
}

void setup() {
  pinMode(valvePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(flowSensorPin, INPUT_PULLUP);
  pinMode(canIntPin, INPUT_PULLUP);

  digitalWrite(buzzerPin, LOW);
  setValve(false);

  attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowISR, FALLING);
  Serial.begin(115200);

  EEPROM.begin(256);
  loadSettings();
  loadSession();

  SPI.begin(canSckPin, canMisoPin, canMosiPin, canCsPin);
  while (CAN.begin(MCP_ANY, CAN_250KBPS, canClock) != CAN_OK) {
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);

  beep(100);
}

void loop() {
  pollCan();

  unsigned long pulsesSnapshot = getPulseCount();
  dispensedVolume = pulsesSnapshot / impulsesPerLiter;

  if (inDispenseMode) {
    if (pulsesSnapshot != lastDispensePulseCount) {
      lastDispensePulseCount = pulsesSnapshot;
      lastPulseTime = millis();
    }

    if (millis() - lastPulseTime > noFlowTimeoutMs) {
      noFlowAlarm = true;
      stopDispense(false);
      beep(220);
    }

    if (dispensedVolume >= targetVolume) {
      stopDispense(false);
      beep(180);
    } else {
      maybeSaveSession();
    }
  }

  if (millis() - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = millis();
    sendStatusFrame();
  }

  if (millis() - lastConfigTx >= CONFIG_TX_MS) {
    lastConfigTx = millis();
    sendConfigFrame();
  }
}
