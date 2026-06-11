#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int flowSensorPin = 2;
const int valvePin = 3;
const int buttonPlus = 4;
const int buttonMinus = 5;
const int buttonCalibrate = 6; // START
const int buttonStop = 7;
const int buzzerPin = 8;

const int addrImpPerL = 0;
const int addrTankSize = 4;
const int addrValvePol = 8;
const int addrTargetVol = 12;
const int addrDispensed = 16;
const int addrInProgress = 20;
const int addrStep = 24;
const int addrLastFull = 28;

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 0;

float impulsesPerLiter = 450.0;
float tankCapacity = 100.0;
bool valvePolarity = true;

float targetVolume = 0;
float dispensedVolume = 0;
float lastFullTank = 0;
bool inDispenseMode = false;
bool resumedFromPowerLoss = false;

float targetStep = 5.0;
const float leakAlarmThresholdLiters = 5.0;

unsigned long stopPressTime = 0;
bool stopHeld = false;

unsigned long lastDispensePulseCount = 0;
unsigned long lastLeakPulseCount = 0;
unsigned long lastPulseTime = 0;

bool valveOpen = false;

float leakStartLiters = 0.0;
float leakLiters = 0.0;
unsigned long leakFirstPulseTime = 0;
unsigned long leakAlarmTime = 0;
bool leakCounting = false;
bool leakAlarmShown = false;

// Pomiar czasu nalewania
unsigned long dispenseStartTime = 0;
unsigned long lastDispenseDuration = 0;

// Skrót + / -
bool shortcutUsed = false;

// Ochrona EEPROM
unsigned long lastSessionSaveTime = 0;
float lastSavedDispensedVolume = -1.0;
float lastSavedTargetVolume = -1.0;
const unsigned long SESSION_SAVE_INTERVAL = 5000; // ms
const float SESSION_SAVE_DELTA = 0.2;             // L

// Czasy przytrzymań
const unsigned long shortcutHoldTime = 1500;      // + i - razem => +500L
const unsigned long plusHoldShowLastFull = 2000;  // wydłużone
const unsigned long minusHoldShowTime = 2500;     // wydłużone
const unsigned long stopHoldReset = 1500;
const unsigned long calibrateHoldMenu = 1500;

void beep(unsigned int ms = 200);
void beepPattern(unsigned int onMs, unsigned int offMs, byte repeats);
void showVolumeMessage(const char* line1, float volume, unsigned long displayMs = 2500, byte decimals = 1);

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

void resetPulseCount() {
  noInterrupts();
  pulseCount = 0;
  interrupts();
}

void flowISR() {
  unsigned long now = micros();
  if (now - lastPulseMicros > 2000) {
    pulseCount++;
    lastPulseMicros = now;
  }
}

bool isButtonPressedDebounced(int pin, unsigned long debounceMs = 30) {
  if (digitalRead(pin) == LOW) {
    delay(debounceMs);
    return digitalRead(pin) == LOW;
  }
  return false;
}

void beep(unsigned int ms) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
}

void beepPattern(unsigned int onMs, unsigned int offMs, byte repeats) {
  for (byte i = 0; i < repeats; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(onMs);
    digitalWrite(buzzerPin, LOW);
    if (i + 1 < repeats) {
      delay(offMs);
    }
  }
}

void showVolumeMessage(const char* line1, float volume, unsigned long displayMs, byte decimals) {
  lcd.clear();
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(volume, decimals);
  lcd.print(" L");
  delay(displayMs);
  lcd.clear();
}

void resetLeakDetection() {
  unsigned long pulses = getPulseCount();
  leakCounting = false;
  leakAlarmShown = false;
  leakFirstPulseTime = 0;
  leakAlarmTime = 0;
  leakLiters = 0;
  leakStartLiters = pulses / impulsesPerLiter;
  lastLeakPulseCount = pulses;
}

void setValve(bool open) {
  bool previousValveState = valveOpen;
  valveOpen = open;

  if (open) {
    digitalWrite(valvePin, valvePolarity ? HIGH : LOW);
  } else {
    digitalWrite(valvePin, valvePolarity ? LOW : HIGH);
    if (previousValveState == true) {
      resetLeakDetection();
    }
  }
}

void saveSettings() {
  EEPROM.put(addrImpPerL, impulsesPerLiter);
  EEPROM.put(addrTankSize, tankCapacity);
  EEPROM.put(addrValvePol, valvePolarity);
  EEPROM.put(addrStep, targetStep);
}

void loadSettings() {
  EEPROM.get(addrImpPerL, impulsesPerLiter);
  EEPROM.get(addrTankSize, tankCapacity);
  EEPROM.get(addrValvePol, valvePolarity);
  EEPROM.get(addrStep, targetStep);

  if (isnan(impulsesPerLiter) || impulsesPerLiter < 10 || impulsesPerLiter > 10000) impulsesPerLiter = 450.0;
  if (isnan(tankCapacity) || tankCapacity <= 0 || tankCapacity > 10000) tankCapacity = 100.0;
  if (isnan(targetStep) || targetStep < 5 || targetStep > 100) targetStep = 5.0;
}

void saveSessionNow() {
  EEPROM.put(addrTargetVol, targetVolume);
  EEPROM.put(addrDispensed, dispensedVolume);
  EEPROM.put(addrInProgress, true);

  lastSessionSaveTime = millis();
  lastSavedDispensedVolume = dispensedVolume;
  lastSavedTargetVolume = targetVolume;
}

void maybeSaveSession() {
  unsigned long now = millis();
  if (now - lastSessionSaveTime < SESSION_SAVE_INTERVAL) return;

  bool targetChanged = abs(targetVolume - lastSavedTargetVolume) >= 0.01;
  bool dispensedChanged = abs(dispensedVolume - lastSavedDispensedVolume) >= SESSION_SAVE_DELTA;

  if (targetChanged || dispensedChanged) {
    saveSessionNow();
  }
}

void clearSession() {
  bool inProgress = false;
  EEPROM.put(addrInProgress, inProgress);
}

void loadSession() {
  bool wasInProgress = false;
  EEPROM.get(addrInProgress, wasInProgress);
  if (wasInProgress) {
    EEPROM.get(addrTargetVol, targetVolume);
    EEPROM.get(addrDispensed, dispensedVolume);

    if (isnan(targetVolume) || targetVolume < 0 || targetVolume > 10000) targetVolume = 0;
    if (isnan(dispensedVolume) || dispensedVolume < 0 || dispensedVolume > 10000) dispensedVolume = 0;

    resumedFromPowerLoss = true;
  }
}

void saveLastFullTank(float volume) {
  EEPROM.put(addrLastFull, volume);
}

void loadLastFullTank() {
  EEPROM.get(addrLastFull, lastFullTank);
  if (isnan(lastFullTank) || lastFullTank < 0 || lastFullTank > 10000) lastFullTank = 0.0;
}

void showMainScreen() {
  lcd.setCursor(0, 0);
  lcd.print("Nalano: ");
  lcd.print(dispensedVolume, 1);
  lcd.print("L   ");

  lcd.setCursor(0, 1);
  lcd.print("Cel: ");
  lcd.print(targetVolume, 1);
  lcd.print("L ");
  lcd.print(inDispenseMode ? "->>" : "-||-");
  lcd.print("  ");
}

void showLastFullTank() {
  lcd.clear();
  lcd.print("Ostatni zbiornik:");
  lcd.setCursor(0, 1);
  lcd.print(lastFullTank, 2);
  lcd.print(" L");
  delay(3000);
  lcd.clear();
}

void showLastDispenseTime() {
  lcd.clear();
  lcd.print("Czas nalewania:");
  lcd.setCursor(0, 1);

  if (lastDispenseDuration > 0) {
    unsigned long totalSeconds = lastDispenseDuration / 1000;
    unsigned int hours = totalSeconds / 3600;
    unsigned int minutes = (totalSeconds % 3600) / 60;

    if (hours > 0) {
      lcd.print(hours);
      lcd.print("h ");
      if (minutes < 10) lcd.print("0");
      lcd.print(minutes);
      lcd.print("min");
    } else {
      float minF = lastDispenseDuration / 60000.0;
      lcd.print(minF, 2);
      lcd.print(" min");
    }
  } else {
    lcd.print("Brak danych");
  }

  delay(3000);
  lcd.clear();
}

void handlePlusMinusShortcut() {
  static unsigned long bothButtonsHeldTime = 0;

  if (digitalRead(buttonPlus) == LOW && digitalRead(buttonMinus) == LOW) {
    if (bothButtonsHeldTime == 0) {
      bothButtonsHeldTime = millis();
    }

    if (!shortcutUsed && (millis() - bothButtonsHeldTime >= shortcutHoldTime)) {
      beep(400);
      targetVolume += 500.0;
      if (targetVolume > tankCapacity) targetVolume = tankCapacity;

      lcd.clear();
      lcd.print("+500L dodane!");
      delay(1500);

      shortcutUsed = true;
    }
  } else {
    bothButtonsHeldTime = 0;
    shortcutUsed = false;
  }
}

void calibrationMenu() {
  lcd.clear();
  lcd.print("Kalibracja...");
  delay(1000);

  bool done = false;
  int mode = 0;

  while (!done) {
    lcd.clear();

    if (mode == 0) {
      lcd.print("Imp/litr: ");
      lcd.print(impulsesPerLiter, 0);
    } else if (mode == 1) {
      lcd.print("Zbiornik: ");
      lcd.print(tankCapacity, 0);
      lcd.print(" L");
    } else if (mode == 2) {
      lcd.print("Pol. zaworu: ");
      lcd.print(valvePolarity ? "HIGH" : "LOW");
    } else if (mode == 3) {
      lcd.print("Krok celu: ");
      lcd.print(targetStep, 0);
      lcd.print(" L");
    }

    lcd.setCursor(0, 1);
    lcd.print("+/- Zm. Start=OK");

    while (true) {
      if (isButtonPressedDebounced(buttonPlus)) {
        beep();
        if (mode == 0) impulsesPerLiter += 1;
        else if (mode == 1) tankCapacity += 50;
        else if (mode == 2) valvePolarity = true;
        else if (mode == 3 && targetStep < 100) targetStep += 5;
        delay(250);
        break;
      }

      if (isButtonPressedDebounced(buttonMinus)) {
        beep();
        if (mode == 0 && impulsesPerLiter > 10) impulsesPerLiter -= 1;
        else if (mode == 1 && tankCapacity > 50) tankCapacity -= 50;
        else if (mode == 2) valvePolarity = false;
        else if (mode == 3 && targetStep > 5) targetStep -= 5;
        delay(250);
        break;
      }

      if (isButtonPressedDebounced(buttonCalibrate)) {
        beep();
        mode++;
        if (mode > 3) {
          saveSettings();
          done = true;
        }
        delay(300);
        break;
      }
    }
  }

  lcd.clear();
  lcd.print("Zapisano.");
  beep(400);
  delay(1000);
  lcd.clear();
}

void setup() {
  pinMode(valvePin, OUTPUT);
  pinMode(buttonPlus, INPUT_PULLUP);
  pinMode(buttonMinus, INPUT_PULLUP);
  pinMode(buttonCalibrate, INPUT_PULLUP);
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(buzzerPin, LOW);

  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowISR, FALLING);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("-TANK CONTROL-");
  lcd.setCursor(0, 1);
  lcd.print("Hardi Commander");
  beep(100);
  delay(2000);
  lcd.clear();

  setValve(false);

  loadSettings();
  loadSession();
  loadLastFullTank();
  resetLeakDetection();

  if (resumedFromPowerLoss) {
    lcd.clear();
    lcd.print("Wznowic nalewanie?");
    lcd.setCursor(0, 1);
    lcd.print("Start=Tak Stop=Nie");

    unsigned long startTime = millis();
    bool answered = false;

    while (millis() - startTime < 10000) {
      if (isButtonPressedDebounced(buttonStop)) {
        beep(350);
        clearSession();
        targetVolume = 0;
        dispensedVolume = 0;
        resetPulseCount();
        resetLeakDetection();
        delay(500);
        answered = true;
        break;
      }

      if (isButtonPressedDebounced(buttonCalibrate)) {
        beep(200);
        inDispenseMode = false;
        setPulseCount((unsigned long)(dispensedVolume * impulsesPerLiter));
        resetLeakDetection();
        answered = true;
        break;
      }
    }

    if (!answered) {
      clearSession();
      targetVolume = 0;
      dispensedVolume = 0;
      resetPulseCount();
      resetLeakDetection();
      lcd.clear();
      lcd.print("Brak decyzji");
      delay(1000);
    }

    lcd.clear();
    setValve(false);
  }
}

void loop() {
  unsigned long pulsesSnapshot = getPulseCount();
  dispensedVolume = pulsesSnapshot / impulsesPerLiter;

  if (!valveOpen) {
    float currentLiters = pulsesSnapshot / impulsesPerLiter;
    bool pulseChanged = (pulsesSnapshot != lastLeakPulseCount);

    if (pulseChanged) {
      if (!leakCounting) {
        leakCounting = true;
        leakFirstPulseTime = millis();
        leakStartLiters = currentLiters;
        leakAlarmShown = false;
      }

      leakLiters = currentLiters - leakStartLiters;
      if (leakLiters < 0) leakLiters = 0;

      lastLeakPulseCount = pulsesSnapshot;
      leakAlarmTime = millis();

      if (!leakAlarmShown && leakLiters >= leakAlarmThresholdLiters) {
        leakAlarmShown = true;
        lcd.clear();
        lcd.print("ALARM! Przeplyw");
        lcd.setCursor(0, 1);
        lcd.print("przy zamkn. zaw.");

        while (true) {
          digitalWrite(buzzerPin, HIGH);
          delay(180);
          digitalWrite(buzzerPin, LOW);
          if (digitalRead(buttonPlus) == LOW || digitalRead(buttonMinus) == LOW ||
              digitalRead(buttonCalibrate) == LOW || digitalRead(buttonStop) == LOW) {
            delay(300);
            break;
          }
          delay(180);
        }

        lcd.clear();
        lcd.print("Nalano przy zamk:");
        lcd.setCursor(0, 1);
        lcd.print(leakLiters, 2);
        lcd.print(" L");
        delay(5000);
        lcd.clear();
      }
    }

    if (leakCounting && (millis() - leakAlarmTime > 5000)) {
      resetLeakDetection();
    }
  }

  if (!inDispenseMode) {
    showMainScreen();

    handlePlusMinusShortcut();

    if (digitalRead(buttonPlus) == LOW && digitalRead(buttonMinus) == LOW) {
      return;
    }

    if (isButtonPressedDebounced(buttonPlus)) {
      unsigned long pressedAt = millis();
      bool longHandled = false;

      while (digitalRead(buttonPlus) == LOW) {
        if (digitalRead(buttonMinus) == LOW) return;
        if (!longHandled && millis() - pressedAt > plusHoldShowLastFull) {
          beep(200);
          showLastFullTank();
          longHandled = true;
        }
      }

      if (longHandled) return;

      beep();
      targetVolume += targetStep;
      if (targetVolume > tankCapacity) targetVolume = tankCapacity;
      delay(250);
    }

    if (isButtonPressedDebounced(buttonMinus)) {
      unsigned long pressedAt = millis();
      bool longHandled = false;

      while (digitalRead(buttonMinus) == LOW) {
        if (digitalRead(buttonPlus) == LOW) return;
        if (!longHandled && millis() - pressedAt > minusHoldShowTime) {
          beep(200);
          showLastDispenseTime();
          longHandled = true;
        }
      }

      if (longHandled) return;

      beep();
      if (targetVolume > 0) targetVolume -= targetStep;
      if (targetVolume < 0) targetVolume = 0;
      delay(250);
    }

    if (digitalRead(buttonStop) == LOW) {
      if (!stopHeld) {
        stopPressTime = millis();
        stopHeld = true;
      } else if (millis() - stopPressTime > stopHoldReset) {
        beep(300);
        dispensedVolume = 0;
        resetPulseCount();
        resetLeakDetection();
        stopHeld = false;
      }
    } else {
      stopHeld = false;
    }

    if (isButtonPressedDebounced(buttonCalibrate)) {
      unsigned long pressedTime = millis();

      while (digitalRead(buttonCalibrate) == LOW) {
        if (millis() - pressedTime > calibrateHoldMenu) {
          beep(400);
          calibrationMenu();
          return;
        }
      }

      if (targetVolume <= 0.0) {
        lcd.clear();
        lcd.print("Ustaw cel > 0");
        beep(300);
        delay(1200);
        lcd.clear();
        return;
      }

      beep();
      setPulseCount((unsigned long)(dispensedVolume * impulsesPerLiter));
      pulsesSnapshot = getPulseCount();

      inDispenseMode = true;
      setValve(true);
      lastDispensePulseCount = pulsesSnapshot;
      lastPulseTime = millis();
      dispenseStartTime = millis();

      saveSessionNow();
    } else {
      setValve(false);
    }

  } else {
    pulsesSnapshot = getPulseCount();
    dispensedVolume = pulsesSnapshot / impulsesPerLiter;
    showMainScreen();

    if (pulsesSnapshot != lastDispensePulseCount) {
      lastDispensePulseCount = pulsesSnapshot;
      lastPulseTime = millis();
    }

    if (millis() - lastPulseTime > 10000) {
      setValve(false);
      clearSession();

      if (dispenseStartTime > 0) {
        lastDispenseDuration = millis() - dispenseStartTime;
        dispenseStartTime = 0;
      }

      inDispenseMode = false;
      lcd.clear();
      lcd.print("Brak przeplywu!");
      beepPattern(180, 120, 4);
      delay(1200);
      lcd.clear();
      return;
    }

    if (dispensedVolume >= targetVolume) {
      setValve(false);
      clearSession();

      if (dispenseStartTime > 0) {
        lastDispenseDuration = millis() - dispenseStartTime;
        dispenseStartTime = 0;
      }

      inDispenseMode = false;
      beepPattern(120, 100, 3);
      saveLastFullTank(dispensedVolume);
      showVolumeMessage("Koniec nalewania", dispensedVolume, 2500, 1);
    } else {
      maybeSaveSession();
    }

    if (isButtonPressedDebounced(buttonStop)) {
      setValve(false);

      if (dispenseStartTime > 0) {
        lastDispenseDuration = millis() - dispenseStartTime;
        dispenseStartTime = 0;
      }

      inDispenseMode = false;
      saveSessionNow();
      beepPattern(150, 100, 2);
      showVolumeMessage("Zatrzymano recz.", dispensedVolume, 2500, 1);
    }
  }
}
