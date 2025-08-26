#include <HardwareSerial.h>

HardwareSerial LoraSerial(1); // use UART1

#define CT_PIN 35
#define VOLT_PIN 34
#define NUM_SAMPLES 400
#define WINDOW_SIZE 10

// Calibration factors
#define CT_CALIBRATION 12.1
#define VOLT_CALIBRATION 75.0

// Fault Detection Thresholds
#define CURRENT_THRESHOLD 1.6
#define SUPPLY_ON_THRESHOLD 3.5
#define FAULT_CONFIRM_TIME 2000
#define BREAKAGE_CONFIRM_TIME 2000
#define RECONNECT_CONFIRM_TIME 2000
#define RESET_CURRENT_THRESHOLD 0.5

// Fault types
enum FaultType {
  NO_FAULT = 0,
  LINE_TO_LINE = 27,
  LINE_BREAKAGE = 10
};

// System States
enum SystemState {
  STATE_NORMAL,
  STATE_OVER_CURRENT_DETECTED,
  STATE_FAULT_LATCHED,
  STATE_RECONNECT_DETECTED
};

// System State
float currentReadings[WINDOW_SIZE] = {0};
float voltageReadings[WINDOW_SIZE] = {0};
int readIndex = 0;
float filteredCurrent = 0.0;
float filteredVoltage = 0.0;

// Store the exact values that were displayed in serial
float displayedCurrent = 0.0;
float displayedVoltage = 0.0;
FaultType displayedFault = NO_FAULT;

// Power state tracking
bool powerOn = false;
unsigned long powerLostTime = 0;
unsigned long powerRestoredTime = 0;

unsigned long lastTransmitTime = 0;
const unsigned long heartbeatInterval = 5000;
const unsigned long loraCooldown = 150;

// Fault Detection State
SystemState currentState = STATE_NORMAL;
FaultType currentFault = NO_FAULT;
unsigned long stateEntryTime = 0;
float preFaultCurrent = 0.0;

// LoRA optimization
unsigned long lastLoraSendTime = 0;
bool loraSendPending = false;
String pendingLoraMessage = "";

// Track if we need to send immediate fault message
bool immediateFaultSendRequired = false;

void setup() {
  Serial.begin(115200);
  LoraSerial.begin(9600, SERIAL_8N1, 16, 17);

  // Initialize arrays
  for (int i = 0; i < WINDOW_SIZE; i++) {
    currentReadings[i] = 0;
    voltageReadings[i] = 0;
  }

  delay(2000);
  Serial.println("ESP32 Smart Pole Monitor - Serial-LoRa Sync Fixed");
}

String getTimestamp() {
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(timestamp);
}

void sendLoraMessageNow(const String &message) {
  if (millis() - lastLoraSendTime >= loraCooldown) {
    String formattedMessage = "$$" + message + "##\r\n";
    LoraSerial.print(formattedMessage);
    lastLoraSendTime = millis();
    Serial.println("LoRa Sent: " + formattedMessage);
  } else {
    pendingLoraMessage = message;
    loraSendPending = true;
  }
}

void processPendingLoraMessage() {
  if (loraSendPending && (millis() - lastLoraSendTime >= loraCooldown)) {
    String formattedMessage = "$$" + pendingLoraMessage + "##\r\n";
    LoraSerial.print(formattedMessage);
    lastLoraSendTime = millis();
    Serial.println("LoRa Sent (pending): " + formattedMessage);
    loraSendPending = false;
    pendingLoraMessage = "";
  }
}

void prepareLoraMessage() {
  // Use the EXACT same values that were displayed in serial
  String minimalMessage = "P01,S01,";
  
  // For fault conditions, set voltage to 0.0
  if (displayedFault == LINE_TO_LINE || displayedFault == LINE_BREAKAGE) {
    minimalMessage += "0.0,";
  } else {
    minimalMessage += String(round(displayedVoltage * 10) / 10.0, 1) + ",";
  }
  
  minimalMessage += String(round(displayedCurrent * 100) / 100.0, 2) + ",";
  minimalMessage += String(displayedFault);
  
  sendLoraMessageNow(minimalMessage);
}

void takeMeasurement() {
  long sumCT = 0, sumVolt = 0;
  long offsetSumCT = 0, offsetSumVolt = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    offsetSumCT += analogRead(CT_PIN);
    offsetSumVolt += analogRead(VOLT_PIN);
  }
  float offsetCT = offsetSumCT / (float)NUM_SAMPLES;
  float offsetVolt = offsetSumVolt / (float)NUM_SAMPLES;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    float valCT = analogRead(CT_PIN) - offsetCT;
    float valVolt = analogRead(VOLT_PIN) - offsetVolt;

    sumCT += valCT * valCT;
    sumVolt += valVolt * valVolt;
  }

  float Vrms_rawCT = sqrt(sumCT / (float)NUM_SAMPLES) * (3.3 / 4095.0);
  float Irms = Vrms_rawCT * CT_CALIBRATION;

  float Vrms_rawVolt = sqrt(sumVolt / (float)NUM_SAMPLES) * (3.3 / 4095.0);
  float Vrms = Vrms_rawVolt * VOLT_CALIBRATION;

  currentReadings[readIndex] = Irms;
  voltageReadings[readIndex] = Vrms;
  readIndex = (readIndex + 1) % WINDOW_SIZE;

  filteredCurrent = 0;
  filteredVoltage = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filteredCurrent += currentReadings[i];
    filteredVoltage += voltageReadings[i];
  }
  filteredCurrent /= WINDOW_SIZE;
  filteredVoltage /= WINDOW_SIZE;

  // Store the exact values that will be displayed
  displayedCurrent = filteredCurrent;
  displayedVoltage = filteredVoltage;
  displayedFault = currentFault; // This will be updated in checkForFaults()
}

void checkForFaults() {
  bool previousPowerState = powerOn;
  powerOn = (filteredVoltage > SUPPLY_ON_THRESHOLD);

  // 1. POWER LOST DETECTION
  if (!powerOn && previousPowerState) {
    powerLostTime = millis();
    Serial.println("Power lost. Starting breakage timer...");
  }
  
  // 2. POWER RESTORED DETECTION
  if (powerOn && !previousPowerState) {
    powerRestoredTime = millis();
    if (currentFault == LINE_BREAKAGE) {
      currentState = STATE_RECONNECT_DETECTED;
      Serial.println("Power restored. Waiting for reconnection confirmation...");
    }
  }

  // 3. CHECK FOR LINE BREAKAGE (POWER LOSS)
  if (!powerOn && currentFault != LINE_TO_LINE) {
    if (currentState != STATE_FAULT_LATCHED && currentState != STATE_RECONNECT_DETECTED) {
      if ((millis() - powerLostTime) > BREAKAGE_CONFIRM_TIME) {
        currentFault = LINE_BREAKAGE;
        currentState = STATE_FAULT_LATCHED;
        immediateFaultSendRequired = true;
        Serial.println("Line Breakage confirmed! Immediate send required.");
      }
    }
  }

  // 4. CHECK FOR RECONNECTION CONFIRMATION
  if (currentState == STATE_RECONNECT_DETECTED) {
    if (millis() - powerRestoredTime > RECONNECT_CONFIRM_TIME) {
      currentFault = NO_FAULT;
      currentState = STATE_NORMAL;
      immediateFaultSendRequired = true;
      Serial.println("Line reconnection confirmed! Fault cleared.");
    } else if (!powerOn) {
      currentState = STATE_FAULT_LATCHED;
      Serial.println("Power lost again during reconnection confirmation.");
    }
  }

  // 5. CHECK FOR OVERCURRENT (LINE-TO-LINE) FAULT
  if (currentState != STATE_FAULT_LATCHED && currentState != STATE_RECONNECT_DETECTED) {
    switch (currentState) {
      case STATE_NORMAL:
        if (filteredCurrent > CURRENT_THRESHOLD) {
          currentState = STATE_OVER_CURRENT_DETECTED;
          stateEntryTime = millis();
          preFaultCurrent = filteredCurrent;
          Serial.println("Overcurrent detected. Waiting for substation trip...");
        }
        break;

      case STATE_OVER_CURRENT_DETECTED:
        if (filteredCurrent < 0.5) {
          currentFault = LINE_TO_LINE;
          currentState = STATE_FAULT_LATCHED;
          immediateFaultSendRequired = true;
          Serial.println("Substation trip detected! Line-to-Line fault. Immediate send required.");
        } else if (millis() - stateEntryTime > FAULT_CONFIRM_TIME) {
          currentState = STATE_NORMAL;
          preFaultCurrent = 0.0;
          Serial.println("Overcurrent condition ended without trip. Resetting.");
        }
        break;
    }
  }
  
  // 6. GLOBAL RESET LOGIC for LINE_TO_LINE faults
  if (currentState == STATE_FAULT_LATCHED && currentFault == LINE_TO_LINE) {
    if (filteredCurrent > RESET_CURRENT_THRESHOLD) {
      currentFault = NO_FAULT;
      currentState = STATE_NORMAL;
      preFaultCurrent = 0.0;
      immediateFaultSendRequired = true;
      Serial.println("Load current restored. Line-to-Line fault cleared.");
    }
  }

  // Update the displayed fault state AFTER all fault processing
  displayedFault = currentFault;
}

void loop() {
  takeMeasurement();
  
  checkForFaults();

  // Serial print the EXACT values that will be sent
  Serial.print("Current: ");
  Serial.print(displayedCurrent, 2);
  Serial.print(" A | Voltage: ");
  Serial.print(displayedVoltage, 1);
  Serial.print(" V | State: ");
  Serial.print(currentState);
  Serial.print(" | Fault: ");
  Serial.println(displayedFault);

  processPendingLoraMessage();

  // Check if immediate fault message is required
  if (immediateFaultSendRequired) {
    prepareLoraMessage();
    immediateFaultSendRequired = false;
    lastTransmitTime = millis();
  }
  // Periodic heartbeat transmission (5 seconds)
  else if (millis() - lastTransmitTime >= heartbeatInterval) {
    if (currentState == STATE_FAULT_LATCHED) {
      prepareLoraMessage();
    } else if (powerOn && currentState == STATE_NORMAL) {
      prepareLoraMessage();
    }
    lastTransmitTime = millis();
  }

  delay(50);
}