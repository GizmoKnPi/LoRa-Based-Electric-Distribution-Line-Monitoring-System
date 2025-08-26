#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// WiFi and MQTT Configuration
const char* ssid = "Your Wifi SSID";
const char* password = "Your WiFi Password";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_status_topic = "scada/grid/substation1/status (Create a Topic)";
const char* mqtt_recharge_topic = "scada/grid/substation1/recharge (Create a Topic)";
const char* mqtt_pole_topic = "scada/grid/pole1/ (Create a Topic)"; 

// LoRa Serial for pole communication - NEW
HardwareSerial PoleLoraSerial(2); // use UART2 on RX2/TX2 pins
#define POLESERIAL_RX 16  // RX2 pin
#define POLESERIAL_TX 17  // TX2 pin

WiFiClient espClient;
PubSubClient client(espClient);

// EXACTLY THE SAME AS YOUR ORIGINAL CODE
#define CT1_PIN 34          // CT1 on D34 (Live)
#define CT2_PIN 35          // CT2 on D35 (Neutral)
#define RELAY1_PIN 5        // Relay 1 on D5 (active low)
#define RELAY2_PIN 18       // Relay 2 on D18 (active low)
#define LED_PIN 2           // Built-in LED
#define THRESHOLD 1.8       // Overcurrent trip level in amps (1.8A)
#define NUM_SAMPLES 400     // Number of ADC samples per measurement
#define DEBOUNCE_DELAY 200  // Delay to prevent inrush current false triggering (ms)

#define CT_CALIBRATION 12.1 // CT calibration factor for 1.6A load

// Low-pass filter factor (0 < alpha <= 1)
float alpha = 0.2;
float filteredCurrent = 0.0;
float filteredNeutral = 0.0;

bool relayTripped = false;       // Track if relays are tripped
unsigned long overCurrentStart = 0; // Timer for overcurrent condition
unsigned long lastTransmitTime = 0;
const unsigned long transmitInterval = 1000; // MQTT transmit every 1 second

// NEW: LoRa message handling variables
String loraBuffer = "";
unsigned long lastLoraCheckTime = 0;
const unsigned long loraCheckInterval = 50; // Check LoRa every 50ms

// Fault types
enum FaultType {
  NO_FAULT = 0,
  LINE_TO_LINE = 27,
  LINE_TO_GROUND = 1
};

FaultType currentFault = NO_FAULT;

// FUNCTION PROTOTYPE - ADD THIS LINE
void sendMQTTMessage(bool isFault = false, FaultType faultType = NO_FAULT);

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert the payload to a string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // Check if the message is received on the recharge topic
  if (String(topic) == mqtt_recharge_topic) {
    if (message == "r" || message == "R") {
      // Reset relays (active low)
      digitalWrite(RELAY1_PIN, LOW);
      digitalWrite(RELAY2_PIN, LOW);
      relayTripped = false;
      overCurrentStart = 0;
      resetFilter(); // Reset filter values
      Serial.println("Relays reset via MQTT (circuit closed). Filter values reset.");
      sendMQTTMessage(false); // Send reset status
      delay(100); // Short delay to allow stable reading after reset
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to the recharge topic
      client.subscribe(mqtt_recharge_topic);
      Serial.println("Subscribed to: " + String(mqtt_recharge_topic));
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

String getTimestamp() {
  // Generate a simple timestamp
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

void sendMQTTMessage(bool isFault, FaultType faultType) {
  StaticJsonDocument<200> doc;
  
  doc["substation_id"] = "S01";
  doc["voltage"] = 12; // Always send 12V as requested
  doc["current"] = filteredCurrent;
  
  if (isFault) {
    doc["fault_code"] = faultType;
    if (faultType == LINE_TO_LINE) {
      doc["fault_type"] = "Line to Line";
    } else if (faultType == LINE_TO_GROUND) {
      doc["fault_type"] = "Line to Ground";
    }
    doc["status"] = "CRITICAL";
    doc["breaker_status"] = "OPEN";
  } else {
    doc["fault_code"] = 0;
    doc["fault_type"] = "NIL";
    doc["status"] = "OK";
    doc["breaker_status"] = "CLOSED";
  }
  
  doc["timestamp"] = getTimestamp();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Publish to MQTT status topic
  client.publish(mqtt_status_topic, jsonString.c_str());
  Serial.println("MQTT Sent to status topic: " + jsonString);
}

// NEW: Function to handle LoRa messages from pole
void handlePoleLoraMessages() {
  while (PoleLoraSerial.available()) {
    char c = PoleLoraSerial.read();
    
    // Add character to buffer
    loraBuffer += c;
    
    // Check if we have a complete message (ended with newline)
    if (c == '\n' || c == '\r') {
      loraBuffer.trim(); // Remove whitespace and line endings
      
      if (loraBuffer.length() > 0) {
        // Forward the raw message to MQTT
        if (client.connected()) {
          client.publish(mqtt_pole_topic, loraBuffer.c_str());
          Serial.println("Pole LoRa forwarded to MQTT: " + loraBuffer);
        } else {
          Serial.println("MQTT not connected, cannot forward Pole LoRa message");
        }
        loraBuffer = ""; // Clear buffer
      }
    }
    
    // Prevent buffer overflow
    if (loraBuffer.length() > 256) {
      Serial.println("LoRa buffer overflow, clearing buffer");
      loraBuffer = "";
    }
  }
}

void resetFilter() {
  // Reset filter values to prevent immediate re-triggering after reset
  filteredCurrent = 0.0;
  filteredNeutral = 0.0;
  currentFault = NO_FAULT;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  
  // Initialize relays as active low (HIGH = off, LOW = on)
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  
  Serial.begin(115200);
  
  // NEW: Initialize Pole LoRa Serial
  PoleLoraSerial.begin(9600, SERIAL_8N1, POLESERIAL_RX, POLESERIAL_TX);
  Serial.println("Pole LoRa serial initialized on RX2/TX2");
  
  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Set MQTT callback function
  
  delay(1000);
  
  // Turn on relays (active low)
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  Serial.println("System started. Relays activated (circuit closed).");
}

void loop() {
  // Reconnect MQTT if needed
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Process incoming MQTT messages
  
  // NEW: Non-blocking LoRa message checking
  if (millis() - lastLoraCheckTime >= loraCheckInterval) {
    handlePoleLoraMessages();
    lastLoraCheckTime = millis();
  }

  // Check for serial command to reset relays
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'r' || command == 'R') {
      // Reset relays (active low)
      digitalWrite(RELAY1_PIN, LOW);
      digitalWrite(RELAY2_PIN, LOW);
      relayTripped = false;
      overCurrentStart = 0;
      resetFilter(); // Reset filter values
      Serial.println("Relays reset via Serial (circuit closed). Filter values reset.");
      sendMQTTMessage(false); // Send reset status
      delay(100); // Short delay to allow stable reading after reset
    }
  }

  // If relays are tripped, skip current measurement but handle MQTT and LoRa
  if (relayTripped) {
    digitalWrite(LED_PIN, HIGH); // LED indicates fault condition
    
    // Send MQTT status periodically
    if (millis() - lastTransmitTime >= transmitInterval) {
      sendMQTTMessage(true, currentFault);
      lastTransmitTime = millis();
    }
    
    // NEW: Still check LoRa even when tripped
    if (millis() - lastLoraCheckTime >= loraCheckInterval) {
      handlePoleLoraMessages();
      lastLoraCheckTime = millis();
    }
    
    delay(50);
    return;
  }

  // --- EXACTLY YOUR ORIGINAL CURRENT MEASUREMENT CODE ---
  long sumCT1 = 0, sumCT2 = 0;
  long offsetSumCT1 = 0, offsetSumCT2 = 0;

  // --- 1) Calculate DC offsets ---
  for (int i = 0; i < NUM_SAMPLES; i++) {
    offsetSumCT1 += analogRead(CT1_PIN);
    offsetSumCT2 += analogRead(CT2_PIN);
  }
  float offsetCT1 = offsetSumCT1 / (float)NUM_SAMPLES;
  float offsetCT2 = offsetSumCT2 / (float)NUM_SAMPLES;

  // --- 2) RMS calculation ---
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float valCT1 = analogRead(CT1_PIN) - offsetCT1;
    float valCT2 = analogRead(CT2_PIN) - offsetCT2;

    sumCT1 += valCT1 * valCT1;
    sumCT2 += valCT2 * valCT2;
  }

  // Convert to RMS current
  float Vrms_rawCT1 = sqrt(sumCT1 / (float)NUM_SAMPLES) * (3.3 / 4095.0);
  float IrmsLive = Vrms_rawCT1 * CT_CALIBRATION;

  float Vrms_rawCT2 = sqrt(sumCT2 / (float)NUM_SAMPLES) * (3.3 / 4095.0);
  float IrmsNeutral = Vrms_rawCT2 * CT_CALIBRATION;

  // --- 3) Low-pass filter for stability ---
  filteredCurrent = (alpha * IrmsLive) + ((1 - alpha) * filteredCurrent);
  filteredNeutral = (alpha * IrmsNeutral) + ((1 - alpha) * filteredNeutral);

  // --- 4) Enhanced Fault Detection Logic ---
  bool lineToLineFault = (filteredCurrent >= THRESHOLD && filteredNeutral >= THRESHOLD);
  bool lineToGroundFault = ((filteredCurrent >= THRESHOLD && filteredNeutral < THRESHOLD * 0.5) || 
                           (filteredNeutral >= THRESHOLD && filteredCurrent < THRESHOLD * 0.5));
  
  if (lineToLineFault || lineToGroundFault) {
    if (overCurrentStart == 0) {
      // Start timing the overcurrent condition
      overCurrentStart = millis();
      // Determine fault type
      if (lineToLineFault) {
        currentFault = LINE_TO_LINE;
        Serial.println("Line-to-Line fault detected, starting timer...");
      } else {
        currentFault = LINE_TO_GROUND;
        Serial.println("Line-to-Ground fault detected, starting timer...");
      }
    } else if (millis() - overCurrentStart >= DEBOUNCE_DELAY) {
      // Overcurrent condition has persisted for the debounce period
      // Turn off relays (active high)
      digitalWrite(RELAY1_PIN, HIGH);
      digitalWrite(RELAY2_PIN, HIGH);
      relayTripped = true;
      
      if (currentFault == LINE_TO_LINE) {
        Serial.println("Line-to-Line fault confirmed! Relays turned OFF.");
      } else {
        Serial.println("Line-to-Ground fault confirmed! Relays turned OFF.");
      }
      
      Serial.println("Send 'r' to reset relays.");
      sendMQTTMessage(true, currentFault); // Send fault message with specific type
      return; // Exit loop to stop further measurements
    }
  } else {
    // Current is below threshold, reset the timer
    overCurrentStart = 0;
    currentFault = NO_FAULT;
  }

  // --- 5) Serial output ---
  Serial.print("AC RMS Current (Live): ");
  Serial.print(filteredCurrent, 2);
  Serial.print(" A | AC RMS Current (Neutral): ");
  Serial.print(filteredNeutral, 2);
  Serial.println(" A");

  // --- 6) MQTT Transmission (NON-BLOCKING) ---
  if (millis() - lastTransmitTime >= transmitInterval) {
    sendMQTTMessage(false); // Send normal heartbeat
    lastTransmitTime = millis();
  }

  // LED indicates normal operation
  digitalWrite(LED_PIN, LOW);

  delay(50); // EXACTLY THE SAME DELAY AS YOUR ORIGINAL CODE
}