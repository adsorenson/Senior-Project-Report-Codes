#include <Adafruit_MCP2515.h>

// Define pins and constants
#define CS_PIN PIN_CAN_CS
#define CAN_BAUDRATE (250000)

// Object instantiation
Adafruit_MCP2515 mcp(CS_PIN);

// Define Pins
const int RPWM = 12;
const int LPWM = 11;
const int Lin_Act_Analog_Pin = A0;
const int Speed = 255; // Max extension/retraction speed
const float strokeLength = 2.0;
float targetLength = 0.7; // Desired extension length (inches)
float retractedLength = 0.00; // Fully retracted length (inches)
const int maxAnalogReading = 700; // Set these values based on your actuator's range
const int minAnalogReading = 262; // Adjust these values based on the actuator's actual readings

// Command definitions
const uint8_t CMD_EXTEND = 7;
const uint8_t CMD_RETRACT = 8;

volatile uint8_t currentCommand = CMD_RETRACT; // Default retracted
uint8_t lastCommand = 0xFF; // Store last processed command

void setup() {
  Serial.begin(115200);
  Serial.println("MCP2515 test!");

  // Check for CAN module
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while (1) delay(10);
  }
  Serial.println("MCP2515 chip found");

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(Lin_Act_Analog_Pin, INPUT);
}

void loop() {
  // Check for new CAN packets
  int packetSize = mcp.parsePacket();
  if (packetSize) {
    uint8_t brakeModeCmd = mcp.read();
    Serial.print("Received packet with command: ");
    Serial.println(brakeModeCmd);

    // Update command only if it's valid and different from the last one
    if ((brakeModeCmd == CMD_RETRACT || brakeModeCmd == CMD_EXTEND) && brakeModeCmd != lastCommand) {
      currentCommand = brakeModeCmd;
      lastCommand = brakeModeCmd;
    }
  }

  // Execute the last received command
  if (currentCommand == CMD_RETRACT) {
    retract();
  } else if (currentCommand == CMD_EXTEND) {
    extend();
  }
}

void extend() {
  int sensorVal = analogRead(Lin_Act_Analog_Pin);
  Serial.print("Extend - Sensor Value: ");
  Serial.println(sensorVal);

  int targetAnalogValue = mapfloat(targetLength, 0.0, strokeLength, float(minAnalogReading), float(maxAnalogReading));
  int lowerBound = targetAnalogValue - 10;
  int upperBound = targetAnalogValue + 10;

  if (sensorVal < lowerBound) {
    analogWrite(RPWM, Speed); // Extend actuator
    analogWrite(LPWM, 0);
    Serial.println("Extending...");
  } else if (sensorVal >= lowerBound && sensorVal <= upperBound) {
    analogWrite(RPWM, 0); // Stop actuator
    analogWrite(LPWM, 0);
    Serial.println("Target length reached.");
  }
}

void retract() {
  int sensorVal = analogRead(Lin_Act_Analog_Pin);
  Serial.print("Retract - Sensor Value: ");
  Serial.println(sensorVal);

  int retractedAnalogValue = mapfloat(retractedLength, 0.0, strokeLength, float(minAnalogReading), float(maxAnalogReading));
  int lowerBound = retractedAnalogValue - 10;
  int upperBound = retractedAnalogValue + 10;

  if (sensorVal > upperBound) {
    analogWrite(RPWM, 0); // Retract actuator
    analogWrite(LPWM, Speed);
    Serial.println("Retracting...");
  } else if (sensorVal >= lowerBound && sensorVal <= upperBound) {
    analogWrite(RPWM, 0); // Stop actuator
    analogWrite(LPWM, 0);
    Serial.println("Fully retracted.");
  }
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax) {
  return (x - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}
