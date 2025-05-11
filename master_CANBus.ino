#include <Adafruit_MCP2515.h>

// Define pins and constants
#define CS_PIN    PIN_CAN_CS
#define CAN_BAUDRATE (250000)

// Object instantiation
Adafruit_MCP2515 mcp(CS_PIN);

// Variables to store command values
uint8_t steerDirCmd = 6;      // Default to stop
uint8_t brakeModeCmd = 8;     // Default to retract
uint8_t driveModeCmd = 0;     // Default to neutral
uint16_t throttleRaw = 0;     // Default to no throttle
uint16_t steerValueRaw = 590; // Default to center position (590)

// Buffer for serial reception
const int BUFFER_SIZE = 32;
char serialBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Timing variables for status printing
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 10; // Print status every 10ms

// For detecting communication timeout
unsigned long lastCommandTime = 0;
const unsigned long COMM_TIMEOUT = 500; // 500ms timeout

void setup() {
  Serial.begin(115200);
  
  // Initialize CAN controller
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  
  Serial.println("MCP2515 initialized and ready!");
}

void loop() {
  // Check for timeout - if no commands received for a while, stop vehicle
  if (millis() - lastCommandTime > COMM_TIMEOUT) {
    // Safety stop if communication is lost
    brakeModeCmd = 7; // Extend brakes
    throttleRaw = 0;  // No throttle
    steerValueRaw = 590; // Center steering
  }
  
  // Process any available serial data
  while (Serial.available() > 0) {
    processSerialData();
  }
  
  // Send CAN messages based on current values
  sendCanMessages();
  
  // Print status occasionally but don't slow down the loop
  if (millis() - lastStatusTime > STATUS_INTERVAL) {
    printStatus();
    lastStatusTime = millis();
  }
  
  // Small delay to prevent flooding the CAN bus
  // Using a shorter delay for more responsive control
  delay(10);
}

void processSerialData() {
  char c = Serial.read();
  
  // End of command marker
  if (c == '\n') {
    // Null terminate the string
    serialBuffer[bufferIndex] = '\0';
    
    // Parse and execute the command
    parseCommand(serialBuffer);
    
    // Reset buffer for next command
    bufferIndex = 0;
    
    // Update the last command time
    lastCommandTime = millis();
    
    return;
  }
  
  // Add character to buffer if not end of command
  if (bufferIndex < BUFFER_SIZE - 1) {
    serialBuffer[bufferIndex++] = c;
  }
}

void parseCommand(const char* command) {
  // Variables to store parsed values
  int brake = 0;
  int mode = 0;
  float linear = 0.0;
  float angular = 0.0;
  
  // Parse the command string - faster version using direct char pointer manipulation
  char* ptr = (char*)command;
  
  // Find brake value
  ptr = strstr(ptr, "B:");
  if (ptr) {
    ptr += 2; // Skip "B:"
    brake = atoi(ptr);
    
    // Set brake mode based on brake value
    if (brake == 1) {
      brakeModeCmd = 7; // Extend brakes
    } else {
      brakeModeCmd = 8; // Retract brakes
    }
  }
  
  // Find mode value
  ptr = strstr(command, "M:");
  if (ptr) {
    ptr += 2; // Skip "M:"
    mode = atoi(ptr);
    
    // Set drive mode based on the mode value
    driveModeCmd = mode;
  }
  
  // Find linear value
  ptr = strstr(command, "L:");
  if (ptr) {
    ptr += 2; // Skip "L:"
    linear = atof(ptr);
  }
  
  // Find angular value
  ptr = strstr(command, "A:");
  if (ptr) {
    ptr += 2; // Skip "A:"
    angular = atof(ptr);
    
    // Map angular value to the new steering range (255-924)
    // angular is -0.7 to 0.7, map to 255-924 where:
    // -0.7 = max left = 255
    // 0.0 = center = 590
    // 0.7 = max right = 924
    
    // First normalize to -1.0 to 1.0
    float normalizedAngular = angular / 0.7;
    // Clamp to range
    if (normalizedAngular > 1.0) normalizedAngular = 1.0;
    if (normalizedAngular < -1.0) normalizedAngular = -1.0;
    
    // Map -1.0 to 1.0 to 255-924
    // Center value (590) + range (334.5 on each side) * normalized value
    steerValueRaw = 590 + (int)(normalizedAngular * 334.5);
    
    // Set steering direction based on angular value
    if (angular > 0.1) {
      steerDirCmd = 4;  // Left
    } else if (angular < -0.1) {
      steerDirCmd = 5;  // Right
    } else {
      steerDirCmd = 6;  // Stop/Center
    }
  }
  
  // Set throttle based on linear velocity
  // Only use positive linear values (0 to 1.5) for throttle (0 to 512)
  if (linear > 0) {
    // Map 0-1.5 to 0-512
    throttleRaw = (uint16_t)((linear / 1.5) * 512);
    
    // Ensure we don't exceed the maximum
    if (throttleRaw > 512) throttleRaw = 512;
  } else {
    // No motion or negative value (treated as no throttle)
    throttleRaw = 0;
  }
}

void sendCanMessages() {
  // Separate throttle value into two bytes
  uint8_t throttleHigh = (throttleRaw >> 8) & 0xFF;
  uint8_t throttleLow = throttleRaw & 0xFF;
  
  // Separate steering value into two bytes
  uint8_t steerHigh = (steerValueRaw >> 8) & 0xFF;
  uint8_t steerLow = steerValueRaw & 0xFF;

  // Send Steering Packet (0x100)
  mcp.beginPacket(0x100);
  mcp.write(steerDirCmd);
  mcp.write(steerHigh);  // Add steering high byte
  mcp.write(steerLow);   // Add steering low byte
  mcp.endPacket();

  // Send Throttle & Drivemode Packet (0x200)
  mcp.beginPacket(0x200);
  mcp.write(driveModeCmd);   // First byte: drive mode command
  mcp.write(throttleHigh);   // Second byte: high byte of throttle value
  mcp.write(throttleLow);    // Third byte: low byte of throttle value
  mcp.endPacket();

  // Send Braking Packet (0x300)
  mcp.beginPacket(0x300);
  mcp.write(brakeModeCmd);
  mcp.endPacket();
}

void printStatus() {
  // Print current values to serial for debugging
  Serial.print("Status - Steer Dir: ");
  Serial.print(steerDirCmd);
  Serial.print(", Steer Value: ");
  Serial.print(steerValueRaw);
  Serial.print(", Brake: ");
  Serial.print(brakeModeCmd);
  Serial.print(", Drive Mode: ");
  Serial.print(driveModeCmd);
  Serial.print(", Throttle: ");
  Serial.println(throttleRaw);
}
