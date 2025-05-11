#include <Adafruit_MCP2515.h>

// Define pins and constants
#define CS_PIN    PIN_CAN_CS
#define CAN_BAUDRATE (250000)

// Object instantiation
Adafruit_MCP2515 mcp(CS_PIN); // initialize CANbus

// Define Pins
const int STEP = 12;
const int DIR = 11;
const int Rswitch = 10;
const int Lswitch = 9;
const int magENC = A0;

// Command definitions
const uint8_t CMD_LEFT  = 4;
const uint8_t CMD_RIGHT = 5;
const uint8_t CMD_STOP  = 6;

// Steering value range
const int STEER_MIN = 255;  // Max left
const int STEER_CENTER = 590;  // Center
const int STEER_MAX = 924;  // Max right

// Steering control variables
volatile bool pulseState = LOW;  // current pulse state
volatile uint8_t currentCommand = CMD_STOP;  // default is stop
volatile uint16_t targetSteerValue = STEER_CENTER;  // target steering value

// Timing variables for nonblocking pulse generation
unsigned long lastStepTime = 0;
unsigned long stepInterval = 250;  // microseconds between toggles

// Control parameters
const int POSITION_TOLERANCE = 20;  // tolerance for position matching
const int MIN_STEP_INTERVAL = 100;  // fastest step interval in microseconds
const int MAX_STEP_INTERVAL = 1000;  // slowest step interval in microseconds

void setup() {
  Serial.begin(115200);
  Serial.println("MCP2515 test!");
  
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while (1) delay(10);
  }
  Serial.println("MCP2515 chip found");
  
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(Lswitch, INPUT_PULLUP);
  pinMode(Rswitch, INPUT_PULLUP);
  pinMode(magENC, INPUT);
}

void loop() {
  // Check for new CAN packets
  int packetSize = mcp.parsePacket();
  
  if (packetSize && mcp.packetId() == 0x100) {  // Check if it's the steering packet
    // Read the steering command
    uint8_t steerDirCmd = mcp.read();
    // Read the steering value (high byte and low byte)
    uint8_t steerHigh = mcp.read();
    uint8_t steerLow = mcp.read();
    
    // Combine to get the 16-bit steering value
    uint16_t receivedSteerValue = (steerHigh << 8) | steerLow;
    
    // Update the target steering value
    targetSteerValue = receivedSteerValue;
    
    // Update the command state
    currentCommand = steerDirCmd;
    
    // Debug output
    Serial.print("Received: CMD=");
    Serial.print(currentCommand);
    Serial.print(", Steer Value=");
    Serial.println(targetSteerValue);
  }
  
  // Read current position
  int currentPosition = analogRead(magENC);
  
  // Read limit switches
  int leftSwitchState = digitalRead(Lswitch);
  int rightSwitchState = digitalRead(Rswitch);
  
  // Determine direction and whether to move
  if (abs(currentPosition - targetSteerValue) > POSITION_TOLERANCE) {
    if (currentPosition < targetSteerValue && rightSwitchState == LOW) {
      // Need to move right
      digitalWrite(DIR, LOW);
      
      // Adjust step interval based on distance
      int distance = targetSteerValue - currentPosition;
      adjustStepInterval(distance);
      
      generatePulse();
    }
    else if (currentPosition > targetSteerValue && leftSwitchState == LOW) {
      // Need to move left
      digitalWrite(DIR, HIGH);
      
      // Adjust step interval based on distance
      int distance = currentPosition - targetSteerValue;
      adjustStepInterval(distance);
      
      generatePulse();
    }
  }
  else {
    // We're at the target position, stop pulsing
    // You might want to implement a hold position feature here
  }
  
  // Periodically print status
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 500) {  // Print every 500ms
    Serial.print("Target: ");
    Serial.print(targetSteerValue);
    Serial.print(", Current: ");
    Serial.print(currentPosition);
    Serial.print(", Interval: ");
    Serial.print(stepInterval);
    Serial.print(", LSwitch: ");
    Serial.print(leftSwitchState);
    Serial.print(", RSwitch: ");
    Serial.println(rightSwitchState);
    
    lastStatusTime = millis();
  }
}

// Adjust step interval based on distance to target
void adjustStepInterval(int distance) {
  // Map distance to step interval
  // Larger distance = faster steps (smaller interval)
  // Smaller distance = slower steps (larger interval)
  
  // Constrain distance to a reasonable range for mapping
  distance = constrain(distance, 1, 300);
  
  // Map distance inversely to step interval
  stepInterval = map(distance, 1, 300, MAX_STEP_INTERVAL, MIN_STEP_INTERVAL);
}

// This function generates a pulse using non-blocking timing
void generatePulse() {
  unsigned long currentTime = micros();
  if (currentTime - lastStepTime >= stepInterval) {
    pulseState = !pulseState;           // Toggle pulse state
    digitalWrite(STEP, pulseState);     // Write new pulse state
    lastStepTime = currentTime;         // Update last step time
  }
}
