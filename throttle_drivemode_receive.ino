#include <Adafruit_MCP2515.h>
#include <Adafruit_MCP4728.h>

// Define pins and constants
#define CS_PIN PIN_CAN_CS
#define CAN_BAUDRATE (250000)
#define TARGET_CAN_ID (0x200)  // Only process messages with this ID

// Object instantiation
Adafruit_MCP2515 mcp(CS_PIN); // initialize CANbus
Adafruit_MCP4728 dac; // initialize DAC

// Function prototypes
void neutral();
void drive();
void sport();
void reverse();
void updateThrottle(int rawThrottle);

// ----- DRIVE MODE FUNCTIONS -----
void neutral() {
  // dac.setChannelValue(MCP4728_CHANNEL_A, 324); // 0.3V (Keep this line uncommented)
  dac.setChannelValue(MCP4728_CHANNEL_B, 3404); // 3.15V
  dac.setChannelValue(MCP4728_CHANNEL_C, 2064); // 1.91V
  dac.setChannelValue(MCP4728_CHANNEL_D, 3458); // 3.20V
  return;
}

void drive(){
  // dac.setChannelValue(MCP4728_CHANNEL_A, 378); // 0.3V
  dac.setChannelValue(MCP4728_CHANNEL_B, 0); // 0V
  dac.setChannelValue(MCP4728_CHANNEL_C, 2064); // 1.91V
  dac.setChannelValue(MCP4728_CHANNEL_D, 3436); // 3.18V
  return;
}

void sport(){
  // dac.setChannelValue(MCP4728_CHANNEL_A, 378); // 0.3V
  dac.setChannelValue(MCP4728_CHANNEL_B, 594); // 0.55V
  dac.setChannelValue(MCP4728_CHANNEL_C, 2031); // 1.88V
  dac.setChannelValue(MCP4728_CHANNEL_D, 0); // 0V
  return;
}

void reverse(){
  // dac.setChannelValue(MCP4728_CHANNEL_A, 378); // 0.3V
  dac.setChannelValue(MCP4728_CHANNEL_B, 3404); // 3.15V
  dac.setChannelValue(MCP4728_CHANNEL_C, 0); // 0V
  dac.setChannelValue(MCP4728_CHANNEL_D, 3436); // 3.18V
  return;
}

// ----- THROTTLE UPDATE FUNCTION -----
void updateThrottle(int rawThrottle) {
  // Map the raw throttle value to the DAC value.
  // *values might need to be changed for proper 0-5mph mapping*
  long throttleVal = map(rawThrottle, 0, 512, 378, 2100);
  
  // Update DAC channel A with the mapped throttle value:
  dac.setChannelValue(MCP4728_CHANNEL_A, throttleVal);
  Serial.print("Throttle updated: raw ");
  Serial.print(rawThrottle);
  Serial.print(" -> DAC value ");
  Serial.println(throttleVal);
  Serial.println();
  return;
}

unsigned long lastPrint = 0;

void setup() {
  // Serial initialization
  Serial.begin(115200);
  // while(!Serial) delay(10);
  
  // check for CAN
  Serial.println("MCP2515 test!");
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
  
  // check for DAC
  Serial.println("AdafruitMCP4728 test!");
  
  // Try to initialize!
  if (!dac.begin()) { // Initialize DAC
    Serial.println("Error initializing MCP4728.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MCP4728 chip found");
  Wire.begin(0x60); // Set DAC address
  Wire.setSDA(2);
  Wire.setSCL(3);
}

void loop() {
  // try to parse packet
  int packetSize = mcp.parsePacket();
  
  // if a packet is received, process it
  if (packetSize) {
    // Check if the packet ID matches our target
    if (mcp.packetId() == TARGET_CAN_ID) {
      lastPrint = millis();
      
      // Print out packetID and length
      Serial.print("Received packet with id 0x");
      Serial.print(mcp.packetId(), HEX);
      Serial.print(" and length ");
      Serial.println(packetSize);
      
      uint8_t driveModeCmd = mcp.read();
      uint8_t throttleHigh = mcp.read();
      uint8_t throttleLow = mcp.read();
      uint16_t throttleRaw = ((uint16_t)throttleHigh << 8) | throttleLow;
      
      // Print received values for debugging (in hex)
      Serial.print("Drive mode: ");
      Serial.println(driveModeCmd, DEC);
      Serial.print("Throttle: ");
      Serial.println(throttleRaw, DEC);
      
      // unpack drive mode and throttle values
      // Update drive mode based on the command:
      // (For example: 0 = neutral, 1 = drive, 2 = sport, 3 = reverse)
      switch (driveModeCmd) {
        case 0:
          neutral();
          break;
        case 1:
          drive();
          updateThrottle(throttleRaw);
          break;
        case 2:
          sport();
          updateThrottle(throttleRaw);
          break;
        case 3:
          reverse();
          updateThrottle(throttleRaw);
          break;
        default:
          Serial.print("Unknown drive mode: ");
          Serial.println(driveModeCmd);
          break;
      }
    } else {
      // If we received a packet but with a different ID, just dump it
      while (mcp.available()) {
        mcp.read(); // Read and discard data
      }
    }
  }
  else {
    // Optionally print a message every few seconds to show the loop is still running.
    if (millis() - lastPrint > 3000) {
      Serial.println("Waiting for packet with ID 0x200...");
      lastPrint = millis();
    }
  }
}
