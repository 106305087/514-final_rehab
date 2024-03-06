#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
#include <AccelStepper.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LED_PIN     9
#define NUM_LEDS    1
CRGB leds[NUM_LEDS];

#define BUTTON_PIN 8

#define MOTOR_PIN_1 1
#define MOTOR_PIN_2 2
#define MOTOR_PIN_3 3
#define MOTOR_PIN_4 4
AccelStepper stepper(AccelStepper::FULL4WIRE, MOTOR_PIN_1, MOTOR_PIN_3, MOTOR_PIN_2, MOTOR_PIN_4);

#define SERVICE_UUID        "49c1c51a-6e2c-48a4-8fde-3e9a02711aed" // Replace with your UUID
#define CHARACTERISTIC_UUID "bbd6bbb3-318c-4c13-b4f9-d60f6aca4a2e" // Replace with your UUID

// #define SERVICE_UUID        "ff77370f-5ca6-42ae-aa47-99ae6fd92793" // Replace with your unique UUID
// #define CHARACTERISTIC_UUID "bf683ee2-db03-40a7-abba-e9a1e9dfcf12" // Replace with your unique UUID

// bool isConnected = false; // This variable tracks the connection status.
// bool displayMode = false; // false for angle, true for bend count

// Bluetooth Low Energy (BLE) variables
static boolean isConnected = false;
static boolean connected = false;
static boolean doScan = false;
// static BLERemoteCharacteristic* pRemoteCharacteristic;
// static BLEAdvertisedDevice* myDevice;

BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

unsigned long lastReconnectAttempt = 0; // This tracks the last reconnect attempt time.
const unsigned long reconnectInterval = 5000; // Attempt to reconnect every 5 seconds.

// Display variables
bool displayMode = true;
float lastAngle = 0.0;
unsigned long lastBendCount = 0;

static unsigned long lastDebounceTime = 0;
static bool lastButtonState = HIGH;

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        isConnected = true;
        Serial.println("Connected to server");
    }

    void onDisconnect(BLEClient* pclient) override {
        isConnected = false;
        Serial.println("Disconnected from server");
    }
};

void setupDisplay();
void setupBLE();
void scanAndConnect();
bool connectToServer(BLEAddress pAddress);
void updateDisplay();
void handleBLE();
void handleButton();
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

void setup() {
    Serial.begin(115200);
    setupDisplay();
    setupBLE();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    handleButton();
    scanAndConnect();

    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(500);
}

void loop() {
    handleBLE();
    handleButton();
    stepper.run();
}

void setupDisplay() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Infinite loop
    }
    display.display();
    delay(2000); // Pause for 2 seconds
    display.clearDisplay();
    display.setTextSize(1.2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("Scanning for BLE Server..."));
    display.display();
}

void setupBLE() {
    BLEDevice::init("");
}

void scanAndConnect() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(5, false);
    Serial.println("Scan done!");
    BLEUUID serviceUUID(SERVICE_UUID);
    for (int i = 0; i < foundDevices.getCount(); i++) {
        BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            Serial.println("Found our device!");
            display.clearDisplay();
            display.setCursor(0,0);
            display.println(F("Found server\nConnecting..."));
            display.display();
            if (connectToServer(advertisedDevice.getAddress())) {
                display.clearDisplay();
                display.setCursor(0,0);
                display.println(F("Connected"));
                display.display();
                return;
            }
        }
    }
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("Server not found"));
    display.display();
}

bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());

    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(pAddress)) {
        Serial.println(" - Connection failed");
        return false;
    }

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(SERVICE_UUID);
      return false;
    }
    Serial.println(" - Found our service");

    BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(CHARACTERISTIC_UUID);
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    isConnected = true;
    return true;
}

void handleBLE() {

    if (!isConnected && (millis() - lastReconnectAttempt > reconnectInterval)) {
      lastReconnectAttempt = millis(); // Update the last attempt time

      display.clearDisplay();
      display.setCursor(0,0);
      display.println(F("Lost connection\nReconnecting..."));
      display.display();

      // Scan for BLE servers and find the one that matches our service UUID
      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setActiveScan(true); // Active scan uses more power, but get results faster
      BLEScanResults foundDevices = pBLEScan->start(5, false); // 5-second scan, don't continue the scan after 5 seconds

      BLEUUID serviceUUID(SERVICE_UUID);
      bool foundServer = false;
      for (int i = 0; i < foundDevices.getCount(); i++) {
          BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
          if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
              if (connectToServer(advertisedDevice.getAddress())) {
                  display.clearDisplay();
                  display.setCursor(0,0);
                  display.println(F("Reconnected"));
                  display.display();
                  foundServer = true;
                  isConnected = true; // Update connection status
                  break;
              }
          }
      }

      if (!foundServer) {
          display.clearDisplay();
          display.setCursor(0,0);
          display.println(F("Reconnect failed"));
          display.display();
          isConnected = false; // Ensure connection status is updated
      }
      pBLEScan->clearResults(); // Clear scan results to free up memory
  }
}

void handleButton() {
    bool currentButtonState = digitalRead(BUTTON_PIN);
    // Check if button state has changed
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > 50) {
        lastDebounceTime = millis(); // Reset the debouncing timer

        // Check if the button was pressed (assuming active low configuration)
        if (currentButtonState == LOW) {
            Serial.println("Button Pressed");

            // Toggle the display mode
            displayMode = !displayMode;
            
            // Update the display based on the new mode
            updateDisplay();
        }
    }

    lastButtonState = currentButtonState; // Save the current button state as the last button state
}


void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1.2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    
    if (!displayMode) {
        display.print("Max Angle: ");
        display.print(lastAngle);
    } else {
        display.print("Bend Count: ");
        display.print(lastBendCount);
    }
    display.display();
}

const long positionLeft = -500;  // Adjust as necessary
const long positionRight = 500;  // Adjust as necessary

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    String dataString = (char*)pData;

    // Print the raw data received to the serial monitor
    Serial.print("Received Data: ");
    Serial.println(dataString);

    // Find indices for key substrings
    int angleIndex = dataString.indexOf("A: ") + 3;
    int bendCountIndex = dataString.indexOf(", B: ") + 4;

    if (angleIndex != -1 && bendCountIndex != -1) {
        int commaIndex = dataString.indexOf(',', angleIndex);
        
        if (commaIndex != -1) {
            // Extract substrings based on identified indices
            String angleString = dataString.substring(angleIndex, commaIndex);
            String bendCountString = dataString.substring(bendCountIndex);
            
            // Convert extracted substrings to appropriate types
            lastAngle = angleString.toFloat();
            lastBendCount = bendCountString.toInt();
            
            // Update display with new data
            updateDisplay();

            // Motor control logic based on the angle
            if (lastAngle < 50) {
                // Turn the motor to the left (counter-clockwise)
                Serial.println("Turning motor left.");
                stepper.moveTo(positionLeft); // Move 100 steps counter-clockwise
            } else {
                // Turn the motor to the right (clockwise)
                Serial.println("Turning motor right.");
                stepper.moveTo(positionRight); // Move 100 steps clockwise
            }
        }
    }
}


//////////////////////////////////



// class MyClientCallback : public BLEClientCallbacks {
//     void onConnect(BLEClient* pclient) {
//         isConnected = true;
//         Serial.println("Connected to server");
//     }

//     void onDisconnect(BLEClient* pclient) {
//         isConnected = false;
//         Serial.println("Disconnected from server");
//     }
// };

// void updateDisplay();

// void notifyCallback(
//   BLERemoteCharacteristic* pBLERemoteCharacteristic,
//   uint8_t* pData,
//   size_t length,
//   bool isNotify) {
//     String dataString = (char*)pData;

//     // Find indices for key substrings
//     int angleIndex = dataString.indexOf("Max Angle: ") + String("Max Angle: ").length();
//     int bendCountIndex = dataString.indexOf(", Bend Count: ") + String(", Bend Count: ").length();

//     if (angleIndex != -1 && bendCountIndex != -1) {
//         int commaIndex = dataString.indexOf(',', angleIndex);
        
//         if (commaIndex != -1) {
//             // Extract substrings based on identified indices
//             String angleString = dataString.substring(angleIndex, commaIndex);
//             String bendCountString = dataString.substring(bendCountIndex);
            
//             // Convert extracted substrings to appropriate types
//             lastAngle = angleString.toFloat();
//             lastBendCount = bendCountString.toInt();
            
//             // Update display with new data
//             updateDisplay();
//         }
//     }
// }

// bool connectToServer(BLEAddress pAddress) {
//     Serial.print("Forming a connection to ");
//     Serial.println(pAddress.toString().c_str());

//     BLEClient*  pClient  = BLEDevice::createClient();
//     Serial.println(" - Created client");

//     // Connect to the remote BLE Server.
//     pClient->connect(pAddress);
//     Serial.println(" - Connected to server");

//     BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
//     if (pRemoteService == nullptr) {
//       Serial.print("Failed to find our service UUID: ");
//       Serial.println(SERVICE_UUID);
//       return false;
//     }
//     Serial.println(" - Found our service");

//     // Obtain a reference to the characteristic in the service of the remote BLE server.
//     BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
//     if (pRemoteCharacteristic == nullptr) {
//       Serial.print("Failed to find our characteristic UUID: ");
//       Serial.println(CHARACTERISTIC_UUID);
//       return false;
//     }
//     Serial.println(" - Found our characteristic");

//     // Read the value of the characteristic.
//     if(pRemoteCharacteristic->canNotify())
//       pRemoteCharacteristic->registerForNotify(notifyCallback);

//     isConnected = true;
//     return true;
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(BUTTON_PIN, INPUT_PULLUP);

//     BLEDevice::init("");

//     if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
//         Serial.println(F("SSD1306 allocation failed"));
//         for(;;);
//     }
//     display.display();
//     delay(2000); // Pause for 2 seconds

//     display.clearDisplay();
//     display.setTextSize(1);      // Normal 1:1 pixel scale
//     display.setTextColor(SSD1306_WHITE); // Draw white text
//     display.setCursor(0,0);     // Start at top-left corner
//     display.println(F("Scanning for\nBLE Server..."));
//     display.display();

//     BLEScan* pBLEScan = BLEDevice::getScan();
//     pBLEScan->setActiveScan(true);
//     BLEScanResults foundDevices = pBLEScan->start(5);
//     Serial.println("Scan done!");
//     bool foundServer = false;
//     BLEUUID serviceUUID(SERVICE_UUID); // Explicitly create a BLEUUID object
//     for (int i = 0; i < foundDevices.getCount(); i++) {
//         BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
//         if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) { // Use the BLEUUID object
//             Serial.println("Found our device!");
//             display.clearDisplay();
//             display.setCursor(0,0);
//             display.println(F("Found server\nConnecting..."));
//             display.display();
//             if (connectToServer(advertisedDevice.getAddress())) {
//                 display.clearDisplay();
//                 display.setCursor(0,0);
//                 display.println(F("Connected"));
//                 display.display();
//                 foundServer = true;
//                 break;
//             }
//         }
//     }
//     if (!foundServer) {
//         display.clearDisplay();
//         display.setCursor(0,0);
//         display.println(F("Server not found"));
//         display.display();
//         isConnected = false;
//     }
// }

// void loop() {
//     // Check if we are still connected, if not, attempt to reconnect every reconnectInterval milliseconds
//     if (!isConnected && (millis() - lastReconnectAttempt > reconnectInterval)) {
//         lastReconnectAttempt = millis(); // Update the last attempt time

//         display.clearDisplay();
//         display.setCursor(0,0);
//         display.println(F("Lost connection\nReconnecting..."));
//         display.display();

//         // Scan for BLE servers and find the one that matches our service UUID
//         BLEScan* pBLEScan = BLEDevice::getScan();
//         pBLEScan->setActiveScan(true); // Active scan uses more power, but get results faster
//         BLEScanResults foundDevices = pBLEScan->start(5, false); // 5-second scan, don't continue the scan after 5 seconds

//         BLEUUID serviceUUID(SERVICE_UUID);
//         bool foundServer = false;
//         for (int i = 0; i < foundDevices.getCount(); i++) {
//             BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
//             if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
//                 if (connectToServer(advertisedDevice.getAddress())) {
//                     display.clearDisplay();
//                     display.setCursor(0,0);
//                     display.println(F("Reconnected"));
//                     display.display();
//                     foundServer = true;
//                     isConnected = true; // Update connection status
//                     break;
//                 }
//             }
//         }

//         if (!foundServer) {
//             display.clearDisplay();
//             display.setCursor(0,0);
//             display.println(F("Reconnect failed"));
//             display.display();
//             isConnected = false; // Ensure connection status is updated
//         }
//         pBLEScan->clearResults(); // Clear scan results to free up memory
//     }

//     delay(1000); // Delay to prevent constant reconnection attempts

//     static unsigned long lastDebounceTime = 0;
//     static bool lastButtonState = HIGH;
//     bool currentButtonState = digitalRead(BUTTON_PIN);
    
//     // Check if button state has changed
//     if (currentButtonState != lastButtonState) {
//         lastDebounceTime = millis();
//     }
    
//     // Only toggle the display mode if the button has been stable for at least 50 milliseconds
//     if ((millis() - lastDebounceTime) > 50) {
//         if (currentButtonState == LOW) { // Assuming active low button
//             displayMode = !displayMode; // Toggle display mode
//             updateDisplay(); // Update the display with the new mode
//         }
//     }
    
//     lastButtonState = currentButtonState; // Update the last button state
    
//     delay(100); // Delay to reduce polling rate
// }

// void updateDisplay() {
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
//     display.setCursor(0,0);
    
//     if (!displayMode) {
//         display.print("Max Angle: ");
//         display.print(lastAngle);
//     } else {
//         display.print("Bend Count: ");
//         display.print(lastBendCount);
//     }
    
//     display.display();
// }