#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h> // For math functions
#include "time.h"
#include <CircularBuffer.h>

#include <WiFi.h>
#include <Firebase_ESP_Client.h>

#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// WiFi credentials
const char* ssid     = "NETGEAR36";
const char* password = "silenttulip242";
// const char* ssid     = "UW MPSK";
// const char* password = "MEAi7_*tb:";   
// soldered sensing devcie
// const char* ssid     = "UW MPSK";
// const char* password = "Js>4}AMjjM";

// NTP settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -8;
const int   daylightOffset_sec = 3600;
bool timeInitialized = false;

// #define DATABASE_URL "https://final-project-db-dadda-default-rtdb.firebaseio.com/" // Replace with your database URL
// #define API_KEY "AIzaSyA4leJXt3aT4czaFpAhiTrun24OuNSeRhQ" // Replace with your API key

#define DATABASE_URL "https://final-database-d82d1-default-rtdb.firebaseio.com/" // Replace with your database URL
#define API_KEY "AIzaSyDKel-U76W2CexlNbHaQv5aqCIYWPE_L-E"

#define STAGE_INTERVAL 12000 // 12 seconds each stage
#define MAX_WIFI_RETRIES 10 // Maximum number of WiFi connection retries

int uploadInterval = 1000; // 1 seconds each upload

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

// Bluetooth UUIDs
#define SERVICE_UUID        "49c1c51a-6e2c-48a4-8fde-3e9a02711aed" // Replace with your unique UUID
#define CHARACTERISTIC_UUID "bbd6bbb3-318c-4c13-b4f9-d60f6aca4a2e" // Replace with your unique UUID

Adafruit_MPU6050 mpu;

float lastSentAngle = -1000; // Initialize with an impossible value for the first comparison
unsigned long lastAngleChangeTime = 0; // Track when the last angle change occurred
const float angleChangeTolerance = 1.5;

// Define a circular buffer for recent accelY readings to calculate a dynamic threshold
CircularBuffer<float, 10> accelYBuffer; // Adjust the size based on your needs
// Variables to keep track of the state of peak detection
bool isAboveThreshold = false;
float dynamicThreshold = 0.0;
float thresholdMultiplier = 1.5; // Adjust based on sensitivity required
float minDifference = 0.05; // Minimum difference to detect a peak, adjust as needed
unsigned long bendCount = 0; // Track the number of bends detected

bool deviceConnected = false; // Track Bluetooth connection status
bool oldDeviceConnected = false; // Track previous Bluetooth connection status

BLECharacteristic *pCharacteristic;
BLEServer *pServer = nullptr; // Global BLEServer pointer

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
      deviceConnected = false;
      BLEDevice::startAdvertising(); // Make sure this is called upon disconnection
      Serial.println("Now advertising for clients...");
    }
};

// // Function prototypes
bool connectToWiFi();
void sendWiFiStatus(const char* statusMessage);
void initializeTime();
void initFirebase();
void sendDataToFirebase(float pitch, unsigned long bendCount, float angularVelocityY);
void printLocalTime();

void setup() {
  Serial.begin(115200);

  Serial.println("Starting BLE work!");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 initialization successful");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // BLE setup
  BLEDevice::init("ESP32_S3_BLE_Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE server is running");

  // Initialize Wi-Fi
  Serial.println("Turning on WiFi");
  connectToWiFi();

  // Initialize NTP
  initializeTime();

  // Initialize Firebase
  Serial.println("Initializing Firebase...");
  initFirebase();

  // Assign the callback function for the long running token generation task
  config.token_status_callback = tokenStatusCallback; // Ensure this callback is defined to handle token status
}

void loop() {

  // Check if connection status has changed
  if (deviceConnected != oldDeviceConnected) {
    oldDeviceConnected = deviceConnected; // Update the connection status
    if (deviceConnected) {
      Serial.println("Device connected");
    } else {
      Serial.println("Device disconnected");
      // Reset the timer to avoid repeated disconnections
      lastAngleChangeTime = 0;
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  // Print the pitch angle to the serial monitor regardless of BLE notifications
  Serial.print("Current Pitch: ");
  Serial.println(pitch);
  
    // Calculate angular velocity from gyroscope data (radians to degrees per second conversion)
  float angularVelocityX = g.gyro.x * 180 / M_PI;
  float angularVelocityY = g.gyro.y * 180 / M_PI;
  float angularVelocityZ = g.gyro.z * 180 / M_PI;

  // // Print angular velocity for debugging
  // Serial.print("Gyro (deg/s) X: "); Serial.print(angularVelocityX);
  // Serial.print(" Y: "); Serial.print(angularVelocityY);
  // Serial.print(" Z: "); Serial.println(angularVelocityZ);

  // Acceleration data in G's
  float accelX = a.acceleration.x / 9.81; // Converting to G's
  float accelY = a.acceleration.y / 9.81;
  float accelZ = a.acceleration.z / 9.81;

  // // Print acceleration for debugging
  // Serial.print("Accel (G) X: "); Serial.print(accelX);
  // Serial.print(" Y: "); Serial.print(accelY);
  // Serial.print(" Z: "); Serial.println(accelZ);

  // Add the latest accelY value to the buffer
  accelYBuffer.push(accelY);

  // Calculate a simple dynamic threshold based on the average of recent values in the buffer
  float sum = 0;
  for (int i = 0; i < accelYBuffer.size(); i++) {
    sum += accelYBuffer[i];
  }
  float average = sum / accelYBuffer.size();
  dynamicThreshold = average * thresholdMultiplier;

  static bool sendNextPitch = false;

  // Check for a peak
  if (!isAboveThreshold && accelY > dynamicThreshold + minDifference) {
    // Peak detected
    Serial.println("Bend detected!");
    isAboveThreshold = true; // We are now above the threshold
    sendNextPitch = true;
    bendCount++;
  } else if (isAboveThreshold && accelY < dynamicThreshold) {
    // Once the value goes below the dynamic threshold, we reset, ready to detect the next peak
    isAboveThreshold = false;
  }


  // if (!deviceConnected && fabs(pitch - lastSentAngle) > angleChangeTolerance) {
  //   float pitchChange = fabs(pitch - lastSentAngle);
  //   if (pitchChange > angleChangeTolerance) {
  //     // Detected significant movement, refresh advertising to encourage reconnection
  //     BLEDevice::startAdvertising(); // Restart advertising
  //     Serial.println("Movement detected, advertising restarted to encourage reconnection.");
  //     lastSentAngle = pitch; // Update the last sent angle to prevent continuous refreshes
  //   }
  // }

  if (deviceConnected) {
    if (sendNextPitch) { // MODIFICATION: Check if the next pitch data should be sent.
      char buf[32];
      snprintf(buf, sizeof(buf), "A: %.2f, B: %lu", pitch, bendCount);
      Serial.println(bendCount);
      pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pCharacteristic->notify();
      Serial.print("Sent next pitch after bend over BLE: "); Serial.println(buf);
      sendNextPitch = false; // MODIFICATION: Reset the flag as the next pitch has been sent.
    }
    
    sendDataToFirebase(pitch, bendCount, angularVelocityY);
  
    // Check if the angle change doesn't exceed 1.5 degrees for more than 1 minute
    if (fabs(pitch - lastSentAngle) <= angleChangeTolerance) {
      if (lastAngleChangeTime == 0) { // First time the angle change is within the threshold
        lastAngleChangeTime = millis();
      } else if (millis() - lastAngleChangeTime > 60000) { // 1 minute has passed
        // Disconnect the client due to inactivity
        pServer->disconnect(pServer->getConnId());
        Serial.println("Disconnected due to inactivity.");
        // Reset the timer to avoid repeated disconnections
        lastAngleChangeTime = 0;
      }
    } else {
      // If the angle change exceeds 1.5 degrees, reset the timer
      lastAngleChangeTime = 0;
    }
  
    lastSentAngle = pitch; // Update the last sent angle regardless of the condition
  }
  
  delay(1000); // Adjust the delay as needed
}

bool connectToWiFi()
{
  // Print the device's MAC address.
  Serial.println(WiFi.macAddress());
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  sendWiFiStatus("Connecting...");
  int wifiCnt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    wifiCnt++;
    if (wifiCnt > MAX_WIFI_RETRIES){
      Serial.println("WiFi connection failed");
      sendWiFiStatus("WiFi connection failed");
      ESP.restart();
      return false;
    }
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  sendWiFiStatus("WiFi connected");
  return true;
}

void initFirebase()
{
  /* Assign the api key (required) */
  config.api_key = API_KEY;
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi is still connected");
    } else {
      Serial.println("WiFi connection lost");
    }
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

void sendDataToFirebase(float pitch, unsigned long bendCount, float angularVelocityY) {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > uploadInterval || sendDataPrevMillis == 0) && timeInitialized) {
    sendDataPrevMillis = millis();

    // Get the current timestamp
    time_t now;
    time(&now);

    // Prepare a JSON object to send the pitch and timestamp
    FirebaseJson json;
    json.set("pitch", pitch);
    json.set("bendCount", bendCount);
    json.set("gyroY", angularVelocityY);
    json.set("timestamp", (unsigned long)now); // Add the timestamp here
    // json.set("timeinfo", buffer); // Add the timeinfo here

    // Write the JSON object to the database path "test/data"
    if (Firebase.RTDB.pushJSON(&fbdo, "test/data2", &json)) {
      Serial.println("PASSED");
      Serial.print("PATH: ");
      Serial.println(fbdo.dataPath());
      Serial.print("TYPE: ");
      Serial.println(fbdo.dataType());
      Serial.print("PITCH: ");
      Serial.println(pitch);
      Serial.print("BEND COUNT: ");
      Serial.println(bendCount);
      Serial.print("GYRO Y: ");
      Serial.println(angularVelocityY);
      Serial.print("TIMESTAMP: ");
      Serial.println((unsigned long)now); // Print the timestamp
      // Serial.print("TIMESINFO: ");
      // Serial.println(buffer); // Print the timeinfo
    } else {
      Serial.println("Upload Firebase FAILED");
      Serial.print("REASON: ");
      Serial.println(fbdo.errorReason());
    }
    count++;
  }
}


void initializeTime() {
  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Wait for time to be set
  long start = millis();
  time_t now;
  while (time(&now) < 24 * 3600) {
    Serial.println("Waiting for NTP time sync");
    delay(500);
    if (millis() - start > 5000) {
      Serial.println("Failed to get NTP time.");
      return;
    }
  }
  Serial.println("Time synchronized");
  timeInitialized = true;
}

void sendWiFiStatus(const char* statusMessage) {
  if (deviceConnected) {
    pCharacteristic->setValue((uint8_t*)statusMessage, strlen(statusMessage));
    pCharacteristic->notify();
    Serial.println(statusMessage);
  }
}