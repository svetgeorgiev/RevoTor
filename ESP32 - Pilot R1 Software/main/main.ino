// include required libraries
#include <FS.h> // file system library
#include <WiFi.h> // Wi-Fi library
#include <Wire.h> // I2C library
#include <SPIFFS.h> // SPI Flash File System library
#include <Arduino.h> // Arduino core library
#include <TinyGPS++.h> // GPS library
#include <SoftwareSerial.h> // serial communication library
#include <ESPAsyncWebSrv.h> // asynchronous web server library

// Replace with your network credentials
const char * ssid = "iS-Home2"; // network name
const char * password = "NDXSTNYYDC"; // network password

const unsigned long WIFI_TIMEOUT = 30000; // 30 seconds
unsigned long wifiStartTime; // variable to store start time for Wi-Fi connection

// Create an instance of the TinyGPS++ object
TinyGPSPlus gps; // GPS object

// Define the homeLatitude and homeLongitude variables
char homeLatString[16]; // string to store home latitude
char homeLngString[16]; // string to store home longitude

const unsigned long GPS_TIMEOUT = 30000; // 30 seconds

AsyncWebServer server(80); // asynchronous web server on port 80

// define SPIFFS
#define SPIFFS_START_ADDR 0x10000 // start address for SPIFFS
#define SPIFFS_SIZE 0x3F0000 // size of SPIFFS

// define RGB LED pins
#define R_PIN 2 // red LED pin
#define G_PIN 15 // green LED pin
#define B_PIN 4 // blue LED pin

// define push button pin
#define BUTTON_PIN 5 // push button pin

// define long push duration in milliseconds
#define LONG_PUSH_DURATION 3000 // 3 seconds

// define buzzer pin and frequency for playing sound
#define BUZZER_PIN 23 // buzzer pin
#define BUZZER_FREQ 500 // buzzer frequency

// define LEDC channel and resolution
#define LEDC_CHANNEL 0 // LEDC channel
#define LEDC_RESOLUTION 8 // LEDC resolution

// initialize LED colors
int redValue = 0; // red LED value
int greenValue = 0; // green LED value
int blueValue = 0; // blue LED value

// initialize push button state and timing variables
int buttonState = 0; // current state of the push button
unsigned long pushStartTime = 0; // start time of push button press
unsigned long pushDuration = 0; // duration of push button press

// initialize current mode
int currentMode = 0; // current mode of the program


void setup() {
  // set LED pins as output
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);

  // set button pin as input with pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // set buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

  // set initial LED OFF 
  analogWrite(R_PIN, 255);
  analogWrite(G_PIN, 255);
  analogWrite(B_PIN, 255);
  
  Wire.begin();
  Serial.begin(9600); 
  Serial2.begin(9600);
  Serial.println("\nFlight Computer Setup Begin");
  
  // Wait for the GPS to get a fix
  unsigned long startTime = millis();

  while (!gps.location.isValid()) {
    // Read the GPS data
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }

    // Check if timeout has occurred
    if (millis() - startTime > GPS_TIMEOUT) {
      Serial.println("GPS Fix Timeout");
      // handle the error or break out of the loop
      break;
    }
  }

  // Update the homeLatitude and homeLongitude variables with the GPS values
  dtostrf(gps.location.lat(), 6, 6, homeLatString);
  dtostrf(gps.location.lng(), 6, 6, homeLngString);


  // set LEDC timer frequency and resolution
  ledcSetup(LEDC_CHANNEL, BUZZER_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, LEDC_CHANNEL);

  

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi:");

  wifiStartTime = millis(); // get current time

  while (WiFi.status() != WL_CONNECTED) {
    // exit loop if timeout is exceeded
    if (millis() - wifiStartTime > WIFI_TIMEOUT) {
      Serial.println("Wi-Fi connection timed out");
      break;
    }

    delay(1000);
    Serial.print(".");

  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("- Successful");
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.begin();
    Serial.println("Ground Station Web Page - Activated");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    // handle Wi-Fi connection error
    Serial.println("Wi-Fi connection failed");
    // Handle the error here (e.g. log the error, display an error message, etc.)
  }

  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    // Handle the error here (e.g. log the error, display an error message, etc.)
  } else {
    // Continue with the setup if SPIFFS is successfully mounted
    switchOnTone(); // Indicate with "Switching ON" sound
    Serial.println("Home Position Updated: " + String(homeLatString) + " / " + String(homeLngString));
  }
}

void modeOne() {
  Serial.println("GROUND MODE - ACTIVATED");
  analogWrite(R_PIN, 210);
  analogWrite(G_PIN, 190);
  analogWrite(B_PIN, 0);
}

void modeTwo() {
  Serial.println("TEST MODE - ACTIVATED");
  analogWrite(R_PIN, 255);
  analogWrite(G_PIN, 255);
  analogWrite(B_PIN, 0);
}

void modeThree() {
  Serial.println("FLIGHT MODE - ACTIVATED");
  int batteryLevel = getBatteryLevel();
  Serial.print("Battery Level: ");
  Serial.print(batteryLevel);
  Serial.println("%");
  if (batteryLevel < 100) {
    Serial.println("WARNING ! - Battery level is under 100% \nUNSUITABLE FOR FLIGHT - RECHARGE BEFORE USE !");
  }
  if (batteryLevel <= 25) {
    analogWrite(R_PIN, 0);
    analogWrite(G_PIN, 255);
    analogWrite(B_PIN, 255);
  } else if (batteryLevel <= 50) {
    analogWrite(R_PIN, 0);
    analogWrite(G_PIN, 0);
    analogWrite(B_PIN, 255);
  } else {
    analogWrite(R_PIN, 255);
    analogWrite(G_PIN, 0);
    analogWrite(B_PIN, 255);
  }
}

void loop() {
  // Check if there is GPS data available
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      // Send the latitude, longitude, altitude, speed (in km/h), and number of satellites over WebSockets
      String message = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.altitude.meters()) + "," + String(gps.speed.mps() * 3.6) + "," + String(gps.satellites.value());
      // TODO: send message over WebSockets
      break;
    }
  }
  // read button state
  buttonState = digitalRead(BUTTON_PIN);

  // if button is pressed, record start time
  if (buttonState == LOW && pushStartTime == 0) {
    pushStartTime = millis(); // record start time
  }
  // if button is released, calculate duration and switch mode or turn off
  else if (buttonState == HIGH && pushStartTime > 0) {
    pushDuration = millis() - pushStartTime; // calculate push duration
    pushStartTime = 0; // reset start time

    if (pushDuration >= LONG_PUSH_DURATION) { // if long push, turn off LED and play sound
      analogWrite(R_PIN, 255);
      analogWrite(G_PIN, 255);
      analogWrite(B_PIN, 255);
      Serial.print("Switching OFF\n");
      switchOffTone(); // Indicate with "Switching ON" sound 
    } else { // if short push, switch between modeOne, modeTwo and modeThree and play sound
      if (currentMode == 1) {
        modeTwo();
        currentMode = 2;
      } else if (currentMode == 2) {
        modeThree();
        currentMode = 3;
      } else {
        modeOne();
        currentMode = 1;
      }
      beepTone(); // Beep for every press of the button
    }
  }
}
