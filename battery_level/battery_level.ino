#include <Wire.h>                    // Include Wire library

unsigned long previousMillis = 0;    // Variable to store the previous time
const unsigned long interval = 1000; // Interval at which to execute the code, in milliseconds

const byte RGB_RED_PIN = 2;          // Define pin number for red LED
const byte RGB_GREEN_PIN = 15;       // Define pin number for green LED
const byte RGB_BLUE_PIN = 4;         // Define pin number for blue LED

void setup() 
{
  pinMode(RGB_RED_PIN, OUTPUT);      // Set red LED pin as output
  pinMode(RGB_GREEN_PIN, OUTPUT);    // Set green LED pin as output
  pinMode(RGB_BLUE_PIN, OUTPUT);     // Set blue LED pin as output
  
  Serial.begin(115200);              // Initialize the Serial port with a baud rate of 115200
  delay(5000);                       // Delay for 5 seconds to allow time for the Serial monitor to open
  Wire.begin();                      // Initialize the Wire library for I2C communication
}

void loop()
{
  unsigned long currentMillis = millis();   // Get the current time

  if (currentMillis - previousMillis >= interval)  // Check if it's time to execute the code
  {
    previousMillis = currentMillis;    // Store the current time as the previous time

    // Your code to execute every second goes here
    Serial.print("Battery Level: ");    // Print a message to the Serial monitor
    Serial.print(getBatteryLevel());   // Print the battery level
    Serial.println("%");                // Print a newline character after the battery level
  }
}

// This function reads the battery level from a device with an address of 0x75
int8_t getBatteryLevel() {
  Wire.beginTransmission(0x75);      // Begin I2C transmission to device with address 0x75
  Wire.write(0x78);                  // Send a request to read a single byte at address 0x78
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1) == 1) { // If the transmission was successful and one byte was received
    switch (Wire.read() & 0xF0) {    // Mask the received byte with 0xF0 and switch on the result
      case 0xE0:                     // If the masked byte is 0xE0
        emptyBattery();              // Call the emptyBattery function
        return 25;                   // Return a battery level of 25
      case 0xC0:                     // If the masked byte is 0xC0
        lowBattery();                // Call the lowBattery function
        return 50;                   // Return a battery level of 50
      case 0x80:                     // If the masked byte is 0x80
        fullBattery();               // Call the fullBattery function
        return 75;                   // Return a battery level of 75
      case 0x00:                     // If the masked byte is 0x00
        fullBattery();               // Call the fullBattery function
        return 100;                  // Return a battery level of 100
      default:                       // If the masked byte is none of the above
        emptyBattery();              // Call the emptyBattery function
        return 0;                    // Return a battery level of 0
    }
  }
  return -1;                         // If the transmission was not successful, return -1
}

// This function sets the RGB LED color based on the battery level
void setRgbLedColor(int red, int green, int blue)
{
  digitalWrite(RGB_RED_PIN, !red);    // Set the red LED pin to the opposite of the red argument
  digitalWrite(RGB_GREEN_PIN, !green);// Set the green LED pin to the opposite of the green argument
  digitalWrite(RGB_BLUE_PIN, !blue);  // Set the blue LED pin to the opposite of the blue argument
}

void emptyBattery() {
  // Set the LED color to red, indicating a low battery level
  setRgbLedColor(HIGH, LOW, LOW);
}

void lowBattery() {
  // Set the LED color to yellow, indicating a medium battery level
  setRgbLedColor(HIGH, HIGH, LOW);
}

void mediumBattery() {
  // Set the LED color to blue, indicating a medium battery level
  setRgbLedColor(LOW, LOW, HIGH);
}

void fullBattery() {
  // Set the LED color to green, indicating a full battery level
  setRgbLedColor(LOW, HIGH, LOW);
}
