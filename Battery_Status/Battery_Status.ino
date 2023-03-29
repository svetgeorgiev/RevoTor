#include <Wire.h> // Include Wire Library


//   TIMER
const unsigned int UpdateInterval = 1000;
unsigned long lastUpdate;

byte hour;
byte minute;
byte second;

  // Declare RGB PINS 
const int RGB_RED_PIN = 14;
const int RGB_GREEN_PIN = 16;
const int RGB_BLUE_PIN = 12;
// END TIMER


int8_t getBatteryLevel()
{
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0
   && Wire.requestFrom(0x75, 1)) {
    switch (Wire.read() & 0xF0) {
    case 0xE0:
      emptyBattery();
      return 25;
    case 0xC0:
      lowBattery();
      return 50;
    case 0x80:
      mediumBattery();
      return 75;
    case 0x00:
      fullBattery();
      return 100;
    default: return 0;
    }
  }
  return -1;
}

void setup() 
{ 
  // Set the RGB pins to output
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  
  Wire.begin();
  Serial.begin();
}

void loop() 
{
  if(millis() - lastUpdate >= UpdateInterval){
    lastUpdate += UpdateInterval;
    
    second++;
    
    if(second == 60){
      second = 0;
      minute++;
    }
    if(minute == 60){
      minute = 0;
      hour++;
    }
    if(hour == 24){
      hour = 0;
    }
    Serial.print(" - ");
    Serial.print(hour);
    Serial.print(':');
    if(minute < 10){
      Serial.print('0');
    }
    Serial.print(minute);
    Serial.print(':');
    if(second < 10){
      Serial.print('0');
    }
    Serial.print(second);
  }
  // Print Battery Level on Serial  
  Serial.print(" - Battery Level: ");
  Serial.print(getBatteryLevel());
  Serial.print("%\n");
  delay(1000);
}
  
