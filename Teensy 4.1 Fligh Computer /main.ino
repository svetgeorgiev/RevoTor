#include <SD.h> // Include SD library to log DATA 
#include <SPI.h> // Include SPI library
#include <Wire.h> // Include Wire library
#include <LoRa.h> // Include LoRa library for Radio Transmitting
#include <Servo.h> // Include Servo library for Wings & Parachute 
#include <I2Cdev.h> // Include I2Cdev library
#include "BMP085.h" // Include BMP085 library for Temperature & Pressure readings
#include "MPU6050.h" // Include MPU6050 library for Orientation
#include "HMC5883L.h" // Include HMC5883L library for compass
#include <PinButton.h> // Include PinButton library for ground control Push Button 
#include <TinyGPSPlus.h> // Include TinyGPS++ library for GPS DATA
#include <ServoEasing.hpp> // Include ServoEasing library for more precise movement over the servo's
#include <SoftwareSerial.h> // Include SoftwareSerial library

#define SCK     13    // GPIO13  - Teensy 4.1 SCK
#define MISO    12   // GPIO12 - Teensy 4.1 MISO
#define MOSI    11   // GPIO11 - Teensy 4.1 MOSI
#define SS      10   // GPIO10 - Teensy 4.1 CS
#define RST     9   // GPIO9 - Teensy 4.1 RESET
#define G0     28   // GPIO28 - Teensy 4.1 IRQ(Interrupt Request)
#define FQ     8680222E2 // Define Frequency

#define RED_LED 33 // RED LED PIN
#define GREEN_LED 34 // GREEN LED PIN
#define BLUE_LED 35 // BLUE LED PIN

#define LEFT_WING_PIN 41 // Left WING
#define RIGHT_WING_PIN 40 // Right WING

const int buzzer = 32; // Buzzer
const int chipSelect = BUILTIN_SDCARD; // Teensy 4.1 BUILDIN_SDCARD
static const int RXPin = 0, TXPin = 1; // GPS PIN connections
static const uint32_t GPSBaud = 9600; // GPS Baud Rate 
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS = 131.0; // Gyro Sensitivity with default +/- 250 deg/s scale

TinyGPSPlus gps;  // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device
ServoEasing Left_Wing; // Left Wing Servo Object
ServoEasing Right_Wing; // Right Wing Servo Object
PinButton myButton(6); // Mode Switching Button Object
File myFile; // SD card Object
  
// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

// Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int scale = 16384;

// Barometer class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;

float temperature;
float pressure;
int32_t lastMicros;

void setup() {
  
  bool state = false;
  unsigned int count = 0;

  pinMode(RED_LED, OUTPUT); // Set Red LED pin output
  pinMode(BLUE_LED, OUTPUT); // Set Blue LED pin output
  pinMode(GREEN_LED, OUTPUT); // Set GREEN LED pin output
  pinMode(buzzer, OUTPUT); // Set BUZZER - pin 9 as an output
  
  // Attach servo to pin and set servo to start position.
  Left_Wing.attach(LEFT_WING_PIN, 4);
  Right_Wing.attach(RIGHT_WING_PIN, 3);
  Left_Wing.setSpeed(140); // This speed is taken if no further speed argument is given.
  Right_Wing.setSpeed(140); // This speed is taken if no further speed argument is given.
  delay(500); // Wait for servo to reach start position.
  
  
  Serial.begin(9600);
  while (!Serial && (count < 30)) {
    delay(200); // Wait for serial port to connect with timeout. Needed for native USB
    //Blink GREEN LED
    digitalWrite(GREEN_LED, state);
    state = !state;
    count++;
  }
  digitalWrite(GREEN_LED, HIGH);
 // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  // ==================== MPU6050 ============================
  accelgyro.initialize();
  Serial.print("Testing Accel/Gyro... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
  // Now we can talk to the HMC5883l

  // ==================== HMC5883L ============================
  mag.initialize();
  Serial.print("Testing Mag...  ");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // ==================== BMP085 ============================
  barometer.initialize();
  Serial.print("Testing Pressure...  ");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
 // ===================== SD CARD =========================
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
    while (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    digitalWrite(RED_LED, HIGH); // RED LED ON
    digitalWrite(GREEN_LED, LOW); // GREEN LED OFF
    tone(buzzer, 200); // Make Sound
    delay(750);
    noTone(buzzer); // Stop sound...
    digitalWrite(RED_LED, LOW); // RED LED ON
    Serial.println("Try Again in 5 seconds...");
    delay(5000);
  }
  Serial.println("SD Card Initialized.");
  myFile = SD.open("FlightLog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.println("time,latitude,longtitude,speed,elevation,accelerometer-XYZ,gyroscope-XYZ,magnetometer-XYZ,heading,temperature,pressure");
    myFile.close();   
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening FlighLog.txt");
    digitalWrite(RED_LED, HIGH); // RED LED ON
    digitalWrite(GREEN_LED, LOW); // GREEN LED OFF
    tone(buzzer, 200); // Make Sound
    delay(750);
    noTone(buzzer); // Stop sound... 
  }
   // ==================== LoRa initialising ============================
   Serial.print("Starting LoRa Transmitter... ");
   SPI.begin();
   LoRa.begin(FQ);
   LoRa.setPins(SS, RST, G0);
    if (!LoRa.begin(FQ)) {
     Serial.println("Starting LoRa failed!");
    while (1);
   }
  Serial.println("connection successfull");
  Serial.print("Transmitting frequency: "); Serial.println(FQ);
 // ==================== GPS initialising ============================
  ss.begin(GPSBaud);

  Serial.println("Waiting for valid GPS fix...");

  // Keep waiting until the GPS fix is valid
  while (!gps.location.isValid())
  {
    smartDelay(500);
    Serial.print('.');
    //Blink GREEN LED
    digitalWrite(GREEN_LED, state);
    state = !state;
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      digitalWrite(RED_LED, HIGH); // RED LED ON
      digitalWrite(GREEN_LED, LOW); // GREEN LED OFF
      tone(buzzer, 200); // Make Sound
      delay(750);
      noTone(buzzer); // Stop sound... 
      Serial.println(F("No GPS data received: check wiring"));
      while (1) {}
    }
  }
  
  Serial.println("\n====// Setup Complete //====");
  tone(buzzer, 1000); // 
  delay(10);
  tone(buzzer, 500); // 
  delay(150);
  tone(buzzer, 1000); // 
  delay(200);
  noTone(buzzer); // Stop sound...
  
  //CLOSE BOTH WINGS
  Left_Wing.write(95); // Left Wing CLOSED
  Right_Wing.write(0); // Right Wing CLOSED
}

 
// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printTime(TinyGPSTime &t)
{
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  smartDelay(0);
}
/*static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
} */

void modeOne() {
  //CLOSE BOTH WINGS
  Left_Wing.write(0); // Left Wing OPEN
  Right_Wing.write(105); // Right Wing OPEN
  
  static unsigned long ms = 0;
  static boolean state = !state;

  // Serial Output Format
  // === Time ===   |===== GPS DATA ===== |  ==== GPS DATA ==== |=== Accel === | === Gyro === | ======= Mag ======= | === Barometer === | === Wing Possition === |
  //    H:M:s       |    LAT      LNG     |    SPEED    ALT     |  X   Y   Z   |  X   Y   Z   |  X   Y   Z  Heading |  Temp   Pressure  | LPOS RPOS PITCH° ROLL° 

  if (millis() - ms > 100) {
    printTime(gps.time); // Print GPS TIME
    Serial.print("> ");
    //Display LON & LAT
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    Serial.print(", "); 
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    Serial.print(", ");
    
    //Display SPEED & ALTITUDE
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2); //Speed
    Serial.print(", ");    
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2); //Altitude    
    Serial.print(", ");
    
    accelgyro.getMotion6( & ax, & ay, & az, & gx, & gy, & gz); // read raw accel/gyro measurements
    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax / ACCEL_SENS);
    Serial.print(" , ");
    Serial.print(ay / ACCEL_SENS);
    Serial.print(" , ");
    Serial.print(az / ACCEL_SENS);
    Serial.print(" , ");
    Serial.print(gx / GYRO_SENS);
    Serial.print(" , ");
    Serial.print(gy / GYRO_SENS);
    Serial.print(" , ");
    Serial.print(gz / GYRO_SENS);
    Serial.print(" , ");
    
    // read raw heading measurements
    mag.getHeading( & mx, & my, & mz);

    // display tab-separated mag x/y/z values
    Serial.print(mx);
    Serial.print(" , ");
    Serial.print(my);
    Serial.print(" , ");
    Serial.print(mz);
    Serial.print(" , ");

    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if (heading < 0) heading += 2 * M_PI;
    Serial.print(heading * 180 / M_PI);
    Serial.print(" , ");
    // display both wings possition format LPOS RPOS PITCH° ROLL°
    Serial.print("LwP-");
    Serial.print(Left_Wing.read());
    Serial.print(" , ");
    Serial.print("RwP-");  
    Serial.print(Right_Wing.read());
    Serial.print(" , ");  
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);

    // wait appropriate time for conversion (4.5ms delay)
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();
    
    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();

    // display measured values if appropriate
    Serial.print(temperature);
    Serial.print(" , ");   
    Serial.print(pressure / 100);
    Serial.print("\t");
    Serial.println(" | "); 
     ms = millis();
      // SD TXT FILE Format
      // time,accelerometer XYZ,gyroscope XYZ,magnetometer XYZ,heading,latitude,longtitude,speed,elevation,lwp,rwp,temperature,pressure
  
      // write the GY87 values to SD card
      myFile = SD.open("FlightLog.txt", FILE_WRITE);
       if (myFile) {
        myFile.print((float) gps.time.hour(), 0); myFile.print(":");
        myFile.print((float) gps.time.minute(), 0); myFile.print(":"); 
        myFile.print((float) gps.time.second(), 0); myFile.print(",");     
        
        myFile.print((float) gps.location.lat(), 6); myFile.print(","); 
        myFile.print((float) gps.location.lng(), 6); myFile.print(","); 
     
        myFile.print((float) gps.speed.kmph()); myFile.print(","); 
        myFile.print((float) gps.altitude.meters()); myFile.print(",");
        
        myFile.print((float) ax / scale);  myFile.print(","); 
        myFile.print((float) ay / scale);  myFile.print(","); 
        myFile.print((float) az / scale);  myFile.print(","); 
     
        myFile.print((float) gx / scale);  myFile.print(","); 
        myFile.print((float) gy / scale);  myFile.print(","); 
        myFile.print((float) gz / scale);  myFile.print(","); 
     
        myFile.print((float) mx / scale);  myFile.print(","); 
        myFile.print((float) my / scale);  myFile.print(","); 
        myFile.print((float) mz / scale);  myFile.print(","); 
      
        myFile.print((float) heading * 180 / M_PI); myFile.print(","); 

        myFile.print("LwP:"); myFile.print(Left_Wing.read()); myFile.print(",");
        myFile.print("RwP:"); myFile.print(Right_Wing.read()); myFile.print(",");
        
        myFile.print((float) temperature); myFile.print(",");
        myFile.println((float) pressure); 
        myFile.close();
    }
       // LoRa RFM95 Transmitting To Base 
      LoRa.beginPacket();
      LoRa.print((float) gps.time.hour(), 0); LoRa.print(":");
      LoRa.print((float) gps.time.minute(), 0); LoRa.print(":");
      LoRa.print((float) gps.time.second(), 0); LoRa.print(", ");
      LoRa.print((float) gps.location.lat(), 6); LoRa.print(", ");
      LoRa.print((float) gps.location.lng(), 6); LoRa.print(", ");
      LoRa.print(gps.speed.kmph()); LoRa.print(", ");
      LoRa.print(gps.altitude.meters()); LoRa.print(", ");
      LoRa.print((float) ax / scale); LoRa.print(", ");
      LoRa.print((float) ay / scale); LoRa.print(", ");
      LoRa.print((float) az / scale); LoRa.print(", ");
      LoRa.print((float) gx / scale); LoRa.print(", ");
      LoRa.print((float) gy / scale); LoRa.print(", ");
      LoRa.print((float) gz / scale); LoRa.print(", ");
      LoRa.print((float) mx / scale); LoRa.print(", ");
      LoRa.print((float) my / scale); LoRa.print(", ");
      LoRa.print((float) mz / scale); LoRa.print(", ");
      LoRa.print((float) heading * 180 / M_PI); LoRa.print(", ");
      LoRa.print("LwP: "); LoRa.print(Left_Wing.read()); LoRa.print(", ");
      LoRa.print("RwP: "); LoRa.print(Right_Wing.read()); LoRa.print(", ");
      LoRa.print ((float) temperature); LoRa.print(", ");
      LoRa.print ((float) pressure);
      LoRa.endPacket();


    // blink LED to indicate activity
    digitalWrite(BLUE_LED, state);
    state = !state;
    }
    {
     smartDelay(250);

     if (millis() > 5000 && gps.charsProcessed() < 10)
     Serial.println(F("No GPS data received: check wiring"));
  }
  
}



void modeTwo() {

  // START TESTING MODE:
  Serial.println("--== TEST MODE ==-- ");
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  tone(buzzer, 1000); // 
  delay(200);
  tone(buzzer, 500); // 
  delay(250);
  tone(buzzer, 1000); // 
  delay(10);
  noTone(buzzer); // Stop sound...

  //OPEN BOTH WINGS
  Left_Wing.write(0); // Left Wing OPEN
    Serial.print("LwP-");
    Serial.print(Left_Wing.read());
    Serial.print(" , ");
  Right_Wing.write(105); // Right Wing OPEN
    Serial.print("RwP-");  
    Serial.println(Right_Wing.read());
  delay(2000);

  //TEST LEFT WING'
  Serial.println("Left Wing Test");
  Left_Wing.write(0); // Left Wing OPEN
      Serial.print("LwP-");
      Serial.print(Left_Wing.read());
      Serial.print(" , ");
  delay(2500);
  Left_Wing.write(95); // Left Wing CLOSED
      Serial.print("LwP-");
      Serial.print(Left_Wing.read());
      Serial.print(" , ");
  delay(2500);
  Left_Wing.write(0); // Left Wing OPEN
      Serial.print("LwP-");
      Serial.println(Left_Wing.read());
  delay(3000);

  //TEST RIGHT WING
  Serial.println("Right Wing Test");
  Right_Wing.write(105); // Right Wing OPEN
    Serial.print("RwP-");  
    Serial.print(Right_Wing.read());
    Serial.print(" , ");
  delay(2500); 
  Right_Wing.write(0); // Right  Wing CLOSED
    Serial.print("RwP-");  
    Serial.print(Right_Wing.read());
    Serial.print(" , ");
  delay(2500);
  Right_Wing.write(105); // Right Wing OPEN
    Serial.print("RwP-");  
    Serial.println(Right_Wing.read());
  delay(3000);

  //CLOSE BOTH WINGS
  Serial.println("Close Wings");
  Left_Wing.write(95); // Left Wing CLOSED
    Serial.print("LwP-");  
    Serial.print(Left_Wing.read());
    Serial.print(" , ");
  Right_Wing.write(0); // Right Wing CLOSED
    Serial.print("RwP-");  
    Serial.println(Right_Wing.read());
  tone(buzzer, 1000); // 
  delay(10);
  tone(buzzer, 500); // 
  delay(250);
  tone(buzzer, 1000); // 
  delay(250);
  noTone(buzzer); // Stop sound...
  Serial.println("--== TEST MODE FINISHED SUCESSFULLY ==--");
  digitalWrite(BLUE_LED, LOW);
 
}
void modeOff() {
  Left_Wing.write(95); // Left Wing CLOSED
  Right_Wing.write(0); // Right Wing CLOSED
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  tone(buzzer, 200); // 
  delay(750);
  noTone(buzzer); // Stop sound... 
  digitalWrite(RED_LED, LOW); 
}


void loop() {
  
  digitalWrite(GREEN_LED, HIGH);
  // remember the selected mode; 255 indicates no mode is active
  static uint8_t mode = 255;
  
  myButton.update();
  if (myButton.isSingleClick()) {

   mode = 1;

  }

  if (myButton.isDoubleClick()) {
   
    mode = 2;
  
  }

  if (myButton.isLongClick()) {

    mode = 0;

  }
// honour the mode
  switch (mode)
  {
    case 0:
      modeOff();
      mode = 255;  // if you want to prevent modeOff to be repeated, set mode to 255
      break;
    case 1:
      modeOne();
      break;
    case 2:
      modeTwo();
      mode = 255; // Preventing Test Mode from repeating - 255
      break;     
  }

}
// close void loop
