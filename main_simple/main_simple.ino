#include <SD.h> // Include SD library to log DATA 
#include <SPI.h> // Include SPI library
#include <Wire.h> // Include Wire library
#include <LoRa.h> // Include LoRa library for Radio Transmitting
#include <I2Cdev.h> // Include I2Cdev library
#include "BMP085.h" // Include BMP085 library for Temperature & Pressure readings
#include "MPU6050.h" // Include MPU6050 library for Orientation
#include "HMC5883L.h" // Include HMC5883L library for compass
#include <PinButton.h> // Include PinButton library for ground control Push Button 
#include <TinyGPSPlus.h> // Include TinyGPS++ library for GPS DATA
#include <ServoEasing.hpp> // Include ServoEasing library for more precise movement over the servo's
#include <SoftwareSerial.h> // Include SoftwareSerial library


#define SCK     13        // Define SCK PIN 
#define MISO    12        // Define MISO PIN 
#define MOSI    11        // Define MOSI PIN 
#define SS      10        // Define CS PIN For LoRa RFN95
#define RST     9         // Define RESET PIN For LoRa RFN95
#define G0      28        // Define IRQ(Interrupt Request) PIN For LoRa RFN95
#define FQ      8680222E2 // Define LoRa Frequency
#define RXPin   0         // Define RX Pin For GPS
#define TXPin   1         // Define TX Pin For GPS
#define GPSBaud 9600      // Define GPS Baud Rate



const int BUZZER = 31;     // Buzzer
const char LED = LED_BUILTIN;   // RED LED PIN 


// SD Card Object
File myFile;
int chipSelect = BUILTIN_SDCARD; // Teensy 4.1 BUILDIN_SDCARD

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;

// Push Button object
PinButton myButton(6); 

// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

/* Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
 specific I2C addresses may be passed as a parameter here */
MPU6050 accelgyro;
const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
const float GYRO_SENS = 133.0; // Gyro Sensitivity with default +/- 250 deg/s scale

int16_t ax, ay, az;
int16_t gx, gy, gz;

/* Barometer class default I2C address is 0x77
 specific I2C addresses may be passed as a parameter here
 (though the BMP085 supports only one address)*/
BMP085 barometer;

float temperature;
float pressure;
int32_t lastMicros;


void setup() {
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
 Wire.begin();
 Wire.setClock(400000); 
      
  boolean state = HIGH;
  
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.begin(115200);
  ss.begin(GPSBaud);
  digitalWrite(LED, HIGH);

  // ===================== SD CARD =========================
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    digitalWrite(LED, HIGH);
    smartDelay(5000);
  }
  Serial.println("SD Card Initialized.");
  myFile = SD.open("FlightLog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.println("time,lat,lng,alt,speed,accelerometer-XYZ,gyroscope-XYZ,magnetometer-XYZ,heading,temperature,pressure");
    myFile.close();
  }

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
 
  // ==================== LoRa initialising ============================
   Serial.print("Starting LoRa Transmitter... ");
   SPI.begin();
   LoRa.begin(FQ);
   LoRa.setPins(SS, RST, G0);
    if (!LoRa.begin(FQ)) {
     Serial.println("Starting LoRa failed!");
     digitalWrite(LED, HIGH);
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
    //Blink LED
    digitalWrite(LED, state);
    state = !state;
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      tone(BUZZER, 200); // Make Sound
      smartDelay(750);
      noTone(BUZZER); // Stop sound... 
      Serial.println(F("No GPS data received: check wiring"));
      while (1) {}
    }
    
  }
  
  Serial.println("Setup Complete");
  
  
  tone(BUZZER, 450); // Start sound
  smartDelay(150);
  tone(BUZZER, 950); 
  smartDelay(150);
  tone(BUZZER, 1350); 
  smartDelay(200);
  noTone(BUZZER); 
}

void modeOne() {

  while (ss.available() > 0)
    gps.encode(ss.read());

  static unsigned long ms = 0;
  static boolean state = HIGH;

  // Serial Output Format
  //  ====== GPS ====== | === Accel === | === Gyro === | ======= Mag ======= | === Barometer === |
  //TIME  LAT  LNG  ALT |  X   Y   Z   |  X   Y   Z   |  X   Y   Z  Heading |  Temp   Pressure   |

  if (millis() - ms > 100) {
    
  //GPS Data
      char buf[80];
      sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      Serial.print(buf);
      Serial.print(" : ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" | ");
      Serial.print(gps.altitude.meters());
      Serial.print(" | ");
      Serial.print(gps.speed.kmph());
      Serial.print(" | ");
  
  // MPU6050 Data  

    // read raw accel/gyro measurements
    accelgyro.getMotion6( & ax, & ay, & az, & gx, & gy, & gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax / ACCEL_SENS);
    Serial.print(" , ");
    Serial.print(ay / ACCEL_SENS);
    Serial.print(" , ");
    Serial.print(az / ACCEL_SENS);
    Serial.print(" | ");
    Serial.print(gx / GYRO_SENS);
    Serial.print(" , ");
    Serial.print(gy / GYRO_SENS);
    Serial.print(" , ");
    Serial.print(gz / GYRO_SENS);
    Serial.print(" | ");

    // read raw heading measurements
    mag.getHeading( & mx, & my, & mz);

    // display tab-separated mag x/y/z values
    Serial.print(mx);
    Serial.print(" , ");
    Serial.print(my);
    Serial.print(" , ");
    Serial.print(mz);
    Serial.print(" | ");

    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if (heading < 0) heading += 2 * M_PI;
    Serial.print(heading * 180 / M_PI);
    Serial.print(" | ");

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
    Serial.print(" | ");
    Serial.print(pressure / 100);
    Serial.println("\t");
  
   // Recording the sama data from the Serial to the SD card 
    myFile = SD.open("FlightLog.csv", FILE_WRITE);
    if (myFile) {
      
      char buf[80];
      sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      myFile.print(buf);
      myFile.print(",");
      myFile.print((float) gps.location.lat(), 6);
      myFile.print(",");
      myFile.print((float) gps.location.lng(), 6);
      myFile.print(",");
      myFile.print((float) gps.altitude.meters());
      myFile.print(",");
      myFile.print((float) gps.speed.kmph());
      myFile.print(",");
      myFile.print((float) ax / ACCEL_SENS);
      myFile.print("-");
      myFile.print((float) ay / ACCEL_SENS);
      myFile.print("-");
      myFile.print((float) az / ACCEL_SENS);
      myFile.print(",");

      myFile.print((float) gx / GYRO_SENS);
      myFile.print("-");
      myFile.print((float) gy / GYRO_SENS);
      myFile.print("-");
      myFile.print((float) gz / GYRO_SENS);
      myFile.print(",");

      myFile.print((float) mx);
      myFile.print("-");
      myFile.print((float) my);
      myFile.print("-");
      myFile.print((float) mz);
      myFile.print(",");

      myFile.print((float) heading * 180 / M_PI);
      myFile.print(",");

      myFile.print((float) temperature);
      myFile.print(",");
      myFile.println((float) pressure);
      myFile.close();

      ms = millis();
      digitalWrite(LED, state);
      state = !state;

    }
 // LoRa RFM95 Transmitting data To Base 
      LoRa.beginPacket();
      LoRa.print((float) gps.time.hour(), 0); LoRa.print(":");
      LoRa.print((float) gps.time.minute(), 0); LoRa.print(":");
      LoRa.print((float) gps.time.second(), 0); LoRa.print(", ");
      LoRa.print((float) gps.location.lat(), 6); LoRa.print(", ");
      LoRa.print((float) gps.location.lng(), 6); LoRa.print(", ");
      LoRa.print(gps.altitude.meters()); LoRa.print(", ");
      LoRa.print(gps.speed.kmph()); LoRa.print(", ");
      LoRa.print((float) ax / ACCEL_SENS); LoRa.print(", ");
      LoRa.print((float) ay / ACCEL_SENS); LoRa.print(", ");
      LoRa.print((float) az / ACCEL_SENS); LoRa.print(", ");
      LoRa.print((float) gx / GYRO_SENS); LoRa.print(", ");
      LoRa.print((float) gy / GYRO_SENS); LoRa.print(", ");
      LoRa.print((float) gz / GYRO_SENS); LoRa.print(", ");
      LoRa.print((float) mx ); LoRa.print(", ");
      LoRa.print((float) my ); LoRa.print(", ");
      LoRa.print((float) mz ); LoRa.print(", ");
      LoRa.print((float) heading * 180/M_PI ); LoRa.print(", ");
      LoRa.print ((float) temperature); LoRa.print(", ");
      LoRa.println ((float) pressure);
      LoRa.endPacket();
 
  }
}

void modeOff() { 
  tone(BUZZER, 1350); // Make sound
  smartDelay(150);
  tone(BUZZER, 950); 
  smartDelay(150);
  tone(BUZZER, 450); 
  smartDelay(200);
  noTone(BUZZER); 
}

void loop() {
 
  // remember the selected mode; 255 indicates no mode is active
  static uint8_t mode = 255;
  
  myButton.update();
  if (myButton.isSingleClick()) {

   mode = 1;
  tone(BUZZER, 1200); // Make sound 
  smartDelay(500);
  noTone(BUZZER);  
  smartDelay(500);
  tone(BUZZER, 1200); // Make sound
  smartDelay(500);
  noTone(BUZZER); 
  smartDelay(500);
  tone(BUZZER, 1800); // Make sound
  smartDelay(750);
  noTone(BUZZER); 
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
  }

}
// close void loop

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
