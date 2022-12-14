#include <SPI.h> // Include SPI library
#include <Wire.h> // Include Wire library
#include <I2Cdev.h> // Include I2Cdev library

#include <SD.h> // Include SD library to log DATA 
File myFile; // SD card Object
const int chipSelect = BUILTIN_SDCARD; // Teensy 4.1 BUILDIN_SDCARD

#include <TinyGPSPlus.h> // TinyGPS++ Lib
TinyGPSPlus gps;  // The TinyGPSPlus object
static const int RXPin = 0, TXPin = 1; // GPS PIN connections
static const uint32_t GPSBaud = 9600; // GPS Baud Rate 

#include <SoftwareSerial.h> // Include SoftwareSerial library
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

#include <BMP085.h> // BMP085 Lib
BMP085 barometer;
float temperature;
float pressure;
int32_t lastMicros;

#include <MPU6050.h> // MPU Lib
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS = 131.0; // Gyro Sensitivity with default +/- 250 deg/s scale

#include <HMC5883L.h> // HMC5883L Lib
HMC5883L mag;
int16_t mx, my, mz;


#include <LoRa.h> // LoRa Lib
#define SCK     13    // GPIO13  - Teensy 4.1 SCK
#define MISO    12   // GPIO12 - Teensy 4.1 MISO
#define MOSI    11   // GPIO11 - Teensy 4.1 MOSI
#define SS      10   // GPIO10 - Teensy 4.1 CS
#define RST     9   // GPIO9 - Teensy 4.1 RESET
#define G0     28   // GPIO28 - Teensy 4.1 IRQ(Interrupt Request)
#define FQ     8680222E2 // Define Frequency

  
void setup() {
  
  unsigned int count = 0;

 Serial.begin(9600);
  while (!Serial && (count < 30)) {
    delay(200); // Wait for serial port to connect with timeout. Needed for native USB
  }
  
  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
  
  // ==================== MPU6050 ============================
  accelgyro.initialize();
  Serial.print("Testing Accel/Gyro... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
  // Now we can talk to the HMC5883l

  // ==================== HMC5883L ============================
 //  mag.initialize();
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
    Serial.println("Try Again in 5 seconds...");
    delay(5000);
  }
  Serial.println("SD Card Initialized.");
  myFile = SD.open("FlightLog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.println("time,latitude,longitude,speed,elevation,accelerometer-XYZ,gyroscope-XYZ,magnetometer-XYZ,heading,temperature,pressure");
    myFile.close();   
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening FlighLog.csv");
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
    
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS data received: check wiring"));
      while (1) {}
    }
  }
  
  Serial.println("\n====// Setup Complete //====");
 
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

void loop() {
  
  static unsigned long ms = 0;
  static boolean state = !state;
  
  // Serial Output Format
  // === Time ===   |===== GPS DATA ===== |  ==== GPS DATA ==== |=== Accel === | === Gyro === | ======= Mag ======= | === Barometer === |  
  //    H:M:s       |    LAT      LNG     |    SPEED    ALT     |  X   Y   Z   |  X   Y   Z   |   X Y Z  Heading    |  Temp   Pressure  |

  if (millis() - ms > 100) {
    printTime(gps.time); // Print GPS TIME
    Serial.print("> ");
    //Display LON & LAT
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    Serial.print(", "); 
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    Serial.print(" | ");
    
    //Display SPEED & ALTITUDE
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2); //Speed
    Serial.print("| ");    
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2); //Altitude    
    Serial.print(" | ");
    
    accelgyro.getMotion6( & ax, & ay, & az, & gx, & gy, & gz); // read raw accel/gyro measurements
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
    if(heading < 0) heading += 2 * M_PI;
    Serial.print(heading * 180/M_PI); 
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
    Serial.print("\t");
    Serial.println(" | "); 
     ms = millis();
      /* SD TXT FILE Format
         time,accelerometer XYZ,gyroscope XYZ,magnetometer XYZ,heading,latitude,longtitude,speed,elevation,lwp,rwp,temperature,pressure
         write the GY87 values to SD card*/
      myFile = SD.open("FlightLog.csv", FILE_WRITE);
       if (myFile) {
        myFile.print((float) gps.time.hour(), 0); myFile.print(":");
        myFile.print((float) gps.time.minute(), 0); myFile.print(":"); 
        myFile.print((float) gps.time.second(), 0); myFile.print(",");     
        
        myFile.print((float) gps.location.lat(), 7); myFile.print(","); 
        myFile.print((float) gps.location.lng(), 7); myFile.print(","); 
     
        myFile.print((float) gps.speed.kmph()); myFile.print(","); 
        myFile.print((float) gps.altitude.meters()); myFile.print(",");
        
        myFile.print((float) ax / ACCEL_SENS);  myFile.print("-"); 
        myFile.print((float) ay / ACCEL_SENS);  myFile.print("-"); 
        myFile.print((float) az / ACCEL_SENS);  myFile.print(","); 
     
        myFile.print((float) gx / GYRO_SENS);  myFile.print("-"); 
        myFile.print((float) gy / GYRO_SENS);  myFile.print("-"); 
        myFile.print((float) gz / GYRO_SENS);  myFile.print(","); 
     
        myFile.print((float) mx );  myFile.print("-"); 
        myFile.print((float) my );  myFile.print("-"); 
        myFile.print((float) mz );  myFile.print(","); 
      
        myFile.print((float) heading * 180/M_PI ); myFile.print(","); 

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
      LoRa.print ((float) pressure);
      LoRa.endPacket();
    }
    {
     smartDelay(250);

     if (millis() > 1000 && gps.charsProcessed() < 10)
     Serial.println(F("No GPS data received: check wiring"));
  }

}
// close void loop
