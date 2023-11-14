#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x5F);


void setup() {
  Serial.begin(9600);
  pwm.begin();
  Wire.setClock(200000);
  setColor(0, 0, 0); // Switch all LED's OFF
}

void loop() {
  // RGB test
  testRGB();
  delay(2000);  // Wait for 2 seconds before repeating the test
}

void testRGB() {
  Serial.println("RGB test!");
    
  // Buzzer
  beep(2047);
  Serial.println("Beep");
  delay(1000);
  
  // Red
  setColor(4095, 0, 0);
  Serial.println("Red");
  delay(1000);
  
  // Green
  setColor(0, 4095, 0);
  Serial.println("Green");  
  delay(1000);
  
  // Blue
  setColor(0, 0, 4095);
  Serial.println("Blue");
  delay(1000);

  // Yellow
  setColor(4095, 4095, 0);
  Serial.println("Yellow");
  delay(1000);

  // Magenta
  setColor(4095, 0, 4095);
  Serial.println("Magenta");
  delay(1000);

  // Cyan
  setColor(0, 4095, 4095);
  Serial.println("Cyan");
  delay(1000);

  // White
  setColor(4095, 4095, 4095);
  Serial.println("White");
  delay(1000);

  // Turn off all LEDs
  setColor(0, 0, 0);
  Serial.println("LED OFF");
  delay(2000);
}

void setColor(uint16_t red, uint16_t green, uint16_t blue) {
  // Invert the color values to address the unexpected behavior
  red = 4095 - red;
  green = 4095 - green;
  blue = 4095 - blue;

  // Set the color of the RGB LEDs
  pwm.setPWM(0, 0, red);     // RED - LED0
  pwm.setPWM(1, 0, green);   // GREEN - LED1
  pwm.setPWM(2, 0, blue);    // BLUE - LED2
}

void beep(uint16_t signal) {
   // Set the buzzer signal
   pwm.setPWM(11, 0, signal);
   delay(100);  // Add a delay to ensure the signal is processed
   pwm.setPWM(11, 0, 0);  // Turn off the buzzer
}
