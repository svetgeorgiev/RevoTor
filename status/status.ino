// Declare RGB PINS 
const int RGB_RED_PIN = 4;
const int RGB_GREEN_PIN = 0;
const int RGB_BLUE_PIN = 5;

// Declare BUZZER PIN
const unsigned char BUZZER = 15;

// Declare 5 seconds delay
const int DELAY_MS = 5000; // delay in ms between changing colors

void setup() {
  
  // Set the RGB pins to output
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode (BUZZER,OUTPUT);
  
  // Turn on Serial Monitor
  Serial.begin(9600); 
  delay(DELAY_MS);
  Serial.println(" Status Demo Begin ");
}

void loop() {

// 5 seconds demo for each Color with Tone in a loop

  errorYellowColor();      // Error 1 - SD Card 
  errorPurpleColor();      // Error 2 - Waiting for GPS Signal
  errorRedColor();         // Error 3 - General Error
  serviceTurqoiseColor();  // Service Begin
  serviceBlueColor();      // Service Complete
  goGreenColor();          // All Services OK!
  sosWhiteColor();         // Touch Down - SOS 
  
  
}
