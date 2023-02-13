
// setRgbLedColor takes in either HIGH or LOW for red, green, and blue.
void setRgbLedColor(int red, int green, int blue)
{
  digitalWrite(RGB_RED_PIN, !red);
  digitalWrite(RGB_GREEN_PIN, !green);
  digitalWrite(RGB_BLUE_PIN, !blue);  
}

void errorRedColor() {
  Serial.println("Red: General Error!");
  setRgbLedColor(HIGH, LOW, LOW);
  tone(BUZZER, 200); // Make Sound
  delay(750);
  noTone(BUZZER); // Stop sound... 
  delay(DELAY_MS);
}

void errorPurpleColor() {
  Serial.println("Purple: Waiting for GPS Signal!");
  setRgbLedColor(HIGH, LOW, HIGH);
  delay(DELAY_MS);
}

void goGreenColor() {
  Serial.println("Green: All System OK - Good to Go!");
  setRgbLedColor(LOW, HIGH, LOW);
  tone(BUZZER, 1000); // 
  delay(10);
  tone(BUZZER, 500); // 
  delay(150);
  tone(BUZZER, 1000); // 
  delay(200);
  noTone(BUZZER); // Stop sound...
  delay(DELAY_MS);
}

void errorYellowColor() {
  Serial.println("Yellow: SD CARD No Detected!");
  setRgbLedColor(HIGH, HIGH, LOW);
  tone(BUZZER, 200); // Make Sound
  delay(750);
  noTone(BUZZER); // Stop sound... 
  delay(DELAY_MS);
}

void serviceTurqoiseColor() {
  Serial.println("Turqoise: Service Mode Begin");
  setRgbLedColor(LOW, HIGH, HIGH);
  tone(BUZZER, 1000); // 
  delay(200);
  tone(BUZZER, 500); // 
  delay(250);
  tone(BUZZER, 1000); // 
  delay(250);
  tone(BUZZER, 1500); // 
  delay(800);
  noTone(BUZZER); // Stop sound...
  delay(DELAY_MS);
}

void serviceBlueColor() {
  Serial.println("Blue: Service Mode Completed ");
  setRgbLedColor(LOW, LOW, HIGH);
  tone(BUZZER, 1000); // 
  delay(200);
  tone(BUZZER, 500); // 
  delay(250);
  tone(BUZZER, 1000); // 
  delay(10);
  noTone(BUZZER); // Stop sound...
  delay(DELAY_MS);
}

void sosWhiteColor() {
  Serial.println("White: SOS Mode - Touch Down -");
  setRgbLedColor(HIGH, HIGH, HIGH);
  tone(BUZZER, 2000); // 
  delay(750);
  noTone(BUZZER); // Stop sound...
  tone(BUZZER, 2000); // 
  delay(750);
  noTone(BUZZER); // Stop sound...
  tone(BUZZER, 2000); // 
  delay(750);
  noTone(BUZZER); // Stop sound...
  tone(BUZZER, 2000); // 
  delay(1000);
  noTone(BUZZER); // Stop sound...
  delay(DELAY_MS);
}
