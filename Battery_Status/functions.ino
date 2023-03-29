
// setRgbLedColor takes in either HIGH or LOW for red, green, and blue.
void setRgbLedColor(int red, int green, int blue)
{
  digitalWrite(RGB_RED_PIN, !red);
  digitalWrite(RGB_GREEN_PIN, !green);
  digitalWrite(RGB_BLUE_PIN, !blue);  
}

void emptyBattery() {
  setRgbLedColor(HIGH, LOW, LOW);
}

void lowBattery() {
  setRgbLedColor(HIGH, HIGH, LOW);
}


void mediumBattery() {
  setRgbLedColor(LOW, LOW, HIGH);
}

void fullBattery() {
  setRgbLedColor(LOW, HIGH, LOW);
}
