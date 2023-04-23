//  Web Settings 
void handleRoot(AsyncWebServerRequest * request) {
  String html;
  String modeText;
  String latitude = String(gps.location.lat(), 6);
  String longitude = String(gps.location.lng(), 6);
  String altitude = String(gps.altitude.meters());
  String speed = String(gps.speed.mps() * 3.6);
  String satellites = String(gps.satellites.value());
  int batteryLevel = getBatteryLevel();

  if (currentMode == 1) {
    modeText = "Ground Mode";
    File file = SPIFFS.open("/ground_mode.html", "r"); // open the HTML file
    html = file.readString(); // read the contents of the file into a String
    file.close(); // close the file

  } else if (currentMode == 2) {
    modeText = "Test Mode";
    File file = SPIFFS.open("/test_mode.html", "r"); // open the HTML file
    html = file.readString(); // read the contents of the file into a String
    file.close(); // close the file
  } else if (currentMode == 3) {
    modeText = "Flight Mode";
    File file = SPIFFS.open("/flight_mode.html", "r"); // open the HTML file
    html = file.readString(); // read the contents of the file into a String
    file.close(); // close the file
  } else {
    modeText = "ACTIVATE!";
    File file = SPIFFS.open("/index.html", "r"); // open the HTML file
    html = file.readString(); // read the contents of the file into a String
    file.close(); // close the file
  }

  // Replace the placeholders in the HTML file with the GPS coordinates
  html.replace("{MODE}", modeText);
  html.replace("{BATTERY_LEVEL}", String(batteryLevel));
  html.replace("{LATITUDE}", latitude);
  html.replace("{LONGITUDE}", longitude);
  html.replace("{ALTITUDE}", altitude);
  html.replace("{SPEED}", speed);
  html.replace("{SATS}", satellites);

  request -> send(200, "text/html", html);
}

void handleData(AsyncWebServerRequest * request) {
  int batteryLevel = getBatteryLevel();
  float satellites = (float) gps.satellites.value();
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  float altitude = gps.altitude.meters();
  float speed = gps.speed.kmph();

  String json = "{\"battery\": " + String(batteryLevel) + ", \"sats\": " + String(satellites) +
    ", \"latitude\": " + String(latitude, 6) + ", \"longitude\": " + String(longitude, 6) +
    ", \"altitude\": " + String(altitude) + ", \"speed\": " + String(speed) + "}";

  request -> send(200, "application/json", json);
}

// Battery Status Function
int8_t getBatteryLevel() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1) == 1) {
    switch (Wire.read() & 0xF0) {
    case 0xE0:
      return 25;
    case 0xC0:
      return 50;
    case 0x80:
      return 75;
    case 0x00:
      return 100;
    default:
      return 0;
    }
  }
  return -1;
}

// Tones and Sounds
void switchOffTone() {
  // Play an switch off tone
  tone(BUZZER_PIN, 1350); // Make Switching OFF sound
  delay(150);
  tone(BUZZER_PIN, 950);
  delay(150);
  tone(BUZZER_PIN, 450);
  delay(200);
  noTone(BUZZER_PIN);
}
void switchOnTone() {
  // Play an switch on tone
  tone(BUZZER_PIN, 450); // Start sound
  delay(150);
  tone(BUZZER_PIN, 950);
  delay(150);
  tone(BUZZER_PIN, 1350);
  delay(200);
  noTone(BUZZER_PIN);
}
void errorTone() {
  // Play an error tone
  tone(BUZZER_PIN, 2000, 250); // Frequency = 2000 Hz, Duration = 250 ms
  delay(300); // Wait for 300 ms
  tone(BUZZER_PIN, 1000, 250); // Frequency = 1000 Hz, Duration = 250 ms
  delay(300); // Wait for 300 ms
  tone(BUZZER_PIN, 500, 250); // Frequency = 500 Hz, Duration = 250 ms
  delay(400); // Wait for 400 ms
  noTone(BUZZER_PIN); // Stop the tone
}
void beepTone() {
  // Play an beep tone
  tone(BUZZER_PIN, BUZZER_FREQ, 100); // play short sound
  delay(100); // wait 10th of a second
  noTone(BUZZER_PIN);
}
