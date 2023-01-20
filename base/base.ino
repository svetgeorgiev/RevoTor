#include <SPI.h> // Include SPI library
#include <LoRa.h> // Include LoRa library for Radio Receiving

#define FQ     8680222E2 // Define Frequency

#define GREEN_LED   A1        // Green Led
#define RED_LED     10        // Red Led 
 
void setup() {
  
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  boolean state = HIGH;
  
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("RevoTOR Ground Base Receiver");
  digitalWrite(GREEN_LED, HIGH);
  if (!LoRa.begin(FQ)) {
    Serial.println("Starting LoRa failed!");
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    while (1);
  }
}

void loop() {

 static boolean state = HIGH;
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Green Led Blink
    digitalWrite(GREEN_LED, state);
    state = !state;
    
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
  }
}
