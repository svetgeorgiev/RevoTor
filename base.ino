#include <SPI.h> // Include SPI library
#include <LoRa.h> // Include LoRa library for Radio Receiving

#define FQ     868E6 // Define Frequency

 
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Ground Base Receiver");

  if (!LoRa.begin(FQ)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
