#include <SPI.h>
#include <LoRa.h>

// Use a specific SPI interface (STM32 often has multiple)
SPIClass LoRaSPI(PC12, PC11, PC10);   // MOSI, MISO, SCK

#define SS_PIN   PD15
#define RST_PIN  PD14
#define DIO0_PIN PE9

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("LoRa Chat Initializing...");

  // bind the LoRa driver to this SPI bus and pins
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed! Check wiring.");
    while (1);
  }

  Serial.println("LoRa Chat Ready");
}

void loop() {
  // check for a received packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg;
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }
    Serial.print("Received: ");
    Serial.println(msg);
  }

  // send whatever is typed into the Serial monitor
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();
      Serial.print("Sent: ");
      Serial.println(msg);
    }
  }
}