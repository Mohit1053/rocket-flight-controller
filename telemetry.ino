#include <HardwareSerial.h>

// Define XBee serial on USART6 pins (PC6 = TX, PC7 = RX)
HardwareSerial XBeeSerial(PC6, PC7); // RX, TX

void setup() {
  Serial.begin(115200);        // USB Serial Monitor
  XBeeSerial.begin(115200);    // XBee baud rate
  delay(2000);

  Serial.println("=== STM32H743ZI-XBee Communication Test ===");
  Serial.println("Type here to send data via XBee");
}

void loop() {
  // Serial Monitor → XBee
  if (Serial.available()) {
    char c = Serial.read();
    XBeeSerial.write(c);
    Serial.print("[Sent]: ");
    Serial.println(c);
  }

  // XBee → Serial Monitor
  if (XBeeSerial.available()) {
    char c = XBeeSerial.read();
    Serial.print("[Received]: ");
    Serial.println(c);
  }
}