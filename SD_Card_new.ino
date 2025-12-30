#include <SPI.h>
#include <SD.h>

// Chip Select pin
#define SD_CS_PIN  PD5

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing SD card...");

  // Initialize SPI
  SPI.begin();  // Uses PA5 (SCK), PA6 (MISO), PA7 (MOSI)

  // Initialize SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized successfully!");

  // Write test
  File testFile = SD.open("himanshu.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Hello from STM32L552!");
    testFile.close();
    Serial.println("Data written to test.txt");
  } else {
    Serial.println("Error opening test.txt for writing");
  }

  // Read test
  testFile = SD.open("himanshu.txt");
  if (testFile) {
    Serial.println("Reading from test.txt:");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
  } else {
    Serial.println("Error opening test.txt for reading");
  }
}

void loop() {
  // nothing here
}