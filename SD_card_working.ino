#include <SPI.h>
#include "SdFat.h"

#define SD_CS_PIN PD14

// Use SPI3 (PC12=MOSI, PC11=MISO, PC10=SCK)
//PB5,PB4,PB3
SPIClass mySPI(PC12, PC11, PC10);

// Create SdFat instance
SdFat sd;
File testFile;

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (!Serial);

  Serial.println("=== STM32H7A3ZI-Q SD Card Test (SPI3 Mode) ===");

  mySPI.begin();

  // Correct way: use SdSpiConfig
  SdSpiConfig spiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &mySPI);

  if (!sd.begin(spiConfig)) {
    Serial.println("SD card init failed!");
    while (1) {
      delay(1000);
      Serial.println("Insert/check SD card wiring...");
    }
  }

  Serial.println("SD card initialized successfully!");

  // Write a test file
  testFile = sd.open("HI", FILE_WRITE);
  if (testFile) {
    testFile.println("Hello from STM32H7A3ZI-Q over SPI35!");
    testFile.close();
    Serial.println("Wrote belo.txt successfully.");
  } else {
    Serial.println("Failed to create belo.txt");
  }

  // Read the test file
  testFile = sd.open("HI", FILE_READ);
  if (testFile) {
    Serial.println("Reading belo.txt:");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
    Serial.println("\nRead complete.");
  } else {
    Serial.println("Failed to open belo.txt for reading.");
  }
}

void loop() {
  // nothing
}