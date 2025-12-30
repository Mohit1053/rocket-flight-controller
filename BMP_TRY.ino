// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME680.h>

// // Create sensor object for I2C
// Adafruit_BME680 bme;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   Serial.println("Adafruit BME680 test");

//   // Initialize BME680 sensor at I2C address 0x77
//   if (!bme.begin(0x77)) {
//     Serial.println("❌ Could not find a valid BME680 sensor at 0x77, check wiring!");
//     while (1);
//   }

//   // Set oversampling and filter
//   bme.setTemperatureOversampling(BME680_OS_8X);
//   bme.setHumidityOversampling(BME680_OS_2X);
//   bme.setPressureOversampling(BME680_OS_4X);
//   bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
//   bme.setGasHeater(320, 150); // 320°C for 150 ms

//   Serial.println("✅ BME680 sensor initialized successfully!");
// }

// void loop() {
//   // Perform a reading
//   if (!bme.performReading()) {
//     Serial.println("Failed to perform reading :(");
//     delay(1000);
//     return;
//   }

//   Serial.print("Temperature = ");
//   Serial.print(bme.temperature);
//   Serial.println(" °C");

//   Serial.print("Pressure = ");
//   Serial.print(bme.pressure / 100.0);
//   Serial.println(" hPa");

//   Serial.print("Humidity = ");
//   Serial.print(bme.humidity);
//   Serial.println(" %");

//   Serial.print("Gas Resistance = ");
//   Serial.print(bme.gas_resistance / 1000.0);
//   Serial.println(" KOhms");

//   Serial.println("---------------------------");
//   delay(2000); // 2 seconds delay
// }


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Create BME680 object
Adafruit_BME680 bme;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("BME680 on STM32L552ZE-Q test");

  // Initialize I2C4: SDA = PD13, SCL = PF14
  Wire.begin(PB9, PB8);

  // Initialize BME680 at I2C address 0x77
  if (!bme.begin(0x77, &Wire)) {
    Serial.println("❌ Could not find a valid BME680 sensor at 0x77, check wiring!");
    while (1);
  }

  // Configure oversampling and filter
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  Serial.println("✅ BME680 initialized successfully!");
}

void loop() {
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    delay(1000);
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas Resistance = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.println("---------------------------");
  delay(2000);
}