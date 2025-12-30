// #include <HardwareSerial.h>

// // -------------------- CONFIG --------------------
// // Using USART2 on Nucleo boards (PA2 = TX, PA3 = RX)
// HardwareSerial gpsSerial(USART2);

// String nmeaSentence = "";

// // -------------------- HELPERS --------------------

// // Convert raw NMEA lat/lon to decimal degrees
// double convertToDecimal(String raw, char hemi) {
//   if (raw == "") return 0.0;
//   double val = raw.toDouble();
//   int degrees = (int)(val / 100);
//   double minutes = val - (degrees * 100);
//   double decimal = degrees + minutes / 60.0;
//   if (hemi == 'S' || hemi == 'W') decimal = -decimal;
//   return decimal;
// }

// // Convert UTC (from GPS) → IST (UTC + 5:30)
// void convertUTCtoIST(int &hour, int &minute, int &second) {
//   hour += 5;
//   minute += 30;
//   if (minute >= 60) { minute -= 60; hour += 1; }
//   if (hour >= 24) hour -= 24;
// }

// // -------------------- GLOBAL STATE --------------------
// double lastSpeed = 0, lastCourse = 0;
// int lastHour = 0, lastMinute = 0, lastSecond = 0;
// double lastLat = 0, lastLon = 0;

// // -------------------- SETUP --------------------
// void setup() {
//   Serial.begin(115200);
//   gpsSerial.begin(9600);  // ✅ Default for NEO-M8N
//   Serial.println("Hour,Minute,Second,Latitude,Longitude,Speed(km/h),Altitude(m)");
// }

// // -------------------- LOOP --------------------
// void loop() {
//   while (gpsSerial.available() > 0) {
//     char c = gpsSerial.read();

//     if (c == '\n') {
//       nmeaSentence.trim();

//       if (nmeaSentence.startsWith("$GPRMC")) parseGPRMC(nmeaSentence);
//       else if (nmeaSentence.startsWith("$GPGGA")) parseGPGGA(nmeaSentence);

//       nmeaSentence = "";
//     } else {
//       nmeaSentence += c;
//     }
//   }
// }

// // -------------------- GPRMC PARSER --------------------
// void parseGPRMC(String sentence) {
//   int idx = 0, last = 0, field = 0;
//   String timeUTC, status, latRaw, latDir, lonRaw, lonDir, speedKnots, course;

//   while ((idx = sentence.indexOf(',', last)) != -1) {
//     String token = sentence.substring(last, idx);
//     last = idx + 1;
//     switch (field) {
//       case 1: timeUTC = token; break;
//       case 2: status = token; break;
//       case 3: latRaw = token; break;
//       case 4: latDir = token; break;
//       case 5: lonRaw = token; break;
//       case 6: lonDir = token; break;
//       case 7: speedKnots = token; break;
//       case 8: course = token; break;
//     }
//     field++;
//   }

//   if (status != "A") return;          // 'A' = active fix
//   if (timeUTC.length() < 6) return;   // invalid time

//   int hour = timeUTC.substring(0, 2).toInt();
//   int minute = timeUTC.substring(2, 4).toInt();
//   int second = timeUTC.substring(4, 6).toInt();
//   convertUTCtoIST(hour, minute, second);

//   lastHour = hour;
//   lastMinute = minute;
//   lastSecond = second;
//   lastLat = convertToDecimal(latRaw, latDir.charAt(0));
//   lastLon = convertToDecimal(lonRaw, lonDir.charAt(0));
//   lastSpeed = speedKnots.toFloat() * 1.852; // knots → km/h
//   lastCourse = course.toFloat();
// }

// // -------------------- GPGGA PARSER --------------------
// void parseGPGGA(String sentence) {
//   int idx = 0, last = 0, field = 0;
//   String latRaw, latDir, lonRaw, lonDir, altitude;

//   while ((idx = sentence.indexOf(',', last)) != -1) {
//     String token = sentence.substring(last, idx);
//     last = idx + 1;
//     switch (field) {
//       case 2: latRaw = token; break;
//       case 3: latDir = token; break;
//       case 4: lonRaw = token; break;
//       case 5: lonDir = token; break;
//       case 9: altitude = token; break;
//     }
//     field++;
//   }

//   double latitude = (latRaw.length() ? convertToDecimal(latRaw, latDir.charAt(0)) : lastLat);
//   double longitude = (lonRaw.length() ? convertToDecimal(lonRaw, lonDir.charAt(0)) : lastLon);

//   // Print combined GPS data
//   Serial.print(lastHour); Serial.print(",");
//   Serial.print(lastMinute); Serial.print(",");
//   Serial.print(lastSecond); Serial.print(",");
//   Serial.print(latitude, 6); Serial.print(",");
//   Serial.print(longitude, 6); Serial.print(",");
//   Serial.print(lastSpeed, 2); Serial.print(",");
//   Serial.println(altitude);
// }


// #include <HardwareSerial.h>

// // -------------------- CONFIG --------------------
// HardwareSerial gpsSerial(USART2);   // Using USART2 (PA2=TX, PA3=RX)

// void setup() {
//   Serial.begin(115200);     // For your PC Serial Monitor
//   gpsSerial.begin(9600);    // Default baud for NEO-M8N
//   Serial.println("=== GPS RAW DATA TEST STARTED ===");
//   Serial.println("Waiting for NMEA sentences...");
// }

// void loop() {
//   while (gpsSerial.available() > 0) {
//     char c = gpsSerial.read();
//     Serial.write(c);  // Directly forward every GPS character to Serial Monitor
//   }
// }


#include <HardwareSerial.h>
#include <TinyGPS++.h>

// -------------------- CONFIG --------------------
HardwareSerial gpsSerial(USART2);   // Use USART2 (PA2 = TX, PA3 = RX)
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);   // NEO-M8N default baud rate
  Serial.println("=== NEO-M8N GPS Parser Started ===");
}

void loop() {
  // Read all available GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // When a new location is available, print all info
  if (gps.location.isUpdated()) {
    Serial.println("====================================");
    Serial.print("Latitude:  "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude:  "); Serial.print(gps.altitude.meters()); Serial.println(" m");
    Serial.print("Speed:     "); Serial.print(gps.speed.kmph()); Serial.println(" km/h");
    
    if (gps.date.isValid() && gps.time.isValid()) {
      Serial.print("Date: "); 
      Serial.print(gps.date.day()); Serial.print("/");
      Serial.print(gps.date.month()); Serial.print("/");
      Serial.println(gps.date.year());
      
      Serial.print("Time (UTC): ");
      Serial.print(gps.time.hour()); Serial.print(":");
      Serial.print(gps.time.minute()); Serial.print(":");
      Serial.println(gps.time.second());
    } else {
      Serial.println("Date/Time: Invalid (no fix yet)");
    }
    
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    Serial.print("HDOP: "); Serial.println(gps.hdop.hdop());
    Serial.println("====================================");
  }
}

