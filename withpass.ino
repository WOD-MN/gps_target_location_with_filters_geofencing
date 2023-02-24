#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define LED_PIN 10

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

float targetLat;
float targetLon;

#define THRESHOLD 100 //specific the distance as much as you like )

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("Enter in this format 'password,lat,lon' to set target location.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("saiman,")) {
      int separatorIndex1 = input.indexOf(',', 7);
      int separatorIndex2 = input.indexOf(',', separatorIndex1 + 1);
      if (separatorIndex1 != -1 && separatorIndex2 != -1) {
        targetLat = input.substring(separatorIndex1 + 1, separatorIndex2).toFloat();
        targetLon = input.substring(separatorIndex2 + 1).toFloat();
        Serial.println("Target location set:");
        Serial.print("Latitude: ");
        Serial.println(targetLat, 6);
        Serial.print("Longitude: ");
        Serial.println(targetLon, 6);
      }
    }
  }
  if (targetLat != 0 && targetLon != 0) {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        float lat = gps.location.lat();
        float lon = gps.location.lng();
        float distance = gps.distanceBetween(lat, lon, targetLat, targetLon);

        if (distance <= THRESHOLD) {
          digitalWrite(LED_PIN, HIGH);
        } else {
          digitalWrite(LED_PIN, LOW);
        }
        Serial.println(lat, 6);
        Serial.println(lon, 6);
        Serial.println(distance);
        Serial.println();
        delay(1000);
      }
    }
  }
}
