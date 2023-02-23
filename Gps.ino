#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN 3
#define GPS_TX_PIN 2
#define LED_PIN 13

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

#define TARGET_LAT 37.7749
#define TARGET_LON -122.4194
#define THRESHOLD 100

// Filtering variables
const int FILTER_WINDOW_SIZE = 5;
float latFilter[FILTER_WINDOW_SIZE];
float lonFilter[FILTER_WINDOW_SIZE];
int filterIndex = 0;

// Time-based filtering variables
unsigned long lastReadingTime = 0;
const unsigned long MIN_READING_INTERVAL_MS = 1000;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      unsigned long currentTime = millis();

      // Time-based filtering
      if (currentTime - lastReadingTime < MIN_READING_INTERVAL_MS) {
        continue;
      }
      lastReadingTime = currentTime;

      // Store current GPS reading in the filter
      latFilter[filterIndex] = gps.location.lat();
      lonFilter[filterIndex] = gps.location.lng();
      filterIndex = (filterIndex + 1) % FILTER_WINDOW_SIZE;

      // Calculate average latitude and longitude over the filter window
      float latSum = 0, lonSum = 0;
      for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        latSum += latFilter[i];
        lonSum += lonFilter[i];
      }
      float lat = latSum / FILTER_WINDOW_SIZE;
      float lon = lonSum / FILTER_WINDOW_SIZE;

      // Calculate distance to target location
      float distance = gps.distanceBetween(lat, lon, TARGET_LAT, TARGET_LON);

      // Check if within threshold distance and turn on LED
      if (distance <= THRESHOLD && gps.satellites() > 0 && gps.hdop() < 3.0) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }

      // Print GPS data for debugging
      Serial.print("Latitude: ");
      Serial.print(lat, 6);
      Serial.print(", Longitude: ");
      Serial.print(lon, 6);
      Serial.print(", Distance: ");
      Serial.print(distance);
      Serial.print(", Satellites: ");
      Serial.print(gps.satellites());
      Serial.print(", HDOP: ");
      Serial.println(gps.hdop());
    }
  }
}
