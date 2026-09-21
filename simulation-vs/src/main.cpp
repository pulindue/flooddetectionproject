#include <Arduino.h>


// Pin Definitions (matching your Wokwi setup)
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// Physical Drain Configuration (Assumed 1000 mm depth; adjust to your drain size)
const float DRAIN_DEPTH_MM = 1000.0;  //depth eka adala widiht adjust krnna

// Filtering Configuration
const int FILTER_WINDOW_SIZE = 5;
float readings[FILTER_WINDOW_SIZE];
int readIndex = 0;
float total = 0.0;
float filteredWaterLevel = 0.0;

// Surge Detection Variables
float previousWaterLevel = 0.0;
unsigned long previousTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 1000; // Sample every 1 second

// Surge Threshold (mm/sec rise rate)
const float SURGE_THRESHOLD_MM_S = 10.0; // Triggers alert if rising faster than 10 mm/s

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize moving average buffer
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    readings[i] = 0.0;
  }
}

float measureRawDistanceMM() {
  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse duration in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m max range)
  if (duration == 0) {
    return -1.0; // Measurement timeout or out of range
  }

  // Speed of sound = 0.343 mm/us (divided by 2 for round-trip)
  return (duration * 0.343) / 2.0;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousTime >= SAMPLE_INTERVAL_MS) {
    float rawDistanceMM = measureRawDistanceMM();

    if (rawDistanceMM >= 0) {
      // 1. Calculate water level from sensor mounting height: h = drain_depth - distance
      float currentWaterLevel = DRAIN_DEPTH_MM - rawDistanceMM;
      if (currentWaterLevel < 0) currentWaterLevel = 0;

      // 2. Apply Moving Average Filter to eliminate surface wave/ripple noise
      total = total - readings[readIndex];
      readings[readIndex] = currentWaterLevel;
      total = total + readings[readIndex];
      readIndex = (readIndex + 1) % FILTER_WINDOW_SIZE;
      filteredWaterLevel = total / FILTER_WINDOW_SIZE;

      // 3. Compute Rate of Rise (delta_h / delta_t)
      float timeDeltaSeconds = (currentTime - previousTime) / 1000.0;
      float rateOfRise = (filteredWaterLevel - previousWaterLevel) / timeDeltaSeconds;

      // 4. Determine Alert State
      String status = "NORMAL";
      if (rateOfRise >= SURGE_THRESHOLD_MM_S) {
        status = "SURGE WARNING (RAPID INFLOW)";
      } else if (filteredWaterLevel >= (DRAIN_DEPTH_MM * 0.7)) {
        status = "OVERFLOW CRITICAL (CAPACITY LIMIT)";
      }



      // 5. Output Telemetry Data to Serial Monitor
      Serial.println("----------------------------------------");
      Serial.print("Raw Distance: "); Serial.print(rawDistanceMM); Serial.println(" mm");
      Serial.print("Filtered Water Level: "); Serial.print(filteredWaterLevel); Serial.println(" mm");
      Serial.print("Rate of Rise (dh/dt): "); Serial.print(rateOfRise); Serial.println(" mm/s");
      Serial.print("System Status: "); Serial.println(status);

      // Save state for subsequent iteration
      previousWaterLevel = filteredWaterLevel;
      previousTime = currentTime;
    } else {
      Serial.println("Sensor read timeout. Verifying echo pulse...");
      previousTime = currentTime;
    }
  }
}

//ERROR404