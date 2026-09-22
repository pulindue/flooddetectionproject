#include <Arduino.h>
#include "esp_sleep.h"

// Set to true when running inside Wokwi simulator********
// Set to false when flashing to real ESP32 hardware**********
#define IS_SIMULATION true

// --- Pin Definitions ---
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// --- Physical Drain Configuration ---
const float DRAIN_DEPTH_MM = 1000.0; // Drain depth in mm

// --- Power Saving Sleep Configuration ---
#define TIME_TO_SLEEP_SEC 5 
#define uS_TO_S_FACTOR 1000000ULL

// --- Filtering Configuration ---
const int FILTER_WINDOW_SIZE = 5;

// RTA wala thiyena hinda , deep sleep eken passe values reset wenne na 
RTC_DATA_ATTR float readings[FILTER_WINDOW_SIZE] = {0};
RTC_DATA_ATTR int readIndex = 0;
RTC_DATA_ATTR float total = 0.0;
RTC_DATA_ATTR float filteredWaterLevel = 0.0;
RTC_DATA_ATTR float previousWaterLevel = 0.0;
RTC_DATA_ATTR bool isFirstBoot = true;

// Surge Threshold (mm/sec rise rate)
const float SURGE_THRESHOLD_MM_S = 10.0;

float measureRawDistanceMM() {
  // Ensure trigger line is clear and settled
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  
  // 10 microsecond trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse (40ms timeout corresponds to ~6.8m maximum distance)
  long duration = pulseIn(ECHO_PIN, HIGH, 40000);
  if (duration <= 0) {
    return -1.0;
  }
  
  // Speed of sound = 343 m/s = 0.343 mm/us
  return (duration * 0.343) / 2.0;
}

void processDrainMeasurement() {
  float rawDistanceMM = measureRawDistanceMM();

  if (rawDistanceMM >= 0) {
    // 1. Calculate water level from surface distance
    float currentWaterLevel = DRAIN_DEPTH_MM - rawDistanceMM;
    if (currentWaterLevel < 0) currentWaterLevel = 0;

    // 2. Moving Average Filter using RTC persistent buffer
    if (isFirstBoot) {
      for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        readings[i] = currentWaterLevel;
      }
      total = currentWaterLevel * FILTER_WINDOW_SIZE;
      filteredWaterLevel = currentWaterLevel;
      previousWaterLevel = currentWaterLevel;
      isFirstBoot = false;
    } else {
      total = total - readings[readIndex];
      readings[readIndex] = currentWaterLevel;
      total = total + readings[readIndex];
      readIndex = (readIndex + 1) % FILTER_WINDOW_SIZE;
      filteredWaterLevel = total / FILTER_WINDOW_SIZE;
    }

    // 3. Compute Rate of Rise (dh/dt)
    float timeDeltaSeconds = (float)TIME_TO_SLEEP_SEC;
    float rateOfRise = (filteredWaterLevel - previousWaterLevel) / timeDeltaSeconds;

    // 4. Alert Status Logic
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

    previousWaterLevel = filteredWaterLevel;
  } else {
    Serial.println("Sensor read timeout or invalid reading. Skipping cycle update...");
  }
}

void setup() {
  Serial.begin(115200);

  // Configure pins and guarantee default idle state
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  if (IS_SIMULATION) {
    Serial.println("Simulation Mode Initialized.");
  } else {
    // Hardware deep sleep execution flow
    Serial.println("\n=== Node Wakeup ===");
    
    // Allow sensor line voltage and transducer to stabilize after wake
    delay(50); 
    
    processDrainMeasurement();
    
    Serial.println("Entering deep sleep...");
    Serial.flush(); // Ensure all serial bytes transmit before shutting down CPU

    // Configure wakeup timer and enter deep sleep
    esp_sleep_enable_timer_wakeup((uint64_t)TIME_TO_SLEEP_SEC * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}

void loop() {    // meeka run wenne suimulation eke witarai , deep sleep yana hinda real hardware use karaddi
  if (IS_SIMULATION) {
    processDrainMeasurement();
    delay(TIME_TO_SLEEP_SEC * 1000);
  }
}