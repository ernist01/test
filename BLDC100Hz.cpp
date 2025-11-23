/*
 * Teensy 4.1 BLDC Motor Control (100Hz PWM)
 * * Hardware:
 * - Teensy 4.1
 * - BLDC ESC (Electronic Speed Controller)
 * * Connections:
 * - ESC Signal -> Pin 2
 * - ESC Ground -> GND
 * * Logic:
 * Standard ESCs expect a pulse between 1000us (Stop) and 2000us (Full Speed).
 * At 100Hz, the total period is 10ms (10,000us).
 * We calculate the specific duty cycle to match that microsecond duration.
 */

#include <Arduino.h>

// --- Configuration ---
const int pwmPin = 2;          // The pin connected to ESC signal
const int pwmFreq = 100;       // 100Hz Frequency
const int pwmResolution = 12;  // 12-bit resolution (0 - 4095 values) for smoother control

// --- ESC Limits ---
const int minPulse = 1000;     // 1000 microseconds (Stop)
const int maxPulse = 2000;     // 2000 microseconds (Full Throttle)

// Function Prototype
void writeMotorMicroseconds(int us);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(2000); // Wait for Serial to stabilize
  
  Serial.println("Initializing Teensy 4.1 BLDC Control...");

  // Configure the PWM Pin
  pinMode(pwmPin, OUTPUT);
  
  // Set the specific Frequency and Resolution
  // Teensy 4.x specific hardware abstraction
  analogWriteFrequency(pwmPin, pwmFreq);
  analogWriteResolution(pwmResolution);

  // --- ARMING SEQUENCE ---
  // Most ESCs need to see 0% throttle (1000us) for a few seconds 
  // to calibrate and "arm" before they will spin.
  Serial.println("Arming ESC (Sending Min Throttle)...");
  writeMotorMicroseconds(minPulse); 
  
  // Wait 3 seconds for the ESC to play its initialization tones
  delay(3000); 
  
  Serial.println("ESC Armed. Starting Loop.");
}

void loop() {
  // --- RAMP UP DEMO ---
  Serial.println("Ramping Up...");
  
  // Gently increase speed from 1000us to 1300us (approx 30% power)
  // CAUTION: 2000us is full power. We stop at 1300us for safety in this demo.
  for (int signal = 1000; signal <= 1300; signal += 5) {
    writeMotorMicroseconds(signal);
    delay(50); // Small delay to ramp smoothly
  }

  delay(1000); // Hold speed for 1 second

  // --- RAMP DOWN DEMO ---
  Serial.println("Stopping...");
  for (int signal = 1300; signal >= 1000; signal -= 5) {
    writeMotorMicroseconds(signal);
    delay(50);
  }

  delay(2000); // Wait 2 seconds before restarting cycle
}

/**
 * Helper function to convert microseconds to Duty Cycle value
 * * Calculation Logic:
 * 1. Frequency = 100Hz -> Period = 1/100 = 0.01s = 10,000 microseconds.
 * 2. Resolution = 12-bit -> Max Value = 4095 (2^12 - 1).
 * 3. Duty Cycle Ratio = Desired_Microseconds / Period_Microseconds.
 * 4. AnalogWrite Value = Duty_Cycle_Ratio * Max_Resolution_Value.
 */
void writeMotorMicroseconds(int us) {
  // Clamp the value for safety so we don't send garbage to the ESC
  if (us < minPulse) us = minPulse;
  if (us > maxPulse) us = maxPulse;

  // Calculate period in microseconds (1000000 us / 100 Hz = 10000 us)
  float periodUs = 1000000.0 / pwmFreq; 
  
  // Calculate the integer value for analogWrite
  // (us / 10000) * 4095
  int dutyValue = (int)(((float)us / periodUs) * (float)((1 << pwmResolution) - 1));

  analogWrite(pwmPin, dutyValue);
}
