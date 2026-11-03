/*
  HC-SR04 classroom radar: one UNO R4 WiFi, two sensors, optional third.
  See ../../tutorial/README.md before wiring or uploading.
  Serial Monitor: 115200 baud. All distances are centimetres.
*/

#include <Arduino.h>
#include <math.h>

// Start with 1 for the first exercise, then use 2; 3 enables the centre check.
constexpr uint8_t SENSOR_COUNT = 2;
constexpr uint8_t TRIGGER_PINS[] = {2, 4, 6};  // Left, right, centre.
constexpr uint8_t ECHO_PINS[] = {3, 5, 7};
constexpr float BASELINE_CM = 10.0f;  // Measured left-to-right module spacing.
constexpr float OFFSET_CM[] = {0.0f, 0.0f, 0.0f};  // Measured minus true range.
constexpr float THIRD_TOLERANCE_CM = 3.0f;  // Teaching threshold, not accuracy.
constexpr unsigned long ECHO_TIMEOUT_US = 25000UL;
constexpr unsigned long QUIET_MS = 65UL;
constexpr float RAD_TO_DEGREES = 180.0f / 3.14159265f;
static_assert(SENSOR_COUNT >= 1 && SENSOR_COUNT <= 3, "Use 1, 2 or 3 sensors");
static_assert(BASELINE_CM > 0.0f, "Measure a positive baseline");

bool usableRange(float cm) {
  return isfinite(cm) && cm >= 2.0f && cm <= 400.0f;
}

float readDistance(uint8_t sensor) {
  digitalWrite(TRIGGER_PINS[sensor], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINS[sensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINS[sensor], LOW);

  const unsigned long echoUs =
      pulseIn(ECHO_PINS[sensor], HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) {
    return -1.0f;  // No complete echo; this is NOT a zero-distance target.
  }
  const float cm = echoUs / 58.0f - OFFSET_CM[sensor];
  return usableRange(cm) ? cm : -1.0f;
}

// Same coordinate convention as SevIQ's bridge2d.py, with strict rejection.
// Sensors: (-baseline/2, 0), (+baseline/2, 0). Forward is +y, right is +x.
bool solvePosition(float left, float right, float baseline, float &x, float &y) {
  if (!usableRange(left) || !usableRange(right) ||
      !isfinite(baseline) || baseline <= 0.0f) {
    return false;
  }
  // Both triangle inequalities matter: the circles must intersect.
  if (fabsf(left - right) > baseline || left + right < baseline) {
    return false;
  }
  const float candidateX = (left * left - right * right) / (2.0f * baseline);
  const float fromLeft = candidateX + baseline / 2.0f;
  const float ySquared = left * left - fromLeft * fromLeft;
  if (ySquared <= 0.0f) {
    return false;  // No forward intersection (including a tangent at y=0).
  }
  x = candidateX;
  y = sqrtf(ySquared);  // Choose the forward of the two mirror solutions.
  return true;
}

bool centerAgrees(float center, float x, float y) {
  if (!usableRange(center) || !isfinite(x) || !isfinite(y) || y <= 0.0f) {
    return false;
  }
  const float predictedCenter = sqrtf(x * x + y * y);
  return fabsf(center - predictedCenter) <= THIRD_TOLERANCE_CM;
}

void setup() {
  Serial.begin(115200);
  const unsigned long startMs = millis();
  while (!Serial && millis() - startMs < 3000UL) {
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(TRIGGER_PINS[i], OUTPUT);
    digitalWrite(TRIGGER_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }
  delay(QUIET_MS);
  Serial.println("ms,left_cm,right_cm,center_cm,x_cm,y_cm,bearing_deg,status");
}

void loop() {
  // A new set every time: an old good reading must not survive a missing echo.
  float ranges[] = {-1.0f, -1.0f, -1.0f};
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ranges[i] = readDistance(i);
    // GLOBAL quiet time: wait after every sensor, including the last one.
    // The next transmitter cannot start while this one is still being read.
    delay(QUIET_MS);
  }

  float x = 0.0f;
  float y = 0.0f;
  bool havePosition = false;
  const char *status = "RANGE_ONLY";
  if (SENSOR_COUNT >= 2) {
    havePosition = solvePosition(ranges[0], ranges[1], BASELINE_CM, x, y);
    status = havePosition ? "PAIR_ESTIMATE" : "NO_FIX";
  }
  if (SENSOR_COUNT == 3 && havePosition) {
    if (!usableRange(ranges[2])) {
      havePosition = false;
      status = "THIRD_MISSING";
    } else if (!centerAgrees(ranges[2], x, y)) {
      havePosition = false;
      status = "THIRD_DISAGREES";
    } else {
      status = "THIRD_CONSISTENT";
    }
  }

  Serial.print(millis());  // End of the set, not a simultaneous capture time.
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(',');
    Serial.print(ranges[i], 1);
  }
  Serial.print(',');
  if (havePosition) {
    Serial.print(x, 1);
    Serial.print(',');
    Serial.print(y, 1);
    Serial.print(',');
    Serial.print(atan2f(x, y) * RAD_TO_DEGREES, 1);
    Serial.print(',');
  } else {
    Serial.print("NA,NA,NA,");
  }
  Serial.println(status);
}
