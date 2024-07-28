#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <AverageThermistor.h>
#include <Preferences.h>

#include "LimitedServo.cpp"

// Display config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// NTC thermistor config
#define NTC_THERMISTOR_PIN 34
#define REFERENCE_RESISTANCE 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define ESP32_ANALOG_RESOLUTION 4095
#define ESP32_ADC_VREF_MV 3300

Thermistor *thermistor;

// Temperature servo config
#define TEMP_SERVO_PIN 4

LimitedServo tempServo(TEMP_SERVO_PIN, 11, 110);

// Hall sensor config
#define HALL_PIN 2

// RPM measurement config
#define RPM_SAMPLE_SIZE 8
#define RPM_SERVO_PIN 15

LimitedServo rpmServo(RPM_SERVO_PIN, 0, 150);

volatile int rotationCount = 0;

// Persistent storage config
Preferences preferences;

volatile int totalRotations = 0;

// Count a rotation via the hall sensor and save it to the persistent storage
void countRotation()
{
  rotationCount++;
  totalRotations++;
  preferences.putInt("rotationCount", totalRotations);
}

void setup()
{
  Serial.begin(115200);

  // Set up the persistently saved values
  preferences.begin("motor-unit", false);
  totalRotations = preferences.getInt("rotationCount", 0);

  // Set up the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.display();

  // Set up the thermistor
  Thermistor *origin = new NTC_Thermistor_ESP32(
      NTC_THERMISTOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE,
      ESP32_ADC_VREF_MV,
      ESP32_ANALOG_RESOLUTION);

  // Make the thermistor measurements more stable by calculating average
  thermistor = new AverageThermistor(
      origin,
      10,
      10);

  // Set initial servo positions
  tempServo.setPosition(110);
  rpmServo.setPosition(0);

  // Set up the hall sensor and hook in to the interrupt
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), countRotation, FALLING);
}

// Map temperature to servo angle
int mapTemperatureToAngle(long temperature)
{
  if ((temperature > 90))
  {
    return map(temperature, 90, 120, 38, 11);
  }
  else if (temperature < 0)
  {
    return map(temperature, -20, 0, 110, 98);
  }
  else if (temperature < 60)
  {
    return map(temperature, 0, 60, 98, 73);
  }
  else
  {
    return map(temperature, 60, 90, 73, 38);
  }
}

// Map RPM to servo angle
int mapRpmToAngle(long rpm)
{
  return map(rpm, 100, 6000, 0, 150);
}

void loop()
{
  // Measure RPM — if it takes too long, skip the rest of the measurements
  long start = micros();
  while (rotationCount < RPM_SAMPLE_SIZE)
  {
    // Leave the measurement if it takes more than 2 seconds
    if (micros() - start > 2000000)
    {
      break;
    }
  }

  // Calculate RPM
  float seconds = (micros() - start) / 1000000.0;
  long rpm = rotationCount / seconds * 60.0;
  rotationCount = 0;

  // Set RPM servo position
  rpmServo.setPosition(mapRpmToAngle(rpm));

  // Get temperature
  long temperature = thermistor->readCelsius();

  // Set temperature servo position
  tempServo.setPosition(mapTemperatureToAngle(temperature));

  // Handle displaying the values
  display.clearDisplay();
  display.setCursor(0, 0);

  display.println(rpm);
  display.print(totalRotations);

  display.print(' ');
  display.println(temperature);

  display.display();
}