#include <Wire.h>
#include <string>

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
#define CHAR_BASE_SIZE 6

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

volatile unsigned long rotationCount = 0;

// Persistent storage config
Preferences preferences;

volatile unsigned long totalRotations = 0;
volatile unsigned long motoMilliseconds = 0;

#define BTN_PIN 5

// Count a rotation via the hall sensor and save it to the persistent storage
void countRotation()
{
  rotationCount++;
  totalRotations++;
}

long temperature = 0;
unsigned long rpm = 0;

#define RPM_TEXT "rpm"

// Clear display and the print all values
void syncDisplay()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);

  // Print RPM so there is space for 4 digits
  auto rpmStr = std::to_string(rpm);
  display.setCursor((4 - rpmStr.length()) * CHAR_BASE_SIZE * 2, 0);
  display.setTextSize(2);
  display.print(rpm);

  // Print RPM lable
  display.setCursor(display.getCursorX() + 4, 6);
  display.setTextSize(1);
  display.print(RPM_TEXT);

  // Print temperature
  auto tpmText = std::to_string(temperature);
  display.setCursor((SCREEN_WIDTH - tpmText.length() * CHAR_BASE_SIZE * 2) - CHAR_BASE_SIZE - 4, 0);
  display.setTextSize(2);
  display.print(temperature);

  // Print temperature label
  auto x = display.getCursorX();
  display.setCursor(x + 4, 6);
  display.setTextSize(1);
  display.println("C");

  // Print moto miliseconds
  display.setTextSize(1);
  display.setCursor(0, 22);
  display.print(motoMilliseconds);

  display.setCursor(display.getCursorX() + 4, 22);
  display.setTextSize(1);
  display.print("ms");

  // Print total rotations
  display.setCursor(display.getCursorX() + 2 * CHAR_BASE_SIZE, 22);
  display.setTextSize(1);
  display.println(totalRotations);

  // Flush display
  display.display();
}

unsigned long startMillis = 0;

void setup()
{
  Serial.begin(115200);

  // Set up the persistently saved values
  preferences.begin("motor-unit", false);

  // Reset total rotations if the button is pressed during start
  pinMode(BTN_PIN, INPUT_PULLUP);
  if (digitalRead(BTN_PIN) == LOW)
  {
    preferences.putInt("totalRotations", 0);
    preferences.putInt("motoMilliseconds", 0);
  }

  totalRotations = preferences.getInt("totalRotations", 0);

  // Set up the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  syncDisplay();

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

  startMillis = millis();
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
  auto start = micros();

  // Calculate moto milliseconds since the last measurement and add it to the total
  auto currentMillis = start / 1000;
  auto deltaMillis = currentMillis - startMillis;

  if (deltaMillis > 0)
  {
    startMillis = currentMillis;
    motoMilliseconds += deltaMillis;
    preferences.putInt("motoMilliseconds", motoMilliseconds);
  }

  // Measure RPM — if it takes too long, skip the rest of the measurements

  while (rotationCount < RPM_SAMPLE_SIZE)
  {
    syncDisplay();

    // Leave the measurement if it takes more than 2 seconds
    if (micros() - start > 2000000)
    {
      break;
    }
  }

  // Calculate RPM
  auto seconds = (micros() - start) / 1000000.0;
  rpm = rotationCount / seconds * 60.0;
  rotationCount = 0;

  // Set RPM servo position
  rpmServo.setPosition(mapRpmToAngle(rpm));

  // Get temperature
  temperature = thermistor->readCelsius();

  // Set temperature servo position
  tempServo.setPosition(mapTemperatureToAngle(temperature));

  syncDisplay();

  preferences.putInt("totalRotations", totalRotations);
}