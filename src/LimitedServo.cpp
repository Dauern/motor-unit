#include <ESP32Servo.h>

// A servo with a limited range.
// It checks the range so we cannot accidentally set the servo to a position that will break it.
class LimitedServo
{
public:
  // Initialize the servo with the given pin and limits.
  LimitedServo(int pin, int min, int max)
  {
    this->min = min;
    this->max = max;
    this->servo.attach(pin);
  }

  // Set the servo position and make sure it is within the limits.
  void setPosition(int position)
  {
    if (position < min)
    {
      position = min;
    }
    if (position > max)
    {
      position = max;
    }

    this->position = position;

    this->servo.write(position);
  }

  // Get the current servo position.
  int position;

private:
  Servo servo;
  int min;
  int max;
};