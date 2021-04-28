#include <Arduino.h>

// this is for the Arduino Nano
// It uses both interrupt pins to detect any change in the quadrature encoder signals, both rising and falling.

#define PWM1 9 // pins that go to the H-Bridge inputs
#define PWM2 10

#define QUAD1 2 // Interrupt pins that respond to the quadrature encoder outputs - use 1k as protective resistors
#define QUAD2 3

volatile long position; // internal counter that the interrupt code modifies

// The interrupt function; it is triggered each time pin 2 / 3 change their value.
// So we get the most detail and can be resistant against vibrations
void intcount()
{
  static uint8_t lastref = 0;               // store the last reference in the Quadrature cycle that is 00 - 01 - 11 - 10
  uint8_t seq[4] = {B00, B01, B11, B10};    // Mapping of encoder bits to the count, i.e. swap values 2 and 3
  uint8_t portstat = (PIND & B1100) >> 2;   // Read both inputs in parallel (instead of digitalRead)
  uint8_t nowref = seq[portstat];           // where are we in the cycle? 0 -> 0, 1-> 1, 2->3, 3->2
  if (seq[(lastref - 1) & B11] == portstat) // Now check which is the neighboring value; do we go up or down in the cycle array?
  {
    position--;
  }
  if (seq[(lastref + 1) & B11] == portstat)
  {
    position++;
  }
  lastref = nowref; // remember for the next call
}

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), intcount, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), intcount, CHANGE);
}

void loop()
{

#define STEEP 8     // how fast the program accelerates/decelerates PWM value as the position approaches the target
#define MINSPEED 20 // minimum PWM value, adjust to something that's bit enough to make the motor turn
#define ZONE 24     // Deadzone around target that is considered "good enough" (since the motor will take time to come to a halt)

  static int32_t target = 1000; // just a first target
  static int32_t current = 0; // a buffer
  static unsigned long start = 0; // the moment the movement starts; to calculate acceleration
  static unsigned long nextprint = 0; // a clock for the diagnostic output every 0.5 seconds
  int32_t delta, ramp = 0; // values for calculation 
  unsigned long tick = millis(); // remember the current millisecond

  current = position; // read the position so it doesn't change within the cycle, even if interrupts happen
  ramp = (tick - start) / STEEP; // ramp-up PWM value as the milliseconds pass after start


  if (current > target) // do we need to turn "downwards"?
  {
    delta = current - target; // remaining steps
    analogWrite(9, min(ramp + MINSPEED, min(255, (delta / STEEP) + MINSPEED))); // set one channel to PWM
    analogWrite(10, 0); // the other channel is simply set to ground
  }
  else
  {
    delta = target - current; // same as above, but we're turning "upwards" to reach the target
    analogWrite(9, 0); // so this motor pin channel is now ground
    analogWrite(10, min(ramp + MINSPEED, min(255, (delta / STEEP) + MINSPEED))); // and this channel is driven by PWM
  }

  if (delta < ZONE) // we're inside the deadband of our target position
  {

    Serial.println("Reached ZONE");
    analogWrite(PWM1, 0); // Shut down both PWM signals
    analogWrite(PWM2, 0);
    delay(500); // just for testing; let the motor run out.
    Serial.print("Stopped at ");
    Serial.print(position);
    Serial.print(" of ");
    Serial.println(target);

    // have we been running towards a random test position?
    if (target > 0) 
    {
      target = 0; // yes: go back to zero
    }
    else
    {
      target = random(500, 10000); // no: go to a random position
    }
    Serial.print("New target = ");
    Serial.println(target);
    start = millis(); // Begin the acceleration ramp
  }

// a diagnostic print that happens every 500 ms
  if (tick > nextprint)
  {
    Serial.println(position);
    nextprint = tick + 500;
  }
}