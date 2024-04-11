#include <FastLED.h>
#include "PWM.hpp"
#include <Servo.h>


#define NUM_LEDS 1
#define PWM_MAX_VALUE 14900
#define PWM_MIN_VALUE 13900
#define DATA_PIN 16

CRGB leds[NUM_LEDS];

PWM ch3(2);//ch3
PWM ch2(3);//ch2
PWM ch1(4);//ch1
PWM hall(0);//hall

Servo tServo;
Servo cServo;
Servo sServo;


void setup() {
  ch1.begin(true);
  ch2.begin(true);
  ch3.begin(true);
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  tServo.attach(5);
  cServo.attach(7);
  sServo.attach(6);
  Serial.begin(115200); // Serial for debug
}
long ageTimer = 0;
boolean blinkState = false;
boolean disconnected = true;
void loop() {
  tServo.write(90);
  cServo.write(90);
  sServo.write(90);
  if(millis()-ageTimer > 250)
  {
    ageTimer = millis();
    if(ch1.getAge()>20000)
    {
      blinkState = !blinkState;
      disconnected = true;
    }
    else
    {
      disconnected = false;
      blinkState = false;
    }
  }
  if(disconnected)
  {
    if(blinkState)
    {
      leds[0] = CRGB::Red;
    }
    else
    {
      leds[0] = CRGB::Black;
    }
  }
  else
  {
    int ch1Val = ch1.getValue();
    if(ch1Val>PWM_MAX_VALUE)
    {
      ch1Val = PWM_MAX_VALUE;
    }
    if(ch1Val<PWM_MIN_VALUE)
    {
      ch1Val = PWM_MIN_VALUE;
    }
    byte ch1b = map(ch1Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
    int ch2Val = ch2.getValue();
    if(ch2Val>PWM_MAX_VALUE)
    {
      ch2Val = PWM_MAX_VALUE;
    }
    if(ch2Val<PWM_MIN_VALUE)
    {
      ch2Val = PWM_MIN_VALUE;
    }
    byte ch2b = map(ch2Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
    int ch3Val = ch3.getValue();
    if(ch3Val>PWM_MAX_VALUE)
    {
      ch3Val = PWM_MAX_VALUE;
    }
    if(ch3Val<PWM_MIN_VALUE)
    {
      ch3Val = PWM_MIN_VALUE;
    }
    byte ch3b = map(ch3Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
    leds[0] = CRGB(ch1b/4, ch2b/4, ch3b/4);
  }
  FastLED.show();
}
