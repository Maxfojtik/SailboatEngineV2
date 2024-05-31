#include <FastLED.h>
#include "PWM.hpp"
#include <Servo.h>
#include "ArduPID.h"



#define NUM_LEDS 1
#define PWM_MAX_VALUE 14900
#define PWM_MIN_VALUE 13900
#define DATA_PIN 16

#define MIN_RPM 1000
#define MAX_RPM 8000
#define CONTROLLER_P .05
#define CONTROLLER_I 0
#define CONTROLLER_D 0

#define IDLE 0
#define PULL_STARTING 1
#define RUNNING 2

CRGB leds[NUM_LEDS];

ArduPID myController;

PWM ch3(2);//ch3
PWM ch2(3);//ch2
PWM ch1(4);//ch1
PWM hall(0);//hall

byte ch3b;
byte ch2b;
byte ch1b;


Servo tServo;//180-50, 50 full
Servo cServo;//180-90, 90 open
Servo sServo;//70, 110, 170, forward, neutral, backward

int state = 0;
int currentGear = 0;//0 = neu, 1 = forward, -1 = backward

double setpoint = 0;
double input;
double output;



void setup() {
  ch1.begin(true);
  ch2.begin(true);
  ch3.begin(true);
  hall.begin(true);
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  tServo.attach(5);
  cServo.attach(7);
  sServo.attach(6);
  Serial.begin(115200); // Serial for debug
  FastLED.setBrightness(65);
  myController.begin(&input, &output, &setpoint, CONTROLLER_P, CONTROLLER_I, CONTROLLER_D);
}
long ageTimer = 0;
boolean blinkState = false;
boolean disconnected = true;
long connectedTime = 0;
long timer = 0;

long lastTimeChangedGear = 0;
int safeRPM = 3500;
long timeTrans = 0;
double ch1bAvg = 0;

void loop() {
  if(timer-millis()>10)
  {
    timer = millis();
    readController();
    determineIfDisconnected();
    determineStateAndGear();
    setServos();
    showLEDBasedOnState();
    Serial.print(state);
    Serial.print("\t");
    Serial.print(rpm());
    Serial.print("\t");
    Serial.print(ch1b);
    Serial.print("\t");
    Serial.print(ch1bAvg);
    Serial.print("\t");
    Serial.print(ch2b);
    Serial.print("\t");
    Serial.print(ch3b);
    Serial.print("\t");
    Serial.println(currentGear);
    // myController.debug(&Serial, "", PRINT_INPUT    | // Can include or comment out any of these terms to print
    //                                           PRINT_OUTPUT   | // in the Serial plotter
    //                                           PRINT_SETPOINT |
    //                                           PRINT_P        |
    //                                           PRINT_I        |
    //                                           PRINT_D);
  }
}

void setState(int statei)
{
    state = statei;
    timeTrans = millis();
}
void determineStateAndGear()
{
  // Serial.println(rpm());
  if(state == IDLE && rpm() > 100 && millis() - timeTrans > 500)
  {
    setState(PULL_STARTING);
  }
  if(state == PULL_STARTING && rpm()>100)
  {
    timeTrans = millis();
  }
  if(state == PULL_STARTING && rpm() == 0 && millis() - timeTrans > 10000)
  {
    setState(IDLE);
  }
  if(state == PULL_STARTING && rpm() > 1000)
  {
    setState(RUNNING);
  }
  if(state == RUNNING && rpm() < 100)
  {
    setState(PULL_STARTING);
  }
  // if(ch3b > 127)
  // {
  //   setState(IDLE);
  // }
  // else
  // {
  //   setState(RUNNING);
  // }
  ch1bAvg = ch1bAvg + (ch1b-ch1bAvg)*.005;
  if(connectedTime > 500)
  {
    if(state == RUNNING)
    {
      if(millis()-lastTimeChangedGear>1000)
      {
        if(ch1bAvg<256/8)
        {
          lastTimeChangedGear = millis();
          ch1bAvg = 128;
          if(currentGear==0 && rpm()<safeRPM)
          {
            currentGear = 1;
          }
          else if(currentGear == -1)
          {
            currentGear = 0;
          }
        }
        if(ch1bAvg>256*7/8)
        {
          lastTimeChangedGear = millis();
          ch1bAvg = 128;
          if(currentGear==0 && rpm()<safeRPM)
          {
            currentGear = -1;
          }
          else if(currentGear == 1)
          {
            currentGear = 0;
          }
        }
      }
    }
    // Serial.println(currentGear);
  }
  if(state == RUNNING)
  {
    byte chThrottle = map(ch2b, 128, 255, 0, 255);
    chThrottle = constrain(chThrottle, 0,255);
    int desiredRPM = map(chThrottle, 0, 255, MIN_RPM, MAX_RPM);
    if(desiredRPM<2000)
    {
      desiredRPM = 2000;
    }
    input = rpm();
    setpoint = desiredRPM;
    myController.compute();
  }
}

void setServos()
{
  if(state == IDLE)//off
  {
    tServo.write(180);//defaults
    cServo.write(100);
    sServo.write(110);
  }
  else if(state == PULL_STARTING)//pull starting
  {
    tServo.write(120);//about 1/4 throttle
    cServo.write(180);//full choke
    sServo.write(110);//neutral
  }
  else if(state == RUNNING)//running
  {   
    byte outputAngle = map(output, 0, 255, 180, 50);//0-180
    //180-50, 50 full
    if(outputAngle<50)
    {
      outputAngle = 50;
    }
    if(outputAngle > 180)
    {
      outputAngle = 180;
    }
    tServo.write(outputAngle);//throttle controlled based on PID
    cServo.write(100);
    if(currentGear==-1)
    {
      sServo.write(170);//back
    }
    else if(currentGear==1)
    {

      sServo.write(70);//forward
    }
    else
    {
      sServo.write(110);//neutral
    }
  }
}

void showLEDBasedOnState()
{
  if(state==0)
  {
    leds[0] = CRGB(0,millis()%2000>1950 ? 65:0,0);
  }
  else if(state==1)
  {
    leds[0] = CRGB(millis()%500>50 ? 65:0,millis()%500>50 ? 65:0,0);
  }
  else if(state==2)
  {
    byte val = millis()%500>50 ? 65:0;
    if(currentGear==0)
    {
      leds[0] = CRGB(val,val,val);
    }
    else if(currentGear==1)
    {
      leds[0] = CRGB(0,val,0);
    }
    else if(currentGear==-1)
    {
      leds[0] = CRGB(0,0,val);
    }
  }
  else
  {
    leds[0] = CRGB(millis()%500>250 ? 65:0,0,0);
  }
  FastLED.show();
}


void showLEDBasedOnController()
{
  if(disconnected)
  {
    if(millis()-ageTimer > 250)
    {
      ageTimer = millis();
      if(ch1.getAge()>20000)
      {
        blinkState = !blinkState;
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
        blinkState = false;
      }
    }
  }
  else
  {
    leds[0] = CRGB(abs(ch1b-127)/2, abs(ch2b-127)/2, ch3b);
  }
  FastLED.show();
}

void determineIfDisconnected()
{
  disconnected = ch1.getAge()>20000;
  if(!disconnected)
  {
    connectedTime++;
  }
  else
  {
    connectedTime = 0;
  }
  
}

void readController()
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
    ch1b = map(ch1Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
    int ch2Val = ch2.getValue();
    if(ch2Val>PWM_MAX_VALUE)
    {
      ch2Val = PWM_MAX_VALUE;
    }
    if(ch2Val<PWM_MIN_VALUE)
    {
      ch2Val = PWM_MIN_VALUE;
    }
    ch2b = map(ch2Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
    int ch3Val = ch3.getValue();
    if(ch3Val>PWM_MAX_VALUE)
    {
      ch3Val = PWM_MAX_VALUE;
    }
    if(ch3Val<PWM_MIN_VALUE)
    {
      ch3Val = PWM_MIN_VALUE;
    }
    ch3b = map(ch3Val, PWM_MIN_VALUE, PWM_MAX_VALUE, 0, 255);
}
int rpm()
{
  if(hall.getAge()>500000)//500ms
  {
    return 0;
  }
  return 60.0 * 1e6 / (hall.getValue()*6);
}
