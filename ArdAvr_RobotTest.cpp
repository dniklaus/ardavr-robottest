// Do not remove the include below
#include "ArdAvr_RobotTest.h"

#include "LcdKeypad.h"
#include "TimerContext.h"
#include "SN754410Driver.h"
#include "MotorPWM.h"
#include "UltrasonicSensor.h"
#include "UltrasonicSensorHCSR04.h"

LcdKeypad lcdKeypad;
MotorPWM* motorL;
MotorPWM* motorR;
UltrasonicSensor* ultrasonicSensorFront;


bool isRunning = false;

// H-bridge enable pin for speed control
int speedPin1 = 44;
int speedPin2 = 45;

// H-bridge leg 1
int motor1APin = 46;
int motor3APin = 47;

// H-bridge leg 2
int motor2APin = 48;
int motor4APin = 49;

// value for motor speed
int speed_value_motor = 0;
bool isSpeedIncreasing = true;
bool isFwd = true;
bool isObstacleDetected = false;

// LCD Backlight Intensity
int lcdBackLightIntensity = 100;


// Ultrasonic Sensor
unsigned int triggerPin = 34;
unsigned int echoPin    = 36;
unsigned long dist      = 0;   // [cm]

//The setup function is called once at startup of the sketch
void setup()
{
  ultrasonicSensorFront = new UltrasonicSensorHCSR04(triggerPin, echoPin);

  motorL = new SN754410_Driver(speedPin1, motor1APin, motor2APin);
  motorR = new SN754410_Driver(speedPin2, motor3APin, motor4APin);

  lcdKeypad.setBackLightIntensity(lcdBackLightIntensity);
}

void speedControl()
{
  dist = ultrasonicSensorFront->getDistanceCM();
  isObstacleDetected = isFwd && (dist > 0) && (dist < 10);

  if (lcdKeypad.isRightKey())
  {
    isRunning = false;
  }
  else if (lcdKeypad.isLeftKey())
  {
    isRunning = true;
  }

  if (isRunning)
  {
    if ((speed_value_motor == 255) || isObstacleDetected)
    {
      isSpeedIncreasing = false;
    }
    else if (speed_value_motor == 0)
    {
      isSpeedIncreasing = true;
      isFwd = !isFwd;
    }
    if (isSpeedIncreasing && (speed_value_motor < 255))
    {
      speed_value_motor++;
    }
    else if ((speed_value_motor > 0))
    {
      speed_value_motor--;
    }
  }
  else
  {
    if (speed_value_motor > 0)
    {
      if (isObstacleDetected)
      {
        speed_value_motor -= 4;
      }
      else
      {
        speed_value_motor--;
      }
    }
  }
}

void lcdBackLightControl()
{
  if (lcdKeypad.isUpKey() && (lcdBackLightIntensity < 255))
  {
    lcdBackLightIntensity++;
  }
  else if (lcdKeypad.isDownKey() && (lcdBackLightIntensity > 0))
  {
    lcdBackLightIntensity--;
  }

  lcdKeypad.setBackLightIntensity(lcdBackLightIntensity);
}

void updateDisplay()
{
  lcdKeypad.setCursor(0, 0);
//  lcdKeypad.print("in: ");
//  lcdKeypad.print(lcdKeypad.isNoKey()     ? "NO_KEY    " :
//                  lcdKeypad.isUpKey()     ? "UP_KEY    " :
//                  lcdKeypad.isDownKey()   ? "DOWN_KEY  " :
//                  lcdKeypad.isLeftKey()   ? "LEFT_KEY  " :
//                  lcdKeypad.isRightKey()  ? "RIGHT_KEY " :
//                  lcdKeypad.isSelectKey() ? "SELECT_KEY" : "----------");
  lcdKeypad.print("Dist:  ");
  lcdKeypad.print(dist > 99 ? "" : dist > 9 ? " " : "  ");
  lcdKeypad.print(dist);
  lcdKeypad.print(" [cm]");

  lcdKeypad.setCursor(0, 1);
  lcdKeypad.print("Speed: ");
  lcdKeypad.print(speed_value_motor > 99 ? "" : speed_value_motor > 9 ? " " : "  ");
  lcdKeypad.print(speed_value_motor);
  lcdKeypad.setCursor(11, 1);
  lcdKeypad.print(isRunning ? (isFwd ? "[FWD]" : "[REV]") : "[---]");
}

void updateActors()
{
  int speedAndDirection = speed_value_motor * (isFwd ? 1 : -1);
  motorL->setSpeed(speedAndDirection);
  motorR->setSpeed(speedAndDirection);
}

// The loop function is called in an endless loop
void loop()
{
  TimerContext::instance()->handleTick();

  lcdBackLightControl();
  speedControl();
  updateDisplay();
  updateActors();

  delay(0);
}
