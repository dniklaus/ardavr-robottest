// Do not remove the include below
#include "ArdAvr_RobotTest.h"

#include "LcdKeypad.h"
#include "Blanking.h"
#include "TimerContext.h"
#include "Timer.h"
#include "TimerAdapter.h"
#include "SN754410Driver.h"
#include "MotorPWM.h"
#include "UltrasonicSensor.h"
#include "UltrasonicSensorHCSR04.h"
#include "EEPROM.h"
#include "PID_v1.h"

#include <avr/power.h>
#include <avr/sleep.h>

LcdKeypad lcdKeypad;
MotorPWM*         motorL;
MotorPWM*         motorR;
UltrasonicSensor* ultrasonicSensorFront;
Timer*            speedSensorReadTimer;
Timer*            displayTimer;
Timer*            speedCtrlTimer;
Blanking*         displayBlanking;
PID*              pidLeft = 0;
PID*              pidRight = 0;

bool isRunning = false;
bool isIvmAccessMode = false;
bool isIvmRobotIdEditMode = false;

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
unsigned int triggerPin  = 34;
unsigned int echoPin     = 36;
unsigned long dist       = UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED;   // [cm]

// Battery Voltage Surveillance
float    battVoltage     = 0;   // [100mv]

const int BATT_SENSE_PIN     = A9;
const float BATT_WARN_THRSHD = 6.20;
const float BATT_SENS_FACTOR_1 = 2.0;
const float BATT_SENS_FACTOR_2 = 2.488;
const float BATT_SENS_FACTOR_3 = 2.550;
const float BATT_SENS_FACTOR_4 = 2.091;
const float BATT_SENS_FACTOR_5 = 2.456;
float BATT_SENS_FACTOR = 2.0;

// Wheel Speed Sensors
const unsigned int SPEED_SENSORS_READ_TIMER_INTVL_MILLIS = 100;

const int IRQ_PIN_18 = 5;
const int IRQ_PIN_19 = 4;
const int IRQ_PIN_20 = 3;
const int IRQ_PIN_21 = 2;

const int L_SPEED_SENS_IRQ = IRQ_PIN_20;
const int R_SPEED_SENS_IRQ = IRQ_PIN_21;

volatile unsigned long int speedSensorCountLeft  = 0;
volatile unsigned long int speedSensorCountRight = 0;

 double leftWheelSpeed  = 0.0;
 double rightWheelSpeed = 0.0;

const double PID_KP = 1;
const double PID_KI = 100;
const double PID_KD = 1;

double leftWheelOutput = 0;
double rightWheelOutput = 0;

double leftWheelSetPoint = 0;
double rightWheelSetPoint = 0;

void readSpeedSensors()
{
  noInterrupts();
  rightWheelSpeed = speedSensorCountRight; speedSensorCountRight = 0;
  leftWheelSpeed  = speedSensorCountLeft;  speedSensorCountLeft  = 0;
  Serial.println(rightWheelSpeed);
  Serial.println(leftWheelSpeed);
  interrupts();
}

void controlSpeed()
{
  Serial.println(pidLeft->Compute());
  Serial.println(pidRight->Compute());

  Serial.println((int)(leftWheelOutput ));
  Serial.println((int)(rightWheelOutput ));
  Serial.println();


  motorL->setSpeed((int)(leftWheelOutput * 10.0));
  motorR->setSpeed((int)(rightWheelOutput * 10.0));
}


class SpeedSensorReadTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    readSpeedSensors();
    controlSpeed();
  }
};

void selectMode();
void speedControl();
void updateActors();

class SpeedCtrlTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    speedControl();
    updateActors();
  }
};

void updateBattVoltageSenseFactor()
{
  unsigned char robotId = EEPROM.read(0);
  switch (robotId)
  {
    case 1:
      BATT_SENS_FACTOR = BATT_SENS_FACTOR_1;
      break;
    case 2:
      BATT_SENS_FACTOR = BATT_SENS_FACTOR_2;
      break;
    case 3:
      BATT_SENS_FACTOR = BATT_SENS_FACTOR_3;
      break;
    case 4:
      BATT_SENS_FACTOR = BATT_SENS_FACTOR_4;
      break;
    case 5:
      BATT_SENS_FACTOR = BATT_SENS_FACTOR_5;
      break;
    default:
      BATT_SENS_FACTOR = 2.0;
  }
}

void readBattVoltage()
{
  unsigned int rawBattVoltage = analogRead(BATT_SENSE_PIN);
  battVoltage = rawBattVoltage * BATT_SENS_FACTOR * 5 / 1023;
}


void updateDisplay();

class DisplayTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    readBattVoltage();
    selectMode();
    updateDisplay();
  }
};

void countLeftSpeedSensor()
{
  noInterrupts();
  speedSensorCountLeft++;
  interrupts();
}

void countRightSpeedSensor()
{
  noInterrupts();
  speedSensorCountRight++;
  interrupts();
}

//The setup function is called once at startup of the sketch
void setup()
{
  Serial.begin(115200);

  leftWheelOutput = 0;
  rightWheelOutput = 0;

  leftWheelSetPoint = 15;
  rightWheelSetPoint = 15;

  ultrasonicSensorFront = new UltrasonicSensorHCSR04(triggerPin, echoPin);

  pidLeft  = new PID(&leftWheelSpeed, &leftWheelOutput, &leftWheelSetPoint, PID_KP, PID_KI, PID_KD,  DIRECT);
  pidLeft->SetMode(AUTOMATIC);
  pidRight = new PID(&rightWheelSpeed, &rightWheelOutput, &rightWheelSetPoint, PID_KP, PID_KI, PID_KD,  DIRECT);
  pidRight->SetMode(AUTOMATIC);

  speedSensorReadTimer  = new Timer(new SpeedSensorReadTimerAdapter(), Timer::IS_RECURRING, SPEED_SENSORS_READ_TIMER_INTVL_MILLIS);

  attachInterrupt(L_SPEED_SENS_IRQ, countLeftSpeedSensor,  RISING);
  attachInterrupt(R_SPEED_SENS_IRQ, countRightSpeedSensor, RISING);

  motorL = new SN754410_Driver(speedPin1, motor1APin, motor2APin);
  motorR = new SN754410_Driver(speedPin2, motor3APin, motor4APin);

  displayTimer = new Timer(new DisplayTimerAdapter(), Timer::IS_RECURRING, 200);

  //speedCtrlTimer = new Timer(new SpeedCtrlTimerAdapter(), Timer::IS_RECURRING, 200);

  displayBlanking = new Blanking();

  lcdKeypad.setBackLightIntensity(lcdBackLightIntensity);

  updateBattVoltageSenseFactor();

}

void selectMode()
{
  if (!isRunning)
  {
    if (lcdKeypad.isSelectKey())
    {
      isIvmAccessMode = true;
    }
    else if (!isIvmRobotIdEditMode && lcdKeypad.isRightKey())
    {
      isIvmAccessMode = false;
    }

    if (isIvmAccessMode)
    {
      if (lcdKeypad.isLeftKey())
      {
        isIvmRobotIdEditMode = true;
      }
    }

    if (isIvmRobotIdEditMode)
    {
      unsigned char robotId = EEPROM.read(0);

      if (lcdKeypad.isSelectKey())
      {
        isIvmRobotIdEditMode = false;
      }
      if (lcdKeypad.isUpKey())
      {
        robotId++;
        EEPROM.write(0, robotId);
        updateBattVoltageSenseFactor();
      }
      if (lcdKeypad.isDownKey())
      {
        robotId--;
        EEPROM.write(0, robotId);
        updateBattVoltageSenseFactor();
      }
    }
  }
}

void speedControl()
{
  if (0 != ultrasonicSensorFront)
  {
    dist = ultrasonicSensorFront->getDistanceCM();
  }
  isObstacleDetected = isFwd && (dist > 0) && (dist < 15);

  if (lcdKeypad.isRightKey() || isObstacleDetected)
  {
    isRunning = false;
  }
  else if (lcdKeypad.isLeftKey() && !isIvmAccessMode)
  {
    isRunning = true;
  }

  if (isRunning)
  {
    speed_value_motor = 255;
//    if ((speed_value_motor == 255))
//    {
//      isSpeedIncreasing = false;
//    }
//    else if (speed_value_motor == 0)
//    {
//      isSpeedIncreasing = true;
//      isFwd = !isFwd;
//    }
//
//    int speedDelta = 50;
//
//    if (isSpeedIncreasing)
//    {
//      if (speed_value_motor <= (255 - speedDelta))
//      {
//        speed_value_motor += speedDelta;
//      }
//      else if (speed_value_motor < 255)
//      {
//        speed_value_motor++;
//      }
//    }
//    else
//    {
//      if ((speed_value_motor >= speedDelta))
//      {
//        speed_value_motor -= speedDelta;
//      }
//      else if (speed_value_motor > 0)
//      {
//        speed_value_motor--;
//      }
//    }
  }
  else
  {
    speed_value_motor = 0;
//    if (speed_value_motor > 0)
//    {
//      if (isObstacleDetected)
//      {
//        speed_value_motor = 0;
//      }
//      else if (speed_value_motor > 10)
//      {
//        speed_value_motor -= 10;
//      }
//      else
//      {
//        speed_value_motor--;
//      }
//    }
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

  if (isIvmAccessMode)
  {
    lcdKeypad.print("IVM Data");
    lcdKeypad.print("        ");

    lcdKeypad.setCursor(0, 1);

    lcdKeypad.print("Robot ID: ");

    if (isIvmRobotIdEditMode && displayBlanking->isSignalBlanked())
    {
      lcdKeypad.print("      ");
    }
    else
    {
      lcdKeypad.print(EEPROM.read(0));
    }
    lcdKeypad.print("     ");
  }
  else
  {
    lcdKeypad.print("Dst:");
    if (dist == UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED)
    {
      lcdKeypad.print("infin ");
    }
    else
    {
      lcdKeypad.print(dist > 99 ? "" : dist > 9 ? " " : "  ");
      lcdKeypad.print(dist);
      lcdKeypad.print("cm ");
    }

    if (displayBlanking->isSignalBlanked() && (battVoltage < BATT_WARN_THRSHD))
    {
      lcdKeypad.print("      ");
    }
    else
    {
      lcdKeypad.print("B:");
      lcdKeypad.print(battVoltage);
      lcdKeypad.print("[V]");
    }

    lcdKeypad.setCursor(0, 1);
    lcdKeypad.print("v ");
    lcdKeypad.print("l:");
    lcdKeypad.print(leftWheelSpeed > 999 ? "" : leftWheelSpeed > 99 ? " " : leftWheelSpeed > 9 ? "  " : "   ");
    lcdKeypad.print(leftWheelSpeed);
    lcdKeypad.print(" r:");
    lcdKeypad.print(rightWheelSpeed > 999 ? "" : rightWheelSpeed > 99 ? " " : rightWheelSpeed > 9 ? "  " : "   ");
    lcdKeypad.print(rightWheelSpeed);
  }
}

void updateActors()
{

}

// The loop function is called in an endless loop
void loop()
{
  TimerContext::instance()->handleTick();
  lcdBackLightControl();
}
