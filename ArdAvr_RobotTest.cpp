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
#include "Ivm.h"
#include "IF_IvmMemory.h"
#include "IvmSerialEeprom.h"
#include "LintillaIvm.h"

#include <aJSON.h>

#include <avr/power.h>
#include <avr/sleep.h>

aJsonStream serial_stream(&Serial);

class DistanceCount;
DistanceCount* lDistCount = 0;
DistanceCount* rDistCount = 0;

LcdKeypad lcdKeypad;
MotorPWM*         motorL;
MotorPWM*         motorR;
UltrasonicSensor* ultrasonicSensorFront;
Timer*            speedSensorReadTimer;
Timer*            displayTimer;
Timer*            speedCtrlTimer;
Blanking*         displayBlanking;
Ivm*              ivm;

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
bool isFwd = true;
bool isObstacleDetected = false;

// LCD Backlight Intensity
bool isLcdBackLightOn   = true;

// Ultrasonic Sensor
unsigned int triggerPin  = 34;
unsigned int echoPin     = 36;
unsigned long dist       = UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED;   // [cm]

// Battery Voltage Surveillance
float       battVoltage        = 0;   // [V]

const int   BATT_SENSE_PIN     = A9;

const float BATT_WARN_THRSHD   = 6.20;
const float BATT_SHUT_THRSHD   = 6.00;

const float BATT_SENS_FACTOR_1 = 2.0;
const float BATT_SENS_FACTOR_2 = 2.450;
const float BATT_SENS_FACTOR_3 = 2.530;
const float BATT_SENS_FACTOR_4 = 2.000;
const float BATT_SENS_FACTOR_5 = 2.456;

float       BATT_SENS_FACTOR   = 2.0;

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


volatile long int leftWheelSpeed  = 0;
volatile long int rightWheelSpeed = 0;

class DistanceCount
{
public:
  DistanceCount()
  : m_cumulativeDistanceCount(0)
  { };

  void reset()
  {
    m_cumulativeDistanceCount = 0;
  };

  void add(unsigned long int delta)
  {
    m_cumulativeDistanceCount += delta;
  }

  unsigned long int cumulativeDistanceCount()
  {
    return m_cumulativeDistanceCount;
  }

private:
  unsigned long int m_cumulativeDistanceCount;
};

void readSpeedSensors()
{
  noInterrupts();

  leftWheelSpeed  = speedSensorCountLeft;  speedSensorCountLeft  = 0;
  rightWheelSpeed = speedSensorCountRight; speedSensorCountRight = 0;

  lDistCount->add(leftWheelSpeed);
  rDistCount->add(rightWheelSpeed);

  interrupts();
}

class SpeedSensorReadTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    readSpeedSensors();
  }
};

void selectMode();
void speedControl();
void updateActors();
void lcdBackLightControl();

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
  unsigned char robotId = ivm->getDeviceId(); // EEPROM.read(0);
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

void sleepNow();

void readBattVoltage()
{
  unsigned int rawBattVoltage = analogRead(BATT_SENSE_PIN);
  battVoltage = rawBattVoltage * BATT_SENS_FACTOR * 5 / 1023;

  if (BATT_WARN_THRSHD >= battVoltage)
  {
    isLcdBackLightOn = false;
    lcdBackLightControl();
  }

  // emergency shutdown on battery low alarm
  if (BATT_SHUT_THRSHD >= battVoltage)
  {
    sleepNow();
  }
}

void sleepNow()
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep modus.
   *
   * In the avr/sleep.h file, the call names of these sleep modus are to be found:
   *
   * The 5 different modes are:
   * SLEEP_MODE_IDLE -the least power savings
   * SLEEP_MODE_ADC
   * SLEEP_MODE_PWR_SAVE
   * SLEEP_MODE_STANDBY
   * SLEEP_MODE_PWR_DOWN -the most power savings
   *
   * the power reduction management <avr/power.h> is described in
   * http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
   */

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here

  sleep_enable(); // enables the sleep bit in the mcucr register
                  // so sleep is possible. just a safety pin
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  sleep_mode(); // here the device is actually put to sleep!!

  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();  // first thing after waking from sleep:
                    // disable sleep...

  power_all_enable();
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

  ivm = new LintillaIvm();

  ultrasonicSensorFront = new UltrasonicSensorHCSR04(triggerPin, echoPin);

  lDistCount = new DistanceCount();
  rDistCount = new DistanceCount();

  speedSensorReadTimer  = new Timer(new SpeedSensorReadTimerAdapter(), Timer::IS_RECURRING, SPEED_SENSORS_READ_TIMER_INTVL_MILLIS);

  attachInterrupt(L_SPEED_SENS_IRQ, countLeftSpeedSensor,  RISING);
  attachInterrupt(R_SPEED_SENS_IRQ, countRightSpeedSensor, RISING);

  motorL = new SN754410_Driver(speedPin1, motor1APin, motor2APin);
  motorR = new SN754410_Driver(speedPin2, motor3APin, motor4APin);

  displayTimer = new Timer(new DisplayTimerAdapter(), Timer::IS_RECURRING, 200);

  speedCtrlTimer = new Timer(new SpeedCtrlTimerAdapter(), Timer::IS_RECURRING, 200);

  displayBlanking = new Blanking();

  updateBattVoltageSenseFactor();

  lcdBackLightControl();
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
      unsigned char robotId = ivm->getDeviceId();

      if (lcdKeypad.isSelectKey())
      {
        isIvmRobotIdEditMode = false;
      }
      if (lcdKeypad.isUpKey())
      {
        robotId++;
        ivm->setDeviceId(robotId);
        updateBattVoltageSenseFactor();
      }
      if (lcdKeypad.isDownKey())
      {
        robotId--;
        ivm->setDeviceId(robotId);
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

  if (lcdKeypad.isRightKey() || isObstacleDetected || (battVoltage < BATT_WARN_THRSHD))
  {
    isRunning = false;
  }
  else if (lcdKeypad.isLeftKey() && !isIvmAccessMode)
  {
    isRunning = true;
    lDistCount->reset();
    rDistCount->reset();
  }

  if (isRunning)
  {
    speed_value_motor = 255;
  }
  else
  {
    speed_value_motor = 0;
  }
}

void lcdBackLightControl()
{
  if (!isIvmAccessMode)
  {
    if (lcdKeypad.isUpKey() && (!isLcdBackLightOn))
    {
      isLcdBackLightOn = true;
    }
    else if (lcdKeypad.isDownKey() && (isLcdBackLightOn))
    {
      isLcdBackLightOn = false;
    }
  }
  lcdKeypad.setBackLightOn(isLcdBackLightOn);
}

void updateDisplay()
{
  lcdBackLightControl();

  lcdKeypad.setCursor(0, 0);

  if (isIvmAccessMode)
  {
    lcdKeypad.print("IVM Data (V.");
    lcdKeypad.print(ivm->getIvmVersion());
    lcdKeypad.print(")     ");

    lcdKeypad.setCursor(0, 1);

    lcdKeypad.print("Robot ID: ");

    if (isIvmRobotIdEditMode && displayBlanking->isSignalBlanked())
    {
      lcdKeypad.print("      ");
    }
    else
    {
      lcdKeypad.print(ivm->getDeviceId());
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


    int lWSpd = static_cast<int>(leftWheelSpeed);
    int rWspd = static_cast<int>(rightWheelSpeed);

    lcdKeypad.setCursor(0, 1);
    lcdKeypad.print("v ");
    lcdKeypad.print("l:");
    lcdKeypad.print(lWSpd > 99 ? "" : lWSpd > 9 ? " " : "  ");
    lcdKeypad.print(lWSpd);
    lcdKeypad.print(" r:");
    lcdKeypad.print(rWspd > 99 ? "" : rWspd > 9 ? " " : "  ");
    lcdKeypad.print(rWspd);
  }
}

void updateActors()
{
  int speedAndDirection = speed_value_motor * (isFwd ? 1 : -1);

  motorL->setSpeed(speedAndDirection);
  motorR->setSpeed(speedAndDirection);
}

/* Process message like: {"straight":{"distance": 10,"topspeed": 100},"turn":{"angle": 45},"emergencyStop":{"stop":false}}*/
void processLintillaMessageReceived(aJsonObject *msg)
{
//  bool emergencyStopValue = false;
//  int topspeed = 0;
//  int distanceValue = 0;
//  int angleValue = 0;
  double batteryVoltage = 0.0;

  /* Lintilla Command List Example
  {
      "commands": [
          {
              "straight": {
                  "distance": 10,
                  "topspeed": 100
              }
          },
          {
              "turn": {
                  "angle": 45
              }
          },
          {
              "straight": {
                  "distance": 20,
                  "topspeed": 50
              }
          },
          {
              "emergencyStop": null
          },
          {
              "readVoltage": null
          }
      ]
  }
  */

  aJsonObject* commands = aJson.getObjectItem(msg, "commands");
  if (0 == commands)
  {
    Serial.println("NO commands");
  }
  else
  {
    Serial.println("commands");

    aJsonObject* aCommand = commands->child;
    while (0 != aCommand)
    {
      aJsonObject* readVoltageCmd = aJson.getObjectItem(aCommand, "readVoltage");
      if (0 != readVoltageCmd)
      {
        batteryVoltage = battVoltage;

        aJsonObject* readVoltageMsgRoot = aJson.createObject();
        aJsonObject* readVoltageMsgElem = aJson.createObject();

        aJson.addItemToObject(readVoltageMsgRoot, "readVoltage", readVoltageMsgElem);
        aJson.addNumberToObject(readVoltageMsgElem, "voltage", batteryVoltage);

        aJson.print(readVoltageMsgRoot, &serial_stream);
        Serial.println(); /* Add newline. */

        aJson.deleteItem(readVoltageMsgElem);
        aJson.deleteItem(readVoltageMsgRoot);
      }
      aJson.deleteItem(readVoltageCmd);

      aJsonObject* straightCmd = aJson.getObjectItem(aCommand, "straight");
      if (0 != straightCmd)
      {
        Serial.println("straight");
      }
      aJson.deleteItem(straightCmd);

      aCommand = aCommand->next;
    }
  }
}

// The loop function is called in an endless loop
void loop()
{
  TimerContext::instance()->handleTick();

  if (serial_stream.available()) {
    /* First, skip any accidental whitespace like newlines. */
    serial_stream.skip();
  }

  if (serial_stream.available()) {
    /* Something real on input, let's take a look. */
    aJsonObject *msg = aJson.parse(&serial_stream);
    processLintillaMessageReceived(msg);
    aJson.deleteItem(msg);
  }

}
