#include <Arduino.h>

#include <QTRSensors.h>

#include <DRV8835MotorShield.h>

// #include "DRV8835MotorShield.h"

// #define LED_PIN 13

// #include <stdint.h>

QTRSensors qtr;
DRV8835MotorShield motors;

// QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10}, 8);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int speed;
int speedm1;
int speedm2;
int speedmax = 200;
int delaystart = 5;
int delaydef = 2;
int position_ante;
int G;
int D;

void demarrage()
{
  Serial.println("start");
  for (int speed=0; speed <= speedmax; speed++)
  {
    Serial.println("start_for");
    motors.setM1Speed(speed);
    motors.setM2Speed(speed);
    delay(delaystart);
  }
}

void setup()
{

  // configure the sensors

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A4, A3, A2, A1, A0, 4, 5, 6}, SensorCount);
  qtr.setEmitterPin(A5);

  // delay(1000);
  digitalWrite(2, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  demarrage();
}

void tournantdroite3(int speed)
{
  int speedm1 = speed;
  Serial.println("tournerdroite3");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("tournerdroite3_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }
}

void tournantdroite2(int speed)
{
  int speedm1 = speed;
  Serial.println("tournerdroite2");
  for (int speed; speed >= 50; speed--)
  {
    Serial.println("tournerdroite2_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }
}

void tournantdroite1(int speed)
{
  int speedm1 = speed;
  Serial.println("tournerdroite1");
  for (int speed; speed >= 100; speed--)
  {
    Serial.println("tournerdroite1_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }
}

void tournantgauche3(int speed)
{
  int speedm2 = speed;
  Serial.println("tournergauche3");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("tournergauche3_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }
}

void tournantgauche2(int speed)
{
  int speedm2 = speed;
  Serial.println("tournergauche2");
  for (int speed; speed >= 50; speed--)
  {
    Serial.println("tournergauche2_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }
}

void tournantgauche1(int speed)
{
  int speedm2 = speed;
  Serial.println("tournergauche1");
  for (int speed; speed >= 100; speed--)
  {
    Serial.println("tournergauche1_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }
}

void toutdroit()
{
  Serial.println("tout_droit");
  for (int speed; speed == speedmax; speed++)
  {
    Serial.println("tout_droit_for");
    motors.setM2Speed(speed);
    motors.setM1Speed(speed);
    //delay(delaydef);
  }
}

void turnRight()
{
  //Serial.println("turnRight");
  for (int speed=0; (speed <= speedmax && D == 1); speed++)
  {
    //Serial.println("turnRight----");
    motors.setM1Speed(0);
    motors.setM2Speed(speed);
    if (sensorValues[0] == 1000 || sensorValues[1] == 1000 || sensorValues[2] == 1000)
    {
      G = 1;
      D = 0;
    }
    if (sensorValues[5] == 1000 || sensorValues[6] == 1000 || sensorValues[7] == 1000)
    {
      G = 0;
      D = 1;
    }
    delay(delaydef);
  }
  Serial.println("JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS    JE SUIS SORTIS   JE SUIS SORTIS");
  /*for (int speed=0; speed <= speedmax; (speed=speed*10))
  {
    Serial.println("turnRight++++");
    motors.setM1Speed(speed/2);
    delay(delaydef);
  }*/
}

void turnLeft()
{
  //Serial.println("turnLeft");
  for (int speed=0; (speed <= speedmax && G == 1); speed++)
  {
    //Serial.println("turnLeft----");
    motors.setM2Speed(0);
    motors.setM1Speed(speed);
    if (sensorValues[0] == 1000 || sensorValues[1] == 1000 || sensorValues[2] == 1000)
    {
      G = 1;
      D = 0;
    }
    if (sensorValues[5] == 1000 || sensorValues[6] == 1000 || sensorValues[7] == 1000)
    {
      G = 0;
      D = 1;
    }
    delay(delaydef);
  }
  Serial.println("JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS    JE SUIS SORTIS   JE SUIS SORTIS");
  /*for (int speed=0; speed <= speedmax; (speed=speed*10))
  {
    Serial.println("turnLeft++++");
    motors.setM2Speed(speed/2);
    delay(delaydef);
  }*/
}

void stop()
{
  Serial.println("stop");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("stop_for");
    motors.setSpeeds(speed, speed);
    delay(delaydef);
  }
}

int getPosition()
{
  //Serial.println("getPosition");
  return qtr.readLineBlack(sensorValues);
}

int getSpeed()
{
  Serial.println("getSpeed");
  return speed;
}

void makeDecision()
{
  int result;
  /*for(int i=4; i<=7; i++)
  {
    if(sensorValues[i]==1000)
    {
      result = i;
      i=7;
    }
  }
  for(int i=3; i>=0; i--)
  {
    if(sensorValues[i]==1000)
    {
      result = i;
      i=0;
    }
  }*/

  /*if(position_ante>0 && position_ante<7000)
  {
    switch(result)
    {
      case 7:
        tournantdroite3(speed);
        break;
      case 6:
        tournantdroite2(speed);
        break;
      case 5:
        tournantdroite1(speed);
        break;
      case 2:
        tournantgauche1(speed);
        break;
      case 1:
        tournantgauche2(speed);
        break;
      case 0:
        tournantgauche3(speed);
        break;
    }
  }
  else if (position_ante==7000){
    tournantdroite3(speed);
  }
  else if (position_ante==0){
    tournantgauche3(speed);
  }*/
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[7] == 1000 && position_ante > 0 && position_ante < 7000)
  {
    tournantdroite3(speed);
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[6] == 1000 && position_ante > 0 && position_ante < 7000)
  {
    tournantdroite2(speed);
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] == 1000 && position_ante > 0 && position_ante < 7000)
  {
    tournantdroite1(speed);
  }
  if (sensorValues[7] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[2] == 1000 && position_ante > 4000)
  {
    tournantgauche1(speed);
  }
  if (sensorValues[7] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[1] == 1000 && position_ante > 4000)
  {
    tournantgauche2(speed);
  }
  if (sensorValues[7] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[0] == 1000 && position_ante < 3000)
  {
    tournantgauche3(speed);
  }
  /*if((sensorValues[3]==1000||sensorValues[4]==1000)&&sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000&&position_ante<5000&&position_ante>3000){
    toutdroit();
  }*/
  if (sensorValues[0] == 1000 && sensorValues[4] == 1000 && sensorValues[7] == 1000)
  {
    stop();
  }
  /*if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000){
    toutdroit();
  }*/
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[7] < 1000 && position_ante == 7000)
  {
    tournantdroite3(speed);
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[7] < 1000 && position_ante == 0)
  {
    tournantgauche3(speed);
  }
}

void makeDecision2()
{
  if (sensorValues[0] == 1000 || sensorValues[1] == 1000 || sensorValues[2] == 1000)
  {
    G = 1;
    D = 0;
    /*Serial.println("turnLeft----");
    motors.setM2Speed(0);
    motors.setM1Speed(5);*/
    turnLeft();
  }
  if (sensorValues[5] == 1000 || sensorValues[6] == 1000 || sensorValues[7] == 1000)
  {
    G = 0;
    D = 1;
    /*Serial.println("turnRight----");
    motors.setM1Speed(0);
    motors.setM2Speed(5);*/
    turnRight();
  }
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 10000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  // makeDecision();

  // toutdroit();

  // stop();

  // demarrage();

  /*for (int speed; speed <= 400; speed++)
  {
    motors.setM2Speed(speed);
    motors.setM1Speed(speed);
    // speedm1=-speed+1;
    /*for (int speedm1; speedm1>=-speed;speedm1--)
    {
      motors.setM1Speed(speedm1);
    }

    delay(2);
  }*/

  makeDecision2();

  delay(250);

  int position_ante = getPosition();

  //int speed_ante = getSpeed();
}

DRV8835MotorShield::DRV8835MotorShield() : _M1DIR(7), _M1PWM(9), _M2DIR(8), _M2PWM(10)
{
}

DRV8835MotorShield::DRV8835MotorShield(uint8_t M1DIR,
                                       uint8_t M1PWM,
                                       uint8_t M2DIR,
                                       uint8_t M2PWM) : _M1DIR(M1DIR), _M1PWM(M1PWM),
                                                        _M2DIR(M2DIR), _M2PWM(M2PWM)
{
}

void DRV8835MotorShield::initPinsAndMaybeTimer()
{
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(_M1PWM, LOW);
  pinMode(_M1PWM, OUTPUT);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  pinMode(_M2PWM, OUTPUT);
  digitalWrite(_M2PWM, LOW);
  digitalWrite(_M1DIR, LOW);
  pinMode(_M1DIR, OUTPUT);
  digitalWrite(_M1DIR, LOW);
  digitalWrite(_M2DIR, LOW);
  pinMode(_M2DIR, OUTPUT);
  digitalWrite(_M2DIR, LOW);
#ifdef DRV8835MOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    // timer 1 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
  }
#endif
}

void DRV8835MotorShield::setM1Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  /*if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }*/
  if (speed > 400) // max
    speed = 400;

#ifdef DRV8835MOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    OCR1A = speed;
  }
  else
  {
    analogWrite(_M1PWM, speed * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_M1PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipM1) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_M1DIR, HIGH);
  else
    digitalWrite(_M1DIR, LOW);
}

// speed should be a number between -400 and 400
void DRV8835MotorShield::setM2Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400) // max PWM duty cycle
    speed = 400;

#ifdef DRV8835MOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    OCR1B = speed;
  }
  else
  {
    analogWrite(_M2PWM, speed * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_M2PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipM2) // flip if speed was negative or _flipM2 setting is active, but not both
    digitalWrite(_M2DIR, HIGH);
  else
    digitalWrite(_M2DIR, LOW);
}

// set speed for both motors
// speed should be a number between -400 and 400
void DRV8835MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

void DRV8835MotorShield::flipM1(boolean flip)
{
  _flipM1 = flip;
}

void DRV8835MotorShield::flipM2(boolean flip)
{
  _flipM2 = flip;
}