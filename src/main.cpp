#include <Arduino.h>

#include <QTRSensors.h>

#include <HCSR04.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int speed;
int speedm1;
int speedm2;
int speedmax = 50;
int delaystart = 5;
int delaydef = 2;
int position_ante;
int P;
int I;
int D;
int error;
int PIDvalue;
int previousError;
int Kd = 1;
int Ki = 0;
int Kp = 0.02;
int PWM_SETPOINT = 20;

byte triggerPin = 21;
byte echoPin = 12;

void demarrage()
{

  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(9, LOW);

  analogWrite(9, speedmax);
  analogWrite(10, speedmax);

  Serial.println("start");
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
  HCSR04.begin(triggerPin, echoPin);
  delay(1000);
  demarrage();
}

void stop()
{
  /*Serial.println("stop");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("stop_for");
    motors.setSpeeds(speed, speed);
    delay(delaydef);
  }*/

  analogWrite(9, 0);
  analogWrite(10, 0);
}

void Line_Follow(uint16_t position)
{
  error = position - 3500;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  speedm1 = PWM_SETPOINT - PIDvalue;
  speedm2 = PWM_SETPOINT + PIDvalue;

  // if (Left_speed >255){Left_speed = 200;}
  // if (Right_speed >255){Right_speed = 200;}
  if (speedm2 < 0)
  {
    speedm2 = 0;
  }
  if (speedm1 < 0)
  {
    speedm1 = 0;
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

  double *distances = HCSR04.measureDistanceCm();

  while (distances[0] < 15.00)
  {

    analogWrite(9, 0);
    analogWrite(10, 0);
  }

  Line_Follow(position);

  analogWrite(9, speedm1);
  analogWrite(10, speedm2);
}