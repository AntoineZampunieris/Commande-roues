#include <Arduino.h>

#include <QTRSensors.h>

// #include <DRV8835MotorShield.h>

// #include "DRV8835MotorShield.h"

// #define LED_PIN 13

// #include <stdint.h>

QTRSensors qtr;
// DRV8835MotorShield motors;

// QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10}, 8);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int speed;
int speedm1;
int speedm2;
int speedmax = 50;
int delaystart = 5;
int delaydef = 2;
int position_ante;
//int G;
//int D;
int P;
int I;
int D;
int error;
int PIDvalue;
int previousError;
int Kd=1;
int Ki=0;
int Kp=0.02;
int PWM_SETPOINT=20;


void demarrage()
{
  /*Serial.println("start");
  for (int speed=0; speed <= speedmax; speed++)
  {
    Serial.println("start_for");
    motors.setM1Speed(speed);
    motors.setM2Speed(speed);
    delay(delaystart);
  }*/
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
  delay(1000);
  demarrage();
}

void tournantdroite3()
{
  /*int speedm1 = speed;
  Serial.println("tournerdroite3");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("tournerdroite3_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(9, 0);
  analogWrite(10, speedmax);
  Serial.println("Tournant droite 3");
}

void tournantdroite2()
{
  /*int speedm1 = speed;
  Serial.println("tournerdroite2");
  for (int speed; speed >= 50; speed--)
  {
    Serial.println("tournerdroite2_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(9, speedmax / 3);
  analogWrite(10, speedmax);
  Serial.println("Tournant droite 2");
}

void tournantdroite1()
{
  /*int speedm1 = speed;
  Serial.println("tournerdroite1");
  for (int speed; speed >= 100; speed--)
  {
    Serial.println("tournerdroite1_for");
    motors.setM1Speed((speedm1 + speed) / 2);
    motors.setM2Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(9, speedmax / 2);
  analogWrite(10, speedmax);
  Serial.println("Tournant droite 1");
}

void tournantgauche3()
{
  /*int speedm2 = speed;
  Serial.println("tournergauche3");
  for (int speed; speed >= 0; speed--)
  {
    Serial.println("tournergauche3_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(10, speedmax);
  analogWrite(10, 0);
  Serial.println("tournergauche3");
}

void tournantgauche2()
{
  /*int speedm2 = speed;
  Serial.println("tournergauche2");
  for (int speed; speed >= 50; speed--)
  {
    Serial.println("tournergauche2_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(10, speedmax);
  analogWrite(10, speedmax / 3);
  Serial.println("tournergauche2");
}

void tournantgauche1()
{
  /*int speedm2 = speed;
  Serial.println("tournergauche1");
  for (int speed; speed >= 100; speed--)
  {
    Serial.println("tournergauche1_for");
    motors.setM2Speed((speedm2 + speed) / 2);
    motors.setM1Speed(speed);
    delay(delaydef);
  }*/
  analogWrite(10, speedmax);
  analogWrite(10, speedmax / 2);
  Serial.println("tournergauche1");
}

/*void toutdroit()
{
  Serial.println("tout_droit");
  for (int speed; speed == speedmax; speed++)
  {
    Serial.println("tout_droit_for");
    motors.setM2Speed(speed);
    motors.setM1Speed(speed);
    //delay(delaydef);
  }
}*/

void turnRight()
{
  // Serial.println("turnRight");
  /*for (int speed=0; (speed <= speedmax && D == 1); speed++)
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
  Serial.println("JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS   JE SUIS SORTIS    JE SUIS SORTIS   JE SUIS SORTIS");*/
  /*for (int speed=0; speed <= speedmax; (speed=speed*10))
  {
    Serial.println("turnRight++++");
    motors.setM1Speed(speed/2);
    delay(delaydef);
  }*/

  analogWrite(9, 0);
  analogWrite(10, speedmax);

  /*if (sensorValues[0] == 1000 || sensorValues[1] == 1000 || sensorValues[2] == 1000)
{
  G = 1;
  D = 0;
}
if (sensorValues[5] == 1000 || sensorValues[6] == 1000 || sensorValues[7] == 1000)
{
  G = 0;
  D = 1;
}*/
}

void turnLeft()
{
  // Serial.println("turnLeft");
  /*for (int speed=0; (speed <= speedmax && G == 1); speed++)
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
  analogWrite(10, 0);
  analogWrite(9, speedmax);
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

int getPosition()
{
  // Serial.println("getPosition");
  return qtr.readLineBlack(sensorValues);
}

int getSpeed()
{
  Serial.println("getSpeed");
  return speed;
}

void makeDecision()
{
  /*int result;
  for(int i=4; i<=7; i++)
  {
    if(sensorValues[i]==1000)
    {
      result = i;
      i=7;
      Serial.println("dceidedroite");
    }
  }
  for(int i=3; i>=0; i--)
  {
    if(sensorValues[i]==1000)
    {
      result = i;
      i=0;
      Serial.println("dceidegauche");
    }
  }

  if(position_ante>0 && position_ante<7000)
  {
    switch(result)
    {
      case 7:
        tournantdroite3(  );
        break;
      case 6:
        tournantdroite2(  );
        break;
      case 5:
        tournantdroite1(  );
        break;
      case 2:
        tournantgauche1(  );
        break;
      case 1:
        tournantgauche2(  );
        break;
      case 0:
        tournantgauche3(  );
        break;
    }
  }*/
  /*else if (position_ante==7000){
    tournantdroite3(  );
  }
  else if (position_ante==0){
    tournantgauche3(  );
  }*/
  /*if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[7] == 1000 && position_ante < 7000)
  {
    tournantdroite3();
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[6] == 1000 && position_ante < 7000)
  {
    tournantdroite2();
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] == 1000 && position_ante < 7000)
  {
    tournantdroite1();
  }
  if (sensorValues[7] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[2] == 1000 && position_ante < 3000)
  {
    tournantgauche1();
  }
  if (sensorValues[7] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[1] == 1000 && position_ante < 3000)
  {
    tournantgauche2();
  }
  if (sensorValues[7] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[0] == 1000 && position_ante < 3000)
  {
    Serial.println("tourner");
    tournantgauche3();
  }
  /*if((sensorValues[3]==1000||sensorValues[4]==1000)&&sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000&&position_ante<5000&&position_ante>3000){
    toutdroit();
  }*/
  /*if (sensorValues[0] == 1000 && sensorValues[4] == 1000 && sensorValues[7] == 1000)
  {
    stop();
  }
  /*if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000){
    toutdroit();
  }*/
  /*if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[7] < 1000 && position_ante == 7000)
  {
    tournantdroite3();
  }
  if (sensorValues[0] < 1000 && sensorValues[1] < 1000 && sensorValues[2] < 1000 && sensorValues[3] < 1000 && sensorValues[4] < 1000 && sensorValues[5] < 1000 && sensorValues[6] < 1000 && sensorValues[7] < 1000 && position_ante == 0)
  {
    Serial.println("horsdeposition");
    tournantgauche3();
  }*/
}

/*void makeDecision2()
{
  if (sensorValues[0] == 1000 || sensorValues[1] == 1000 || sensorValues[2] == 1000)
  {
    G = 1;
    D = 0;
    /*Serial.println("turnLeft----");
    motors.setM2Speed(0);
    motors.setM1Speed(5);
    turnLeft();
  }
  if (sensorValues[5] == 1000 || sensorValues[6] == 1000 || sensorValues[7] == 1000)
  {
    G = 0;
    D = 1;
    /*Serial.println("turnRight----");
    motors.setM1Speed(0);
    motors.setM2Speed(5);
    turnRight();
  }
}*/

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
 
 
  //if (Left_speed >255){Left_speed = 200;}
  //if (Right_speed >255){Right_speed = 200;}
  if (speedm2 <0){speedm2 = 0;}
  if (speedm1 <0){speedm1 = 0;}
 
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

  //makeDecision();

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

  // makeDecision2();

  Line_Follow(position);

  analogWrite(9,speedm1);
  analogWrite(10,speedm2);

  //delay(250);

  //int position_ante = getPosition();

  // int speed_ante = getSpeed();
}