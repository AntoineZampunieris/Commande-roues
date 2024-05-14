#include <Arduino.h>

#include <QTRSensors.h>

#include <DRV8835MotorShield.h>

#include "DRV8835MotorShield.h"

//#define LED_PIN 13

//#include <stdint.h>

// This example is designed for use with eight RC QTR sensors. These
// reflectance sensors should be connected to digital pins 3 to 10. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 10000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 10000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

QTRSensors qtr;
DRV8835MotorShield motors;

//QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10}, 8);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int speed;



void demarrage(){
  //motors.flipM1(true);
  for (int speed = 0; speed <= 400; speed++)
  {
    
    motors.setSpeeds(speed,speed);
    
    motors.setM1Speed(speed);
    delay(10);
  }
  

}



void setup()
{

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A4, A3, A2, A1, A0, 4, 5, 6}, SensorCount);
  qtr.setEmitterPin(A5);

  delay(1000);
  digitalWrite(2,HIGH);
 
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



void tournantdroite3(int speed){
  int speedm1=speed;
  do{
  for (int speed; speed >=0 ; speed--)
  {
    motors.setM1Speed((speedm1+speed)/2);
    motors.setM2Speed(speed);
    delay(2);
    
  }
  }while(speed>0);
  motors.setM1Speed((speedm1+speed)/2);
  motors.setM2Speed(speed);
  
}
void tournantdroite2(int speed){
  int speedm1=speed;
  do{
  for (int speed; speed >=100 ; speed--)
  {
    motors.setM1Speed((speedm1+speed)/2);
    motors.setM2Speed(speed);
    delay(3);
    
  }
  }while(speed>100);
  motors.setM1Speed((speedm1+speed)/2);
  motors.setM2Speed(speed);
    
}
void tournantdroite1(int speed){
  int speedm1=speed;
  do{
  for (int speed; speed >=200 ; speed--)
  {
    motors.setM1Speed((speedm1+speed)/2);
    motors.setM2Speed(speed);
    delay(4);
    
  }
  }while(speed>200);
      motors.setM1Speed((speedm1+speed)/2);
      motors.setM2Speed(speed);
}

void tournantgauche3(int speed){
  int speedm2=speed;
  do{
  for (int speed; speed >=0 ; speed--)
  {
    motors.setM2Speed((speedm2+speed)/2);
    motors.setM1Speed(speed);
    delay(2);
    
  }
  }while(speed>0);
      motors.setM2Speed((speedm2+speed)/2);
      motors.setM1Speed(speed);
}

void tournantgauche2(int speed){
  int speedm2=speed;
  do{
  for (int speed; speed >=100 ; speed--)
  {
    motors.setM2Speed((speedm2+speed)/2);
    motors.setM1Speed(speed);
    delay(2);
    
  }
  }while(speed>100);
      motors.setM2Speed((speedm2+speed)/2);
      motors.setM1Speed(speed);
}
void tournantgauche1(int speed){
  int speedm2=speed;
  do{
  for (int speed; speed >=200 ; speed--)
  {
    motors.setM2Speed((speedm2+speed)/2);
    motors.setM1Speed(speed);
    delay(2);
    
  }
  }while(speed>200);
      motors.setM2Speed((speedm2+speed)/2);
      motors.setM1Speed(speed);
}

void toutdroit(){
  motors.flipM1(true);
  for (int speed; speed==400 ; speed++)
  {
    motors.setSpeeds(speed,speed);
    delay(2);
    
  }
  
}

void stop(){
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setSpeeds(speed,speed);
    delay(20);
  }
}

int getPosition(){
  return qtr.readLineBlack(sensorValues);
}

int getSpeed(){
  return speed;
}

void makeDecision(){
  if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[7]==1000&&getPosition()>10000&&getPosition()<3000){
    tournantdroite3(speed);
  }
  if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[6]==1000&&getPosition()>10000&&getPosition()<3000){
    tournantdroite2(speed);
  }
  if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]==1000&&getPosition()>10000&&getPosition()<3000){
    tournantdroite1(speed);
  }
  if(sensorValues[7]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[2]==1000&&getPosition()>4000){
    tournantgauche1(speed);
  }
  if(sensorValues[7]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[1]==1000&&getPosition()>4000){
    tournantgauche2(speed);
  }
  if(sensorValues[7]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[0]==1000&&getPosition()<3000){
    tournantgauche3(speed);
  }
  if((sensorValues[3]==1000||sensorValues[4]==1000)&&sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000&&getPosition()<5000&&getPosition()>3000){
    toutdroit();
  }
  if(sensorValues[0]==1000&&sensorValues[4]==1000&&sensorValues[7]==1000){
    stop();
  }
  /*if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000){
    toutdroit();
  }
  if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000&&getPosition()>10000){
    tournantdroite3(speed);
  }
  if(sensorValues[0]<1000&&sensorValues[1]<1000&&sensorValues[2]<1000&&sensorValues[3]<1000&&sensorValues[4]<1000&&sensorValues[5]<1000&&sensorValues[6]<1000&&sensorValues[7]<1000&&getPosition()<3000){
    tournantgauche3(speed);
  }*/
  else{
    toutdroit();
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

  makeDecision();
  

  delay(250);
  
  getPosition();

  getSpeed();
}

DRV8835MotorShield::DRV8835MotorShield() :
  _M1DIR(7), _M1PWM(9), _M2DIR(8), _M2PWM(10)
{
}

DRV8835MotorShield::DRV8835MotorShield(uint8_t M1DIR,
                                       uint8_t M1PWM,
                                       uint8_t M2DIR,
                                       uint8_t M2PWM) :
  _M1DIR(M1DIR), _M1PWM(M1PWM),
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
  if (speed > 400)  // max
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
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // max PWM duty cycle
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
void DRV8835MotorShield::setSpeeds(int m1Speed, int m2Speed){
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