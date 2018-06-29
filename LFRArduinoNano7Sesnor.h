#ifndef __LFR_ARDUINO_NANO_7SENS_H
#define __LFR_ARDUINO_NANO_7SENS_H

#include "Arduino.h"


#include <TimerOne.h>
#include <EEPROM.h>


class LFR
{
  public:
	void Init();
	void Initialize(float _kp, float _ki, float _kd);
	void moveForward(unsigned int _speed);
	void stop();
	void turnRight(unsigned int _speed);
	void turnLeft(unsigned int _speed);
  private:
	int pwmLeftMotorPin = 9;
	int pwmRightMotorPin = 10;
	int dirALeftMotorPin = 8;
	int dirBLeftMotorPin = 11;
	int dirARightMotorPin = 12;
	int dirBRightMotorPin = 13;
	
	#define numSens 7
	
	unsigned char lineSensor;
	
	enum _sens{
		right3Sens,
		right2Sens,
		right1Sens,
		midSens,
		left1Sens,
		left2Sens,
		left3Sens
	};
	
	int sens[numSens]={A0, A1, A2, A3, A4, A5, A6};
	unsigned int maxSens[numSens], minSens[numSens], refSens[numSens];
	unsigned char sensIO[numSens];
	
	int batSensPin = A7;

	int startButton_pin = 0;
	int menuButton_pin = 1;
	
	const unsigned long samplingPID = 50;
	
	float  Input, lasterrorSetPoint;
	float Kp, Ki, Kd;
	float iInput;
	float PID;
	
	unsigned long lastSamplingLcd, lastSamplingClearLcd, lastTimePID;
	
	const unsigned long samplingLcd = 200;
	
	char delay_ms_and_checkPin(unsigned long wtime);
	void Callibration();
	void rightMotorPWM(int _speed);
	void leftMotorPWM(int _speed);
	void readSensor();
	void writeSenstoLCD();
	void controlPID(int setPointSpeed);
	
	
};
#endif
