#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "LFRArduinoNano7Sesnor.h"
// initialize the library with the numbers of the interface pins

	
/* Initialize()
 ******************************************************************************/ 
 void LFR::Init(){
	 
	pinMode(pwmLeftMotorPin, OUTPUT);
	pinMode(pwmRightMotorPin, OUTPUT); 
	pinMode(dirALeftMotorPin, OUTPUT);
	pinMode(dirBLeftMotorPin, OUTPUT);
	pinMode(dirARightMotorPin, OUTPUT);
	pinMode(dirBRightMotorPin, OUTPUT);
	Timer1.initialize(200);  // 200 us = 5 kHz
	LFR::rightMotorPWM(0);
	LFR::leftMotorPWM(0);
	delay(1500);
 }
 

void LFR::turnRight(unsigned int _speed){
	LFR::leftMotorPWM(_speed);

}

void LFR::turnLeft(unsigned int _speed){
	LFR::rightMotorPWM(_speed);
	LFR::leftMotorPWM(-(_speed-50));
	
}

void LFR::stop(){
	LFR::rightMotorPWM(0);
	LFR::leftMotorPWM(0);
}


void LFR::rightMotorPWM(int _speed){
	_speed = constrain(_speed, -1023, 1023);
	if(_speed>0)
	{
		digitalWrite(dirARightMotorPin, LOW);
		digitalWrite(dirBRightMotorPin, HIGH);
		Timer1.pwm(pwmRightMotorPin, _speed);
	}
	else if(_speed<0)
	{
		digitalWrite(dirARightMotorPin, HIGH);
		digitalWrite(dirBRightMotorPin, LOW);
		Timer1.pwm(pwmRightMotorPin, -_speed);
	}
	else{
		digitalWrite(dirARightMotorPin, LOW);
		digitalWrite(dirBRightMotorPin, LOW);
		Timer1.pwm(pwmRightMotorPin, _speed);
	}
}

void LFR::leftMotorPWM(int _speed){
	_speed = constrain(_speed, -1023, 1023);
	if(_speed>0)
	{
		digitalWrite(dirALeftMotorPin, HIGH);
		digitalWrite(dirBLeftMotorPin, LOW);
		Timer1.pwm(pwmLeftMotorPin, _speed);
	}
	else if(_speed<0)
	{
		digitalWrite(dirALeftMotorPin, LOW);
		digitalWrite(dirBLeftMotorPin, HIGH);
		Timer1.pwm(pwmLeftMotorPin, -_speed);
	}
	else{
		digitalWrite(dirALeftMotorPin, LOW);
		digitalWrite(dirBLeftMotorPin, LOW);
		Timer1.pwm(pwmLeftMotorPin, _speed);
	}
}

void LFR::writeSenstoLCD(){
	if((millis() - lastSamplingLcd) >= samplingLcd){	
		
		// lcd.setCursor(0,1);  lcd.print(lineSensor, BIN);		
		lastSamplingLcd = millis();	
	}
}

void LFR::controlPID(int setPointSpeed){
	float pInput, dInput;
	float errorSetPoint;
	int pwmRight, pwmLeft;
	
	if((millis() - lastTimePID) >= samplingPID)
	{
		if(lineSensor == B0001000)			{Input = 0;}
		else if (lineSensor == B0001100)	{Input = 1;}
		else if (lineSensor == B0000100)	{Input = 3;}
		else if (lineSensor == B0000110)	{Input = 4;}
		else if (lineSensor == B0000010)	{Input = 10;}
		// else if (lineSensor == B0000001)	{Input = 20;}
		else if (lineSensor == B0011000)	{Input = -1;}
		else if (lineSensor == B0010000)	{Input = -3;}
		else if (lineSensor == B0110000)	{Input = -4;}
		else if (lineSensor == B0100000)	{Input = -10;}
		// else if (lineSensor == B1000000)	{Input = -20;}
		
		errorSetPoint = 0 - Input;
		pInput  = Kp*errorSetPoint;
		iInput += Ki*errorSetPoint;
		iInput	= constrain (iInput, -100, 100);
		dInput = (errorSetPoint-lasterrorSetPoint)*Kd;
		PID = pInput + dInput;
		pwmRight = setPointSpeed + PID;
		pwmLeft = setPointSpeed - PID;
		LFR::rightMotorPWM (pwmRight);
		LFR::leftMotorPWM  (pwmLeft);
		
		lasterrorSetPoint = errorSetPoint;
		lastTimePID = millis();
	}		
}
