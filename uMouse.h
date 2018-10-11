/*
  uMouse.h - Library for uMouse - Robot maze solving
  Copyright (c) 2018 AHN Team
  
  *uMouse PTIT 2018*
  * 
  * --------------
  * Using
  * ---------------------------------------------
  * 				speedPin		directPin
  * Left motor			3				5
  * Right motor			6				A1
  * 
  * ---------------------------------------------
  * 				triggerPin		echoPin
  * Left SR04			A4				A2
  * Right SR04			1				A0
  * Front SR04			11				12
  * 
  * ---------------------------------------------
  * 
*/

#ifndef _UMOUSE_H
#define _UMOUSE_H
#include "Arduino.h"
#include <math.h>


class SimpleKalmanFilter 
{
	public:
		void init(float mea_e=0.5, float est_e=0.5, float q=0.01);
		float updateEstimate(float mea);
		void setMeasurementError(float mea_e);
		void setEstimateError(float est_e);
		void setProcessNoise(float q);
		void reset();
		float getKalmanGain();
	  
	private:
		float mea_e;
		float est_e;
		float q;
		float current_est;
		float last_est;
		float kalman_gain; 
};

/*
 * 
 *	Class Motor
 *  
 */
 
class Motor{
   public:
		/*
		 * Setup motor
		 */
		void setMotor(uint8_t speedPin, uint8_t directPin);
    
    
		/*
		 * Set speed for motor
		 * * 0 <= speed <= 255
		 */
		void setSpeed(uint8_t speed);
    
    
		/*
		 * Set direct for motor
		 * * direct = 1  -> Motor forward
		 * * direct = 0  -> Motor stop
		 * * direct = -1 -> Motor back 
		 */
		void setDirect(int8_t direct);
		
		
		/*
		 * Enable Motor
		 */	
		void enable();
    
  private:
		uint8_t speedPin, directPin;
		uint8_t speed;
		int8_t  direct;
};

/*
 * 
 * Class Robot
 * 
 */

class Robot{
  public:
		/*
		 * Constructor
		 * Setup 2 motor 
		 */
		Robot();
  		
  		
  		/*
  		 * Robot forward, back, crolLeft, crolRight with speed
  		 * * 0 <= speed <= 255
  		 */ 
		void forward(uint8_t speed);
    
		void back(uint8_t speed);
		
		void crolLeft(uint8_t speed);
		
		void crolRight(uint8_t speed);
		
		
		/*
		 * Robot move with left motor speed and right motor speed
		 * * 0 <= speedL, speedR <= 255
		 */
		void turn(uint8_t speedL, uint8_t speedR);
		
		
		/*
		 * Robot stop
		 */
		void stop();
    
  private:
		Motor leftMotor, rightMotor;
};


/*
 * 
 * Class SR04
 * 
 */

class SR04{
	public:
		/*
		 * Set sensor
		 * Trigger pin and Echo pin off HC-SR04
		 */
		void setSensor(uint8_t triggerPin, uint8_t echoPin);
		
		
		/*
		 * Read distance from HC-SR04
		 * Value return is int32_t
		 * Unit: centimeter
		 */
		int32_t read();
		
	private:
		uint8_t echoPin, triggerPin;	
};

/*
 * 
 * Class Sensor
 * 
 */

class Sensor{
	public:
		/*
		 * Constructor
		 * Setup trigger pin and echo pin of 3 sensor Left, Front, Right
		 */
		Sensor();
		
		
		/*
		 * Read distance from 3 sensor HC-SR04
		 * Value return is int32_t
		 * Unit: centimeter
		 */
		int32_t readLeft();
		int32_t readLeft_filter();
		
		int32_t readRight();
		int32_t readRight_filter();
		
		int32_t readFront();
		int32_t readFront_filter();
		
		void readAll(int32_t *distance);
		
	private:
		SR04 frontSensor, leftSensor, rightSensor;
		SimpleKalmanFilter filterLeft, filterRight, filterFront;
		
};

/*
 * 
 * Class Epprom
 * 
 */

class Epprom{
	public:
		/*
		 * Read EPPROM at address
		 * 0 <= Address <= 1023
		 * Return value uint8_t
		 */
		uint8_t readAt(uint8_t address);
		
		/*
		 * Read memory cells where have value diffrent 255
		 * Value return by reference *val
		 */
		uint8_t read(uint8_t *val);
		
		/*
		 * Write EPPROM at address
		 * 0 <= Address <= 1023
		 */
		void writeAt(uint8_t address);
		
		/*
		 *	Write a array to the EPPROM 
		 */
		void write(uint8_t *val);
};


class Console{
	public:
		/*
		 * 
		 */
		Console(uint16_t baud);
		
		/*
		 *
		 */
		void init(); 
		/*
		 *
		 */
		 
		void log(String str);
		void log(uint16_t val);
		void log(uint32_t val);
		void log(int16_t val);
		void log(int32_t val);
	private:
		uint16_t baud;
		
};


class PID{
	public:
		PID();
		int16_t calculate(int16_t error);
		
	private:
		float Kp, Kd, Ki;
		float P, I, D, PID_value;
		int16_t previous_error;
};


#endif
