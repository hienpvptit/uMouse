#include "uMouse.h"

void SimpleKalmanFilter::init(float mea_e, float est_e, float q)
{
	this->mea_e = mea_e;
	this->est_e = est_e;
	this->q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
	this->kalman_gain = this->est_e / (this->est_e + this->mea_e);
	this->current_est = this->last_est + this->kalman_gain * (mea - this->last_est);
	this->est_e =  (1.0 - this->kalman_gain) * this->est_e + fabs(this->last_est - this->current_est) * this->q;
	this->last_est = this->current_est;
	return this->current_est;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
	this->mea_e = mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
	this->est_e = est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
	this->q = q;
}

void SimpleKalmanFilter::reset()
{
	this->current_est = 0;
	this->last_est = 0;
	this->kalman_gain = 0;
}

float SimpleKalmanFilter::getKalmanGain() {
  return this->kalman_gain;
}

/*--------------------------------------------------------------------*/

void Motor::setMotor(uint8_t speedPin, uint8_t directPin){
	this->speedPin = speedPin;
    this->directPin = directPin;
    pinMode(this->speedPin, OUTPUT);
    pinMode(this->directPin, OUTPUT);
    this->direct = 0;
    this->speed = 0;
}

void Motor::setSpeed(uint8_t speed){
	this->speed = speed;
}

void Motor::setDirect(int8_t direct){
	this->direct = direct;
}

void Motor::enable(){
	if(this->direct==-1){
    	digitalWrite(this->directPin, LOW);
        analogWrite(this->speedPin, this->speed);   
    }
    else if(direct==0){
        analogWrite(this->speedPin, 0);  
    }
    else{
        digitalWrite(this->directPin, HIGH);
        analogWrite(this->speedPin, this->speed);
    }
}

/*--------------------------------------------------------------------*/

Robot::Robot(){
    this->leftMotor.setMotor(3, 5);
    this->rightMotor.setMotor(6, A1);
}

void Robot::forward(uint8_t speed){
    this->leftMotor.setDirect(1);
    this->leftMotor.setSpeed(speed);
    this->rightMotor.setDirect(1);
    this->rightMotor.setSpeed(speed);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

void Robot::back(uint8_t speed){
    this->leftMotor.setDirect(-1);
    this->leftMotor.setSpeed(speed);
    this->rightMotor.setDirect(-1);
    this->rightMotor.setSpeed(speed);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

void Robot::crolLeft(uint8_t speed){
	this->leftMotor.setDirect(-1);
	this->leftMotor.setSpeed(speed);
	this->rightMotor.setDirect(1);
	this->rightMotor.setSpeed(speed);
	this->leftMotor.enable();
	this->rightMotor.enable();
}

void Robot::crolRight(uint8_t speed){
	this->leftMotor.setDirect(1);
	this->leftMotor.setSpeed(speed);
	this->rightMotor.setDirect(-1);
	this->rightMotor.setSpeed(speed);
	this->leftMotor.enable();
	this->rightMotor.enable();
}


void Robot::turn(uint8_t speedL, uint8_t speedR){
	this->leftMotor.setDirect(1);
	this->leftMotor.setSpeed(speedL);
	this->rightMotor.setDirect(1);
	this->rightMotor.setSpeed(speedR);
	this->leftMotor.enable();
	this->rightMotor.enable();
}

void Robot::stop(){
    this->leftMotor.setSpeed(0);
    this->rightMotor.setSpeed(0);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

/*--------------------------------------------------------------------*/
void SR04::setSensor(uint8_t triggerPin, uint8_t echoPin){
	this->triggerPin = triggerPin;
	this->echoPin = echoPin;
	pinMode(this->triggerPin, OUTPUT);
	pinMode(this->echoPin, INPUT);
}

int32_t SR04::read(){
	digitalWrite(this->triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(this->triggerPin, HIGH);
	delayMicroseconds(10);
    digitalWrite(this->triggerPin, LOW);
    uint32_t durationMicroSec = pulseIn(this->echoPin, HIGH);
    int32_t distanceCm = (int32_t)(durationMicroSec / 2.0 * 0.0343);
    return (distanceCm == 0 || distanceCm > 400) ? -1 : distanceCm;
}

/*--------------------------------------------------------------------*/
Sensor::Sensor(){
	this->filterLeft.init();
	this->filterLeft.reset();
	this->filterRight.init();
	this->filterRight.reset();
	this->filterFront.init();
	this->filterFront.reset();
	this->leftSensor.setSensor(A4, A2);  
    this->rightSensor.setSensor(1, A0);
    this->frontSensor.setSensor(11, 12);
}

int32_t Sensor::readLeft(){
	return this->leftSensor.read();
}

int32_t Sensor::readLeft_filter(){
	int32_t tmp;
	tmp = this->leftSensor.read();
	if(tmp==-1)
		return -1;
	return (int32_t)filterLeft.updateEstimate(tmp);
}

int32_t Sensor::readRight(){
	return this->rightSensor.read();
}

int32_t Sensor::readRight_filter(){
	int32_t tmp;
	tmp = this->rightSensor.read();
	if(tmp==-1)
		return -1;
	return (int32_t)filterRight.updateEstimate(tmp);
}

int32_t Sensor::readFront(){
	return this->frontSensor.read();
}

int32_t Sensor::readFront_filter(){
	int32_t tmp;
	tmp = this->frontSensor.read();
	if(tmp==-1)
		return -1;
	return (int32_t)filterFront.updateEstimate(tmp);
}


void Sensor::readAll(int32_t *distance){
	distance[0] = this->leftSensor.read();
	distance[1] = this->frontSensor.read();
	distance[2] = this->rightSensor.read();	
}

/*--------------------------------------------------------------------*/
Console::Console(uint16_t baud){
	this->baud = baud;
}

void Console::init(){
	Serial.begin(9600);
}

void Console::log(String str)
{
	str += ";";
	Serial.print(str);
}

void Console::log(uint16_t val)
{
	String str = String(val);
	str += ";";
	Serial.print(str);
}

void Console::log(uint32_t val)
{
	String str = String(val);
	str += ";";
	Serial.print(str);
}

void Console::log(int16_t val)
{
	String str = String(val);
	str += ";";
	Serial.print(str);
}
void Console::log(int32_t val)
{
	String str = String(val);
	str += ";";
	Serial.print(str);
}

/*----------------------------------------------*/
PID::PID()
{
	this->Kp = 4; 
	this->Ki = 0; 
	this->Kd = 2;
	this->P = 0;
	this->I = 0;
	this->D = 0;
	this->previous_error = 0;
}

int16_t PID::calculate(int16_t error)
{
	this->P = error;
	this->I += error;
	if(this->I > 1000 && this->I<-1000)
		this->I = 0;
	this->D = error - this->previous_error;
	this->PID_value = (this->Kp*this->P) + (this->Ki*this->I) + (this->Kd*this->D);
	this->previous_error=error;
	return (int16_t)this->PID_value;
}



