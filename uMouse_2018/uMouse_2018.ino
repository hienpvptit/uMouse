#include "uMouse.h"
Robot robot;
Sensor sensor;
PID pid;

long left, right, front, error;

void setup() {

}


void loop() {
  robotForwardPid();
}


void robotForwardPid(){
  left = sensor.readLeft();
  right = sensor.readRight();
  if(left==-1 || right==-1)
    return;
  error = left - right;
  error = constrain(error, -30, 30);
  if(1) //left<15 && right<15
  {
    long pidValue = pid.calculate(error);
    robot.turn(constrain(200-pidValue, 0, 255), constrain(200+pidValue, 0, 255));
  }
  else 
    robot.stop();

}


void moveCm(int distance)
{
  long oldDistance;
  for(int i=0; i<10; i++)
    oldDistance = sensor.readFront_filter();
  do
  {
    //robotForwardPid();
    robot.forward(100);
    front = sensor.readFront();  
    Serial.println(front);
  }while(oldDistance - front < distance);
  robot.stop();
}

void debugSensor()
{
  left = sensor.readLeft_filter();
  right = sensor.readRight_filter();
  front = sensor.readFront_filter();
  Serial.print(left); Serial.print(" ");
  Serial.print(front); Serial.print(" ");
  Serial.println(right);  
}

