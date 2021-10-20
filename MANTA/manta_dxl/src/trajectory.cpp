#include "manta_dxl/trajectory.h"

#include <chrono>
#include <sys/time.h>
#include <ctime>

#include <cmath>

#include <ros/ros.h>

Trajectory::Trajectory(float _maxVel, float _acc, float _dec, float _thresh)
{
  target = 0;
  curPos = 0;
  curVel = 0;
  maxVel = _maxVel;
  velGoal = _maxVel;
  acc = _acc;
  if (_dec == -1)
    dec = _acc;
  else
    dec = _dec;
  oldTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  threshold = _thresh;
  noTasks = true;
  target_time = 100000000;
}

Trajectory::~Trajectory()
{
}



bool Trajectory::setTargetPos(float _targetPos, float _time)
{
  target = _targetPos;

  target_time = _time*1000;
  _time = _time + sqrt(2 * threshold / dec);

  float sqrtTerm = (_time * _time) - 2 * abs(_targetPos - curPos) * (1 / acc + 1 / dec);

  if (sqrtTerm >= 0)
  {
    velGoal = (_time - sqrt(sqrtTerm)) / (1 / acc + 1 / dec);

    if (velGoal > maxVel)
      velGoal = maxVel;
  }
  else
  {
    velGoal = maxVel;
  }

  noTasks = false;
  startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  if (velGoal == maxVel)
    return false;
  else
    return true;
}




float Trajectory::update(float dT)
{
unsigned long currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  
  dT /= 1000.0;
    float timeError = currTime-startTime;

    float posError = target - curPos;

    if (abs(posError) > threshold) 
    {

      bool dir = true;
      if (posError < 0)
        dir = false;

      float acceleration = acc;
      if ((curVel * curVel / (2 * dec)) >= abs(posError))
        acceleration = -dec;

      if (dir)
        curVel += acceleration * dT;
      else
        curVel -= acceleration * dT;

      if (curVel > velGoal)
        curVel = velGoal;
      else if (curVel < -velGoal)
        curVel = -velGoal;

      float dP = curVel * dT;

      if (abs(dP) < abs(posError))
        curPos += dP;
      else
        curPos = target;

      //Serial.print(millis()); Serial.print(",");
      //Serial.print(curPos); Serial.print(",");
      //Serial.print(curVel); Serial.print(",");
      //if (abs(curVel) != velGoal) Serial.println(acceleration * (-1 + 2*dir));
      //else Serial.println(0);
    }
    else if(abs(timeError) <target_time)
    {
      curPos = curPos;
    } 
    else
    { 

      curVel = 0;
      curPos = target;
      noTasks = true;
    }

    // ROS_INFO(" %ld %f ",currTime-startTime,target_time);

  // Error check for NaN
  //if (curPos != curPos) {
  //	//printf(F("ERROR: Dynamics calculation error"));
  //  curPos = target;
  //	curVel = 0;
  //}

  return curPos;
}

float Trajectory::update()
{
  // Calculate Time Change
  unsigned long newTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  float dT = float(newTime - oldTime);
  oldTime = newTime;
  return update(dT);
}

void Trajectory::reset(float newPos)
{
  curPos = newPos;
  curVel = 0;
  target = 0;
  velGoal = maxVel;
  noTasks = true;
  oldTime =  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

}

void Trajectory::setPos(float newPos)
{
  curPos = newPos;
}
