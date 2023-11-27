#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

IRsensor SharpIR;
SonarSensor HCSR04;
void WallFollowingController::Init(void)
{
    SharpIR.Init();
    HCSR04.Init();
}

float WallFollowingController::Run(float target_dist)
{
  //assignment 2: write a PD controller that outputs speed as a function of distance error
  float curr_dist = SharpIR.FilterData(false);           //obtain the current distance
  float diff = target_dist - curr_dist;             //find the difference between the target and current
  float prev_error = target_dist - prev_e_dist;     //establish the most recent error measurement
  float dist_error = diff - prev_error;             //determine the rate at which the difference is changing
  float speed = (diff * Kp) + (dist_error * Kd);    //adjust according to the difference, and at the rate at
  prev_e_dist = curr_dist;                          //which the difference changes
  return speed;                                     //return the adjusted motor speed.
}