#include <Romi32U4.h>
#include "Encoders.h"
#include "PIDcontrollers.h"

// apriltag stuff
#include "apriltagdatum.h"
#include <Arduino.h>
#include <Wire.h>
#include <openmv.h>
//#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder;
AprilTagDatum dakota;
OpenMV camera2;

void PIDcontrollers::Init(void)
{
    MagneticEncoder.Init();
}

// TODO: replace  the following function with the one written for the Lab 5 Bonus signoff,
// since it performs much more consistently.
void PIDcontrollers::FollowAtDistance () {
    if(MagneticEncoder.UpdateEncoderCounts()){
        camera2.readTag(dakota);
        float e_left = (dakota.cx - median);
        float e_right = (median - dakota.cx);
        float e_Area = targetArea - (dakota.h * dakota.w);
        //float e_dist = targetDistance - hornet.FilterData(false);

        E_left += e_left;
        E_right += e_right;

        float v_left = Kpd*e_Area + Kp*e_left + Ki*E_left;
        float v_right = Kpd*e_Area + Kp*e_right + Ki*E_right;
        // float v_left = Kpd*e_Area * (targetWheelspeed + Kp*e_left + Ki*E_left) + (Kp*e_left + Ki*E_left);
        // float v_right = Kpd*e_Area * (targetWheelspeed + Kp*e_right + Ki*E_right) + (Kp*e_left + Ki*E_left);

        motors.setEfforts(v_left,v_right);
    }
}

void PIDcontrollers::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        
        // Serial.print(MagneticEncoder.ReadVelocityLeft());
        // Serial.print('\t');
        // Serial.println(MagneticEncoder.ReadVelocityRight());
    }
}

boolean PIDcontrollers::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*(degree/180.0); //assignment 1: convert degree into counts
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}
boolean PIDcontrollers::Straight(int target_velocity, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= (unsigned long)time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean PIDcontrollers::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= (unsigned long)time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

bool PIDcontrollers::Reverse(int target_velocity, int distance) //in mm/s and cm
{
    motors.setEfforts(0, 0);
    
    uint32_t duration = 1000*((distance*10)/(float)target_velocity); //in ms
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= duration){
        Run(-target_velocity,-target_velocity);
    }

    motors.setEfforts(0, 0);
    return 1;
}

void PIDcontrollers::FollowLine(int targetSpeed) {
    float 


    motors.setEfforts(leftSpeed, rightSpeed);
}

void PIDcontrollers::Stop()
{
    motors.setEfforts(0,0);
}