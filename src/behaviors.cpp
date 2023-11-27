#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"
#include "algorithms.h"

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//motor-speed controller
SpeedController PIcontroller;
Romi32U4Motors drivetrain;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((data[0]) < threshold) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_pickup = LSM6.ReadAcceleration();
    data[0] = (med_x.Filter(data_pickup.X) * 0.061);
    data[1] = (med_y.Filter(data_pickup.Y) * 0.061);
    data[2] = (med_z.Filter(data_pickup.Z) * 0.061);
    if((abs(data[2])) > threshold_pickup) return 1;     // this returns only if the romi is being lifted.
    else return 0;
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

// IDLE, DRIVE, REVERSE, TURN
void Behaviors::Run(void)
{
    auto data_crash = LSM6.ReadAcceleration();

    switch (robot_state)
    {
    case IDLE:
    {
        if(buttonA.getSingleDebouncedRelease()) {
            robot_state = DRIVE; 
            PIcontroller.Stop(); //action
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
        } 
        else { //transition condition
            robot_state = IDLE; 
            PIcontroller.Stop(); //action 
        }   
    break;
    }

    case DRIVE:
    {
        
        Serial.print('\t');
        Serial.print('\t');
        Serial.print((med_x.Filter(data_crash.X) * 0.061));
        Serial.print('\t');
        Serial.print('\t');
        Serial.print((med_y.Filter(data_crash.Y) * 0.061));
        Serial.print('\t');
        Serial.print('\t');
        Serial.print((med_z.Filter(data_crash.Z) * 0.061));
        Serial.print('\t');
        Serial.print('\t');           
        Serial.println(millis());


        if (buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp() == true) { //transition condition
            robot_state = IDLE;
            PIcontroller.Stop();
        }

        if (DetectCollision()) {
            robot_state = REVERSE;
            PIcontroller.Stop();
        }

        else {
            PIcontroller.Run(speed, speed);
        }
    break;
    }

    case REVERSE:
    {   // back up for 10 centimeters
        PIcontroller.Reverse(50, 10);
        robot_state = TURN;
        
        //else if (backup condition == true) switch to turn state.
    break;
    }

    case TURN:
    {
        PIcontroller.Turn(90, false);
        robot_state = DRIVE;

    break;
    }
    
    }
}