#include <Romi32U4.h>
#include "Speed_controller.h"
#include "Wall_following_controller.h"

#include "Behaviors.h"
#include "Algorithms.h"

#include "IMU.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

//sensors
IMU_sensor LSM6;
IRsensor ranger;
SonarSensor hornet;

Romi32U4ButtonA buttonA;

// complext data-type median filter
Algorithm med_x;
Algorithm med_y;
Algorithm med_z;

//motor-speed controller
SpeedController PIcontroller;
WallFollowingController PDcontroller;
Romi32U4Motors drivetrain;


void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
    PDcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.ComplexTypeFilter(data_acc.X)*0.061;
    data[1] = med_y.ComplexTypeFilter(data_acc.Y)*0.061;
    data[2] = med_z.ComplexTypeFilter(data_acc.Z)*0.061;
    if((data[0]) < threshold) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_pickup = LSM6.ReadAcceleration();
    data[0] = (med_x.ComplexTypeFilter(data_pickup.X) * 0.061);
    data[1] = (med_y.ComplexTypeFilter(data_pickup.Y) * 0.061);
    data[2] = (med_z.ComplexTypeFilter(data_pickup.Z) * 0.061);
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
        } else { //transition condition
            robot_state = IDLE; 
            PIcontroller.Stop(); //action 
        }   
    break;
    }

    case DRIVE:
    {
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