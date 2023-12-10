#include <Romi32U4.h>
#include "Speed_controller.h"
#include "Wall_following_controller.h"

#include "Behaviors.h"
#include "Algorithms.h"
#include "Position_estimation.h"

#include "IMU.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

//sensors
IMU_sensor leyte;
IRsensor ranger;
SonarSensor hornet;

Romi32U4ButtonA buttonA;

// complext data-type median filter
Algorithm gyro_x_stab;
Algorithm gyro_y_stab;
Algorithm gyro_z_stab;

//motor-speed controller
SpeedController DriveControl;
WallFollowingController FollowControl;
Romi32U4Motors drivetrain;


void Behaviors::Init(void)
{
    leyte.Init();
    ranger.Init();
    hornet.Init();
    gyro_x_stab.Init();
    gyro_y_stab.Init();
    gyro_z_stab.Init();
    DriveControl.Init();
    FollowControl.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = leyte.ReadAcceleration();
    data[0] = gyro_x_stab.ComplexTypeFilter(data_acc.X)*0.061;
    data[1] = gyro_y_stab.ComplexTypeFilter(data_acc.Y)*0.061;
    data[2] = gyro_z_stab.ComplexTypeFilter(data_acc.Z)*0.061;
    if((data[0]) < threshold) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_pickup = leyte.ReadAcceleration();
    data[0] = (gyro_x_stab.ComplexTypeFilter(data_pickup.X) * 0.061);
    data[1] = (gyro_y_stab.ComplexTypeFilter(data_pickup.Y) * 0.061);
    data[2] = (gyro_z_stab.ComplexTypeFilter(data_pickup.Z) * 0.061);
    if((abs(data[2])) > threshold_pickup) return 1;     // this returns only if the romi is being lifted.
    else return 0;
}

void Behaviors::Stop(void)
{
    DriveControl.Stop();
}

// IDLE, DRIVE, REVERSE, TURN
void Behaviors::Run(void)
{
    auto data_crash = leyte.ReadAcceleration();

    switch (robot_state)
    {
    case IDLE:
    {
        if(buttonA.getSingleDebouncedRelease()) {
            robot_state = DRIVE; 
            DriveControl.Stop(); //action
        } else { //transition condition
            robot_state = IDLE; 
            DriveControl.Stop(); //action 
        }   
    break;
    }

    case DRIVE:
    {
        if (buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp() == true) { //transition condition
            robot_state = IDLE;
            DriveControl.Stop();
        }

        if (DetectCollision()) {
            robot_state = REVERSE;
            DriveControl.Stop();
        }

        else {
            DriveControl.Run(speed, speed);
        }

    break;
    }

    case REVERSE:
    {   // back up for 10 centimeters
        DriveControl.Reverse(50, 10);
        robot_state = TURN;
        
        //else if (backup condition == true) switch to turn state.
    break;
    }

    case TURN:
    {
        DriveControl.Turn(90, false);
        robot_state = DRIVE;

    break;
    }
    
    }
}