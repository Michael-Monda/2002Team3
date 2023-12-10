#include <Romi32U4.h>
#include "PIDcontrollers.h"
#include "Wall_following_controller.h"

#include "Behaviors.h"
#include "Algorithms.h"
#include "Position_estimation.h"

#include "IMU.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

#include "openmv.h"
#include "apriltagdatum.h"

//sensors
IMU_sensor leyte;       // I like to give them names because I think it's easier to associate them with their functions.
IRsensor ranger;
SonarSensor hornet;

Romi32U4ButtonA buttonA;

// complext data-type median filter
Algorithm gyro_x_stab;  // this object used to find the median x acceleration of the romi over a given interval.
Algorithm gyro_y_stab;  // this object used to find the median y acceleration of the romi over a given interval.
Algorithm gyro_z_stab;  // this object used to find the median z acceleration of the romi over a given interval.

//motor-speed controller
PIDcontrollers DriveControl;
WallFollowingController FollowControl;
Romi32U4Motors drivetrain;

// initialize all the things.
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

    // TODO:
    // based upon my conversation with professor Nemitz this week, there may be some
    // good solutions to non-normal collisions by creating a second threshold in the
    // y-direction and insituting an "or" operand in the if statement.

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

// TODO:
// below is the STATE MACHINE code for our final project.
// if we need to define new functions specific to actions
// done only for the final project, please define them in
// a new class so they can be properly organized. If not
// able, please denote them clearly and I will modify the
// workspace folder later today.

// additionally, consider creating a function which gets
// the size of an apriltag and compares it with the target,
// in order for the robot to change into a new state only
// when it sees it from a desired locaiton.

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