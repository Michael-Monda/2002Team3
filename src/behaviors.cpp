// @author RBE2002 Team #3:
// Michael Monda
// Samantha Booher
// Chris Adzima
// Aditri Thakur

// physical object imports
#include <Romi32U4.h>
#include "Speed_controller.h"
#include "Wall_following_controller.h"

// virtual object imports
#include "Behaviors.h"
#include "Algorithms.h"
#include "Position_estimation.h"

// sensor imports
#include "IMU.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

// sensor declarations
IMU_sensor leyte;
IRsensor ranger;
SonarSensor hornet;

Romi32U4ButtonA buttonA;

// complex data-type median filter
Algorithm medAccel_x;
Algorithm medAccel_y;
Algorithm medAccel_z;

// virtual object declarations
SpeedController intrepid;
WallFollowingController tarawa;
Romi32U4Motors saipan;

void Behaviors::Init(void)
{
    leyte.Init();
    medAccel_x.Init();
    medAccel_y.Init();
    medAccel_z.Init();
    intrepid.Init();
    tarawa.Init();
}

// this needs to be configured to measure the collision strength from any direction
// on the plane. running theory is that pythagorean theorem could achieve this.
boolean Behaviors::DetectCollision(void)
{
    auto data_acc = leyte.ReadAcceleration();
    data[0] = medAccel_x.ComplexTypeFilter(data_acc.X)*0.061;
    data[1] = medAccel_y.ComplexTypeFilter(data_acc.Y)*0.061;
    data[2] = medAccel_z.ComplexTypeFilter(data_acc.Z)*0.061;
    if((data[0]) < threshold) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_pickup = leyte.ReadAcceleration();
    data[0] = (medAccel_x.ComplexTypeFilter(data_pickup.X) * 0.061);
    data[1] = (medAccel_y.ComplexTypeFilter(data_pickup.Y) * 0.061);
    data[2] = (medAccel_z.ComplexTypeFilter(data_pickup.Z) * 0.061);
    if((abs(data[2])) > threshold_pickup) return 1;     // this returns only if the romi is being lifted.
    else return 0;
}

void Behaviors::Stop(void)
{
    intrepid.Stop();
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
            intrepid.Stop(); //action
        } else { //transition condition
            robot_state = IDLE; 
            intrepid.Stop(); //action 
        }   
    break;
    }

    case DRIVE:
    {
        if (buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp() == true) { //transition condition
            robot_state = IDLE;
            intrepid.Stop();
        }

        if (DetectCollision()) {
            robot_state = REVERSE;
            intrepid.Stop();
        }

        else {
            intrepid.Run(speed, speed);
        }

    break;
    }

    case REVERSE:
    {   // back up for 10 centimeters
        intrepid.Reverse(50, 10);
        robot_state = TURN;
        
        //else if (backup condition == true) switch to turn state.
    break;
    }

    case TURN:
    {
        intrepid.Turn(90, false);
        robot_state = DRIVE;

    break;
    }
    
    }
}