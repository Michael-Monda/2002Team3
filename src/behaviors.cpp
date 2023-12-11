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

// romiNumber = 0; // 1 - Harry, 2 - Cedric, 3 - Krum, 4 - Fleur

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
    case IDLE: //KEEP IDLE
    {
        if((buttonA.getSingleDebouncedRelease()) && (romiNumber == 1)){
            robot_state = HARRY; 
            PIcontroller.Stop(); //action
        } else if ((buttonA.getSingleDebouncedRelease()) && (romiNumber == 2)) { //transition condition
            robot_state = CEDRIC; 
            PIcontroller.Stop(); //action 
        } else if ((buttonA.getSingleDebouncedRelease()) && (romiNumber == 3)) { //transition condition
            robot_state = KRUM; 
            PIcontroller.Stop(); //action
        } else if ((buttonA.getSingleDebouncedRelease()) && (romiNumber == 4)) { //transition condition
            robot_state = FLEUR; 
            PIcontroller.Stop(); //action 
        } else { //transition condition
            robot_state = IDLE; 
            PIcontroller.Stop(); //action  
    break;
    }
    
    case HARRY:{
        if (romiNumber == 1) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                PIcontroller.Stop();
            }
            else {
                //line follow
                //go straight two blocks (past one cross section)
                //turn right 
                // go straight to nect cross section 
                //turn right
                //go straight
                //turn left 
                //go straight to next cross section
                //turn left to buzzer 
                //hit buzzer 
                //do a 180 
                // go foward to next cross sections 
                //turn left
                //go to next cross section
                //turn left 
                // go to next cross section 
                //turn right 
                //see april tag 
                // reverse 10cm then sense krum turn 90 degres to the right and collide with krum then turn around find line and go back to line following
                //turn right
                //go to cross section
                //turn left
                // go through one cross section to the next one
                //turn left
                //go up ramp and stop like five cm away from april tag
                // see april tag and do a spin
            }      
        }
        break; 

    }
    case CEDRIC:{
        if (romiNumber == 2) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                PIcontroller.Stop();
            } else {
                // line follow to hit bum

                // drive  
    // miss first intersecti              on
                    // turn right at second intersection
                    // drive
                    // turn right at first intersection
                    // drive
                    // turn left at first intersection
                    // drive
                    // turn left at first intersection
                    // drive and hit bump switch
                    // reverse and turn
                    // miss first intersection
                    // turn left at second intersection
                    // drive
                    // turn left at first intersection
                    // drive
                    // turn right at first intersection
                    // drive 
                    // turn right at first intersection
                    // drive
                    // see APRIL tag at certain distance - stop
                        // wait for Harry to reach general location on map - start moving again
                    // turn left to reach l

//                   drive
                    // turn left at first intersection
                    // drive
                    // miss first intersection
                    // turn left at second intersection
                    // drive certain distance until seeing APRIL tag at end
                        // drive forward (forward kinematics)
                        // turn left (forward kinematics)
                        // drive off edge (forward kinematics)  en
                 //ezam fo dne  dnehcaer dna  ,hctiws 
                // if () { // find APRIL tag - tangled in vines
                //     PIcontroller.Stop(); // stuck in hedge
                //     if () { // if Harry reaches certain location, Cedric moves forward in the maze 
                //         // follows lines to reach end of maze
                //     }
                // }
                // if { // find APRIL tag - blast twi-wizard cup
                //     // turn to left and drive off side with forward kinematics
                // }
                // if { // Harry has started moving
                    
                // }
            }
        break; 
        }
    }
    case KRUM:{
        if (romiNumber == 3) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                PIcontroller.Stop();
            }
        }
        break; 
    }
    case FLEUR:{
        if (romiNumber == 4) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                PIcontroller.Stop();
            }
        }
        else {
            //FOLLOWLINE//
                //line follow using Speed Controller/PID
            //SEES FIRST DOUBLE WHITE// ignore
                //move foward a little 
                //line follow using Speed Controller/PID
            //SEES SECOND DOUBLE WHITE// right turn
                //move foward a little 
                //turn right a little 
                //turn right until LR hit the white line
                //line follow using Speed Controller/PID
            //SEES THIRD DOUBLE WHITE// right turn
                //move foward a little 
                //turn right a little 
                //turn right until LR hit the white line
                //line follow using Speed Controller/PID
            //SEES FOURTH DOUBLE WHITE//
                //move foward a little 
                //turn left a little 
                //turn lift until RR hit the white line
                //line follow using Speed Controller/PID
            // SEES FIFTHDOUBLE WHITE//
                //move foward a little 
                //turn left a little 
                //turn lift until RR hit the white line
                //line follow using Speed Controller/PID
            // HITS BUMP SWITCH//
                //back up a little 
                //turns to left a little 
                //turn left until RR hit the white line
            //SEES AN APRIL TAG//
                //try to read it 
            //APRIL TAG ID is __
                //line follow using Speed Controller/PID
            //APRIL TAG IS __ IN AWAY//
            

        }
        break; 
        
    }
    // case DRIVE: 
    // {
    //     if (buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp() == true) { //transition condition
    //         robot_state = IDLE;
    //         PIcontroller.Stop();
    //     }

    //     if (DetectCollision()) {
    //         robot_state = REVERSE;
    //         PIcontroller.Stop();
    //     }

    //     else {
    //         PIcontroller.Run(speed, speed);
    //     }

    // break;
    // }

    // case REVERSE: 
    // {   // back up for 10 centimeters
    //     PIcontroller.Reverse(50, 10);
    //     robot_state = TURN;
        
    //     //else if (backup condition == true) switch to turn state.
    // break;
    // }

    // case TURN: 
    // {
    //     PIcontroller.Turn(90, false);
    //     robot_state = DRIVE;

    // break;
    // }
    
    }
}
}