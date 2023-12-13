#include <Romi32U4.h>
#include "PIDcontrollers.h"
#include "Wall_following_controller.h"
#include <openmv.h>

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
AprilTagDatum dakota;
OpenMV camera;

Romi32U4ButtonA buttonA;

// complext data-type median filter
Algorithm gyro_x_stab;  // this object used to find the median x acceleration of the romi over a given interval.
Algorithm gyro_y_stab;  // this object used to find the median y acceleration of the romi over a given interval.
Algorithm gyro_z_stab;  // this object used to find the median z acceleration of the romi over a given interval.

//motor-speed controller
PIDcontrollers DriveControl;
WallFollowingController FollowControl;
Romi32U4Motors drivetrain;

// define which number romi is which "character"

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

    switch (romiNumber) {
        case IDLE:                  // romi number == 0
            robot_state = IDLE;
        break;

        case HARRY:                 // romi number == 1
            robot_state = HARRY;    
        break;

        case CEDRIC:                // romi number == 2
            robot_state = CEDRIC;   
        break;

        case KRUM:                  // romi number == 3
            robot_state = KRUM;     
        break;

        case FLEUR:                 // romi number == 4
            robot_state = FLEUR;
        break;
    }
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

boolean Behaviors::ComptagSize(void) {
    int currSize = dakota.w * dakota.h;
    if (currSize == targetTagSize) {
        return true;
    } else return false;
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

    camera.readTag(dakota);                // ALWAYS read the tag in every condition.
    tagCount = camera.getTagCount();

    switch (robot_state)
    {
    case IDLE:
    {
        if(buttonA.getSingleDebouncedRelease()) {
            robot_state = romiNumber; 
            DriveControl.Stop(); //action
        } else { //transition condition
            robot_state = IDLE; 
            DriveControl.Stop(); //action 
        }   
    break;

    }

     case HARRY:{
        if (romiNumber == 1) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                DriveControl.Stop();
            } else if ((tagCount == 1) && (dakota.id == )) { // Replace with target tag ID for Harry
                DriveControl.Stop();
                // Insert code for accessing Krum location and returning to hit him
                if () {

                }
            } else {
                //line follow
                //go straight two blocks (past one cross section)
                //turn right
                // go straight to nect cross section
                //turn right
                //go straight
                //turn left
                //go straight to next cross section
                //turn left to buzzer
                // drive and hit bump switch
                DriveControl.Run(150, 150);
                // reverse and turn
                if (DetectCollision()) {
                    DriveControl.Reverse(110, 50);
                }
                DriveControl.Turn(30, 1);
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
                DriveControl.Stop();
            } else if ((tagCount == 1) && (dakota.id == )) { // Replace with target tag for Cedric
                DriveControl.Stop();
                // Insert code for accessing Harry location and waiting for him to continue
                if () {

                }
            } else {
                // line follow to hit bum

                // drive  
                // miss first intersection
                    // turn right at second intersection
                    // drive
                    // turn right at first intersection
                    // drive
                    // turn left at first intersection
                    // drive
                    // turn left at first intersection

                // drive and hit bump switch
                DriveControl.Run(150, 150);
                // reverse and turn
                if (DetectCollision()) {
                    DriveControl.Reverse(110, 50);
                }
                DriveControl.Turn(30, 1);
                while(analogRead(leftReflectance) <= 600) { // Aditri subject to change
                    DriveControl.Turn(200, 1);
                }
                // Aditri add code for driving with sensors

                    // miss first intersection
                    // turn left at second intersection
                    // drive
                    // turn left at first intersection
                    // drive
                    // turn right at first intersection
                    // drive
                    // turn right at first intersection
                    // drive
                DriveControl.FollowAtDistance();    // this will adjust the romi position until target = curr size
                if (ComptagSize()) {                // see APRIL tag at certain distance - stop
                    Stop();
                }

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

    // TODO: I think the entrance conditions into these cases need to be revised
    // as it stands right now, the machine will check for the "KRUM" state
    // as configured in the switch statement in the Init() function. It then
    // checks AGAIN if the romiNumber is 3, which makes no sense because the
    // robotState KRUM will only occure if and only if the romiNumber == KRUM (3).
    // Addtitionally, the FLEUR state else case is placed incorrect
    case KRUM:{
        if (romiNumber == 3) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                DriveControl.Stop();
            } else if ((tagCount == 1) && (dakota.id == )) { // Replace with target tag for Krum
                DriveControl.Stop();
                // Insert code for accessing Fleur location and returning to hit her
                if () {

                }
            } else {
                // drive and hit bump switch
                DriveControl.Run(150, 150);
                // reverse and turn
                if (DetectCollision()) {
                    DriveControl.Reverse(110, 50);
                }
                while(analogRead(leftReflectance) <= 600) { // Aditri subject to change
                    DriveControl.Turn(200, 1);
                }
            }
        }
        break;
    }
    case FLEUR:{
        if (romiNumber == 4) {
            if (buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE;
                DriveControl.Stop();
            } else if ((tagCount == 1) && (dakota.id == )) { // Replace with target tag for Fleur
                DriveControl.Stop();
                // Insert code for accessing Krum location and returning to hit him
                if () {

                }
            } else {
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
        }
            // drive and hit bump switch
            DriveControl.Run(150, 150);
            // reverse and turn
            if (DetectCollision()) {
                DriveControl.Reverse(110, 50);
            }
            DriveControl.Turn(30, 1);
            while(analogRead(leftReflectance) <= 600) { // Aditri subject to change
                DriveControl.Turn(200, 1);
            }
        break;
    }
    
    }
}
// HAVE TO EDIT TO OUR TAGS THIS IS FROM LAB 5 
// void Behaviors::NoState(void)
// {
//     motors1.setEfforts(0, 0);
//     Serial.println("Searching");
//     uint8_t tagCount = camera1.getTagCount();
//     if (tagCount)
//     {
//         Serial.println("there is a tag");
//         AprilTagDatum tag;
//         if (camera1.readTag(tag))
//         {
//             Serial.println("I read the data");
//             if (tag.id == 4)
//             { // if a tag is seen and id is 4 (FindAprilTags() == 1)
//                 Serial.println("drive tag");
//                 Serial.println(tag.w);
//                 Serial.println(tag.h);
//                 if (MagneticEncoder1.UpdateEncoderCounts())
//                 {
//                     float e_left = (tag.cx - 80.0f);
//                     float e_right = (80.0f - tag.cx);
//                     float e_Area = 4000.0f - (tag.h * tag.w); //2500 //5000
//                     // float e_dist = targetDistance - hornet.FilterData(false);

//                     E_left += 0;//e_left;
//                     E_right += 0;//e_right;

//                     float v_left = Kpd * e_Area + Kp * e_left + Ki * E_left;
//                     float v_right = Kpd * e_Area + Kp * e_right + Ki * E_right;
//                     motors1.setEfforts(v_left, v_right);
//                 }
//                 // break;
//             }
//             else
//             { // if a tag is seen (FindAprilTags() == 2)
//                 Serial.println("wrong tag");
//                 digitalWrite(PIN_A3, LOW);
//                 delay(200);
//                 digitalWrite(PIN_A3, HIGH);
//                 delay(200);
//                 digitalWrite(PIN_A3, LOW);
//                 delay(200);
//                 // break;
//             }
//         }
//     }
// }
// sensors

void Behaviors::NoState(void)
{
    motors1.setEfforts(0, 0);
    Serial.println("Searching");
    uint8_t tagCount = camera1.getTagCount();
    if (tagCount)
    {
        Serial.println("A task is coming up");
        AprilTagDatum tag;
        if (camera1.readTag(tag))
        {
            Serial.println("I see a skrewt");
            if (tag.id == 1) // this is what we can use for harry and cedrics first tags
            { 
                Serial.println(" I am comming up on the challenge");
                Serial.println(tag.w);
                Serial.println(tag.h);
                if (MagneticEncoder1.UpdateEncoderCounts())
                {
                    float e_left = (tag.cx - 80.0f);
                    float e_right = (80.0f - tag.cx);
                    float e_Area = 4000.0f - (tag.h * tag.w); //this is to fix how far away the robot is from the tag (4000.0f)
                    float v_left = Kpd * e_Area + Kp * e_left + Ki * E_left;
                    float v_right = Kpd * e_Area + Kp * e_right + Ki * E_right;
                    motors1.setEfforts(v_left, v_right);
                    Serial.println("The skrewt has been defeated");
                    break; 

                }
                //break;
            }
            else if (tag.id == 2) // This should be the one that fleur sees to edject her from the game
            //we can also do this for krum to make him bad -- all that we would have to change is the print statments to make him "go bad"
            { // if a tag is seen (FindAprilTags() == 2)
                Serial.println(" I am comming up on the challenge");
                Serial.println(tag.w);
                Serial.println(tag.h);
                if (MagneticEncoder1.UpdateEncoderCounts())
                {
                    float e_left = (tag.cx - 80.0f);
                    float e_right = (80.0f - tag.cx);
                    float e_Area = 4000.0f - (tag.h * tag.w); //this is to fix how far away the robot is from the tag (4000.0f)
                    float v_left = Kpd * e_Area + Kp * e_left + Ki * E_left;
                    float v_right = Kpd * e_Area + Kp * e_right + Ki * E_right;
                    motors1.setEfforts(v_left, v_right);
                digitalWrite(PIN_A3, LOW);
                delay(200);
                digitalWrite(PIN_A3, HIGH);
                delay(200);
                digitalWrite(PIN_A3, LOW);
                delay(200);
                Serial.println("Bye Bye");
                // break;
            }
            else if (tag.id == 3) // This should be the one that can be used for the cup
            { 
                Serial.println("I see the cup coming up");
                Serial.println(tag.w);
                Serial.println(tag.h);
                if (MagneticEncoder1.UpdateEncoderCounts())
                {
                    float e_left = (tag.cx - 80.0f);
                    float e_right = (80.0f - tag.cx);
                    float e_Area = 4000.0f - (tag.h * tag.w); //this is to fix how far away the robot is from the tag (4000.0f)
                    float v_left = Kpd * e_Area + Kp * e_left + Ki * E_left;
                    float v_right = Kpd * e_Area + Kp * e_right + Ki * E_right;
                    motors1.setEfforts(v_left, v_right);
                //ADD THE 360 SPIN 
                Serial.println("Whooohooo");
                // break;
            }
        }
            else if (tag.id == 4) // This should be the one that can be used for the cup
            { 
                Serial.println("I see the cup coming up yay... wait a minute");
                Serial.println(tag.w);
                Serial.println(tag.h);
                if (MagneticEncoder1.UpdateEncoderCounts())
                {
                    float e_left = (tag.cx - 80.0f);
                    float e_right = (80.0f - tag.cx);
                    float e_Area = 4000.0f - (tag.h * tag.w); //this is to fix how far away the robot is from the tag (4000.0f)
                    float v_left = Kpd * e_Area + Kp * e_left + Ki * E_left;
                    float v_right = Kpd * e_Area + Kp * e_right + Ki * E_right;
                    motors1.setEfforts(v_left, v_right);
                    motor.setEfforts(50,-50,10);
                    motor.setEfforts(75,75); 
                Serial.println("AHHHHHHHHHH");
                // break;
            }
        }
    }
}}}