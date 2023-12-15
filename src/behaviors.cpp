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
#include "Encoders.h"

#include "openmv.h"
#include "apriltagdatum.h"


//sensors
IMU_sensor leyte;       // I like to give them names because I think it's easier to associate them with their functions.
IRsensor ranger;
SonarSensor hornet;
AprilTagDatum dakota;
OpenMV camera;
int leftReflectance = 20;   
int rightReflectance = 22;  

Romi32U4ButtonA buttonA;

// complext data-type median filter
Algorithm gyro_x_stab;  // this object used to find the median x acceleration of the romi over a given interval.
Algorithm gyro_y_stab;  // this object used to find the median y acceleration of the romi over a given interval.
Algorithm gyro_z_stab;  // this object used to find the median z acceleration of the romi over a given interval.

//motor-speed controller
PIDcontrollers DriveControl;
WallFollowingController FollowControl;
Romi32U4Motors drivetrain;
Encoder wheelEncoders;

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

    // TODO: determine which romis need to react to which tags.
    switch (romiNumber) {
        case IDLE:                  // romi number == 0
            robot_state = IDLE;
        break;

        case HARRY:                 // romi number == 1
            robot_state = HARRY;    // HARRY carries dakota ID 6 on its back   
        break;                      // HARRY needs to recognize tags x, y

        case CEDRIC:                // romi number == 2
            robot_state = CEDRIC;   // CEDRIC carries dakota ID 7 on its back
        break;                      // CEDRIC needs to recognize tags x, y

        case KRUM:                  // romi number == 3
            robot_state = KRUM;     // KRUM carries dakota ID 8 on its back
        break;                      // KRUM needs to recognize tags x, y

        case FLEUR:                 // romi number == 4
            robot_state = FLEUR;    // FLUER carries dakota ID 9 on its back
        break;                      // FLEUR needs to recognize tags x, y
    }
}

int Behaviors::getLeftValue(void) {
    int leftSensVal = analogRead(5);
    return leftSensVal;
}

int Behaviors::getRightValue(void) {
    int rightSensVal = analogRead(12);
    return rightSensVal;
}

void Behaviors::handleIntersection() {
    int timer = millis();
    if (millis() <= timer + driveInterval) {
        DriveControl.Run(40, 40);
        machine = true;
    } else if (machine) {
        machine = false;
        DriveControl.Turn(90, false);
    }
    
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = leyte.ReadAcceleration();
    data[0] = gyro_x_stab.ComplexTypeFilter(data_acc.X)*0.061;
    data[1] = gyro_y_stab.ComplexTypeFilter(data_acc.Y)*0.061;
    data[2] = gyro_z_stab.ComplexTypeFilter(data_acc.Z)*0.061;

    if(((data[0]) < threshold) || (abs(data[1])) > horThresh) return 1;
    // horizontal threshold accounts for non-normal collision incidence angles
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

void Behaviors::Run(void) {
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
            switch (harryState) {
            
            case IDLEH:
                if (buttonA.getSingleDebouncedRelease()) {
                    harryState = prevHarryState;
                } else {
                    harryState = IDLEH;
                    DriveControl.Stop();
                }
            break;
            
            case LINEFOLLOW:
                if (buttonA.getSingleDebouncedRelease()) {
                    prevHarryState = harryState;
                    harryState = IDLEH;
                }
                // separate "if" statement for syntax purposes.
                DriveControl.LineFollow();
                
                //condition 1: harry identifities the correct tag.
                if (camera.getTagCount() != 0 && (dakota.id == harryTargetA || dakota.id == cedricTargetA)) {
                    harryState = SEEAPRILTAG;
                    Serial.println("I see a skrewt");
                    Serial.println("I must be coming up on the challenge :)");  
                } else if (getLeftValue() >= thresh && getRightValue() >= thresh) {
                    harryState = HANDLEINT;
                }
            break;

            case SEEAPRILTAG:
                DriveControl.FollowAtDistance();
                if (!wheelEncoders.UpdateEncoderCounts() && (dakota.id == harryTargetA || dakota.id == cedricTargetA)) {
                    Serial.print("The challenge is complete. The skrewt is defeated");
                    harryState = TURN;
                } else if (!wheelEncoders.UpdateEncoderCounts() && (dakota.id == harryTargetB || dakota.id == cedricTargetB)) {
                    Serial.println("I see the cup coming up");
                    delay (200);
                    Serial.println("Whooohooo"); // Harry wins
                }
            break;

            case HANDLEINT:
                handleIntersection();
                harryState = LINEFOLLOW;
            break;

            case TURN:
                DriveControl.Turn(30, 1);
                //DriveControl.TurnTillLine();
                harryState = LINEFOLLOW;
            break;

            default:    // default state is linefollowing
                harryState = LINEFOLLOW;
            break;
            }
        break;
        }




        //     if (romiNumber == 1) { // If the Romi used is harry
        //     Serial.println("I see a task is coming up");

        //         if (buttonA.getSingleDebouncedRelease()) {
        //             robot_state = IDLE;
        //             DriveControl.Stop(); // When the camera doesn't view any tags of type 16h5;
        //         }
        //         if (camera.getTagCount() != 0 && (dakota.id == harryTargetA || dakota.id == cedricTargetA))
        //         {
                                         
                       
        //             if (wheelEncoders.UpdateEncoderCounts()) {
        //                 DriveControl.FollowAtDistance(); // Align themselves at a set distance from the APRIL tag
                        
        //             } else if (dakota.id == harryTargetB) { // The second APRIL tag Harry sees - the cup at the end
                        
        //                 // Serial.println(dakota.w);
        //                 // Serial.println(dakota.h);
                        
        //                 if (wheelEncoders.UpdateEncoderCounts()) { // Drive directly over the platform, a set distance from the APRIL tag
        //                     DriveControl.FollowAtDistance();
        //                 }
        //                 //ADD THE 360 SPIN 
                    
        //             }
                
        //         } else {
        //             DriveControl.Run(150, 150);
        //                 // reverse and turn
        //             if (DetectCollision()) {
        //                 DriveControl.Reverse(110, 50);
        //                 DriveControl.Turn(30, 1);
        //             }
        //         }
        //     }
        // break;
        // }
//_______________________________________________________________________________________ ignore below :(2-*
        case CEDRIC:{
            if (romiNumber == 2) {
                if (buttonA.getSingleDebouncedRelease()){
                    robot_state = IDLE;
                    DriveControl.Stop();
                } if (tagCount != 0) { // When the camera doesn't view any tags of type 16h5
                    Serial.println("I see a task is coming up");

                    if (tagCount != 0) {
                        Serial.println("I see a skrewt");

                        if (dakota.id == harryTargetA || dakota.id == cedricTargetA) // this is what we can use for harry and cedrics first tags
                        { 
                            Serial.println(" I am comming up on the challenge");
                            Serial.println(dakota.w);
                            Serial.println(dakota.h);

                            if (wheelEncoders.UpdateEncoderCounts()) {
                                DriveControl.FollowAtDistance();
                            }
                        } else if (dakota.id == cedricTargetB) {// This should be the one that can be used for the cup
                            Serial.println("I see the cup coming up yay... wait a minute");
                            Serial.println(dakota.w);
                            Serial.println(dakota.h);
                            if (wheelEncoders.UpdateEncoderCounts())  {
                                DriveControl.FollowAtDistance();
                            } else if (!wheelEncoders.UpdateEncoderCounts()) {
                            drivetrain.setEfforts(50,-50);
                            drivetrain.setEfforts(75,75); 
                            Serial.println("AHHHHHHHHHH");
                            }
                        }
                    }
                } else {
                    DriveControl.Run(150, 150);
                    // reverse and turn
                    if (DetectCollision()) {
                        DriveControl.Reverse(110, 50);
                        DriveControl.Turn(30, 1);
                    }
                    DriveControl.FollowAtDistance();    // this will adjust the romi position until target = curr size
                    if (ComptagSize()) {                // see APRIL tag at certain distance - stop
                        Stop();
                    }
                }
            }
        break;
        }
    

        // TODO: I think the entrance conditions into these cases need to be revised
        // as it stands right now, the machine will check for the "KRUM" state
        // as configured in the switch statement in the Init() function. It then
        // checks AGAIN if the romiNumber is 3, which makes no sense because the
        // robotState KRUM will only occure if and only if the romiNumber == KRUM (3).
        // Addtitionally, the FLEUR state else case is placed incorrect
        case KRUM:{
            if (romiNumber == 3) { // If the Romi used is Krum
                if (buttonA.getSingleDebouncedRelease()){
                    robot_state = IDLE;
                    DriveControl.Stop();
                } if (tagCount != 0) { // When the camera doesn't view any tags of type 16h5
                    Serial.println("I see a task is coming up");
                    if (tagCount != 0) {
                        Serial.println("I see a skrewt");
                    } if (dakota.id == fleurTarget || dakota.id == krumTarget) {    // This should be the one that fleur sees to edject her from the game
                        Serial.println(" I am coming up on the challenge");
                        Serial.println(dakota.w);
                        Serial.println(dakota.h);
                        if (wheelEncoders.UpdateEncoderCounts()) {
                            while (true) {
                                digitalWrite(PIN_A3, LOW);
                                delay(200);
                                digitalWrite(PIN_A3, HIGH);
                                delay(200);
                                digitalWrite(PIN_A3, LOW);
                                delay(200);
                                Serial.println("Bye Bye");
                            }
                            // Fleur moment
                        }
                    }
                }
            } // reverse and turn
            if (DetectCollision()) {
                DriveControl.Reverse(110, 50);
            }

        break;
        }

        case FLEUR:
        {
            if (romiNumber == 4) {
                if (buttonA.getSingleDebouncedRelease()){
                    robot_state = IDLE;
                    DriveControl.Stop();
                 } if (tagCount != 0) { // When the camera doesn't view any tags of type 16h5
                    Serial.println("I see a task is coming up");

                    if (camera.readTag(dakota)) {
                        Serial.println("I see a skrewt");

                    } if (dakota.id == fleurTarget || dakota.id == krumTarget) {    // This should be the one that fleur sees to edject her from the game
                        Serial.println(" I am comming up on the challenge");
                        Serial.println(dakota.w);
                        Serial.println(dakota.h);

                        if (wheelEncoders.UpdateEncoderCounts()) {
                            while (true) {
                                digitalWrite(PIN_A3, LOW);
                                delay(200);
                                digitalWrite(PIN_A3, HIGH);
                                delay(200);
                                digitalWrite(PIN_A3, LOW);
                                delay(200);
                                Serial.println("Bye Bye");
                            }
                        // Krum moment
                        }
                    }
                }
            } else {
                DriveControl.LineFollow(); 

                if (DetectCollision()) {
                    DriveControl.Reverse(110, 50);
                    DriveControl.Turn(30, 1);
                }
            }
        break;
        }
    }
}

//sam code, to be inserted into the state machine
void Behaviors::NoState(void) {
    drivetrain.setEfforts(0, 0);
    Serial.println("Searching");
    uint8_t tagCount = camera.getTagCount();

    if (tagCount != 0) {
        Serial.println("I see a task is coming up");

        if (camera.readTag(dakota))
        {
            Serial.println("I see a skrewt");

            if (dakota.id == harryTargetA || dakota.id == cedricTargetA) // this is what we can use for harry and cedrics first tags
            { 
                Serial.println(" I am comming up on the challenge");
                Serial.println(dakota.w);
                Serial.println(dakota.h);

                if (wheelEncoders.UpdateEncoderCounts()) {
                    DriveControl.FollowAtDistance();
                }
                //break;
            } else if (dakota.id == fleurTarget || dakota.id == krumTarget) {    // This should be the one that fleur sees to edject her from the game

                Serial.println(" I am comming up on the challenge");
                Serial.println(dakota.w);
                Serial.println(dakota.h);

                if (wheelEncoders.UpdateEncoderCounts()) {
                    while (true) {
                        digitalWrite(PIN_A3, LOW);
                        delay(200);
                        digitalWrite(PIN_A3, HIGH);
                        delay(200);
                        digitalWrite(PIN_A3, LOW);
                        delay(200);
                        Serial.println("Bye Bye");
                    }
                    // break;
                } else if (dakota.id == harryTargetB) {// This should be the one that can be used for the cup 
                    Serial.println("I see the cup coming up");
                    Serial.println(dakota.w);
                    Serial.println(dakota.h);
                    if (wheelEncoders.UpdateEncoderCounts()) {
                        DriveControl.FollowAtDistance();
                    }
                    //ADD THE 360 SPIN 
                    Serial.println("Whooohooo");
                    // break;
                }
            }
            else if (dakota.id == cedricTargetB) {// This should be the one that can be used for the cup
                Serial.println("I see the cup coming up yay... wait a minute");
                Serial.println(dakota.w);
                Serial.println(dakota.h);
                if (wheelEncoders.UpdateEncoderCounts())  {
                    DriveControl.FollowAtDistance();
                // break;
                } else if (!wheelEncoders.UpdateEncoderCounts()) {
                    drivetrain.setEfforts(50,-50);
                    drivetrain.setEfforts(75,75); 
                    Serial.println("AHHHHHHHHHH");
                }
            }
        }
    }
}