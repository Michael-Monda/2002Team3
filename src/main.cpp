#include <Arduino.h>
#include <Romi32U4.h>
#include "algorithms.h" // this does not make sense
//sensors
#include "IR_sensor.h"
#include "Sonar_sensor.h"
#include "Encoders.h"
//behaviors
#include "Speed_controller.h"
#include "Wall_following_controller.h"

enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVING};
ROBOT_STATE robot_state = ROBOT_IDLE;

Romi32U4ButtonA buttonA; 
SpeedController PIcontroller;
WallFollowingController PDcontroller;
IRsensor ranger;
SonarSensor hornet;

int j;

void setup() {
  PIcontroller.Init();
  PDcontroller.Init();
}

void loop() {
    switch(robot_state)
    {
        case ROBOT_IDLE:
            if(buttonA.getSingleDebouncedRelease()) robot_state = ROBOT_DRIVING;
        break;

    case ROBOT_DRIVING:

        // PIcontroller.Run(50, 50);
        // int speed = PDcontroller.Run(30); //target distance from wall in [cm]
        // PIcontroller.Run(50 + speed, 50 - speed); //speed in [mm/s]

        //sensor debug code
        Serial.println("");
        for (j = 1; j < 26; j++) {
            hornet.PrintData(true);
            delay(50);
        }
        
        if (buttonA.getSingleDebouncedRelease() || j >= 25) 
        {
            PIcontroller.Stop();
            robot_state = ROBOT_IDLE;
        }
        break;
  }
}