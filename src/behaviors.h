#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

// TODO: add a new variable which represents a collision threshold in the y-direction.
class Behaviors{
    private:
    // -150 thresh for 150mm/s works extremely well.
        int threshold = -110;    //-100 for 50mm/s, 
        int horThresh = 55;      // threshold value compared with abs(y acceleration) to account for non-normal collisions
        int threshold_pickup = 1500;
        int speed = 150;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, HARRY, CEDRIC, KRUM, FLEUR};    //DRIVE, REVERSE, TURN
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        ROBOT_STATE romiNumber = IDLE;
        int targetTagSize = 4400;  // adjust this value to be the area of the apriltag in pixels at the desired distance.
        int tagCount;
        const float Kp = 1.35; //Adapt the parameters until your robot moves at the speed you command it to drive //1.35
        const float Ki = 1.35; //0.1 //1
        const float Kpd = 0.035; //0.1 //0.05
        float E_left; 
        float E_right;
        
        // AprilTags to be identified in sequence
        const unsigned int harryTargetA = 0;
        const unsigned int harryTargetB = 1;
        const unsigned int cedricTargetA = 2;
        const unsigned int cedricTargetB = 3;
        const unsigned int krumTarget = 4;
        const unsigned int fleurTarget = 5;
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        void NoState(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
        boolean ComptagSize(void);
};

#endif