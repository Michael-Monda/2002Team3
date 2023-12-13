#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

// TODO: add a new variable which represents a collision threshold in the y-direction.
class Behaviors{
    private:
    // -150 thresh for 150mm/s works extremely well.
        int threshold = -110;    //-100 for 50mm/s, 
        int threshold_pickup = 1500;
        int speed = 150;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, HARRY, CEDRIC, KRUM, FLEUR};    //DRIVE, REVERSE, TURN
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        ROBOT_STATE romiNumber = IDLE;
        int targetTagSize = 4400;  // adjust this value to be the area of the apriltag in pixels at the desired distance.
        int tagCount;
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
        boolean ComptagSize(void);
};

#endif