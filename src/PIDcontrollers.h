#ifndef PIDCONTROLLERS
#define PIDCONTROLLERS

#include <Romi32U4.h>

class PIDcontrollers{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1;
        const float Kpd = 10.7;
        // LINE FOLLOWING STUFF 
        // reflectance sensor port numbers
        const int leftSensor = 22;  
        const int rightSensor = 20;
        int leftSpeed;
        int rightSpeed;

        float E_left; 
        float E_right;
        // float E_dist = 0;
        int counts;
        uint16_t median = 80;
        // TODO: adjust target area, for identification at a specific distance.
        uint16_t targetArea = 1089; //33 pixels by 33 pixels

    public:
        const int leftReflectanceThresh;
        const int rightReflectanceThresh;
        int leftReflectance;
        int rightReflectance;

        void Init(void);
        void Run(float, float);
        void FollowAtDistance();
        void FollowLine(int);
        void Stop(void);
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean Reverse(int, int);
};

#endif