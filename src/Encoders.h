#ifndef ENCODER
#define ENCODER

#include <Romi32U4.h>

class Encoder{
    private:
        const float revCount = 1440.0; //counts per wheel revolution
        const float wheelRadius = 35.0; //radius of wheel in [mm]
        const float wheelCircum = 2*PI*wheelRadius; //circumference of wheel
        const unsigned int interval = 50; // time in [ms], how often encoders are being updated
        
    public:
        void Init(void);
        int ReadEncoderCountLeft(void);
        int ReadEncoderCountRight(void);
        float ReadVelocityLeft(void); 
        float ReadVelocityRight(void); 
        void PrintVelocities(void);
        boolean UpdateEncoderCounts(void);
};

#endif