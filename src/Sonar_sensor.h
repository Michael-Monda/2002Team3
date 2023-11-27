#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Romi32U4.h>
#include <algorithms.h>

class SonarSensor{
    private:
        const int pin_TRIG = 0;
        const int pin_ECHO = 1;
    public:
        void Init(void); 
        float ReadData(bool); 
        void PrintData(bool);
        float FilterData(bool);
};

#endif