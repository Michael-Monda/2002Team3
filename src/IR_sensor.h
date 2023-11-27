#ifndef IR_SENSOR
#define IR_SENSOR

#include <Romi32U4.h>
#include <algorithms.h>

class IRsensor{
    private:
        const int pin_IR = A0;
    public:
        void Init(void);
        float ReadData(bool);
        void PrintData(bool);
        float FilterData(bool);
};

#endif