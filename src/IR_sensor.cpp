#include <Romi32U4.h>
#include "IR_sensor.h"
#include <algorithms.h>
using namespace std;

Algorithm sortRadar;
void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

void IRsensor::PrintData(bool testing)
{
    Serial.print(FilterData(testing));
    if (testing == false) Serial.println(" cm");
    else Serial.println("");
}

float IRsensor::ReadData(bool adcValue) {
    // assignment 1.1
    // read out and calibrate your IR sensor, to convert readouts to distance in [cm]
    // I think this is the function which actually GRABS data (like rangefinder.getDistance)
    // therefore, this should be a really simple "analogRead" function, and the median should
    // be obtained in main() by passing this function into an array.
    if (adcValue == false) {
        float adc = analogRead(pin_IR); // now, convert to cm from ADC value.
        float rangeCM = (1 / ((((adc * 5) / 1023) - 0.214) / 18.9));
        float errata = 0.6; //cm
        return (rangeCM + errata);
    } else {
        float adc = analogRead(pin_IR);
        return adc;
    }
    

}

float IRsensor::FilterData(bool calibrating) {
    // TODO: install filter for sensor data to return the median??

    float medianDeterminant[5];
    float instanceMedianDist;
    int i;
    for (i = 0; i <= 4; i++) {
        medianDeterminant[i] = ReadData(calibrating);   // need ot figure out how to get the reading here.
    }
    sortRadar.SortAscending(medianDeterminant, 5);                            // i think these can only be called in main()...
    instanceMedianDist = sortRadar.FetchMedian(medianDeterminant, 5, false);  // i think these can only be called in main()..     .

    return instanceMedianDist;
}