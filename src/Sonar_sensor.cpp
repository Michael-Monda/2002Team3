#include <Romi32U4.h>
#include "Sonar_sensor.h"
#include <algorithms.h>

Algorithm sortSonar;
void SonarSensor::Init(void)
{
    pinMode(pin_TRIG,OUTPUT);
    pinMode(pin_ECHO, INPUT);   
}

void SonarSensor::PrintData(bool printEchoPeriod)
{
    Serial.print(FilterData(printEchoPeriod));
    if (printEchoPeriod == false) Serial.println(" cm");
    else Serial.println("");
}

float SonarSensor::ReadData(bool returnEchoPeriod)   // returns echo time in microseconds
{
    //assignment 1.2
    //read out and calibrate your sonar sensor, to convert readouts to distance in [cm]
    digitalWrite(pin_TRIG, HIGH);
    delay(10);
    digitalWrite(pin_TRIG, LOW);
    if (returnEchoPeriod == true) {
        float us = (pulseIn(pin_ECHO, HIGH));
        return us;
    } else {
        float us = (pulseIn(pin_ECHO, HIGH));    // find function to aproximate distance in cm from echo meriod (μs).
        float errata = 0.7;
        float rangeCM = ((us - 78.6) / 55.8);
        return (rangeCM + errata);
    }
    
}

float SonarSensor::FilterData(bool filterEchoPeriod) {
    float medianDeterminant[5];
    float instanceMedianDist;
    int i;

    for (i = 0; i <= 4; i++) {
        medianDeterminant[i] = ReadData(filterEchoPeriod);   // need ot figure out how to get the reading here.
    }
    sortSonar.SortAscending(medianDeterminant, 5);                            // i think these can only be called in main()...
    instanceMedianDist = sortSonar.FetchMedian(medianDeterminant, 5, false);  // i think these can only be called in main()...

  return instanceMedianDist;
}