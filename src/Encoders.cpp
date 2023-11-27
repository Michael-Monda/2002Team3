#include <Romi32U4.h>
#include "Encoders.h"

int leftCount = 0;
int rightCount = 0;
int prevLeftCount = 0;
int prevRightCount = 0;
float previous_time = 0;
uint32_t lastUpdate = 0;

Romi32U4Encoders encoders;

void Encoder::Init(void)
{
    //nothing to initialize, however, good practice to have a init function anyway
}

void Encoder::PrintVelocities(void)
{
    Serial.print("Velocity of left wheel: ");
    Serial.print(ReadVelocityLeft());
    Serial.print('\t');
    Serial.print("Velocity of right wheel: ");
    Serial.println(ReadVelocityRight());
}   

int Encoder::ReadEncoderCountLeft(void)
{
  return leftCount;
}

int Encoder::ReadEncoderCountRight(void)
{
  return rightCount;
}

float Encoder::ReadVelocityLeft(void)
{
    float measurement = (wheelCircum/revCount)*(leftCount-prevLeftCount)/((float)interval/1000);
    return measurement;
}

float Encoder::ReadVelocityRight(void)
{
    float measurement = (wheelCircum/revCount)*(rightCount-prevRightCount)/((float)interval/1000);
    return measurement;
}

boolean Encoder::UpdateEncoderCounts(void)
{
  uint32_t now = millis();
  if((now - lastUpdate) >= interval)
  {    
    prevLeftCount = leftCount;
    prevRightCount = rightCount;
    leftCount = encoders.getCountsLeft();
    rightCount = encoders.getCountsRight();
    previous_time = millis();
    lastUpdate = now;
    return 1;
  }
  return 0;
}