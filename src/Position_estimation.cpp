#include "Position_estimation.h"
#include "Encoders.h"
#include "math.h"

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    float R = (l/2) * ((target_speed_left + target_speed_right) / (target_speed_left - target_speed_right));
    float omega = (target_speed_right - target_speed_left) / l;
    float circumV = omega * R;

    time_now = millis();
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {
        if (target_speed_left == target_speed_right){
           x = x + (circumV * (cos(theta) * (time_now - time_prev))); 
           y = y + (circumV * (sin(theta) * (time_now - time_prev))); 
           theta = theta;
        }
        else{

            x = x - (R * (sin(theta))) + (R * (sin (theta + (omega * (time_now - time_prev)))));
            y = y - (R * (cos(theta))) - (R * (cos (theta + (omega * (time_now - time_prev)))));
            theta = theta + (omega * (time_now -time_prev));
        }
    }
}

