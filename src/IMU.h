#ifndef IMU
#define IMU

#include <Romi32U4.h>

class IMU_sensor{
    private:
        int data[3] = {0};
        char report[120];
        
    public:
        struct acceleration_data {
            int X;
            int Y;
            int Z;
        };
        void Init(void);
        void PrintAcceleration(void);
        acceleration_data ReadAcceleration(void);
};

#endif

// void LSM6::enableDefault(void)
// {
//   if (_device == device_DS33)
//   {
//     // Accelerometer

//     // 0x80 = 0b10000000
//     // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
//     //writeReg(CTRL1_XL, 0x80);
//     setFullScaleAcc(ACC_FS2);
//     setAccDataOutputRate(ODR13);

//     // Gyro

//     // 0x80 = 0b010000000
//     // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
//     //writeReg(CTRL2_G, 0x80);
//     setFullScaleGyro(GYRO_FS245);
//     setGyroDataOutputRate(ODR13);
//     // Common

//     // 0x04 = 0b00000100
//     // IF_INC = 1 (automatically increment register address)
//     writeReg(CTRL3_C, 0x04);
//   }
// }