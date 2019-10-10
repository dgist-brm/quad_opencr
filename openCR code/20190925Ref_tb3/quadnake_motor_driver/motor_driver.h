/* Authors: Bongsub Song */

#ifndef QUADNAKE_MOTOR_DRIVER_H_
#define QUADNAKE_MOTOR_DRIVER_H_

#include "variant.h"
#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
//ROM PART
#define ADDR_X_LIMIT_VELOCITY           44

//RAM PART
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_CURRENT             102
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_CURRENT          126
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210 - Small and XM540-W150 - Big)
#define BIG_MOTOR_LIMIT_MAX_TOQUE       265
#define BIG_MOTOR_LIMIT_MAX_VELOCITY    265
#define BIG_MOTOR_LIMIT_MAX_ANGLE
#define BIG_MOTOR_LIMIT_MIN_ANGLE
#define SMALL_MOTOR_LIMIT_MAX_TOQUE       265
#define SMALL_MOTOR_LIMIT_MAX_VELOCITY    265
#define SMALL_MOTOR_LIMIT_MAX_ANGLE
#define SMALL_MOTOR_LIMIT_MIN_ANGLE

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_CURRENT              2
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define BAUDRATE                        57600 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define TORQUE_CONTANT_VALUE_SMALL      3.5 //mNm per current unit on 12V
#define TORQUE_CONTANT_VALUE_BIG        4.5 //mNm per current unit on 12V

#define DEBUG_SERIAL  SerialBT2

class motorDriver
{
    public:
    motorDriver();
    ~motorDriver();

    bool init(); //모터 초기속도, 동작범위, 통신 초기화등 초기 설정
 
    bool setTorque(uint8_t id, bool onoff);
    //bool setMaxTorque(uint8_t id, bool onoff);
    //bool setVel(uint8_t id, bool onoff);
    
    void close();

    private:

    uint32_t baudrate_;
    float protocol_ver_;
    uint8_t motorids[4][8];

    dynamixel::PortHandler *poh_;
    dynamixel::PacketHandler *pah_;

    void limitVelocity(uint8_t id, uint32_t vel);   //unit 430 = 0.229rpm, uinit 530 = 
};

#endif