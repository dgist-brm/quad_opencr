/*******************************************************************************
* Copyright 2019 Bongsub Song
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Bongsub Song */

#ifndef QUADNAKE_MOTOR_DRIVER_H_
#define QUADNAKE_MOTOR_DRIVER_H_

#include "variant.h"
#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
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

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor

#define BAUDRATE                        57600 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define TORQUE_CONTANT_VALUE_SMALL      3.5 //mNm per current unit on 12V
#define TORQUE_CONTANT_VALUE_BIG        4.5 //mNm per current unit on 12V

#define DEBUG_SERIAL  SerialBT2

class QuadnakeMotorDriver
{
 public:
  QuadnakeMotorDriver();
  ~QuadnakeMotorDriver();
  bool init();
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool writeVelocity(int64_t left_value, int64_t right_value);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  bool torque_;

  uint16_t dynamixel_limit_max_velocity_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};

#endif
