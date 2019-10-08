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

#include "quadnake_motor_driver.h"

QuadnakeMotorDriver::QuadnakeMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

QuadnakeMotorDriver::~QuadnakeMotorDriver()
{
  close();
}

bool QuadnakeMotorDriver::init()
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  //dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

bool QuadnakeMotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, 0xFE, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
 
  return onoff;
}

bool QuadnakeMotorDriver::getTorque()
{
  return torque_;
}

void QuadnakeMotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

// bool QuadnakeMotorDriver::readEncoder(int8_t id, int32_t &angle_value)
// {
//   int dxl_comm_result = COMM_TX_FAIL;              // Communication result
//   bool dxl_addparam_result = false;                // addParam result
//   bool dxl_getdata_result = false;                 // GetParam result

//   // Set parameter
//   dxl_addparam_result = groupSyncReadEncoder_->addParam(id);
//   if (dxl_addparam_result != true)
//     return false;

//   // Syncread present position
//   dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//     Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

//   // Check if groupSyncRead data of Dynamixels are available
//   dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   if (dxl_getdata_result != true)
//     return false;

//   // Get data
//   angle_value  = groupSyncReadEncoder_->getData(id,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

//   groupSyncReadEncoder_->clearParam();
//   return true;
// }

// bool QuadnakeMotorDriver::writeVelocity(uint8_t id, int64_t velocity)
// {
//   bool dxl_addparam_result;
//   int8_t dxl_comm_result;

//   uint8_t left_data_byte[4] = {0, };
//   uint8_t right_data_byte[4] = {0, };


//   left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(velocity));
//   left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(velocity));
//   left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(velocity));
//   left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(velocity));

//   dxl_comm_result = groupSyncWriteVelocity_->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
//     return false;
//   }

//   groupSyncWriteVelocity_->clearParam();
//   return true;
// }
bool QuadnakeMotorDriver::writePosition(int8_t id, int64_t position)
{
  int8_t dxl_comm_result;
  int8_t dxl_error = 0;

  uint8_t data_byte[4] = {0, };

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(position));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(position));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(position));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(position));

  dxl_comm_result = packetHandler_->write4TxRx(portHandler_,id,ADDR_X_GOAL_POSITION,(uint8_t*)&data_byte,dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
    return false;

  return true;
}

bool readPosition(uint8_t id, int32_t &position)
{
  int8_t dxl_comm_result;
  int8_t error;

  dxl_comm_result = packetHandler_->read4TxRx(portHandler_,id,ADDR_X_PRESENT_POSITION,position,error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }
  return true;
}
// bool QuadnakeMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
// {
//   bool dxl_comm_result = false;
  
//   float wheel_velocity_cmd[2];

//   float lin_vel = value[LEFT];
//   float ang_vel = value[RIGHT];

//   wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
//   wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

//   wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
//   wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

//   dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
//   if (dxl_comm_result == false)
//     return false;

//   return true;
// }
