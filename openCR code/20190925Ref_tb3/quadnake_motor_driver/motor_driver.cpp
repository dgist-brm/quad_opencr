/* Authors: Bongsub Song */

#include "motor_driver.h"

motorDriver::motorDriver()
	:baudrate_(BAUDRATE),
	protocol_ver_(PROTOCOL_VERSION)
{
	bool initError = false; //True is no error

	initError = init();

	if(!initError)
	{
		DEBUG_SERIAL.println("Failed to init motor during creating [motorDriver] class instance");
		close();
	}
}

motorDriver::~motorDriver()
{
	close();
}

bool motorDriver::init()
{
	DEBUG_SERIAL.begin(57600);
	poh_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	pah_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	bool initError = false;
	//before torque on
	//set velocity limit and angle limit and so on.

	//after torque on
	initError = setTorque(0xFE,true);
	if(!initError)
	{
		DEBUG_SERIAL.println("Failed to set motor torque on during running [init] function.");
		close();
	}
}

bool motorDriver::setTorque(uint8_t id, bool onoff)
{
	uint8_t dxl_error;
	pah_->write1ByteTxRx(poh_,id,ADDR_X_TORQUE_ENABLE,onoff,&dxl_error);
	if(dxl_error != COMM_SUCCESS)
	{
		DEBUG_SERIAL.println("Failed to write command during running [setTorque] function.");
		return false;
	}
	return true;
}

bool motorDriver::movetoPosition(uint8_t id, int32_t position)
{
	uint8_t dxl_error;
	pah_->write4ByteTxRx(poh_,id,ADDR_X_GOAL_POSITION,position,&dxl_error);

	if(dxl_error != COMM_SUCCESS)
	{
		DEBUG_SERIAL.println("failed to write command during running [movetoPosition] function.");
		return false;
	}
	return true;
}

void motorDriver::close()
{
	poh_->closePort();
}

void motorDriver::limitVelocity(uint8_t id, uint32_t vel)
{
	uint8_t dxl_error;

	if(id != 0xFE)
	{
		pah_->write4ByteTxRx(poh_,id,ADDR_X_LIMIT_VELOCITY,vel,&dxl_error);
	}
}