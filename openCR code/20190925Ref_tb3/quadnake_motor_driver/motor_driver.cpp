/* Authors: Bongsub Song */

#include "motor_driver.h"

motorDriver::motorDriver()
	:baudrate_(BAUDRATE),
	protocol_ver_(PROTOCOL_VERSION)
{
	bool initError = false; //True is no error

	motorids[0] = { 1,2,3,4,5,6,7,8 };
	motorids[1] = { 11,12,13,14,15,16,17,18 };
	motorids[2] = { 21,22,23,24,25,26,27,28 };
	motorids[3] = { 31,32,33,34,35,36,37,38 };

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
	pah_->write1ByteTxRx(poh_,id,ADDR_X_TORQUE_ENABLE,onoff,dxl_error);
	if(dxl_error != COMM_SUCCESS)
	{
		DEBUG_SERIAL.println("Failed to write command during running [setTorque] function.");
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
		pah_->write4ByteTxRx(poh_,id,ADDR_X_LIMIT_VELOCITY,vel,dxl_error);
	}
}