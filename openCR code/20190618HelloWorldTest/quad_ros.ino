#include "motor_setting.h"

#include <ros.h>
#include <std_msgs/Byte.h>
#include<std_msgs/UInt8.h>

DynamixelWorkbench dxl_wb;

ros::NodeHandle nh;
void commandCb( const std_msgs::UInt8& motor_command);
ros::Subscriber<std_msgs::UInt8> sub("motor_cmd", commandCb);

void setup()
{
    nh.initNode();
    nh.subscribe(sub);

    Serial.begin(57600);
    
    const char *log;

    for(int i = 1; i < 5; i++)
    {
        bool result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);

        if(result)
        {

        }
        else
        {
            Serial.print("Fail to init ");
            Serial.print(i);
            Serial.println(" ID motor");
        }
    }
    for(int i = 1; i < 5; i++)
    {
        bool result = dxl_wb.jointMode(i, 0, 0);
        if(result)
        {

        }
        else
        {
            Serial.print("Fail to change mode ");
            Serial.print(i);
            Serial.println(" ID motor");
        }
    }
    Serial.println("motor init done!");
}

void loop()
{
    nh.spinOnce();
}

void commandCb( const std_msgs::UInt8& motor_command)
{
    if(motor_command.data != 0)
    {
        uint8_t dxl_id = motor_command.data;

        dxl_wb.torqueOn(dxl_id);
        dxl_wb.goalPosition(dxl_id, (int32_t)0);
        dxl_wb.torqueOff(dxl_id);
    }
}