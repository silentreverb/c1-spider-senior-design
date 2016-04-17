#include <iostream>
#include "ros/ros.h"
#include <wiringPiI2C.h>

int fd;

void i2c_write(uint8_t reg, uint16_t value) {
	wiringPiI2CWriteReg16(fd, reg, value);
}

void i2c_read(uint8_t reg, uint16_t *value) {
	*value = wiringPiI2CReadReg16(fd, reg);
}

void calibrate() {
	uint16_t calValue = 10240;
	i2c_write(0x05, calValue);
}

void configure() {
	uint16_t confValue = 0x2000 | 0x1800 | 0x0400 | 0x0018 | 0x0007;
	i2c_write(0x00, confValue);
}

float getBusVoltage() {
	uint16_t value;
	i2c_read(0x02, &value);
	
	return (float)((value >> 3) * 4) * 0.001;
}

float getCurrent() {
	calibrate();
	
	uint16_t value;
	i2c_read(0x04, &value);
	
	return (float)value / 25;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "current_sensor");
	ros::NodeHandle n = ros::NodeHandle("current_sensor");
	
	fd = wiringPiI2CSetup(0x40);
	calibrate();
	configure();
	
	ros::Rate loopRate(40);
	
	while(ros::ok())
	{
		cout "Voltage: " getBusVoltage() << " V, Current: " << getCurrent() << " mA" << endl;
		
		ros::spinOnce();
		loopRate.sleep();
	}
	
	return 0;
}
