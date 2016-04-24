#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>


using namespace std;

int fd;

void i2c_write(uint8_t reg, uint16_t value) {
	unsigned char wbuf[3];
    wbuf[0] = reg;
    wbuf[1] = (uint8_t)((value >> 8) & 0xFF);
    wbuf[2] = (uint8_t)(value & 0xFF);
    
    write(fd, wbuf, 3);
}

void i2c_read(uint8_t reg, uint16_t *value) {
	unsigned char rbuf[2];
    write(fd, &reg, 1);
    read(fd, rbuf, 2);
    *value = uint16_t(rbuf[0] << 8 | rbuf[1]);
}

void begin() {
    fd = open("/dev/i2c-1", O_RDWR);
    ioctl(fd, I2C_TENBIT, 0);
    ioctl(fd, I2C_SLAVE, 0x40);
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
	ros::init(argc, argv, "ampsensor");
	ros::NodeHandle n = ros::NodeHandle("ampsensor");
	
    ros::Publisher busVoltagePub = n.advertise<std_msgs::Float32>("bus_voltage", 1);
    std_msgs::Float32 busVoltageMsg;
    
    ros::Publisher motorCurrentPub = n.advertise<std_msgs::Float32>("motor_current", 1);	
    std_msgs::Float32 motorCurrentMsg;

    begin();
	calibrate();
	configure();
	
	ros::Rate loopRate(20);
	
	while(ros::ok())
	{
		busVoltageMsg.data = getBusVoltage();
        motorCurrentMsg.data = getCurrent();
        cout << "Voltage: " << busVoltageMsg.data << " V, Current: " << motorCurrentMsg.data << " mA" << endl;

        busVoltagePub.publish(busVoltageMsg);
        motorCurrentPub.publish(motorCurrentMsg);

		ros::spinOnce();
		loopRate.sleep();
	}
	
	return 0;
}
