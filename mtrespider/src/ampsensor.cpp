#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "ros/ros.h"
#include <mtrespider/ina219.h>


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
	
    ros::Publisher samplesPub = n.advertise<mtrespider::ina219>("samples", 1);	
    mtrespider::ina219 samplesMsg;

    samplesMsg.header.seq = 0;

    system("sudo modprobe aml-i2c");
    
    begin();
	calibrate();
	configure();
	
	ros::Rate loopRate(20);
	
	while(ros::ok())
	{
		samplesMsg.voltage = getBusVoltage();
        samplesMsg.current = getCurrent();
        samplesMsg.header.stamp = ros::Time::now();

        cout << samplesMsg << endl;

        samplesPub.publish(samplesMsg);
        samplesMsg.header.seq++;        

		ros::spinOnce();
		loopRate.sleep();
	}

    system("sudo modprobe -r aml-i2c");
	
	return 0;
}
