// Includes
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <wiringPi.h>
#include <iostream>

// Constants
#define SENSOR_CONSTANT 1  // ADC value to force conversion factor
#define FORCE_THRESH 512 // Force threshold indicating a ferromagnetic surface is present

using namespace std;

int main(int argc, char **argv)
{
    // Setup wiringPi library to be able to read force sensor voltage
    wiringPiSetup();
    
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "ferrosensor");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher surfaceDetectedPub = n.advertise<std_msgs::Bool>("/mtrespider/ferrosensor/detected", 1);
    std_msgs::Bool msg;
    msg.data = true;
    
    // Read voltage at a 10 Hz rate
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        // Read voltage from force sensor and convert to measured force
        int sensorValue = analogRead(25);
        double magForce = SENSOR_CONSTANT * sensorValue;
        
        // Check measured force against threshold value calculated by Raymond
        if (magForce > FORCE_THRESH) {
			msg.data = true; // Ferromagnetic surface detected
		}
		else {
			msg.data = false; // No ferromagnetic surface detected, alert operator on HMI
		}
		
		// Publish detection result to /mtrespider/ferrosensor/detected ROS topic
        surfaceDetectedPub.publish(msg);

		// Sleep for 0.1 s
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}
