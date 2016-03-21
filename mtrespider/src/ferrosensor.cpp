// Includes
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include <wiringPi.h>
#include <iostream>

// Constants
#define SENSOR_THRESH 512 // Force threshold indicating a ferromagnetic surface is present

using namespace std;

int main(int argc, char **argv)
{
    // Setup wiringPi library to be able to read force sensor voltage
    wiringPiSetup();
    
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "ferrosensor");
    ros::NodeHandle n = ros::NodeHandle("ferrosensor");

    // Publishers
    ros::Publisher sensorValuePub = n.advertise<std_msgs::Int16>("fsr_reading", 1);
    std_msgs::Int16 sensorValueMsg;
    sensorValueMsg.data = 0;
    
    ros::Publisher surfaceDetectedPub = n.advertise<std_msgs::Bool>("detected", 1);
    std_msgs::Bool surfaceDetectedMsg;
    surfaceDetectedMsg.data = false;
    
    // Read voltage at a 10 Hz rate
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        // Read voltage from force sensor and convert to measured force
        int sensorValue = analogRead(0);
        sensorValueMsg.data = sensorValue;
        
        if(sensorValue != 0) {
			ROS_INFO("FSR Reading: %d", sensorValue); 
		}
		
        // Check measured force against threshold value calculated by Raymond
        if (sensorValue > SENSOR_THRESH) {
			surfaceDetectedMsg.data = true; // Ferromagnetic surface detected
		}
		else {
			surfaceDetectedMsg.data = false; // No ferromagnetic surface detected, alert operator on HMI
		}
		
		// Publish detection result to /mtrespider/ferrosensor/detected ROS topic
        sensorValuePub.publish(sensorValueMsg);
        surfaceDetectedPub.publish(surfaceDetectedMsg);
		
		// Sleep for 0.1 s
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}
