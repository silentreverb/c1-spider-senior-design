// Includes
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <iostream>

// Constants
#define ARM_UP true // Transducer arm is lifted up 
#define ARM_DOWN false // Transducer arm is in contact with the pipe surface

#define SPRING_K 0.0013505802 // Spring constant in (kg*m)/deg
#define SPRING_THETA 0 // Spring angular displacement in deg
#define SPRING_L 0.0804672 // Moment arm length in m
#define MOTOR_TAU 1 // Motor torque constant in kg*m
#define MOTOR_R 1 // Motor spool radius in m

using namespace std;

// Initialize variables
bool positionSetpoint = ARM_DOWN; // Desired transducer arm position
float ctrlCurrent = 0; // Motor control current
float forceSetpoint = -1; // Desired normal force acting on transducer

// This runs when a new position setpoint has been published on the 
// /mtrespider/transducerarm/position_setpoint topic.
void positionSetpointCallback(const std_msgs::Bool::ConstPtr& msg) {
	positionSetpoint = msg->data; // Update variable with new desired position
}

// This runs when a new force setpoint has been published on the 
// /mtrespider/transducerarm/force_setpoint topic.
void forceSetpointCallback(const std_msgs::Float32::ConstPtr& msg) {
	forceSetpoint = msg->data; // Update variable with new force setpoint
}

int main(int argc, char **argv)
{
	wiringPiSetup();
	
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "transducerarm");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber positionSetpointSub = n.subscribe("/mtrespider/transducerarm/position_setpoint",1, positionSetpointCallback);
    ros::Subscriber forceSetpointSub = n.subscribe("/mtrespider/transducerarm/force_setpoint",1, forceSetpointCallback);
    
    // Send motor control signals at a 20 Hz rate
    ros::Rate loopRate(20);

	// Main loop
    while(ros::ok())
    {
        if(positionSetpoint == ARM_UP) {
			// TODO: Insert code to lift arm off surface
			cout << "Arm UP, Motor ON" << endl;
		}
        else if(positionSetpoint == ARM_DOWN && forceSetpoint == -1) {
			// TODO: Insert code to turn off motor entirely
			cout << "Arm DOWN, Motor OFF" << endl;
        }
        else if(positionSetpoint == ARM_DOWN) {
			ctrlCurrent = (MOTOR_R/MOTOR_TAU)*((SPRING_K*SPRING_THETA)/SPRING_L - forceSetpoint);
			// TODO: Insert code to send the control signal to the motor
			cout << "Arm DOWN, Motor ON" << endl;
		}
		
        // Sleep for 0.05 s
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}
