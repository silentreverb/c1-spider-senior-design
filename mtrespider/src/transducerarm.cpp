// Includes
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

// Constants
#define ARM_UP true // Transducer arm is lifted up 
#define ARM_DOWN false // Transducer arm is in contact with the pipe surface

#define SPRING_K 0.0013505802 // Spring constant in (N*m)/deg
#define SPRING_THETA 38.64 // Spring angular displacement in deg
#define SPRING_L 0.0804672 // Moment arm length in m
#define MOTOR_K 4.774648 // Motor torque constant in (N*m)/A
#define MOTOR_R 1 // Motor spool radius in m

using namespace std;

// Initialize variables
bool positionSetpoint = ARM_DOWN; // Desired transducer arm position
float ctrlCurrent = 0; // Motor control current
float forceSetpoint = 0; // Desired normal force acting on transducer

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
	// GPIO library setup function
	wiringPiSetup();
	
	// Configure 3A motor driver pin
	pinMode(21, OUTPUT); 
	digitalWrite(21, LOW);
	
	// Configure 4A motor driver pin
	pinMode(22, OUTPUT);
	digitalWrite(22, LOW);
	
	// Configure 3-4 EN motor driver pin with Pulse Width Modulation (PWM)
	pinMode(23, OUTPUT);
	//system("sudo modprobe pwm-meson npwm=2; sudo modprobe pwm-ctrl"); // Enable hardware PWM drivers
	//system("echo 0 > /sys/devices/platform/pwm-ctrl/duty1"); // Init to 0% duty (0 A current)
	//system("echo 50 > /sys/devices/platform/pwm-ctrl/freq1"); // 50 Hz pulse frequency
	//system("echo 1 > /sys/devices/platform/pwm-ctrl/enable1"); // Enable PWM
	
    softPwmCreate(23, 0, 1023);

    //ROS node init and NodeHandle init
    ros::init(argc, argv, "transducerarm");
    ros::NodeHandle n = ros::NodeHandle("transducerarm");

    // Subscribers
    ros::Subscriber positionSetpointSub = n.subscribe("position_setpoint",1, positionSetpointCallback);
    ros::Subscriber forceSetpointSub = n.subscribe("force_setpoint",1, forceSetpointCallback);
    
    // Publishers
    ros::Publisher currentPositionPub = n.advertise<std_msgs::Bool>("position_current", 1);
    std_msgs::Bool currentPositionMsg;
    currentPositionMsg.data = positionSetpoint;
    
    // Send motor control signals at a 20 Hz rate
    ros::Rate loopRate(20);

	// Main loop
    while(ros::ok())
    {
        if(positionSetpoint == ARM_UP) {
			// TODO: Insert code to lift arm off surface
			cout << "Arm UP, Motor ON" << endl;
			
			// 100% duty (max current)
			//system("echo 1023 > /sys/devices/platform/pwm-ctrl/duty1");
            softPwmWrite(23, 1023);			

			// Spin motor CW
			digitalWrite(21, HIGH); 
			digitalWrite(22, LOW);
		}
        else if(positionSetpoint == ARM_DOWN && forceSetpoint == 0) {
			// 0% duty (no current)
			//system("echo 0 > /sys/devices/platform/pwm-ctrl/duty1");
            softPwmWrite(23, 0);		    
	
			// No motor rotation
			digitalWrite(21, LOW);
			digitalWrite(22, LOW);
        }
        else if(positionSetpoint == ARM_DOWN) {
			ctrlCurrent = (MOTOR_R/MOTOR_K)*((SPRING_K*SPRING_THETA)/SPRING_L - forceSetpoint);
			// TODO: Insert code to send the control signal to the motor
			cout << "Arm DOWN, Motor ON" << endl;
			
			// 50% duty (50% max current)
			//system("echo 512 > /sys/devices/platform/pwm-ctrl/duty1");
            softPwmWrite(23, 512);		
	
			// Spin motor CW
			digitalWrite(21, HIGH);
			digitalWrite(22, LOW);
		}
		
		// Publish current position so HMI knows what the arm is doing
		currentPositionMsg.data = positionSetpoint;
		currentPositionPub.publish(currentPositionMsg);
		
        // Sleep for 0.05 s
        ros::spinOnce();
        loopRate.sleep();
    }
    
    // Disable motor on exit to avoid damaging anything
    //system("echo 0 > /sys/devices/platform/pwm-ctrl/enable1");
    digitalWrite(21, LOW);
	digitalWrite(22, LOW);
    digitalWrite(23, LOW);
    //system("sudo modprobe -r pwm-ctrl; sudo modprobe -r pwm-meson");
	
    return 0;
}
