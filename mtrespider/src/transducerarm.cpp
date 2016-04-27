// Includes
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "mtrespider/ina219.h"
#include <wiringPi.h>
#include <iostream>
#include <string>
#include <PIDController.h>

// Constants
#define ARM_UP true // Transducer arm is lifted up 
#define ARM_DOWN false // Transducer arm is in contact with the pipe surface

#define SPRING_K 1.32446467 // Spring constant in (N*cm)/deg
#define SPRING_THETA 38.64 // Spring angular displacement in deg
#define SPRING_L 8.04672 // Moment arm length in cm
#define MOTOR_K 0.477464829821 // Motor torque constant in (N*cm)/mA
#define MOTOR_R 0.635 // Motor spool radius in cm

using namespace std;

// Initialize variables
bool positionSetpoint = ARM_DOWN; // Desired transducer arm position
float forceSetpoint = 1.25; // Desired normal force acting on transducer
int lastPwmDuty = -1;
float busVoltage = 12;
float motorCurrent = 0;
PIDController* tauCtrlGnd = new PIDController(25,10,0,-511,512);
PIDController* tauCtrlAir = new PIDController(25,10,0,-511,512);

void setControlSystem() {
    if(positionSetpoint == ARM_UP) {
        // Spin motor CW
        digitalWrite(21, HIGH);
        digitalWrite(22, LOW);

        tauCtrlGnd->off();
        tauCtrlAir->on();
        
        double tau_desired = (SPRING_K*MOTOR_R*60)/SPRING_L;
        tauCtrlAir->targetSetpoint(tau_desired);
    }
    else if(positionSetpoint == ARM_DOWN && forceSetpoint == 1.25) {
        // No motor rotation
        digitalWrite(21, LOW);
        digitalWrite(22, LOW);

        tauCtrlGnd->off();
        tauCtrlAir->off();
    }
    else if(positionSetpoint == ARM_DOWN) {
        // Spin motor CW
        digitalWrite(21, HIGH);
        digitalWrite(22, LOW);

        tauCtrlAir->off();
        tauCtrlGnd->on();

        double tau_desired = MOTOR_R*((SPRING_K*SPRING_THETA)/SPRING_L);
        tauCtrlGnd->targetSetpoint(tau_desired);
    }   
}

// This runs when a new position setpoint has been published on the 
// /mtrespider/transducerarm/position_setpoint topic.
void positionSetpointCallback(const std_msgs::Bool::ConstPtr& msg) {
	positionSetpoint = msg->data; // Update variable with new desired position
    setControlSystem();
}

// This runs when a new force setpoint has been published on the 
// /mtrespider/transducerarm/force_setpoint topic.
void forceSetpointCallback(const std_msgs::Float32::ConstPtr& msg) {
	forceSetpoint = msg->data; // Update variable with new force setpoint
    setControlSystem();
}

void setPwmDuty(int duty) {
    if(duty < 0) {
        duty = 0;
    }
    else if(duty > 1023) {
        duty = 1023;
    }

    if(!(duty == lastPwmDuty)) {
        stringstream ss;
        ss << duty;
        string str = "echo " + ss.str() + " > /sys/devices/pwm-ctrl.42/duty0";
        system(str.c_str());
        lastPwmDuty = duty;
    }
}

void ina219Callback(const mtrespider::ina219::ConstPtr& msg) {
    busVoltage = msg->voltage;
    motorCurrent = msg->current;

    double tau_actual = MOTOR_K*motorCurrent;
    cout << "Desired tau: " << tauCtrlAir->getSetpoint() << ", Actual tau: " << tau_actual << endl;

    if(positionSetpoint == ARM_UP) {
        int corr = tauCtrlAir->calc(tau_actual) + 511;
        cout << "Corr: " << corr << endl;
        setPwmDuty(corr);
    }
    else if(positionSetpoint == ARM_DOWN && forceSetpoint != 1.25) {
        int corr = tauCtrlGnd->calc(tau_actual) + 511;
        cout << "Corr: " << corr << endl;
        setPwmDuty(corr);
    }
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
	system("sudo modprobe pwm-meson; sudo modprobe pwm-ctrl"); // Enable hardware PWM drivers
	setPwmDuty(0); // Init to 0% duty (0 A current)
	system("echo 1000 > /sys/devices/pwm-ctrl.42/freq0"); // 25 kHz pulse frequency
	system("echo 1 > /sys/devices/pwm-ctrl.42/enable0"); // Enable PWM

    //ROS node init and NodeHandle init
    ros::init(argc, argv, "transducerarm");
    ros::NodeHandle n = ros::NodeHandle("transducerarm");

    // Subscribers
    ros::Subscriber positionSetpointSub = n.subscribe("position_setpoint",1, positionSetpointCallback);
    ros::Subscriber forceSetpointSub = n.subscribe("force_setpoint",1, forceSetpointCallback);
    ros::Subscriber ina219Sub = n.subscribe("samples",1, ina219Callback);    

    // Publishers
    ros::Publisher currentPositionPub = n.advertise<std_msgs::Bool>("position_current", 1);
    std_msgs::Bool currentPositionMsg;
    currentPositionMsg.data = positionSetpoint;
    
    // Send motor control signals at a 20 Hz rate
    ros::Rate loopRate(20);

	// Main loop
    while(ros::ok())
    {	
		// Publish current position so HMI knows what the arm is doing
		currentPositionMsg.data = positionSetpoint;
		currentPositionPub.publish(currentPositionMsg);
		
        // Sleep for 0.05 s
        ros::spinOnce();
        loopRate.sleep();
    }
    
    // Disable motor on exit to avoid damaging anything
    system("echo 0 > /sys/devices/pwm-ctrl.42/enable0");
    digitalWrite(21, LOW);
	digitalWrite(22, LOW);
    system("sudo modprobe -r pwm-ctrl; sudo modprobe -r pwm-meson");
	
    return 0;
}
