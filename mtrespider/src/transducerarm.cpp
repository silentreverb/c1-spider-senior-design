// Includes
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "mtrespider/ina219.h"
#include <wiringPi.h>
#include <iostream>
#include <string>

// Constants
#define ARM_UP true // Transducer arm is lifted up 
#define ARM_DOWN false // Transducer arm is in contact with the pipe surface

#define SPRING_L 8.04672 // Moment arm length in cm
#define SPOOL_R 0.635 // Motor spool radius in cm
#define MOTOR_V 1.62091728
#define MOTOR_V_CW 2.1212672 
#define CONTACT_F_MAX_DIST 0.2

using namespace std;

// Initialize variables
bool positionSetpoint = ARM_DOWN; // Desired transducer arm position
float forceSetpoint = 1.25; // Desired normal force acting on transducer
int lastPwmDuty = -1;
int dir = 0;
double rate;
double rate_cw;
float busVoltage = 12;
float motorCurrent = 0;
double currentPos = 0;
double targetPos = 0;
ros::WallTime lastTime;
ros::WallTimer timer;

void setPwmDuty(int duty);

void calcOnTime() {
    int duty;
    if(positionSetpoint == ARM_UP) {
        targetPos = 2.5;
    }
    else if(positionSetpoint == ARM_DOWN) {
        targetPos = CONTACT_F_MAX_DIST*(1-0.8*forceSetpoint);
    }
    
    double deltaPos = targetPos - currentPos;
    if(abs(deltaPos) <= 0.03) {
        duty = 719;
        rate = 0.5259240;
        rate_cw = 1.4922788;
    }
    else {
        duty = 1023;
        rate = MOTOR_V;
        rate_cw = MOTOR_V_CW;
    }
    if(deltaPos > 0) {
        timer.stop();
        dir = 1;
        digitalWrite(21, HIGH);
        digitalWrite(22, LOW);
        setPwmDuty(duty);
        timer.setPeriod(ros::WallDuration(deltaPos/rate));
        timer.start();       
    }
    else if(deltaPos < 0) {
        timer.stop();
        dir = -1;
        digitalWrite(21, LOW);
        digitalWrite(22, HIGH);
        deltaPos = -1*deltaPos;
        setPwmDuty(duty);
        timer.setPeriod(ros::WallDuration(deltaPos/rate_cw));
        timer.start();
    }
}

// This runs when a new position setpoint has been published on the 
// /mtrespider/transducerarm/position_setpoint topic.
void positionSetpointCallback(const std_msgs::Bool::ConstPtr& msg) {
	positionSetpoint = msg->data; // Update variable with new desired position
    calcOnTime();
}

// This runs when a new force setpoint has been published on the 
// /mtrespider/transducerarm/force_setpoint topic.
void forceSetpointCallback(const std_msgs::Float32::ConstPtr& msg) {
	forceSetpoint = msg->data; // Update variable with new force setpoint
    calcOnTime();   
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

/*void ina219Callback(const mtrespider::ina219::ConstPtr& msg) {
    busVoltage = msg->voltage;
    motorCurrent = msg->current;
}
*/

void updatePosEst() {
    if(dir == 1) {
        currentPos = currentPos + rate*(ros::WallTime::now().toSec() - lastTime    .toSec());
    }
    else if(dir == -1) {
        currentPos = currentPos - rate_cw*(ros::WallTime::now().toSec() - lastTime    .toSec());
    }

    lastTime = ros::WallTime::now();
}

void timerCallback(const ros::WallTimerEvent& event) {
    updatePosEst(); 

    dir = 0;
    digitalWrite(21, HIGH);
    digitalWrite(22, HIGH);
    setPwmDuty(1023);
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
	system("echo 1023 > /sys/devices/pwm-ctrl.42/freq0"); // 25 kHz pulse frequency
	system("echo 1 > /sys/devices/pwm-ctrl.42/enable0"); // Enable PWM

    //ROS node init and NodeHandle init
    ros::init(argc, argv, "transducerarm");
    ros::NodeHandle n = ros::NodeHandle("transducerarm");

    // Subscribers
    ros::Subscriber positionSetpointSub = n.subscribe("position_setpoint",1, positionSetpointCallback);
    ros::Subscriber forceSetpointSub = n.subscribe("force_setpoint",1, forceSetpointCallback);
    //ros::Subscriber ina219Sub = n.subscribe("samples",1, ina219Callback);    

    // Publishers
    ros::Publisher currentPositionPub = n.advertise<std_msgs::Bool>("position_current", 1);
    std_msgs::Bool currentPositionMsg;
    currentPositionMsg.data = positionSetpoint;
    
    // Send motor control signals at a 20 Hz rate
    timer = n.createWallTimer(ros::WallDuration(1), timerCallback, true);
    timer.stop();
    lastTime = ros::WallTime::now();

    ros::Rate loopRate(1000);

	// Main loop
    while(ros::ok())
    {	               
        updatePosEst();
        
        cout << "Current Position: " << currentPos << " cm" << endl;
        cout << "Target Position: " << targetPos << " cm" << endl << endl;

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
