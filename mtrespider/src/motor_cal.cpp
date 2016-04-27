// Includes
#include "ros/ros.h"
#include <termios.h>
#include <wiringPi.h>
#include <iostream>
#include <string>
#include <limits>

// Constants
#define SPRING_L 8.04672 // Moment arm length in cm
#define SPOOL_R 0.635 // Motor spool radius in cm
#define MOTOR_V 1.5959

using namespace std;

// Initialize variables
int lastPwmDuty = -1;

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

int getchNonBlocking()
{
    struct termios initial_settings,
               new_settings;
    int n;

    unsigned char key;



    tcgetattr(0,&initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);

    n = getchar();

    key = n;


    tcsetattr(0, TCSANOW, &initial_settings);

    return key;
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
    ros::init(argc, argv, "motor_cal");
    ros::NodeHandle n = ros::NodeHandle("motor_cal");

    setPwmDuty(1023);
    cout << "Please enter PWM duty: ";
    int duty;
    cin >> duty;
    cout << "Press any key to start..." << endl;
    cin.clear();
    cin.ignore( numeric_limits <streamsize> ::max(), '\n' );
    digitalWrite(21, HIGH);
    digitalWrite(22, LOW);
    setPwmDuty(duty);
    ros::WallTime start = ros::WallTime::now();
    ros::WallTime finish;

    cin.clear();
    cin.ignore( numeric_limits <streamsize> ::max(), '\n' );

    finish = ros::WallTime::now();
    digitalWrite(21, HIGH);
    digitalWrite(22, HIGH);
    setPwmDuty(1023);
    cout << "Please enter the number of revolutions: ";
    int revs;
    cin >> revs;
    double rpm = 60*revs / (finish.toSec() - start.toSec());
    cout << "Motor RPM: " << rpm << endl << flush;
    cin.ignore( numeric_limits <streamsize> ::max(), '\n' );
    
    // Disable motor on exit to avoid damaging anything
    system("echo 0 > /sys/devices/pwm-ctrl.42/enable0");
    digitalWrite(21, LOW);
	digitalWrite(22, LOW);
    system("sudo modprobe -r pwm-ctrl; sudo modprobe -r pwm-meson");
	
    return 0;
}
