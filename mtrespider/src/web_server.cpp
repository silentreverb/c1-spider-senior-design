// Includes
#include "ros/ros.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{ 
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "web_server");
    ros::NodeHandle n = ros::NodeHandle("web_server");

    FILE* in = popen("lighttpd -D -f /home/odroid/ros/catkin_ws/src/mtrespider/web_server/lighttpd.conf","r");

    ros::spin();
    
    pclose(in);
    
    return 0;
}
