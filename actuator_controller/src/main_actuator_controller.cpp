/**
 * @brief	This class manages the connection and communication
          with the Faulhaber Motions controllers
 *
 * @file 	main_actuator_controller.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "actuator_controller_node.h"
#include <signal.h>
#include <iostream>

ActuatorController* acNode = NULL;

void handleSignal(int signal){
 if(acNode != NULL){
     ROS_WARN("Shutting down ActuatorController!");
     acNode->~ActuatorController();
 } else
     ROS_ERROR("No ActuatorController to shut down!");
 ros::shutdown();
 exit(1);
}

int main(int argc, char **argv)
{
    //Set-up ROS
    ros::init(argc,argv,"actuator_controller_node");
    // Necessary for signal handler to work..
    ros::NodeHandle nh;


    // Setup signal handler to achieve clean shutdown on ctrl-c (including proper network disconnect)
    struct sigaction signalHandler;
    signalHandler.sa_handler = handleSignal;
    sigemptyset(&signalHandler.sa_mask);
    signalHandler.sa_flags = 0;
    sigaction(SIGINT, &signalHandler, NULL);

    signal(SIGPIPE, SIG_IGN);


    ros::Time::init();
    //Set-up Actuator controller node
    ROS_INFO("Creating new ActuatorController");
    acNode = new ActuatorController(argc,argv, "actuator_controller_node");
    acNode->run();
    return 0;
}
