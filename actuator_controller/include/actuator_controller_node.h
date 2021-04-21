/**
 * @brief	Header for ROS communication with low
 *        level Faulhaber Motion Controllers
 *
 * @file 	actuator_controller_node.h
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#ifndef ACTUATOR_CONTROLLER_NODE_H
#define ACTUATOR_CONTROLLER_NODE_H

#include "algorithm"
#include "math.h"
#include <functional>
#include <sstream>

#include "ros/ros.h"
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "std_msgs/Bool.h"
#include <std_msgs/Int16.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <actuator_controller/setInt.h>

#include "actuator_controller/actuator_state.h"
#include "actuator_controller/actuator_vel_cmd.h"
#include "actuator_controller/actuator_pos_cmd.h"

#include "actuator_state.h"
#include "faulhaber.h"
#include "limit_check.h"

#include <Timer.hpp>
#include <math.h>
#include <chrono>
#include <mutex>


//#define DEBUG_OUTPUT
//#define DEBUG_TIME
#define NUM_ACTUATORS
#define ENCODER_TRESHOLD 0.0

enum class Status: char{IDLE = 0, CALCULATING = 1, PROCESSING = 2 ,ABORTED = 99, ERROR = 100};


class ActuatorController{


    uint TEST_Pmin = 9;
    uint TEST_Pmax = 13;
    double TEST_PminVel = 1;
    double TEST_PmaxVel = 10;
    double TEST_Pcutoff = 30;

    actuator_controller::actuator_pos_cmd TEST_lastPosCmd;

    bool TEST_deactAdaptiveP = false;

    double TEST_times = 0;
    double TEST_peakTime = 0;
    uint TEST_nTimes = 0;
    bool TEST_gotFirstCmd = false;

    int32_t TEST_posVal = -999;
    int16_t TEST_velVal = -999;

    ros::Time TEST_preWrite;
    ros::Time TEST_postWrite;

    vector<int16_t> TEST_curTarget = {0,0,0,0,0,0};

    bool readCycle = false;

    ros::Time lastCommandStamp;

    bool shuttingDown = false;

    bool hard_limit;
    bool limitCheckEnabled = true;
    bool timeoutCheckEnabled = true;
    bool calibrating = false;
    bool limitReached = false;

    // Node frequency configuration
    double robotRate = 50;
    double rosRate = 100;

    std::chrono::time_point<std::chrono::system_clock> centeringStart, last_command_time, TEST_lastCmdTime;
    double timeout=1;

    boost::mutex mx_cmd, mx_states;

    // Vectors containing objects of actuators and their corresponding state
    std::vector<std::shared_ptr<Faulhaber>> actuators;
    std::vector<std::shared_ptr<ActuatorState>> act_states;

    ros::NodeHandle *nh;
    ros::NodeHandle *nh_local;

    // ROS interprocess communication
    ros::Subscriber vel_sub,cmd_sub, rate_sub, pos_sub;
    ros::Publisher  pub_actState, pub_status;

    // ROS services
    ros::ServiceServer get_out_of_limits_srv, home_srv, goToZero_srv, maintenance_srv, toggleLimits_srv, srv_setPvel, srv_setIvel;

    // ROS topic configuration
    std::string cmdTopicName, stateTopicName, velCmdTopicName, posCmdTopicName;

    //Flags
    bool initDev, MaintenanceMode;

public:

    /**
     * @brief constructor of class JointController
     * @param argc number of arguments, argv specification of arguments, node_name description of this node
     * @return
     */
    ActuatorController(int argc, char *argv[], const char *node_name);

    /**
     * @brief destructor of class JointController
     * @param
     * @return
     */
    ~ActuatorController();

    /**
     * @brief ROS main loop
     * @param
     * @return
     */
    void run();

    /**
     * @brief update indexed actuator value
     * @param idx actuator index, cmdType boolean activation
     * @return
     */
    void actuatorValueUpdated(uint idx, bool cmdType);

private:

    limit_check *checker_; // Current actuator limit

    enum class MODES{
        VELOCITY_MODE,
        POSITION_MODE
    };

    MODES actuator_mode; // Desired actuator control mode

    /**
     * @brief Obtain parameters from ROS parameter server and init devices
     * @param
     * @return  result of initialisation procedure (true/false)
     */
    bool initActuators();

    /**
     * @brief Obtain parameters from ROS parameter server
     * @param
     * @return
     */
    void getROSParameter(void);

    /**
     * @brief Write to character device (kernel module) to reset fault
     * @param req query for resetting the device, res result of query
     * @return  result of reset procedure (true/false)
     */
    bool resetFault(std_msgs::Bool &req, std_msgs::Bool &res);

    /**
     * @brief Callback to apply velocities on low level (RS232 or I2C) connections
     * @param msg desired motor velocity from incoming ros message
     * @return
     */
    void velCmdCallback(const actuator_controller::actuator_vel_cmd &msg);

    /**
     * @brief Callback to apply position on low level (RS232 or I2C) connections
     * @param msg desired motor position from incoming ros message
     * @return
     */
    void posCmdCallback(const actuator_controller::actuator_pos_cmd &msg);

    /**
     * @brief Init local environment
     * @param
     * @return  result of initialisation procedure (true/false)
     */
    bool Init();

    /**
     * @brief shutdown Shutdown actuators in case of node shutdown, emergency or crash
     * @param
     * @return
     */
    void shutdown();

    /**
     * @brief softShutdown stops drives without disabling
     * @param
     * @return
     */
    void softShutdown();

     /**
      * @brief pubStateData Publish actuator state data
      * @param
      * @return
      */
     void pubStateData();

     /**
      * @brief calibrateActuators runs the homing sequences for the entire set of actuators
      * @param
      * @return  results of the calibration procedure (true/false)
      */
     bool calibrateActuators();

     /**
      * @brief checkActuatorsLimits assesses the current state of actuator configuration vs limits
      * @param
      * @return  results of the limit check (true/false)
      */
     bool checkActuatorsLimits();

     /**
      * @brief getOutOfLimits slowly moves the considered actuators out of the limit zone
      * @param
      * @return
      */
     void getOutOfLimits();

     /**
      * @brief centerActuators moves the actuators to the center position of acquired limit positions after calibration sequence
      * @param  outOfSingularity flag for singularity avoidance, ignoreLimits flat for overriding the limits configuration
      * @return
      */
     void centerActuators(bool outOfSingularity = false, bool ignoreLimits = false);

     /**
      * @brief maintenancePosition  moves the actuators to the maintance configuration
      * @param
      * @return
      */
     void maintenancePosition();

     /**
      * @brief checkForTimeout implements a cyclic check to identitfy motion controller timeouts
      * @param
      * @return  result of cyclic check (true/false)
      */
     bool checkForTimeout();

     /**
      * @brief checkForTimeout implements a cyclic check to identitfy motion controller timeouts
      * @param
      * @return  result of cyclic check (true/false)
      */
     void publishStatus(Status statusIdx, std::string additionalInfo = "");

     void rateCB(const std_msgs::Int16ConstPtr& rate);


     // ROS services
     bool srvGetOutOfLimits(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
     bool srvCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
     bool srvGoToZero(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
     bool srvMaintenance(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
     bool srvToggleLimits(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
     bool srvSetPVel(actuator_controller::setInt::Request &req, actuator_controller::setInt::Response &res);
     bool srvSetIVel(actuator_controller::setInt::Request &req, actuator_controller::setInt::Response &res);


     // TCP communication
     void tcp_status(std::string status);
     void tcp_connected();
     void tcp_disconnected();
     void tcp_positionReceived(uint actuatorIdx, double pos);
     void tcp_velocityReceived(uint actuatorIdx, double vel);
     void tcp_accelerationReceived(uint actuatorIdx, double vel);
     void tcp_commandUpdateComplete(bool velocityMode);
     void tcp_calibrate();
     void tcp_center(bool outOfSingularity);
     void tcp_outOfLimits();
     void tcp_maintenance();
     void tcp_receivedState(vector<double> vals, bool velocityMode);
     void tcp_setRate(double rate);
     void tcp_receivedCommand(vector<string> vals, bool velocityMode);
     void tcp_receivedVelocityCommand(vector<int16_t> vals);
     void tcp_receivedPositionCommand(vector<int32_t> vals);
     void tcp_toggleLimits(bool on);

};

#endif
