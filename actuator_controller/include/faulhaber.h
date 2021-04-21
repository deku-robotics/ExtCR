 /**
  * @brief	Faulhaber class definition for low level access of faulhaber motion controllers and derives from
  *         virtual Actuator class
  *
  * @file 	faulhaber.h
  * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
  * @license please see GIT repository license declaration

  * @date	January 2018
  */

#ifndef FAULHABER_H
#define FAULHABER_H

#define FAULHABER_STATUS_ACK 0x00
#define FAULHABER_STATUS_OK  0x01
#define FAULHABER_STATUS_COMM_ERROR 0xE0
#define FAULHABER_STATUS_TTY_ERROR  0xE1
#define FAULHABER_STATUS_ERROR_MASK 0xF0

#define FAULHABER_ANSWER_MODE 0x0F
#define FAULHABER_ANSWER_MODE_SYNC 0x00
#define FAULHABER_ANSWER_MODE_ASYNC 0x01
#define FAULHABER_ANSWER_MODE_ACK 0x02


#define FAULHABER_MAX_POS_LIMIT 1.8*10e9

#include "boost/bind.hpp"
#include "actuator.h"
#include "callbackUart.hpp"
#include <Timer.hpp>
#include "string"
#include "fstream"
#include "sstream"
#include "mutex"
#include "condition_variable"
#include "thread"
#include <deque>
#include "stack"
#include <exception>
#include <chrono>




// ROS
#include "ros/ros.h"


class Faulhaber : public Actuator {

public:

    Faulhaber(std::string name, int id, std::string dev_type, std::string dev_add, int transm, int encticks, std::shared_ptr<ActuatorState> actState);

    ~Faulhaber();


    /**
     * @brief sendCmd sends command and parameters without Query
     * @param cmd, param
     * @param param
     */
    bool sendCmd(std::string cmd, int param);
    bool sendCmd(std::string cmd);


    /**
     * @brief sendQueryCmd sends query command and reads answer
     * @param cmd command to send
     * @return
     */
    int32_t sendQueryCmd(std::string cmd);
    int32_t sendSyncQueryCmd(std::string cmd);

    /**
     * @brief stopPosQueryLoop
     */
    void stopPosQueryLoop();

    /**
     * @brief cbReceived
     */
    void cbReceived(const char *dataSync, unsigned int len);

    /**
     * @brief Enable Faulhaber servo drive
     */
    bool enbl();

    /**
     * @brief Disable Faulhaber servo drive
     */
    bool disbl();

    /**
     * @brief This method commands the desired velocity unit[rad/s]
     * @param vel Desired velocity
     * @return true if successful
     */
    bool cmdVel(int16_t vel);

    /**
     * @brief This method commands the desired rotational position [rad]
     * @param pos Desired position
     * @return true if successful
     */
    bool cmdPos(int32_t pos);

    /**
     * @brief This method sets the current position to zero
     * @param
     * @return true if successful
     */
    bool homeActuator();
    /**
     * @brief This method sets the acceleration for ramp generator in [1/s²]
     * @param vel Desired velocity
     * @return true if successful
     */
    bool setAcceleration(float acceleration);

    /**
     * @brief This method sets the maximal allowed velocity for ramp generator in [rpm]
     * @param vel Desired velocity
     * @return true if successful
     */
    bool setMaxVel(float maxVel);

    /**
     * @brief This method gets the current velocity of the servo drive
     * @param velocity Current velocity [rad/s]
     * @return true if successful
     */
    int32_t getVel(bool writeOnly=false);
    int32_t getLastVel();
    void getVelQuery();

    /**
     * @brief This method gets the current position of the servo drive
     * @param pos Current position [encoder ticks]
     * @return true if successful
     */
    int32_t getPos(bool writeOnly=false);
    int32_t getLastPos();
    void getPosQuery();


    int32_t readSync();
    size_t readBinary();

    /**
     * @brief This method provides the actuator id
     * @paramtrue;
//            m.unlock();
     * @return actuator id
     */
    int getID();


    /**
     * @brief setTransmission this method sets the i
     * @param trans
     */
    bool setTransmission(int trans);

    /**
     * @brief getTransmission this method provides the actual transmission
     * @return returns current transmission
     */
    int getTransmission();


    int getEncTicks();

    /**
      * @brief This method provides the actuator name
      * @param
      * @return actuator name
      */
    std::string getName();

    /**
       * @brief This method provides the actuator type
       * @param
       * @return actuator type
       */
    std::string getType();

    /**
       * @brief This method provides the actuator address
       * @param
       * @return actuator address
       */
    std::string getAddr();

    /**
     * @brief initBinInterface opens Binary interface to Faulhaber motor
     */
    bool OpenBinInterface();

    /**
     * @brief initBinInterface close Binary interface to Faulhaber motor
     */
    void CloseBinInterface();

    /**
     * @brief setPosLimits sets the actuators lower and upper position limits
     * @param llow lower position limit required to be > 0
     * @param lup upper position limit required to be < 0
     * @return bool if succesful false if not
     */
    bool setPosLimits(int llow, int lup);

    /**
     * @brief ActPosLimits activates obtained position limits
     * @param active true if activated / false if deactivated
     * @return true bool if succesful false if not
     */
    bool ActPosLimits(bool active);


    /**
     * @brief getBinTrace conducts a trace on the binary interface
     * @return
     */
    bool sendTraceRequest();

    bool readTraceData();


    /**
     * @brief setTraceSettings set Mode1 to position and Mode2 to velocity
     */
    bool setTraceSettings();


    /**
     * @brief Factory for communication instances
     */
    //Communication * getComInstance(std::string &dev_type);


    bool isNewPosAvailable();
    bool isReadDone();


    ros::Duration getLastPosDeltaT();
    ros::Time getLastPosQueryTime();

    bool isInitialized();

    int32_t getZeroPos();

    bool setPvel(uint val);
    uint getPvel();


private:

    // Flags
    bool asyncMode = false;
    bool bin_interface = false;

    uint Pvel = 7;

    // Last time a new position has been read / queried
    ros::Time lastPosQueryTime;
    // Duration between last two position queries
    ros::Duration lastPosDeltaT;

    bool initialized = false;

    static std::mutex mx_cmd;

    std::mutex commMutex, mx_posvel;

    Timer cmdRateTimer;
    double cmdRate = 500;

    bool posQueryRunning = true;
    std::thread* posQueryThread;

    int32_t lastPos = 0;
    int16_t lastVel = 0;

    bool newPosAvailable = false;

    CallbackUart *uart;

    int  id;
    std::string name;

    std::string dev_type;
    bool is_spi = false;
    std::string dev_add;

    // Mechanical parameters
    int transm; // Transmission of the gearing
    int encticks; //Encoder ticks per revolution

    int lastStatus;
    int counter;

    std::string readSyncData;
    std::string readSyncBuffer;
    size_t nReadSync = 0;
    int32_t dataSync = 0;

    std::string readDataAsync;


    bool readDataAsyncFlag = false;

    bool delimReached = false;
    bool ready = false;
    bool notified = false;

    bool acqDataf, readFinishf;
    bool en_flag;

    std::vector<char> readQueue;
    bool lastCmdType = false; // false for pos, true for vel

    size_t lastSize = 0;
    size_t endIdx = std::string::npos;
    std::shared_ptr<ActuatorState> actState;
    double zeroPos = 6; //[mm]

};

#endif
