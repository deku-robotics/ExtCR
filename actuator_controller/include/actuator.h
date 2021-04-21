 /**
  * @brief	Versatile actuator interface definition
  *
  * @file 	actuator.h
  * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
  * @license please see GIT repository license declaration

  * @date	January 2018
  */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "string"
#include "com_interface.h"
#include "actuator_state.h"



class Actuator
{

public:
    /**
     * @brief constructor of virtual class Actuator
     */
//    Actuator();

//    ~Actuator();

    /**
     * @brief constructor of class JointController
     */
    virtual bool enbl() = 0;

    /**
     * @brief constructor of class JointController
     */
    virtual bool disbl() = 0;

    /**
     * @brief constructor of class JointController
     */
    virtual bool cmdVel(int16_t vel) = 0;

    /**
     * @brief constructor of class JointController
     */
    virtual bool cmdPos(int32_t pos) = 0;

    /**
     * @brief constructor of class JointController
     */
    virtual void getVelQuery() = 0;

    /**
     * @brief constructor of class JointController
     */
    virtual int32_t getLastPos() = 0;



public:
 std::string name;
 std::string dev_type;
 std::string dev;

 bool en_flag;


};

#endif
