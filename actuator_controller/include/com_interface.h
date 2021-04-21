 /**
  * @brief	Virtual communication interface definition
  *         to adress the aspect of versatility w.r.t. UART and i2c interfaces
  *
  * @file 	com_interface.h
  * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
  * @license please see GIT repository license declaration

  * @date	January 2018
  */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>

class Communication{

public:

    /**
     * @brief Virtual methode to open device communication
     */
    virtual void open() = 0;

    /**
     * @brief Virtual methode to write to device
     * @param cmd
     */
    virtual bool write(std::string &s) = 0;

    /**
     * @brief Virtual methode to read from device
     */
    virtual std::string read() =  0;

    /**
     * @brief Virtual methode to catch occuring errors
     */
    virtual void error() = 0;

    /**
     * @brief Virtual methode to close device
     */
    virtual void close() = 0;

    /**
     * @brief Destructor
     */
    virtual ~Communication() {}


};



#endif
