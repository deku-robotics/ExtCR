/**
 * @brief	This class handles the limit configuration on the BeagleBoneBlack
 *
 * @file 	limit_check.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "limit_check.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>

limit_check::limit_check()
{
    gpio_manager_ = GPIO::GPIOManager::getInstance();

	  lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_16"));
    lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_8"));
    lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_12"));
	  lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_17"));
    lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_9"));
    lower_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_13"));


	  upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_19"));
    upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_11"));
    upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_15"));
  	upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_18"));
    upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_10"));
    upper_limits.push_back(GPIO::GPIOConst::getInstance()->getGpioByKey("P8_14"));

    for (int i=0;i<6;i++)
    {
        gpio_manager_->setDirection(lower_limits[i], GPIO::INPUT);
        gpio_manager_->setDirection(upper_limits[i], GPIO::INPUT);
    }
}

limit_check::~limit_check()
{
    gpio_manager_->~GPIOManager();
}

bool limit_check::is_limit_reached(int actuator_number, bool lower_limit)
{
    bool output;
    if(lower_limit)
        output=gpio_manager_->getValue(this->lower_limits[actuator_number]);
    else
        output=gpio_manager_->getValue(this->upper_limits[actuator_number]);

    if(output)
        return false;
    if(!output)
        return true;
}
