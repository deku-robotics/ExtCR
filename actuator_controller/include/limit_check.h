/**
 * @brief	gpio limit check definition
 *
 * @file 	limit_check.h
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#ifndef LIMIT_CHECK_H
#define LIMIT_CHECK_H

#include <vector>

#include <BeagleBoneBlack-GPIO/GPIO/GPIOConst.h>
#include <BeagleBoneBlack-GPIO/GPIO/GPIOManager.h>

class limit_check
{
public:
    limit_check();
    ~limit_check();

    bool is_limit_reached(int actuator_number, bool lower_limit=true); // if bool lowerLimit is false, the upper limit will be checked
private:

    GPIO::GPIOManager* gpio_manager_;

    std::vector<int> lower_limits;
    std::vector<int> upper_limits;

};

#endif // LIMIT_CHECK_H
