/**
 * @brief	SPI interface for BeagleBoneBlack
 *
 * @file 	spi.h
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include <string>
#include <string.h>
#include <iostream>


class spi{
public:
  
    spi(std::string devPath);
    ~spi();
    int exchangeData(const char* buffer);

    bool isInitialized();


private:
    bool initialized = false;
    bool opened = false;
    std::string devPath;
    int device = -1;

    u_char spiMode = SPI_MODE_0;
    u_char bitsPerWord = 8;
    uint speed = 1000000;

};
