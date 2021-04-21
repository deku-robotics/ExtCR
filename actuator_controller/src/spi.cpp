/**
 * @brief	This interfaces handles the spi connection and communication
          on the BeagleBone Black
 *
 * @file 	spi.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include <spi.hpp>

spi::spi(std::string devPath){
    this->devPath = devPath;

    device = open(devPath.c_str(), O_RDWR);

    if(device < 0){
        std::cout << "SPI ERROR: Couldn't open device!" << std::endl;
        return;
    } else{
        std::cout << "SPI: Opened device " << devPath  << "!" << std::endl;
        opened = true;
    }

    if(ioctl(device, SPI_IOC_WR_MODE, &spiMode) == -1){
        std::cout << "SPI: Couldn't set mode!" << std::endl;
        return;
    }

    if(ioctl(device, SPI_IOC_RD_MODE, &spiMode) == -1){
        std::cout << "SPI: Couldn't read mode!" << std::endl;
        return;
    } else{
        std::cout << "SPI: Mode: " << (int)spiMode << std::endl;
    }

    if(ioctl(device, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) == -1){
        std::cout << "SPI: Couldn't set bits per word!" << std::endl;
        return;
    }

    if(ioctl(device, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) == -1){
        std::cout << "SPI: Couldn't read bits per word!" << std::endl;
        return;
    } else{
        std::cout << "SPI: Bits per word: " << (int)bitsPerWord << std::endl;
    }

    if(ioctl(device, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1){
        std::cout << "SPI: Couldn't set speed!" << std::endl;
        return;
    }

    if(ioctl(device, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1){
        std::cout << "SPI: Couldn't read speed!" << std::endl;
        return;
    } else{
        std::cout << "SPI: Speed: " << speed / 1000 << " kHz" << std::endl;
    }



    initialized = true;


    std::cout << "SPI: Init end." << std::endl;
}


spi::~spi(){
    if(opened)
        if(!close(device))
            std::cout << "SPI: Couldn't close device " << devPath << "!" << std::endl;
}

bool spi::isInitialized(){
    return initialized;
}

int spi::exchangeData(const char* buffer){
    const char* written = buffer;
    uint size = strlen(buffer);

    char received[size];
    memset(received, 0,size);

    std::cout << "size: " << size << std::endl;
    struct spi_ioc_transfer xfer[size];

    for(uint i=0; i < size; i++){

                memset(&xfer[i], 0, sizeof(xfer[i]));
                xfer[i].tx_buf = (ulong)(buffer + i);
                xfer[i].rx_buf = (ulong)(received + i);
                xfer[i].len = sizeof(*(buffer+i));
                xfer[i].bits_per_word = bitsPerWord;
                xfer[i].cs_change = 1;
                xfer[i].delay_usecs = 0;
                xfer[i].speed_hz = speed;
                xfer[i].pad = 0;
    }

    int status = ioctl(device, SPI_IOC_MESSAGE(size), &xfer);
    if(status < 1){
        std::cout << "SPI: Couldn't write message: " << status << std::endl;
        std::cout << "Error: " << strerror(errno) << std::endl;
    } else
        std::cout << "SPI: Received " << received << " | " << status << std::endl;

    return status;
}
