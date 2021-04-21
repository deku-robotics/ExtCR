 /**
  * @brief	Uart class definition providing an interface to serial /Uart / RS232 devices
  *
  * @file 	faulhaber.h
  * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
  * @license please see GIT repository license declaration

  * @date	January 2018
  */

#ifndef UART_H
#define UART_H

//Boost
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/utility.hpp"
#include "boost/function.hpp"
#include "boost/shared_array.hpp"
#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"
#include <thread>

#include <ros/ros.h>

class UartImpl;


class Uart : private boost::noncopyable
{
public:
    Uart();

    Uart(const std::string& devname, u_int32_t baud_rate,
           boost::asio::serial_port_base::parity opt_parity=
               boost::asio::serial_port_base::parity(
                   boost::asio::serial_port_base::parity::none),
           boost::asio::serial_port_base::character_size opt_csize=
               boost::asio::serial_port_base::character_size(8),
           boost::asio::serial_port_base::flow_control opt_flow=
               boost::asio::serial_port_base::flow_control(
                   boost::asio::serial_port_base::flow_control::none),
           boost::asio::serial_port_base::stop_bits opt_stop=
               boost::asio::serial_port_base::stop_bits(
                   boost::asio::serial_port_base::stop_bits::one));

    ~Uart();

    void open();

    void open(const std::string& devname, u_int32_t baud_rate,
            boost::asio::serial_port_base::parity opt_parity=
                boost::asio::serial_port_base::parity(
                    boost::asio::serial_port_base::parity::none),
            boost::asio::serial_port_base::character_size opt_csize=
                boost::asio::serial_port_base::character_size(8),
            boost::asio::serial_port_base::flow_control opt_flow=
                boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::none),
            boost::asio::serial_port_base::stop_bits opt_stop=
                boost::asio::serial_port_base::stop_bits(
                    boost::asio::serial_port_base::stop_bits::one));

    bool isOpen() const;

    bool errorStatus() const;

    void close();

    bool write(const char *data, size_t size);

    void write(const std::vector<char>& data);

    bool writeString(const std::string& s);

    void error();


    size_t writeSync(std::string toWrite, bool sendControlByte = false, bool readAfterWrite = false);

    size_t writeBinary(unsigned char *data, size_t size, bool sendControlByte = false, bool readAfterWrite = false);

    size_t readSync(std::string& buffer, uint bufferSize);

    size_t readBinary(char* buffer, uint bufferSize);

    static const int readBufferSize=512;

private:

    void doRead();

    void doClose();

    void doWrite();

     void writeEnd(const boost::system::error_code& error);

    void readEnd(const boost::system::error_code& error, size_t bytes_transferred);

protected:
     void setErrorStatus(bool e);



     boost::shared_ptr<UartImpl> uimpl;

     void setReadCallback(const
                boost::function<void (const char*, size_t)>& callback);

     void clearReadCallback();

};

class CallbackUart : public Uart
{
public:
    CallbackUart();

    CallbackUart(const std::string& devname, u_int32_t baud_rate,
                 boost::asio::serial_port_base::parity opt_parity=
                     boost::asio::serial_port_base::parity(
                         boost::asio::serial_port_base::parity::none),
                 boost::asio::serial_port_base::character_size opt_csize=
                     boost::asio::serial_port_base::character_size(8),
                 boost::asio::serial_port_base::flow_control opt_flow=
                     boost::asio::serial_port_base::flow_control(
                         boost::asio::serial_port_base::flow_control::none),
                 boost::asio::serial_port_base::stop_bits opt_stop=
                     boost::asio::serial_port_base::stop_bits(
                         boost::asio::serial_port_base::stop_bits::one));

    void setCallback(const
                     boost::function<void (const char*, size_t)>& callback);
    void clearCallback();

    virtual ~CallbackUart();

};

#endif
