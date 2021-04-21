/**
 * @brief	This class manages the UART callbacks of the RS232 connection
 *
 * @file 	callbackUart.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "callbackUart.hpp"

CallbackUart::CallbackUart() : Uart()
{

}

CallbackUart::CallbackUart(const std::string &devname, u_int32_t baud_rate, boost::asio::serial_port_base::parity opt_parity,
                           boost::asio::serial_port_base::character_size opt_csize,
                           boost::asio::serial_port_base::flow_control opt_flow,
                           boost::asio::serial_port_base::stop_bits opt_stop) :
    Uart(devname,baud_rate,opt_parity, opt_csize, opt_flow, opt_stop)
{

}

void CallbackUart::setCallback(const boost::function<void (const char *, size_t)> &callback)
{
    setReadCallback(callback);
}

void CallbackUart::clearCallback()
{
    clearReadCallback();
}

CallbackUart::~CallbackUart()
{
    clearReadCallback();
}
