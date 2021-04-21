/**
 * @brief	Low level uart implementation (using Boost) for BeagleBoneBlack and Faulhaber controllers
 *
 * @file 	uart.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "uart.h"

class UartImpl: private boost::noncopyable
{
public:
    UartImpl() : io(), port(io)/*, backgroundThread()*/, open(false),
    error(false) {}

    boost::asio::io_service io;
    boost::asio::serial_port port;
    bool open;
    bool error;
    mutable boost::mutex errorMutex;

    std::vector<char> writeQueue;
    boost::shared_array<char> writeBuffer;
    size_t writeBufferSize;
    boost::mutex writeQueueMutex;
    char readBuffer[Uart::readBufferSize];

    boost::function<void (const char*,size_t)> callback;

};

Uart::Uart() : uimpl(new UartImpl){}

Uart::Uart(const std::string &devname, u_int32_t baud_rate, boost::asio::serial_port_base::parity opt_parity, boost::asio::serial_port_base::character_size opt_csize, boost::asio::serial_port_base::flow_control opt_flow, boost::asio::serial_port_base::stop_bits opt_stop) : uimpl(new UartImpl)
{
     open(devname,baud_rate,opt_parity,opt_csize,opt_flow,opt_stop);
}

void Uart::open() {}

void Uart::open(const std::string& devname, u_int32_t baud_rate,
        boost::asio::serial_port_base::parity opt_parity,
        boost::asio::serial_port_base::character_size opt_csize,
        boost::asio::serial_port_base::flow_control opt_flow,
        boost::asio::serial_port_base::stop_bits opt_stop)
{
    if(isOpen()) close();

    setErrorStatus(true);//If an exception is thrown, error_ remains true
    uimpl->port.open(devname);
    uimpl->port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    std::cout << "Opening port with baud rate " << baud_rate << std::endl;
    uimpl->port.set_option(opt_parity);
    uimpl->port.set_option(opt_csize);
    uimpl->port.set_option(opt_flow);
    uimpl->port.set_option(opt_stop);

    setErrorStatus(false);//If we get here, no error
    uimpl->open=true; //Port is now open
}

bool Uart::isOpen() const
{
    return uimpl->open;
}

bool Uart::errorStatus() const
{
    boost::lock_guard<boost::mutex> l(uimpl->errorMutex);
    return uimpl->error;
}

void Uart::close()
{
    if(!isOpen()) return;

    uimpl->open=false;
    uimpl->io.post(boost::bind(&Uart::doClose, this));
//    uimpl->backgroundThread.join();
    uimpl->io.reset();
    if(errorStatus())
    {
        throw(boost::system::system_error(boost::system::error_code(),
                "Error while closing the device"));
    }
}

void Uart::doClose()
{
    boost::system::error_code ec;
    uimpl->port.cancel(ec);
    if(ec) setErrorStatus(true);
    uimpl->port.close(ec);
    if(ec) setErrorStatus(true);
}

bool Uart::write(const char *data, size_t size)
{
    {
        boost::lock_guard<boost::mutex> l(uimpl->writeQueueMutex);
        uimpl->writeQueue.insert(uimpl->writeQueue.end(),data,data+size);
    }
    uimpl->io.post(boost::bind(&Uart::doWrite, this));

    return true;
}

void Uart::write(const std::vector<char>& data)
{
    {
        boost::lock_guard<boost::mutex> l(uimpl->writeQueueMutex);
        uimpl->writeQueue.insert(uimpl->writeQueue.end(),data.begin(),
                data.end());
    }
    uimpl->io.post(boost::bind(&Uart::doWrite, this));
}

bool Uart::writeString(const std::string& s)
{
    {
        boost::lock_guard<boost::mutex> l(uimpl->writeQueueMutex);
        uimpl->writeQueue.insert(uimpl->writeQueue.end(),s.begin(),s.end());
    }
    uimpl->io.post(boost::bind(&Uart::doWrite, this));

    return true;
}

Uart::~Uart()
{
    if(isOpen())
    {
        try {
            close();
        } catch(...)
        {
            //Don't throw from a destructor
        }
    }
}


void Uart::doRead()
{
    uimpl->port.async_read_some(boost::asio::buffer(uimpl->readBuffer,readBufferSize),
            boost::bind(&Uart::readEnd,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void Uart::readEnd(const boost::system::error_code& error,
        size_t bytes_transferred)
{
    std::cout << "readEnd" << std::endl;
    if(error)
    {
        //In this case it is not a real error, so ignore
        std::cout << "UART ERROR: " << error.message() << std::endl;
        if(isOpen())
        {
            doClose();
            setErrorStatus(true);
        }
    } else {
        if(uimpl->callback)
            uimpl->callback(uimpl->readBuffer, bytes_transferred);
        doRead();
    }
}

void Uart::doWrite()
{
    //If a write operation is already in progress, do nothing
    if(uimpl->writeBuffer==0)
    {
        boost::lock_guard<boost::mutex> l(uimpl->writeQueueMutex);
        uimpl->writeBufferSize=uimpl->writeQueue.size();
        uimpl->writeBuffer.reset(new char[uimpl->writeQueue.size()]);
        copy(uimpl->writeQueue.begin(),uimpl->writeQueue.end(),
                uimpl->writeBuffer.get());
        uimpl->writeQueue.clear();
        boost::asio::async_write(uimpl->port,boost::asio::buffer(uimpl->writeBuffer.get(),
                uimpl->writeBufferSize),
                boost::bind(&Uart::writeEnd, this, boost::asio::placeholders::error));
    }
}

size_t Uart::writeSync(std::string toWrite, bool sendControlByte, bool readAfterWrite){
    if(sendControlByte){
        char c;
        if(readAfterWrite)
            c = 1;
        else
            c = 0;
        toWrite += c;
    }
    return boost::asio::write(uimpl->port, boost::asio::buffer(&toWrite[0], toWrite.size()));

}

size_t Uart::writeBinary(unsigned char* data, size_t size, bool sendControlByte, bool readAfterWrite){
    if(sendControlByte){
        unsigned char appended[size + 1];
        unsigned char c;
        if(readAfterWrite)
            c = 1;
        else
            c = 0;
        memcpy(appended,data,size);
        memcpy(appended+size,&c,1);
        return boost::asio::write(uimpl->port, boost::asio::buffer(appended, size+1));
    } else{
        return boost::asio::write(uimpl->port, boost::asio::buffer(data, size));
    }
}


size_t Uart::readSync(std::string &buffer, uint bufferSize){
    return uimpl->port.read_some(boost::asio::buffer(&buffer[0], bufferSize));
}

size_t Uart::readBinary(char* buffer, uint bufferSize){
    return uimpl->port.read_some(boost::asio::buffer(buffer, bufferSize));
}



void Uart::writeEnd(const boost::system::error_code& error)
{
    if(!error)
    {
        boost::lock_guard<boost::mutex> l(uimpl->writeQueueMutex);
        if(uimpl->writeQueue.empty())
        {
            uimpl->writeBuffer.reset();
            uimpl->writeBufferSize=0;

            return;
        }
        uimpl->writeBufferSize=uimpl->writeQueue.size();
        uimpl->writeBuffer.reset(new char[uimpl->writeQueue.size()]);
        copy(uimpl->writeQueue.begin(),uimpl->writeQueue.end(),
                uimpl->writeBuffer.get());
        uimpl->writeQueue.clear();
        boost::asio::async_write(uimpl->port,boost::asio::buffer(uimpl->writeBuffer.get(),
                uimpl->writeBufferSize),
                boost::bind(&Uart::writeEnd, this, boost::asio::placeholders::error));
    } else {
        setErrorStatus(true);
        doClose();
    }
}

void Uart::setErrorStatus(bool e)
{
    boost::lock_guard<boost::mutex> l(uimpl->errorMutex);
    uimpl->error=e;
}

void Uart::setReadCallback(const boost::function<void (const char*, size_t)>& callback)
{
    uimpl->callback=callback;
}

void Uart::clearReadCallback()
{
    uimpl->callback.clear();
}

void Uart::error() {}
