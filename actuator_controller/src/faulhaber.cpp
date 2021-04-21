/**
 * @brief	This class manages the connection and communication
          with the Faulhaber Motions controllers
 *
 * @file 	faulhaber.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "faulhaber.h"

std::mutex Faulhaber::mx_cmd;

Faulhaber::Faulhaber(std::string name, int id, std::string dev_type, std::string dev_add, int transm, int encticks, std::shared_ptr<ActuatorState> actState) : name(name), id(id), dev_type(dev_type), dev_add(dev_add),
    transm(transm), encticks(encticks)
{
    this->actState = actState;

    try{
        this->uart = new CallbackUart(this->dev_add,115200);
        this->uart->setCallback(boost::bind(&Faulhaber::cbReceived,this,_1,_2));
        initialized = true;
    } catch(...){
        ROS_ERROR_STREAM("Couldn't open device " << dev_add << "!");
        initialized = false;
    }

    if(dev_type == "spi"){
        is_spi = true;
        ROS_INFO("SPI MOTOR");
    } else{
        is_spi = false;
        ROS_WARN("UART MOTOR");
    }

    if(initialized){
        ROS_INFO("Actuator initialized");

        this->readFinishf = false;
        this->acqDataf = false;

        ROS_INFO("Sending config");
        cmdRateTimer.rate(cmdRate);
        this->sendCmd("ANSW0");

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("POR",Pvel);

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("I",30);

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("PP",100);

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("PD",4);

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("SR",1);

        cmdRateTimer.rate(cmdRate);
        this->sendCmd("LCC",1400);
        cmdRateTimer.rate(cmdRate);
        this->sendCmd("LPC",2800);

        this->OpenBinInterface();

        std::cout << "INITIALIZED"  << std::endl;
    } else{
        ROS_ERROR_STREAM("Couldn't init actuator " << id << "!");
    }
}

Faulhaber::~Faulhaber()
{
    this->initialized = false;
}


bool Faulhaber::sendCmd(std::string cmd)
{
    if(!this->initialized)
        return false;

    cmdRateTimer.rate(cmdRate);
    cmd = cmd.append("\r");
    if(asyncMode)
        this->uart->writeString(cmd);
    else{
        if(is_spi)
            this->uart->writeSync(cmd,true);
        else
            this->uart->writeSync(cmd);
    }
    return true;
}


bool Faulhaber::sendCmd(std::string cmd, int param)
{
    if(!this->initialized)
        return false;

    cmdRateTimer.rate(cmdRate);
    cmd = cmd.append(boost::lexical_cast<std::string>(param) + "\r");

    if(asyncMode)
        this->uart->writeString(cmd);
    else{
        if(is_spi)
            this->uart->writeSync(cmd,true);
        else
            this->uart->writeSync(cmd);
    }
    return true;
}



int32_t Faulhaber::sendQueryCmd(std::string cmd)
{
    if(!this->initialized)
        return -199999999;

    lastSize = 0;

    cmd = cmd.append("\r");
    this->readDataAsync.clear();
    this->readDataAsyncFlag = true;
    cmdRateTimer.rate(cmdRate);
    this->uart->writeString(cmd);

    return -99999;
}



int32_t Faulhaber::sendSyncQueryCmd(string cmd){
    if(!this->initialized){
        ROS_ERROR_STREAM(id << "| NOT INITIALIZED!");
        return -199999999;
    }

    lastSize = 0;
    cmd = cmd.append("\r");
    cmdRateTimer.rate(cmdRate);
    if(is_spi)
        this->uart->writeSync(cmd, true, true);
    else
        this->uart->writeSync(cmd);

    return readSync();
}


int32_t Faulhaber::readSync(){
    nReadSync = 0;
    dataSync = -9999;

    readSyncBuffer = std::string(64, '\0');

    while(true)
    {
        nReadSync  = uart->readSync(readSyncBuffer, 64);
        if(nReadSync  > 0)
        {
            this->readSyncData += readSyncBuffer.substr(0, nReadSync );
            if((endIdx = this->readSyncData.find("\r\n")) != std::string::npos)
            {
                break;
            } else {
                continue;
            }
            lastSize = readSyncData.size();
        }
        usleep(10);
    }

    try
    {
        dataSync = std::stol(readSyncData);
    }   catch (std::exception& e)
    {
        std::cout << "readSyncData: " << readSyncData << std::endl;
        ROS_ERROR_STREAM(this->id << "| Standard exception: " << e.what() << " || " << endIdx);
    }

    this->readSyncData.clear();
    return dataSync;
}


size_t Faulhaber::readBinary(){
    size_t nReadBinary = 0;
    char buffer[7];

    uint tries = 0;

    while(tries < 5 && nReadBinary < 7){
        nReadBinary += uart->readBinary(buffer, sizeof(buffer));

        if(nReadBinary > 7)
            break;
        tries++;
        usleep(10);
    }

    if(nReadBinary < 7){
        ROS_ERROR_STREAM(id << "| Couldn't read enough bytes! (" << nReadBinary << ")");
        return nReadBinary;
    }

    if(tries > 10)
        ROS_WARN_STREAM(id << "| bin read tries = " << tries << "!");


    lastPos = (int32_t)( (buffer[3] << 24)
          + (buffer[2] << 16)
          + (buffer[1] << 8)
          + (buffer[0])
            );

    lastVel = (int16_t)( (buffer[5] << 8)  + buffer[4]);


    int8_t tDiff = buffer[6];

    actState->setPos(lastPos);
    actState->setVel(lastVel);
    actState->setTimeDiff(tDiff);

    return nReadBinary;
}

void Faulhaber::stopPosQueryLoop(){
    posQueryRunning = false;
    posQueryThread->join();
}

bool Faulhaber::isNewPosAvailable(){
    return newPosAvailable;
}

void Faulhaber::cbReceived(const char *data, unsigned int len)
{
    if(this->readDataAsyncFlag)
    {
        std::string tmp_data(data,data+len);
         this->readDataAsync += tmp_data;

        if((this->endIdx = this->readDataAsync.find("\r\n"/*, lastSize*/)) != std::string::npos){
            return;
        }
        lastSize = readDataAsync.size();
    }
}

bool Faulhaber::OpenBinInterface()
{
    if(!this->initialized)
        return false;
    //Open Binary interface to Faulhaber Controller (p.55)    }stty -F /dev/ttyUSB0 115200 raw -echo
    if(!this->sendCmd("BINSEND1"))
    {
        ROS_ERROR("opening binary interface failed!");
        return false;
    }
    else
    {
        ROS_ERROR("opening binary interface");
        this->bin_interface = true;
        this->setTraceSettings();
        ROS_ERROR("trace settings done");
        return true;
    }
}

bool Faulhaber::setTraceSettings()
{
    if(!this->initialized)
        return false;

    //Configure binary trace to position, velocity and timestamp (see faulhaber manual p.58)
    unsigned char mode1[] = {0xc8,0xc8} ;  //set mode 1 to position data
    unsigned char mode2[] = {0xcA,0x0}; // set mode 2 to velocity data

    if(this->bin_interface)
    {
        if(is_spi){
            cmdRateTimer.rate(cmdRate);
            this->uart->writeBinary(mode1,sizeof(mode1), true);
            cmdRateTimer.rate(cmdRate);
            this->uart->writeBinary(mode2,sizeof(mode2), true);
        } else{
            cmdRateTimer.rate(cmdRate);
            this->uart->writeBinary(mode1,sizeof(mode1));
            cmdRateTimer.rate(cmdRate);
            this->uart->writeBinary(mode2,sizeof(mode2));
        }

        return true;
    }
}

bool Faulhaber::sendTraceRequest()
{
    unsigned char trace[] = {0xc9}; //Binary query

    if(this->bin_interface)
    {
        cmdRateTimer.rate(cmdRate);
        if(is_spi)
            this->uart->writeBinary(trace,sizeof(trace), true, true);
        else
            this->uart->writeBinary(trace,sizeof(trace));
    } else
    {
        return false;
    }
}

bool Faulhaber::readTraceData(){
   if(bin_interface){
       readBinary();
       return true;
   } else
       return false;
}

void Faulhaber::CloseBinInterface()
{
    if(!this->initialized)
        return;

    if(!this->sendCmd("BINSEND0"))
        ROS_ERROR("closing binary interface failed!");
}

bool Faulhaber::setPosLimits(int llow, int lup)
{
    if(!this->initialized)
        return false;
    //Check data validity (Faulhaber manual p. 79)
    if(((llow < 0) && (abs(llow) < FAULHABER_MAX_POS_LIMIT)) && ((lup > 0) && (abs(llow) < FAULHABER_MAX_POS_LIMIT)))
    {
        if(!this->sendCmd("LL",llow))
        {
            ROS_ERROR("Cannot set position limits! Communications error!");
            return false;
        }
        if(!this->sendCmd("LL",lup))
        {
            ROS_ERROR("Cannot set position limits! Communications error!");
            return false;
        }
        return true;
    } else {
        ROS_ERROR("Wrong position limits, pleasy check validity");
        return false;
    }
}

bool Faulhaber::ActPosLimits(bool active)
{
    if(!this->initialized)
        return false;
    if(active)
    {
        if(!this->sendCmd("APL",1))
        {
            ROS_ERROR("Cannot activate position limits! Communications error!");
            return false;
        } else {
            ROS_WARN("Position limits activated!");
            return true;
        }
    } else {
        if(!this->sendCmd("APL",0))
        {
            ROS_ERROR("Cannot deactivate position limits! Communications error!");
            return false;
        } else {
            ROS_WARN("Position limits deactivated!");
            return true;
        }
    }
}

bool Faulhaber::enbl()
{
    if(!this->initialized)
        return false;
    //if query was sent then acknowdlege communications
    if(this->sendCmd("EN"))
    {

        ROS_INFO_STREAM("Device:" << this->getName() << "ID: " << this->getID());
        this->en_flag = true;
    }  else
    {
        ROS_ERROR("Could not enable drive!");
        this->en_flag = false;
        return false;
    }

    return true;
}

bool Faulhaber::disbl()
{
    if(!this->initialized)
        return false;
    //if query was sent then acknowdlege communications
    if(this->sendCmd("DI")){
        return true;
    }
    else{
        return false;
    }
}

bool Faulhaber::cmdPos(int32_t pos)
{
    if(!this->initialized)
        return false;
    //Check for mode of actuator, if true apply position control else false

    std::string data = "LA"; //Load Absolute Position
    data += boost::lexical_cast<std::string>(pos);
    data += "\r";

    this->sendCmd(data);

    std::string data2 = "M"; //Activate PositionControl and start moving to set position
    data2 += "\r";

    return this->sendCmd(data2);
}

bool Faulhaber::homeActuator()
{
    if(!this->initialized)
        return false;
    return this->sendCmd("HO");
}

bool Faulhaber::cmdVel(int16_t vel)
{
    if(!this->initialized)
        return false;

    //Check for mode of actuator, if true apply velocity control else false
    std::string data = "V";
    data += boost::lexical_cast<std::string>(vel);
    data += "\r";

    return this->sendCmd(data);
}

bool Faulhaber::setAcceleration(float acceleration)
{
    if(!this->initialized)
        return false;
    // Convert command from output to motor wave
    int32_t acceleration_motor = acceleration*this->transm;

    std::string data = "AC"; //Load acceleration 0 - 30 000
    data += boost::lexical_cast<std::string>(acceleration_motor);
    data += "\r";

    this->sendCmd(data);
    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    std::string data2 = "DEC"; //Load negative acceleration 0 - 30 000
    data2 += boost::lexical_cast<std::string>(acceleration_motor);
    data2 += "\r";

    return this->sendCmd(data2);
}

bool Faulhaber::setMaxVel(float maxVel)
{
    // Convert command from output to motor wave
    int32_t maxVel_motor = maxVel*this->transm;

    std::string data = "SP"; //Load Maximum speed 0 - 30 000
    data += boost::lexical_cast<std::string>(maxVel_motor);
    data += "\r";

    return this->sendCmd(data);
}

bool Faulhaber::setTransmission(int trans)
{
    if(!this->initialized)
        return false;
    this->transm = trans;
    return true;
}

int Faulhaber::getTransmission()
{
    return this->transm;
}

int Faulhaber::getEncTicks(){
    return this->encticks;
}

void Faulhaber::getVelQuery()
{
    lastCmdType = true;
    boost::lexical_cast<int16_t>(this->sendQueryCmd("GN"));
}

int32_t Faulhaber::getLastPos()
{
    newPosAvailable = false;
    return lastPos;
}

int32_t Faulhaber::getLastVel(){
    return lastVel;
}


void Faulhaber::getPosQuery(){

    mx_posvel.lock();
    lastCmdType = false;
    boost::lexical_cast<int32_t>(this->sendQueryCmd("POS"));

}


int32_t Faulhaber::getPos(bool writeOnly){


    if(writeOnly){
        this->sendCmd("POS");
        return 0;
    }
    lastPos = this->sendSyncQueryCmd("POS");
    return lastPos;
}

int32_t Faulhaber::getVel(bool writeOnly){
    if(writeOnly){
        this->sendCmd("GN");
        return 0;
    }
    lastVel = this->sendSyncQueryCmd("GN");
    return lastVel;
}

int Faulhaber::getID()
{
    return this->id;
}

std::string Faulhaber::getAddr()
{
    return this->dev_add;
}

std::string Faulhaber::getName()
{
    return this->name;
}

std::string Faulhaber::getType()
{
    return this->dev_type;
}


ros::Duration Faulhaber::getLastPosDeltaT(){
    return lastPosDeltaT;
}

ros::Time Faulhaber::getLastPosQueryTime(){
    return lastPosQueryTime;
}

bool Faulhaber::isInitialized(){
    return initialized;
}

int32_t Faulhaber::getZeroPos(){
    return zeroPos * transm * encticks;
}

bool Faulhaber::setPvel(uint val){
    if(!initialized)
        return false;

    sendCmd("POR",val);
    this->Pvel = val;
    return true;
}

uint Faulhaber::getPvel(){
    return this->Pvel;
}
