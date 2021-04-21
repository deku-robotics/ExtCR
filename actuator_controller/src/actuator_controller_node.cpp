#include "actuator_controller_node.h"


ActuatorController::ActuatorController(int argc, char *argv[], const char *node_name)
{
    this->nh = new ros::NodeHandle;
    this->nh_local = new ros::NodeHandle("~");

    this->hard_limit = false;


    ROS_INFO("CoRoLas Actuator Controller initializing... ");

    // Set-up stack of actuators by iterating through parameters and consecutive set-up of actuators
    this->actuators.clear();
    this->act_states.clear();

    // Obtain data from parameter server
    this->getROSParameter();

    // Init the acutator controller connection
    this->initActuators();

    //set-up ROS topics and services
    this->vel_sub = nh->subscribe(ros::this_node::getName()+this->velCmdTopicName,200,&ActuatorController::velCmdCallback,this, ros::TransportHints().udp());
    this->pos_sub = nh->subscribe(ros::this_node::getName()+this->posCmdTopicName,200,&ActuatorController::posCmdCallback,this, ros::TransportHints().udp());
    this->rate_sub = nh->subscribe<std_msgs::Int16>("/main_controller/rate",10, &ActuatorController::rateCB,this);
    this->pub_actState = nh->advertise<actuator_controller::actuator_state>(ros::this_node::getName()+this->stateTopicName,1);
    this->pub_status = nh->advertise<diagnostic_msgs::DiagnosticStatus>(ros::this_node::getName() + "/status",1);

    //Services
    this->home_srv = this->nh->advertiseService(ros::this_node::getName()+"/home",&ActuatorController::srvCalibrate,this);
    this->get_out_of_limits_srv=this->nh->advertiseService(ros::this_node::getName()+"/get_out_of_limits", &ActuatorController::srvGetOutOfLimits, this);
    this->goToZero_srv = this->nh->advertiseService(ros::this_node::getName() + "/go_to_zero", &ActuatorController::srvGoToZero, this);
    this->maintenance_srv = this->nh->advertiseService(ros::this_node::getName() + "/maintenance", &ActuatorController::srvMaintenance, this);
    this->toggleLimits_srv = this->nh->advertiseService(ros::this_node::getName() + "/toggleLimits", &ActuatorController::srvToggleLimits, this);
    this->srv_setPvel = this->nh->advertiseService(ros::this_node::getName() + "/setPvel", &ActuatorController::srvSetPVel, this);
    this->srv_setIvel = this->nh->advertiseService(ros::this_node::getName() + "/setIvel", &ActuatorController::srvSetIVel, this);

    ROS_INFO("CoRoLas Actuator Controller running... ");

    this->checker_=new limit_check();

    this->actuator_mode=MODES::POSITION_MODE;
    this->last_command_time = std::chrono::system_clock::now();

    std::cout << "Current Mode is " << int(this->actuator_mode)  << std::endl;

    TEST_lastPosCmd.position_cmd = {-1,-1,-1,-1,-1,-1};

    this->Init();
}

void ActuatorController::getROSParameter()
{
    this->nh_local->param<std::string>("cmdTopicName",this->cmdTopicName,"/cmd");
    this->nh_local->param<std::string>("velCmdTopicName",this->velCmdTopicName,"/vel_cmd");
    this->nh_local->param<std::string>("posCmdTopicName",this->posCmdTopicName,"/pos_cmd");
    this->nh_local->param<std::string>("stateTopicName",this->stateTopicName,"/state");
    this->nh_local->param<bool>("MaintenanceMode",this->MaintenanceMode,false);
}

void ActuatorController::velCmdCallback(const actuator_controller::actuator_vel_cmd &msg){

    this->actuator_mode=MODES::VELOCITY_MODE;

    for(uint idx = 0; idx < actuators.size(); idx++)
    {
        actuators[idx]->cmdVel(msg.velocity_cmd[idx]);
    }


    TEST_curTarget = msg.velocity_cmd;
    lastCommandStamp = msg.stamp;

}

void ActuatorController::posCmdCallback(const actuator_controller::actuator_pos_cmd &msg){
    this->actuator_mode=MODES::POSITION_MODE;
    TEST_lastPosCmd = msg;

    for(uint idx = 0; idx < actuators.size(); idx++)
    {
        actuators[idx]->cmdPos(msg.position_cmd[idx]);
    }

    lastCommandStamp = msg.stamp;
}



void ActuatorController::pubStateData()
{

    actuator_controller::actuator_state state_msg;

    for(std::shared_ptr<ActuatorState> state : act_states)
    {
       state_msg.positions.push_back(state->getPos());
       state_msg.velocities.push_back(state->getVel());
       state_msg.timeDiffs.push_back(state->getTimeDiff());
    }

    state_msg.TEST_curTarget = TEST_curTarget;
    state_msg.stamp = lastCommandStamp;
    pub_actState.publish(state_msg);
}

void ActuatorController::maintenancePosition(){
    publishStatus(Status::PROCESSING, "Enabling maintenance mode");
    this->last_command_time = std::chrono::system_clock::now();
    ActuatorController::MODES prevMode = this->actuator_mode;
    this->actuator_mode=MODES::POSITION_MODE;
    for(uint idx = 0; idx < actuators.size(); idx++)
    {
        actuators[idx]->cmdPos(3);
    }
    this->last_command_time = std::chrono::system_clock::now();

    this->MaintenanceMode = true;
    this->calibrateActuators();
    this->MaintenanceMode = false;

    this->actuator_mode = prevMode;
    publishStatus(Status::PROCESSING, "Manipulator in maintenance mode");
}


void ActuatorController::centerActuators(bool outOfSingularity, bool ignoreLimits){
    ROS_INFO("Centering manipulator");
    ros::Rate sleepRate(100);
    publishStatus(Status::PROCESSING, "Centering manipulator");


    ActuatorController::MODES prevMode = this->actuator_mode;
    this->actuator_mode=MODES::POSITION_MODE;
    this->timeoutCheckEnabled = false;
    if(ignoreLimits)
        limitCheckEnabled = false;

    TEST_deactAdaptiveP = true;
    for(uint idx = 0; idx < actuators.size(); idx++)
    {
        if((idx == 0 || idx == 3) && outOfSingularity)
            actuators[idx]->cmdPos(actuators[idx]->getZeroPos() + 100000);
        else
            actuators[idx]->cmdPos(actuators[idx]->getZeroPos());
    }
    this->centeringStart = std::chrono::system_clock::now();
    this->last_command_time = std::chrono::system_clock::now();

    bool centerPosReached = false;
    while(!centerPosReached){
        if(std::chrono::duration<double>(std::chrono::system_clock::now() - centeringStart).count() > 10){
            ROS_WARN("Centering failed (timeout)!");
            publishStatus(Status::ERROR, "Centering failed: Timeout!");
            TEST_deactAdaptiveP = false;
            if(!calibrating)
                limitCheckEnabled = true;
            return;
        }

        centerPosReached = true;
        for(uint idx = 0; idx < actuators.size(); idx++)
        {
            double pos = act_states[idx]->getPos();
            if((idx == 0 || idx == 3) && outOfSingularity){
                if(abs(pos - (actuators[idx]->getZeroPos() + 100000)) > 5){
                    centerPosReached = false;
                }
            } else{
                if(abs(pos - actuators[idx]->getZeroPos()) > 5){
                    centerPosReached = false;
                }
            }
        }
        sleepRate.sleep();
    }
    publishStatus(Status::PROCESSING, "Manipulator centered.");
    TEST_deactAdaptiveP = false;

    if(!calibrating)
        limitCheckEnabled = true;

    this->timeoutCheckEnabled = true;
    this->actuator_mode = prevMode;
    ROS_INFO("Finished centering actuators");
}


bool ActuatorController::initActuators()
{
    // Read configuration parameters from Parameterserver
    ROS_INFO("Reading parameter from Server and init Actuators ....");

    XmlRpc::XmlRpcValue act_list;

    if(!this->nh_local->getParam(ros::this_node::getName() + "/actuators", act_list)) {
        ROS_WARN("Cannot read actuator parameters!");
        return 0;
    }

    if(act_list.getType() != XmlRpc::XmlRpcValue::TypeStruct){
        ROS_WARN("Actuator definition in yaml wrong!");
    }

    //Iterate over each n-th actuator parameter definition
    for(XmlRpc::XmlRpcValue::iterator it=act_list.begin(); it!=act_list.end(); ++it)
    {
        //Check for parameter dev_type and setup instance
        if(act_list[it->first]["type"] == "uart" || act_list[it->first]["type"] == "spi")
        {
            std::shared_ptr<ActuatorState> act_state;
            act_state.reset(new ActuatorState(act_list[it->first]["name"],act_list[it->first]["id"]));

                std::shared_ptr<Faulhaber> actuator;
                actuator.reset(new Faulhaber(static_cast<std::string>(act_list[it->first]["name"]), act_list[it->first]["id"],
                        static_cast<std::string>(act_list[it->first]["type"]), static_cast<std::string>(act_list[it->first]["addr"]), act_list[it->first]["transmission"],
                        act_list[it->first]["enc_ticks"], act_state));

                ROS_INFO("------------------------");
                ROS_INFO("Actuator name: %s", actuator->getName().c_str());
                ROS_INFO("Actuator id: %i", actuator->getID());
                ROS_INFO("Actuator Type: %s", actuator->getType().c_str());
                ROS_INFO("Actuator Addr: %s", actuator->getAddr().c_str());
                ROS_INFO_STREAM("Actuator gearing: " << actuator->getTransmission());
                ROS_INFO("------------------------");

                if(actuator->isInitialized()){
                    this->actuators.push_back(actuator);
                    this->act_states.push_back(act_state);

                    actuator->cmdVel(0);
                } else{
                    ROS_ERROR_STREAM("Couldn't init actuator " << actuator->getID());
                }
        } else
        {
            ROS_WARN("No valid device type !");
        }
    }
    ROS_INFO_STREAM((int) this->actuators.size() << " actuator/s is/are set-up!");

    return false;
}


void ActuatorController::run(){

    Timer rateTimer;
    while(!shuttingDown)
    {

        for(std::shared_ptr<Faulhaber> act : actuators){
            act->sendTraceRequest();
        }

        for(std::shared_ptr<Faulhaber> act : actuators){
            act->readTraceData();
        }

        this->pubStateData();

        if(this->checkActuatorsLimits())
        {
            if(!limitReached){
                publishStatus(Status::ERROR, "Joint limit reached!");
                ROS_ERROR("Limit reached, all actuators disabled");
                limitReached = true;
                for(std::shared_ptr<Faulhaber> act : actuators)
                {
                    act->disbl();
                }
            }
        } else
            limitReached = false;

        ros::spinOnce();
        rateTimer.rate(robotRate);
    }
}



bool ActuatorController::Init(){

    for(std::shared_ptr<Faulhaber> act : actuators){
        //Write new data to actuators [rad/s]
        // float maxVelocity= 8000.0/66.0;
        float maxVelocity = 30000/66.0;
        act->setMaxVel(maxVelocity);
        // float acceleration = 17356.0/66.0;
        float acceleration = 30000/66.0;
        act->setAcceleration(acceleration);
    }
    ROS_INFO("Velocities and accelerations are set");

    return false;
}


void ActuatorController::shutdown()
{
    for(std::shared_ptr<Faulhaber> f : actuators)
    {
        f->stopPosQueryLoop();
        f->cmdVel(0);
        f->disbl();
    }
}

void ActuatorController::softShutdown()
{
    std::vector<std::shared_ptr<Faulhaber>>::iterator iter_faul;

    for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
    {
        (*iter_faul)->cmdVel(0.0);
    }
}

bool ActuatorController::calibrateActuators()
{
    calibrating = true;
    limitCheckEnabled = false;

    if(!MaintenanceMode)
        publishStatus(Status::PROCESSING, "Calibrating...");

    std::vector<std::shared_ptr<Faulhaber>>::iterator iter_faul;
    std::vector<std::shared_ptr<ActuatorState>>::iterator iter_state;

    //Write new CMD from MSG to actuators
    for(uint i=0; i < 6; i++)
    {
        float calibrationspeed = -4.0 * actuators[i]->getTransmission() * 60 / (2*M_PI); // choose low velocity for safety reasons

        //Write new data to actuators
        actuators[i]->enbl();
        usleep(10000);
        actuators[i]->cmdVel(calibrationspeed);
    }
    iter_faul = this->actuators.begin();

    iter_state = this->act_states.begin();

    bool calibrated[6] = { false, false , false , false , false, false };
    while(!calibrated[0] || !calibrated[1] || !calibrated[2] || !calibrated[3] || !calibrated[4] || !calibrated[5])
    {

        for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
        {
            if(!calibrated[std::distance(this->actuators.begin(),iter_faul)])
            {
                calibrated[std::distance(this->actuators.begin(),iter_faul)]=checker_->is_limit_reached(std::distance(this->actuators.begin(),iter_faul));

                if(calibrated[std::distance(this->actuators.begin(),iter_faul)])
                {
                    //Stop
                    (*iter_faul)->cmdVel(0.0);

                    ROS_INFO_STREAM("Previous position is : " << (*iter_faul)->getLastPos());
                    usleep(10000);
                    (*iter_faul)->homeActuator();
                    usleep(10000);
                    ROS_INFO_STREAM("New position is : " << (*iter_faul)->getLastPos());
                    usleep(10000);
                    ROS_INFO_STREAM("Actuator " << std::distance(this->actuators.begin(),iter_faul) << " is now calibrated!");
                }
            }

        }
        usleep(1000);
    }
    if(this->MaintenanceMode)
    {
        ROS_INFO("MAINTENANCE_MODE");
        limitCheckEnabled = true;
        calibrating = false;
        return true;
    }
    if(calibrated[0] && calibrated[1] && calibrated[2] && calibrated[3] && calibrated[4] && calibrated[5])
    {
        centerActuators(false, true);
        limitCheckEnabled = true;
        calibrating = false;
        return true;
    }
    if(!MaintenanceMode)
        publishStatus(Status::PROCESSING, "Calibration failed!");

    limitCheckEnabled = true;
    calibrating = false;
    return false;
}

bool ActuatorController::checkActuatorsLimits()
{
    if(!limitCheckEnabled)
        return false;

    std::vector<std::shared_ptr<Faulhaber>>::iterator iter_faul;

    bool onLimit=false;

    for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
    {
        if(checker_->is_limit_reached(std::distance(this->actuators.begin(),iter_faul))){
            onLimit=true;
        }
        //Upper Limits
        if(checker_->is_limit_reached(std::distance(this->actuators.begin(),iter_faul), false)){
            onLimit=true;
        }
    }
    return onLimit;
}

bool ActuatorController::checkForTimeout()
{
    std::vector<std::shared_ptr<Faulhaber>>::iterator iter_faul;

    if(this->actuator_mode==MODES::VELOCITY_MODE && (std::chrono::duration<double>(std::chrono::system_clock::now()-this->last_command_time).count()) > timeout)
    {
        this->actuator_mode=MODES::VELOCITY_MODE;
        for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
        {
            //Write new data to actuators [rad/s]
            (*iter_faul)->cmdVel(0.0);
        }
        return true;
    }
    return false;
}


void ActuatorController::getOutOfLimits()
{
    std::vector<std::shared_ptr<Faulhaber>>::iterator iter_faul;
    std::vector<std::shared_ptr<ActuatorState>>::iterator iter_state = act_states.begin();

    uint tries = 0;

    while(checkActuatorsLimits() && tries < 10){
        iter_state = act_states.begin();
        for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
        {
            if(checker_->is_limit_reached(std::distance(this->actuators.begin(),iter_faul)))
            {
                ROS_INFO_STREAM("Limit : Actuator " <<  std::distance(this->actuators.begin(),iter_faul) << ", DOWN, going up");

                (*iter_faul)->enbl();
                usleep(10000);
                (*iter_faul)->cmdPos( (*iter_state)->getPos() + 25000);
            }
            //Upper Limits
            else if(checker_->is_limit_reached(std::distance(this->actuators.begin(),iter_faul), false))
            {
                ROS_INFO_STREAM("Limit : Actuator " <<  std::distance(this->actuators.begin(),iter_faul) << ", UP");
                (*iter_faul)->enbl();
                usleep(10000);
                (*iter_faul)->cmdPos( (*iter_state)->getPos() - 25000);
            }
        iter_state++;
        }
        tries++;
    }

    if(tries >105){
        publishStatus(Status::ERROR, "Couldn't get out of limits!");
        return;
    }

    sleep(1);
    for(iter_faul = this->actuators.begin(); iter_faul != this->actuators.end(); iter_faul++)
    {
        (*iter_faul)->enbl();
        ROS_WARN("Device enabled!");
        publishStatus(Status::PROCESSING, "Got out of limits, enabling actuators!");
    }
}

void ActuatorController::publishStatus(Status statusIdx, std::string additionalInfo){
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name = "ActuatorController";
    if(statusIdx == Status::IDLE){
        diag.level = 0;
        diag.message = "IDLE";
    }  else if(statusIdx == Status::PROCESSING){
        diag.level = 0;
        diag.message = "PROC";
        diagnostic_msgs::KeyValue val;
        val.key = "Info";
        val.value = additionalInfo;
        diag.values.push_back(val);
    } else if(statusIdx == Status::ERROR){
        diag.level = 2;
        diag.message = "ERRO";
        diagnostic_msgs::KeyValue val;
        val.key = "ErrorMessage";
        val.value = additionalInfo;
        diag.values.push_back(val);

    } else if(statusIdx == Status::ABORTED){
        diag.level = 1;
        diag.message = "ABRT";
    }
    pub_status.publish(diag);
}

void ActuatorController::rateCB(const std_msgs::Int16ConstPtr &rate){
    this->robotRate = rate->data;
}

ActuatorController::~ActuatorController()
{
    double meanTimes = TEST_times / TEST_nTimes;
    cout << "Times: " << meanTimes << endl;
    cout << "Peak: " << TEST_peakTime << endl;
    shuttingDown = true;

    //Stop all drives and cleanup
    this->shutdown();

    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool ActuatorController::srvSetIVel(actuator_controller::setInt::Request &req, actuator_controller::setInt::Response &res){
    bool success = true;
    for(shared_ptr<Faulhaber> act : actuators){
        if(!act->sendCmd("I",req.data))
            success = false;
    }
    return success;
}

bool ActuatorController::srvSetPVel(actuator_controller::setInt::Request &req, actuator_controller::setInt::Response &res){
    bool success = true;
    for(shared_ptr<Faulhaber> act : actuators){

        if(act->getID() == 0)
            act->sendCmd("POR",3.0);
        else {
            if(!act->sendCmd("POR",req.data))
                success = false;
        }
    }
    return success;
}

bool ActuatorController::srvGetOutOfLimits(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    new std::thread(&ActuatorController::getOutOfLimits, this);
}

bool ActuatorController::srvCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    new std::thread(&ActuatorController::calibrateActuators, this);
}

bool ActuatorController::srvGoToZero(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // If req.data is true, center manipulators in a positon slightly next to the center singularity
    // Otherwise center it so it's completely straight, but within the singularity
    new std::thread(&ActuatorController::centerActuators, this, req.data, false);
}

bool ActuatorController::srvMaintenance(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    new std::thread(&ActuatorController::maintenancePosition, this);
}

bool ActuatorController::srvToggleLimits(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    limitCheckEnabled = req.data;
}
