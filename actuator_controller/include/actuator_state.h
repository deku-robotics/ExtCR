/**
 * @brief	Header for actuator state definition
 *
 * @file 	actuator_state.h
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#ifndef ACTUATOR_STATE_H
#define ACTUATOR_STATE_H

#include "string"
#include "ros/time.h"

#include <mutex>

class ActuatorState
{

public:
ActuatorState() : name(),pos(0),vel(0),fault(false) {}

ActuatorState(const std::string& n, int idt, const int32_t p = 0, const int16_t v = 0, const int16_t a = 0) : name(n), id(idt), pos(p), vel(v), accel(a), fault(false) {}

void setVel(const int16_t v) { vel = v; velUpdated = true;}
int16_t getVel() {velUpdated = false; return vel;}

void setPos(const int32_t p) {pos = p; posUpdated = true;}
int32_t getPos(){posUpdated = false; return pos;}

void setAccel(const int16_t a) {accel = a; accelUpdated = true;}
int16_t getAccel() {accelUpdated = false; return accel;}

void setTimeDiff(const int8_t t){last_tDiff = t;}
int8_t getTimeDiff() {return last_tDiff;}

int getId() {return id;}
std::string getName() {return name;}

bool getPosUpdateState() {return posUpdated;}
bool getVelUpdateState() {return velUpdated;}

void lock() {mx_state.lock();}
void unlock() {mx_state.unlock();}




private:

std::string name;
int id;
int8_t last_tDiff = 0;
int32_t pos;
int16_t vel, accel;
bool enabl;
bool fault;

bool posUpdated = false;
bool velUpdated = false;
bool accelUpdated = false;

std::mutex mx_state;


};

#endif
