/**
 * @brief	A customised timer for debugging purposes
 *
 * @file 	Timer.cpp
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#include "Timer.hpp"

using namespace std;

Timer::Timer(function<void()> f, double timeout)
{
  this->f = f;
  this->timeout = timeout;
}

Timer::Timer(){

}

void Timer::start()
{
    cout << "Creating new timer, timeout: " << timeout << " seconds" << endl;
    startTime = chrono::system_clock::now();
  timerThread = new thread(bind(&Timer::startThread,this));
}

void Timer::startThread()
{
    mx_thread.lock();
  while(active) {
        double passedTime = chrono::duration<double>(chrono::system_clock::now() - startTime).count();
        if(passedTime > timeout && active) {
            mx_thread.unlock();
            try {
                new thread(f);
      } catch(const exception& e) {
        cout << e.what() << endl;
      } catch(...){
        cout << "FUUUCK" << endl;
      }
      active = false;
            mx_thread.lock();
      timerThread = NULL;
            mx_thread.unlock();
        } else
            usleep(100000);
  }
    mx_thread.unlock();
}

void Timer::reset()
{
    startTime = chrono::system_clock::now();
  if(!active) {
    active = true;
    timerThread = new thread(bind(&Timer::startThread,this));
  }
}

void Timer::stop()
{
    if(!active)
        return;
    active = false;

    mx_thread.lock();
  try {
        if(timerThread != NULL){
            timerThread->join();
            delete timerThread;
            timerThread = NULL;
        }
  } catch(const std::exception& e) {
    cout << e.what();
  }
    mx_thread.unlock();
}

void Timer::rate(double rate){
    if(rate_firstPass){
        rate_firstPass = false;
    } else{
        rate_lastPassedTime = chrono::duration<double>(chrono::system_clock::now() - rate_lastPass).count();
        if(1.0/rate > rate_lastPassedTime){
            this_thread::sleep_for(chrono::duration<double>(1.0/rate - rate_lastPassedTime));
        }
    }
    rate_lastPass = chrono::system_clock::now();
}

double Timer::getLastPassedTime(){
    return rate_lastPassedTime;
}

Timer::~Timer()
{
    stop();
}
