/**
 * @brief	Customised timer

 * @file 	faulhaber.h
 * @author	Dennis Kundrat (dennis.kundrat@imes.uni-hannover.de, robotics@kundrat.de)
 * @license please see GIT repository license declaration

 * @date	January 2018
 */

#ifndef TIMER_HPP
#define TIMER_HPP

#include <functional>
#include <ctime>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <mutex>

using namespace std;

class Timer
{
public:
  Timer();
  Timer(function<void()> f, double timeout);
  ~Timer();
  void start();
  void stop();
  void reset();
  void rate(double rate);
  double getLastPassedTime();


private:
  function<void()> f;
  double timeout;
  chrono::time_point<std::chrono::system_clock>  startTime, rate_lastPass;
  bool rate_firstPass = true;
  bool active = true;
  thread* timerThread;
  mutex mx_thread;

  double rate_lastPassedTime = 0;

  void startThread();
};

#endif // TIMER_HPP
