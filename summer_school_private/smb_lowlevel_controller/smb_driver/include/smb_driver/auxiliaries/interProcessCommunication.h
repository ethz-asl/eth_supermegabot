/*
 * auxiliaries.hpp
 *
 *  Created on: 19.01.2015
 *      Author: neunertm
 */


#ifndef AUXILIARIES_HPP_
#define AUXILIARIES_HPP_

// Xenomai
#ifdef __XENO__
#include <native/mutex.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/cond.h>
#include <unistd.h>
#else
// Standard C++ (non-RT)
#include <mutex>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <unistd.h>
#endif


#ifdef DEBUG
#define DEBUG_PRINT(s) std::cout<<s<<std::endl;
#else
#define DEBUG_PRINT(s) do {} while (0)
#endif


// Definitions of mutex, thread and condition variable types depending on OS
#ifdef __XENO__
	typedef RT_MUTEX mutex_t;
	typedef RT_TASK thread_t;
	typedef RT_COND condition_variable_t;
#else  // __XENO__
	// Standard C++ (non-RT)
	typedef std::mutex mutex_t;
	typedef std::thread thread_t;
	typedef std::condition_variable_any condition_variable_t;
#endif // __XENO__


// Platform specific mutex and condition variable handling

bool acquireMutex(mutex_t& mutex, int timeoutUs);

void releaseMutex(mutex_t& mutex);

void acquireMutexAndWaitForCondition(mutex_t& mutex, condition_variable_t& condition, bool keepMutex = false);

void notify(condition_variable_t& condition);

void sleepms(int milliseconds);




#endif /* AUXILIARIES_HPP_ */
