/*
 * auxiliaries.cpp
 *
 *  Created on: 19.01.2015
 *      Author: neunertm, asutosh
 */


#include <smb_driver/auxiliaries/interProcessCommunication.h>

#ifdef __XENO__

	bool acquireMutex(mutex_t& mutex, int timeoutUs)
    {
        RTIME wait;
        if (timeoutUs == 0) { wait = TM_NONBLOCK; }
        if (timeoutUs < 0) { wait = TM_INFINITE; }
        if (timeoutUs > 0) { wait = rt_timer_ns2ticks(timeoutUs*1000); }
        int ret = rt_mutex_acquire(&mutex, wait);
        return (ret == 0);
    }
    void releaseMutex(mutex_t& mutex)
    {
        rt_mutex_release(&mutex);
    }
    void acquireMutexAndWaitForCondition(mutex_t& mutex, condition_variable_t& condition, bool keepMutex)
    {
        acquireMutex(mutex, -1);
        rt_cond_wait(&condition, &mutex, TM_INFINITE);

        if (!keepMutex)
            releaseMutex(mutex);
    }
    void notify(condition_variable_t& condition)
    {
        rt_cond_signal(&condition);
    }

    void sleepms(int milliseconds)
    {
    	usleep(milliseconds * 1000);
    }

#else

bool acquireMutex(mutex_t &mutex, int timeoutUs) {
    //DEBUG_PRINT("Trying to acquire mutex with timeout of: "<<timeoutUs<<" microseconds");
    if (timeoutUs >= 0) {
        if (mutex.try_lock()) { return true; }

        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

        while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start) < std::chrono::microseconds(timeoutUs)) {
            if (mutex.try_lock()) { return true; }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    } else {
        mutex.lock();
        return true;
    }
    return false;
}

void releaseMutex(mutex_t &mutex) {
    mutex.unlock();
}

void acquireMutexAndWaitForCondition(mutex_t &mutex, condition_variable_t &condition, bool keepMutex) {
    mutex.lock();
    condition.wait(mutex);

    if (!keepMutex)
        mutex.unlock();
}

void notify(condition_variable_t &condition) {
    condition.notify_one();
}

void sleepms(int milliseconds)
{
	usleep(milliseconds * 1000);
}

#endif

