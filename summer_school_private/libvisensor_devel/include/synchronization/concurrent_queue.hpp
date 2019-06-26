/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CONCURRENT_QUEUE_H_
#define CONCURRENT_QUEUE_H_

#include <config/config.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "lifo_nonlocking.h"

template<typename>
class concurrent_heap;

template<typename>
class concurrent_queue;

template<class container_T>
class concurrent_base
{
	friend class concurrent_heap<container_T>;
	friend class concurrent_queue<container_T>;
protected:
	container_T mqueue;
	mutable boost::mutex mmutex;
	boost::condition_variable mcond;
public:
	typedef container_T container_type;
	typedef typename container_T::value_type value_type;
	concurrent_base(){
		while(!mqueue.empty())
			mqueue.pop();
	}
	virtual ~concurrent_base(){};
	virtual __inline__ void clear() = 0;
	virtual __inline__ value_type pop(void) = 0;

	__inline__ void push(value_type const& data)
	{
		//bool const was_empty=mqueue.empty();
		using namespace std;
		{ //to unlock before notification
			boost::mutex::scoped_lock lock(mmutex);
			mqueue.push(data);
		}
		//if(was_empty)
       	//{
			mcond.notify_one();
		//}
	}
	__inline__ bool empty() const
	{
		boost::mutex::scoped_lock lock(mmutex);
		return mqueue.empty();
	}
	__inline__  typename container_T::size_type size() const
	{
		boost::mutex::scoped_lock lock(mmutex);
		return mqueue.size();
	}

	void notify(){
		mcond.notify_all();
	}
};

template<class container_T>
class concurrent_heap:public concurrent_base<container_T>
{
public:
	using concurrent_base<container_T>::size;
	using concurrent_base<container_T>::push;
	using concurrent_base<container_T>::empty;

	concurrent_heap(){};

	virtual ~concurrent_heap(){};
	virtual __inline__ typename concurrent_base<container_T>::value_type pop(void){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		while(concurrent_base<container_T>::mqueue.empty())
		{
			concurrent_base<container_T>::mcond.wait(lock);
		}
		typename container_T::value_type topval = concurrent_base<container_T>::mqueue.top();
		concurrent_base<container_T>::mqueue.pop();
		return topval;
	}
	virtual __inline__ void clear(){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		concurrent_base<container_T>::mqueue.clear();
	}
};

template<class container_T>
class concurrent_queue:public concurrent_base<container_T>
{
public:
	using concurrent_base<container_T>::size;
	using concurrent_base<container_T>::push;
	using concurrent_base<container_T>::empty;
	concurrent_queue(){};
	virtual ~concurrent_queue(){};
	__inline__ typename concurrent_base<container_T>::value_type pop(void){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		while(concurrent_base<container_T>::mqueue.empty())
		{
			concurrent_base<container_T>::mcond.wait(lock);
		}
		typename concurrent_base<container_T>::value_type topval = concurrent_base<container_T>::mqueue.front();
		concurrent_base<container_T>::mqueue.pop();
		return topval;
	}
	virtual __inline__ void clear(){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		while(!concurrent_base<container_T>::mqueue.empty()){
			concurrent_base<container_T>::mqueue.pop();
		}
	}
};

template<class container_T>
class concurrent_pqueue:public concurrent_base<container_T>
{
public:
	concurrent_pqueue(){};
	virtual ~concurrent_pqueue(){};
	__inline__ typename concurrent_base<container_T>::value_type pop(void){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		while(concurrent_base<container_T>::mqueue.empty())
		{
			concurrent_base<container_T>::mcond.timed_wait(lock, boost::posix_time::microsec(10));
		}
		typename concurrent_base<container_T>::value_type topval = concurrent_base<container_T>::mqueue.top();
		concurrent_base<container_T>::mqueue.pop();
		return topval;
	}
	virtual __inline__ void clear(){
		boost::mutex::scoped_lock lock(concurrent_base<container_T>::mmutex);
		while(!concurrent_base<container_T>::mqueue.empty()){
			concurrent_base<container_T>::mqueue.pop();
		}
	}
};


template<typename value_T>
class nonlocking_lifo
{
private:
	struct dataWrapper:public nonlocking::Lifo::Node{ //a wrapper, since nonlocking lifo stores pointers that must inherit from Lifo::Node
		value_T data;
		dataWrapper(value_T _data):data(_data){};
	};

	nonlocking::Lifo lifo; //the stack

	public:
	typedef value_T value_type;
	nonlocking_lifo(){};
	virtual ~nonlocking_lifo(){};
	__inline__ value_T pop(void){
		dataWrapper* dtwrap = NULL;
		while(!dtwrap){ //dtwrap will be NULL if queue is empty
			dtwrap = (dataWrapper*)lifo.pop();
			boost::this_thread::interruption_point();
			if(!dtwrap)
				boost::this_thread::sleep(boost::posix_time::millisec(10)); //if queue is empty wait. Otherwise we hammer the queue too badly
		}
		value_T val = dtwrap->data;
		delete dtwrap;
		return val;
	}
	virtual __inline__ void clear(){
		dataWrapper* topval = NULL;
		do{ //topval will be NULL if queue is empty
			topval = (dataWrapper*)lifo.pop();
			if(topval)
				delete topval;
		}while(topval);
	}

	__inline__ void push(value_T const& data)
	{
		dataWrapper* val = new dataWrapper(data);
		boost::this_thread::interruption_point();
		lifo.push(val);
	}
	__inline__ bool empty() const
	{
		return (lifo.head == NULL);
	}
	__inline__ int sync() const
	{
		return lifo.sync;
	}

	//no size() implemented, because this would require a mutex. Simply counting unsave the nodes from head to tail w/o a mutex is not an option!
};

#endif /* CONCURRENT_QUEUE_H_ */
