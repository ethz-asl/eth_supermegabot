/*!
 * @file	Node.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <memory> // for std::shared_ptr
#include <sched.h>
#include <unistd.h> // for getpid()
#include <ros/ros.h>


#include "any_worker/WorkerManager.hpp"
#include "any_worker/WorkerOptions.hpp"
#include "any_node/Param.hpp"
#include "any_node/Topic.hpp"

namespace any_node {

bool setProcessPriority(int priority);

class Node {
 public:
    using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;

    Node() = delete;
    Node(NodeHandlePtr nh);
    virtual ~Node() = default;

    /*
     * (abstract) interface functions
     */

    /*!
     * Init function, used to initialize all members and starting workers (if any).
     * @return      True if successful. Returning false indicates that the node shall shut down.
     */
    virtual bool init() = 0;
    /*!
     * Pre-Cleanup function, which is called by Nodewrap _before_ stopping workers. (Thread safety up to the user!).
     * This function is called even if init() returned false.
     */
    virtual void preCleanup() { }
    /*!
     * Cleanup function, called by Nodewrap _after_ stopping workers.
     * This function is called even if init() returned false.
     */
    virtual void cleanup() = 0;

    /*
     * general
     */

    /*!
     * Method to signal nodewrap to shutdown the node.
     */
    void shutdown();

    /*!
     * Helper functions to add Workers to the WorkerManager
     */
    template<class T>
    inline bool addWorker(const std::string& name, const double timestep, bool(T::*fp)(const any_worker::WorkerEvent&), T* obj, const int priority=0) {
        return workerManager_.addWorker(name, timestep, fp, obj, priority);
    }

    inline bool addWorker(const any_worker::WorkerOptions& options) {
        return workerManager_.addWorker(options);
    }

    /*!
     * Check if WorkerManager is managing a Worker with given name
     * @param name  Name of the worker
     * @return      True if worker was found
     */
    inline bool hasWorker(const std::string& name) {
        return workerManager_.hasWorker(name);
    }

    /*!
     * Stop a worker managed by the WorkerManager
     * @param name  Name of the worker
     * @param wait  Whether to wait until the worker has finished or return immediately
     */
    inline void cancelWorker(const std::string& name, const bool wait = true) {
        workerManager_.cancelWorker(name, wait);
    }

    /*!
     * Method to stop all workers managed by the WorkerManager
     */
    inline void stopAllWorkers() {
        workerManager_.clearWorkers();
    }

    /*
     * accessors
     */
    inline ros::NodeHandle& getNodeHandle() const { return *nh_; }


    /*
     * forwarding to Topic.hpp functions
     */
    template<typename msg>
    inline ros::Publisher advertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false) {
        return any_node::advertise<msg>(*nh_, name, defaultTopic, queue_size, latch);
    }

    template<typename msg>
    inline ThreadedPublisherPtr<msg> threadedAdvertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false, unsigned int maxMessageBufferSize = 10) {
        return any_node::threadedAdvertise<msg>(*nh_, name, defaultTopic, queue_size, latch, maxMessageBufferSize);
    }

    template<class M, class T>
    inline ros::Subscriber subscribe(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
        return any_node::subscribe(*nh_, name, defaultTopic, queue_size, fp, obj, transport_hints);
    }

    template<class M, class T>
    inline ThrottledSubscriberPtr<M, T> throttledSubscribe(double timeStep, const std::string& name, const std::string& defaultTopic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
        return any_node::throttledSubscribe<M, T>(timeStep, *nh_, name, defaultTopic, queue_size, fp, obj, transport_hints);
    }

    template<class T, class MReq, class MRes>
    inline ros::ServiceServer advertiseService(const std::string& name, const std::string& defaultService, bool(T::*srv_func)(MReq &, MRes &), T *obj) {
        return any_node::advertiseService(*nh_, name, defaultService, srv_func, obj);
    }

    template<class MReq, class MRes>
    inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string()) {
        return any_node::serviceClient<MReq, MRes>(*nh_, name, defaultService, header_values);
    }

    template<class Service>
    inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string()) {
        return any_node::serviceClient<Service>(*nh_, name, defaultService, header_values);
    }

    /*
     * forwarding to Param.hpp functions
     */
     template<typename ParamT>
     inline bool getParam(const std::string& key, ParamT& param_val) {
         return any_node::getParam(*nh_, key, param_val);
     }

     template<typename ParamT>
     inline ParamT param(const std::string& key, const ParamT& defaultValue) {
         return any_node::param(*nh_, key, defaultValue);
     }

     template<typename ParamT>
     inline void setParam(const std::string& key, const ParamT& param) {
         any_node::setParam(*nh_, key, param);
     }

 protected:
    NodeHandlePtr nh_;

 private:
    any_worker::WorkerManager workerManager_;

};

} // namespace any_node
