/*!
 * @file	WorkerManager.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

 #pragma once

#include <unordered_map>
#include <functional> // for std::bind
#include <string>
#include <mutex>

#include "any_worker/Worker.hpp"
#include "any_worker/WorkerOptions.hpp"

namespace any_worker {

class WorkerManager {
public:
    WorkerManager();
    virtual ~WorkerManager();


    template<class T>
    inline bool addWorker(const std::string& name, const double timestep, bool(T::*fp)(const WorkerEvent&), T* obj, const int priority=0, const bool autostart=true) {
        return addWorker( WorkerOptions(name, timestep, std::bind(fp, obj, std::placeholders::_1), priority), autostart );
    }

    template<class T>
    inline bool addWorker(const std::string& name, const double timestep,
                          bool(T::*cfp)(const WorkerEvent&), void(T::*rfp)(void),
                          T* obj, const int priority=0, const bool autostart=true) {
      return addWorker( WorkerOptions(name, timestep, std::bind(cfp, obj, std::placeholders::_1), std::bind(rfp, obj), priority), autostart );
    }

    inline bool addWorker(const std::string& name, const double timestep, const WorkerCallback& callback, const int priority=0, const bool autostart=true) {
        return addWorker( WorkerOptions(name, timestep, callback, priority), autostart );
    }

    inline bool addWorker(const std::string& name, const double timestep,
                          const WorkerCallback& callback, const WorkerCallbackFailureReaction& callbackFailureReaction,
                          const int priority=0, const bool autostart=true) {
      return addWorker( WorkerOptions(name, timestep, callback, callbackFailureReaction, priority), autostart );
    }

    bool addWorker(const WorkerOptions& options, const bool autostart=true);

    // the addWorker variant below is commented out because it can lead to strange behaviour. E.g. when the user wants to move a worker from one workerManager to
    //   another, the old owner may have a unordered_map<std::string, Worker>-entry with an invalid worker.
//    bool addWorker(Worker&& worker);

    void startWorker(const std::string& name, const int priority=0);

    void startWorkers();

    bool hasWorker(const std::string& name);

    void stopWorker(const std::string& name, const bool wait=true);

    void stopWorkers(const bool wait=true);

    void cancelWorker(const std::string& name, const bool wait=true);

    void cancelWorkers(const bool wait=true);

    void setWorkerTimestep(const std::string& name, const double timeStep);


    /*!
     * Requests all workers to stop, then joins their threads and deletes their instances.
     */
    void clearWorkers();

    /*!
     * Removes workers which are destructible (see Worker::isDestructible()) from the map (calling their destructors)
     */
    void cleanDestructibleWorkers();

private:
    std::unordered_map<std::string, Worker> workers_;
    std::mutex mutexWorkers_;
};

} // namespace any_worker
