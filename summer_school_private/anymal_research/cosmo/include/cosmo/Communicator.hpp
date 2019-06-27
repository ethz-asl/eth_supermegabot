/*!
 * @file	Communicator.hpp
 * @author	Philipp Leemann
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/typedefs.hpp"
#include "cosmo/CommunicatorOptions.hpp"

#include "message_logger/message_logger.hpp"

#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition_any.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/thread/thread_time.hpp> // for boost::get_system_time

#include <memory>
#include <csignal>
#include <fstream>

namespace cosmo {

/**
 *
 * @tparam     MessageType_  Message that will be stored in shared memory. Don not use dynamic sized memory and std::string!
 * @tparam     Alignment_    Alignment of the memory allocation, has to be a multiple of size of a SIMD memory read instruction (128bits)
 */
template <typename MessageType_, bool isPublisher_, unsigned int Alignment_ = DefaultAlignment>
class Communicator {
public:
    Communicator() = delete;

    /*!
     * Creates or opens shared memory required to publish/subscribe to the given
     * topic
     *
     * @param      topic  Name of the topic
     */
    Communicator(const std::string& topic):
            Communicator(std::make_shared<CommunicatorOptions>(topic))
    {
    }

    /*!
     * Creates or opens shared memory required to publish/subscribe to the given
     * topic. This variant takes an option struct as parameter.
     *
     * @param      options  Option struct
     */
    Communicator(const std::shared_ptr<CommunicatorOptions>& options):
            options_(options)
    {

        allocatePool();
        allocateTopic();
    }

    Communicator(Communicator&&) = default;

    /*!
     * Decrements smNumSubscribers_ or smNumPublishers_ respectively and deallocates the shared memory if the counters are 0
     */
    virtual ~Communicator()
    {
        deallocateTopic();
        deallocatePool();
    }


    /**
     * @brief      Returns the number of cosmo subscribers
     *
     * @return     The number of cosmo subscribers
     */
    inline unsigned int getNumShmSubscribers() const {
        boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> lock{*(this->smMutex_)};
        return *smNumSubscribers_;
    }

    /*!
     * Returns the total number of subscribers, including all channels (shared memory, ros, ...)
     * @return      Total number of subscribers
     */
    virtual unsigned int getNumSubscribers() const {
        return getNumShmSubscribers();
    }

    /**
     * @brief      Return the number of cosmo publishers
     *
     * @return     The number of cosmo publishers
     */
    inline unsigned int getNumShmPublishers() const {
        boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> lock{*(this->smMutex_)};
        return *smNumPublishers_;
    }

    /*!
     * Returns the total number of publishers, including all channels (shared memory, ros, ..)
     * @return      Total number of publishers
     */
    virtual unsigned int getNumPublishers() const {
        return getNumShmPublishers();
    }

    /**
     * @brief      Returns the topic name
     *
     * @return     The topic name
     */
    inline std::string getTopic() const {
        return options_->topic_;
    }

    /**
     * @brief      Returns the memory pool name
     *
     * @return     The memory pool name
     */
    inline std::string getMemoryPoolName() const {
        return options_->memoryPoolName_;
    }

private:
    void allocatePool() {
        managedShm_ = {boost::interprocess::open_or_create, options_->memoryPoolName_.c_str(), options_->memoryPoolSize_,
                       0, options_->memoryPermissions_};

#ifdef COSMO_DEBUG_LEVEL1
        MELO_INFO("Memory deallocated at constructor start: %d", managedShm_.all_memory_deallocated());
#endif

        // todo: add Message_ size or checksum information to check type consistency
        if(options_->topic_.find("__") != std::string::npos) {
            MELO_FATAL("Topic names containing two underscores are reserved for cosmo internal variables!");
        }

        if(!managedShm_.check_sanity()) {
            MELO_FATAL("There is an error with the allocated shared memory!");
        }

        const StringAllocator str_alloc(managedShm_.get_segment_manager());
        smPoolMutex_    = managedShm_.find_or_construct<boost::interprocess::interprocess_sharable_mutex>("__PoolMutex")();
        smPoolCnd_      = managedShm_.find_or_construct<boost::interprocess::interprocess_condition_any>("__PoolCnd")();
        smNumNodes_     = managedShm_.find_or_construct<unsigned int>("__NumNodes")(0u);
        smTopicList_    = managedShm_.find_or_construct<ShmStringVector>("__TopicList")(str_alloc);

        const boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::microseconds(options_->missingProcessTimeout_.count());
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> lock(*smPoolMutex_, abs_time);
        if(lock.owns()) {
            ++(*smNumNodes_);
        }else{
            MELO_FATAL_STREAM("Failed to acquire pool mutex lock during initialization within "
                                      << (double(options_->missingProcessTimeout_.count())/1e6)
                                      << "s. Is there a crashed process blocking the mutex? Please clean shared memory.");
        }

    }

    void allocateTopic() {
        // lock the pool mutex to prevent concurrent construction/destruction
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> poolLock{*smPoolMutex_};

        const NodeInfoAllocator alloc_inst(managedShm_.get_segment_manager());
        try {
          smMutex_ = managedShm_.find_or_construct<boost::interprocess::interprocess_sharable_mutex>((options_->topic_ + "__Mutex").c_str())();
          smCnd_ = managedShm_.find_or_construct<boost::interprocess::interprocess_condition_any>((options_->topic_ + "__Cnd").c_str())();
          smHandle_ = managedShm_.find_or_construct<MemoryManager::handle_t>((options_->topic_ + "__Handle").c_str())();
          smNumPublishers_ = managedShm_.find_or_construct<unsigned int>((options_->topic_ + "__NumSubscribers").c_str())(0u);
          smNumSubscribers_ = managedShm_.find_or_construct<unsigned int>((options_->topic_ + "__NumPublishers").c_str())(0u);
          smCounter_ = managedShm_.find_or_construct<unsigned long int>((options_->topic_ + "__Counter").c_str())(0lu);
          smNodeInfos_ = managedShm_.find_or_construct<ShmNodeInfoVector>((options_->topic_ + "__NodeInfos").c_str())(alloc_inst);
        } catch (boost::interprocess::bad_alloc &ex) {
          MELO_FATAL_STREAM("Failed to construct topic '" << options_->topic_ << "': no available memory.");
          return;
        } catch (...) {
          MELO_FATAL_STREAM("Unknown exception when trying to find or construct topic '" << options_->topic_ << "'.");
          return;
        }

        boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::microseconds(options_->missingProcessTimeout_.count());
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> lock(*smMutex_, abs_time);
        if(lock.owns()) {
            // check if there are existing subscribers or publishers which already allocated the memory for the MessageType_ instance
            if(*smNumPublishers_ + *smNumSubscribers_ > 0) {
#ifdef COSMO_DEBUG_LEVEL1
                MELO_INFO("NOT allocating memory for %s", options_->topic_.c_str());
#endif
                smMessage_ = static_cast<MessageType_*>(managedShm_.get_address_from_handle(*smHandle_));
            }else{
#ifdef COSMO_DEBUG_LEVEL1
                MELO_INFO("Allocating memory for %s", options_->topic_.c_str());
#endif
                try {
                  void* messagePtr = managedShm_.allocate_aligned(sizeof(MessageType_), Alignment_);
                  smMessage_ = new(messagePtr) MessageType_;
                  *smHandle_ = managedShm_.get_handle_from_address(messagePtr);
                } catch (boost::interprocess::bad_alloc &ex) {
                  MELO_FATAL_STREAM("Failed to allocate memory for topic '" << options_->topic_ << "': no available memory.");
                  return;
                } catch (...) {
                  MELO_FATAL_STREAM("Unknown exception when trying to allocate memory for topic '" << options_->topic_ << "'.");
                  return;
                }

                const CharAllocator char_alloc(managedShm_.get_segment_manager());
                smTopicList_->emplace_back(options_->topic_.c_str(), char_alloc);
            }

            if(isPublisher_) {
                ++(*smNumPublishers_);
            }else{
                ++(*smNumSubscribers_);
            }

            // check for missing processes
            for(auto iter = smNodeInfos_->begin(); iter != smNodeInfos_->end(); ) {
                if(!checkProcessExistence(*iter)) {
                    if(iter->isPublisher_) {
                        --(*smNumPublishers_);
                    }else{
                        --(*smNumSubscribers_);
                    }

                    --(*smNumNodes_);

                    MELO_WARN_STREAM("Detected missing process on topic " << options_->topic_ << " (pid: " << iter->pid_ << " / start time: " << iter->startTime_
                                                                       << ", was " << (iter->isPublisher_ ? "Publisher" : "Subscriber") << ")!");

                    iter = smNodeInfos_->erase(iter);
                }else{
                    ++iter;
                }
            }

            const auto pid = getpid();
            smNodeInfos_->emplace_back(pid, getProcessTime(pid), isPublisher_);
        }else{
            --(*smNumNodes_);
            MELO_FATAL_STREAM("Failed to acquire topic '" << options_->topic_ << "' mutex lock during initialization within "
                              << (double(options_->missingProcessTimeout_.count())/1e6)
                              << "s. Is there a crashed process blocking the mutex? Please clean shared memory.");
        }
    }

    void deallocateTopic() {
        // lock the pool mutex to prevent concurrent construction/destruction
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> poolLock{*smPoolMutex_};
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> lock{*smMutex_};
        const auto pid = getpid();
        smNodeInfos_->erase(std::find_if(smNodeInfos_->begin(), smNodeInfos_->end(),
                                         [pid](const ShmNodeInfoVector::value_type& info){ return (info.pid_ == pid && info.startTime_ == getProcessTime(pid)); }));

        if(isPublisher_) {
            --(*smNumPublishers_);
        }else{
            --(*smNumSubscribers_);
        }

        if(*smNumPublishers_ + *smNumSubscribers_ == 0) {
            lock.unlock();
#ifdef COSMO_DEBUG_LEVEL1
            MELO_INFO("deallocating memory for %s", options_->topic_.c_str());
#endif
            auto it = std::find_if(smTopicList_->begin(), smTopicList_->end(), [&](const ShmString& str) { return (str.compare(options_->topic_.c_str()) == 0); });
            if(it == smTopicList_->end()) {
                MELO_ERROR("Did not find topic name in topic list!");
            }else{
//                it->~ShmString();
                const CharAllocator char_alloc(managedShm_.get_segment_manager());
                *it = ShmString(char_alloc); // workaround for boost bug. Assigning an empty string makes use of the boost::basic_string_base optimization for short strings,
                          // which does not use heap memory and therefore not creates a memory leak for the boost vector bug https://svn.boost.org/trac10/ticket/13500
                smTopicList_->erase(it);
            }

            managedShm_.deallocate(smMessage_);
            managedShm_.destroy_ptr(smCnd_);
            managedShm_.destroy_ptr(smHandle_);
            managedShm_.destroy_ptr(smNumPublishers_);
            managedShm_.destroy_ptr(smNumSubscribers_);
            managedShm_.destroy_ptr(smCounter_);
            managedShm_.destroy_ptr(smNodeInfos_);
            managedShm_.destroy_ptr(smMutex_);

        }else{
#ifdef COSMO_DEBUG_LEVEL1
            MELO_INFO("NOT deallocating memory for %s", options_->topic_.c_str());
#endif
        }
    }

    void deallocatePool() {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> lock{*smPoolMutex_};
        --(*smNumNodes_);

        if(*smNumNodes_ == 0u) {
            managedShm_.destroy_ptr(smPoolCnd_);
            managedShm_.destroy_ptr(smNumNodes_);
            managedShm_.destroy_ptr(smTopicList_);

            lock.unlock();
            managedShm_.destroy_ptr(smPoolMutex_);

#ifdef COSMO_DEBUG_LEVEL1
            MELO_INFO("Memory pool deallocated at destructor end: %d", managedShm_.all_memory_deallocated());
#endif
            if(!managedShm_.all_memory_deallocated()) {
                MELO_ERROR("Memory leak detected! Node counter is zero, but shared memory still contains data.");
            }
            boost::interprocess::shared_memory_object::remove(options_->memoryPoolName_.c_str());
        }
    }

    static bool checkProcessExistence(const NodeInfo& info) {
        /*
         * From Kill manual:
        If sig is 0, then no signal is sent, but existence and permission
        checks are still performed; this can be used to check for the
        existence of a process ID or process group ID that the caller is
        permitted to signal.
         */
        return (/*(kill(info.pid_, 0) == 0 || errno == EPERM) && */ getProcessTime(info.pid_) == info.startTime_);
    }

    static unsigned int getProcessTime(const int pid) {
        // see https://github.com/torvalds/linux/blob/master/Documentation/filesystems/proc.txt
        std::ifstream procFile;
        procFile.open("/proc/" + std::to_string(pid) + "/stat");

        if(procFile.is_open()) {
            char str[255];
            procFile.getline(str, 255);  // delim defaults to '\n'

            std::vector<std::string> tmp;
            std::istringstream iss(str);
            std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                      std::back_inserter<std::vector<std::string> >(tmp));

            return tmp.size() >= 21 ? std::stoul(tmp[21]) : 0;
        }

        return 0;
    }

protected:
    // user should not do modifications on the options struct because e.g. changing topic or memory pool name will lead to memory leaks.
    // since we are using shared_ptr this is not possible to achieve?
    const std::shared_ptr<const CommunicatorOptions> options_;

    mutable boost::interprocess::interprocess_sharable_mutex* smPoolMutex_; // protects smPoolCnd_ and smNumNodes_
    boost::interprocess::interprocess_condition_any* smPoolCnd_;
    unsigned int* smNumNodes_;

    mutable boost::interprocess::interprocess_sharable_mutex* smMutex_; // protects all variables below
    boost::interprocess::interprocess_condition_any* smCnd_;
    MemoryManager::handle_t* smHandle_;
    MessageType_ *smMessage_;
    unsigned int* smNumSubscribers_;
    unsigned int* smNumPublishers_;
    unsigned long int* smCounter_;

private:
    MemoryManager managedShm_;

    ShmNodeInfoVector* smNodeInfos_;
    ShmStringVector* smTopicList_;
};

} // namespace shared_communicator
