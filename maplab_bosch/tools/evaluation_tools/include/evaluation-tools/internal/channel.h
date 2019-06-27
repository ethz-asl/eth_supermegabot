#ifndef EVALUATION_TOOLS_CHANNEL_H_
#define EVALUATION_TOOLS_CHANNEL_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <glog/logging.h>

namespace evaluation {
namespace internal {

class ChannelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ChannelBase(const ChannelBase&) = delete;
  void operator=(const ChannelBase&) = delete;
  ChannelBase() = default;
  virtual ~ChannelBase() = default;;
};

template<typename DataType>
class Channel : public ChannelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DataType Type;
  Channel() = default;
  Channel(const DataType& data) : value_(data) {}
  virtual ~Channel() = default;
  const DataType value_;
};

class ChannelGroup {
 public:
  typedef std::unordered_map<std::string, std::shared_ptr<ChannelBase>> ChannelMap;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ChannelGroup() = default;
  ChannelGroup(const ChannelGroup&) = delete;
  void operator=(const ChannelGroup&) = delete;

  bool hasChannel(const std::string& channel_name) const {
    CHECK(!channel_name.empty());
    std::lock_guard<std::mutex> lock(mutex_);
    return (channels_.count(channel_name) > 0);
  }

  template<typename DataType>
  void setChannel(const std::string& channel_name, const DataType& data) {
    CHECK(!channel_name.empty());
    std::lock_guard<std::mutex> lock(mutex_);
    std::shared_ptr<Channel<DataType>> cloned_data(new Channel<DataType>(data));
    CHECK(channels_.emplace(std::make_pair(channel_name, cloned_data)).second)
      << "Group already contains channel " << channel_name;
  }

  template<typename DataType>
  const DataType& getChannel(const std::string& channel_name) const {
    const DataType* data_ptr;
    CHECK(getChannelSafe(channel_name, &data_ptr))
          << "Group does not contain channel " << channel_name;
    CHECK_NOTNULL(data_ptr);
    return *data_ptr;
  }

  template<typename DataType>
  bool getChannelSafe(const std::string& channel_name, const DataType** data_ptr) const {
    CHECK_NOTNULL(data_ptr);
    CHECK(!channel_name.empty());
    std::lock_guard<std::mutex> lock(mutex_);
    ChannelMap::const_iterator it = channels_.find(channel_name);
    if (it == channels_.end()) {
      *data_ptr = nullptr;
      return false;
    }
    typedef Channel<DataType> DerivedChannel;
    std::shared_ptr<DerivedChannel> derived = std::dynamic_pointer_cast<DerivedChannel>(it->second);
    CHECK(derived) << "Channel cast to derived failed channel: " << channel_name;
    *data_ptr = &derived->value_;
    return true;
  }

 private:
  ChannelMap channels_;
  mutable std::mutex mutex_;
};

}  // namespace internal
}  // namespace evaluation
#endif  // EVALUATION_TOOLS_CHANNEL_H_
