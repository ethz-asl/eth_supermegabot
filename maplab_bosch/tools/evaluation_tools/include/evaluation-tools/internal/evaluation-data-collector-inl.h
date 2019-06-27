#ifndef INTERNAL_EVALUATION_DATA_COLLECTOR_INL_H_
#define INTERNAL_EVALUATION_DATA_COLLECTOR_INL_H_

#include <mutex>
#include <string>
#include <unordered_map>

namespace evaluation {
namespace internal {

template<typename DataType>
void EvaluationDataCollectorImpl::pushData(
    const SlotId& slot_id, const std::string& channel_name, const DataType& data) {
  ChannelGroup* slot = getSlotAndCreateIfNecessary(slot_id);
  CHECK_NOTNULL(slot);
  slot->setChannel(channel_name, data);
}

template<typename DataType>
bool EvaluationDataCollectorImpl::getDataSafe(
    const SlotId& slot_id, const std::string& channel_name, const DataType** data) const {
  const ChannelGroup* slot;
  if ((slot = getSlot(slot_id)) == nullptr) {
    return false;
  }
  return slot->getChannelSafe(channel_name, data);
}

template<typename DataType>
std::string EvaluationDataCollectorImpl::printData(
    const SlotId& slot_id, const std::string& channel_name) const {
  std::ostringstream out;
  const DataType* data;
  if (getDataSafe(slot_id, channel_name, &data)) {
    out << std::setprecision(5) << *CHECK_NOTNULL(data) << std::fixed;
  } else {
    out << "Channel not available.";
  }
  return out.str();
}

}  // namespace internal
}  // namespace evaluation
#endif  // INTERNAL_EVALUATION_DATA_COLLECTOR_INL_H_
