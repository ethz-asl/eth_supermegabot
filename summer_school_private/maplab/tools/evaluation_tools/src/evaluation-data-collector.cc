#include "evaluation-tools/evaluation-data-collector.h"

#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>

#include <aslam/common/unique-id.h>
#include <Eigen/Core>

namespace evaluation {
namespace internal {
const EvaluationDataCollectorImpl::SlotId EvaluationDataCollectorImpl::kCommonSlotId =
    EvaluationDataCollectorImpl::SlotId::Random();

EvaluationDataCollectorImpl::ChannelGroup* EvaluationDataCollectorImpl::getSlot(const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);

  SlotIdSlotMap::iterator it = channel_groups_.find(slot_id);
  if (it == channel_groups_.end()) {
    return nullptr;
  }
  return &it->second;
}

const EvaluationDataCollectorImpl::ChannelGroup* EvaluationDataCollectorImpl::getSlot(
    const SlotId& slot_id) const {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  SlotIdSlotMap::const_iterator it = channel_groups_.find(slot_id);
  if (it == channel_groups_.end()) {
    return nullptr;
  }
  return &it->second;
}

EvaluationDataCollectorImpl::ChannelGroup* EvaluationDataCollectorImpl::getSlotAndCreateIfNecessary(
    const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  SlotIdSlotMap::iterator iterator;
  bool inserted = false;
  std::tie(iterator, inserted) = channel_groups_.emplace(std::piecewise_construct,
                                                         std::make_tuple(slot_id),
                                                         std::make_tuple());
  return &iterator->second;
}

void EvaluationDataCollectorImpl::removeSlotIfAvailable(const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  channel_groups_.erase(slot_id);
}

bool EvaluationDataCollectorImpl::hasSlot(const SlotId& slot_id) const {
  return (getSlot(slot_id) != nullptr);
}

bool EvaluationDataCollectorImpl::hasChannel(const SlotId& slot_id,
                                      const std::string& channel_name) const {
  const ChannelGroup* slot;
  if ((slot = getSlot(slot_id)) == nullptr) {
    return false;
  }
  return slot->hasChannel(channel_name);
}

void EvaluationDataCollectorImpl::getAllSlotIds(
    std::unordered_set<SlotId>* slot_ids) const {
  CHECK_NOTNULL(slot_ids)->clear();
  std::lock_guard<std::mutex> lock(m_channel_groups_);

  slot_ids->reserve(channel_groups_.size());
  for (const SlotIdSlotMap::value_type& slot_id_with_channel_group :
      channel_groups_) {
    if (slot_id_with_channel_group.first != kCommonSlotId) {
      slot_ids->emplace(slot_id_with_channel_group.first);
    }
  }
}

}  // namespace internal
}  // namespace evaluation
