/*!
 * @file    KeyContainer.hpp
 * @author  Christian Gehring
 * @date    Mar 20, 2015
 * @version 1.0
 *
 */
#pragma once


#include <exception>
#include <list>
#include <vector>
#include <unordered_map>
#include <std_utils/containers/EnumMap.hpp>

namespace std_utils {

template<typename Item_, typename Enum_>
class MultiKeyContainer
{
 public:
  typedef Item_ Item;
  typedef typename std::remove_pointer<Item_>::type ItemType;
  typedef std::vector<Item_> ItemMap;

  typedef Enum_ EnumType;
  typedef std::tuple<std::string, unsigned int, EnumType> KeyTuple;
  typedef std::vector<KeyTuple> KeyMap;
  typedef std::unordered_map<std::string, unsigned int> NameMap;
  typedef std::unordered_map<unsigned int, unsigned int> IdMap;
  typedef std_utils::EnumMap<EnumType, unsigned int> EnumMap;
  typedef std::integral_constant<bool, std::is_pointer<Item_>::value> truth_type;



  typedef typename ItemMap::size_type size_type;
  typedef typename ItemMap::iterator iterator;
  typedef typename ItemMap::const_iterator const_iterator;
  typedef typename ItemMap::const_reference const_reference;
  typedef typename ItemMap::reference reference;
 protected:
  //! Data container
  ItemMap itemMap_;
  //! Key map
  KeyMap keyMap_;
  //! Map from identifier to id of the item.
  IdMap idMap_;
  //! Map from name to id of the item.
  NameMap nameMap_;
  //! Map from enum to id of the item.
  EnumMap enumMap_;

 public:
  MultiKeyContainer() {

  }
  virtual ~MultiKeyContainer() {

  }


  reference operator[](const std::string& name) {
    return itemMap_[nameMap_[name]];
  }

  const_reference operator[](const std::string& name) const {
    return itemMap_[nameMap_.at(name)];
  }


  reference operator[](EnumType e) {
    return itemMap_[enumMap_[e]];
  }

  const_reference operator[](EnumType e) const {
    return itemMap_[enumMap_.at(e)];
  }

  reference operator[](unsigned int id) {
    return itemMap_[idMap_.at(id)];
  }

  const_reference operator[](unsigned int id) const {
    return itemMap_[idMap_.at(id)];
  }

  reference at(const std::string& name) {
    return itemMap_[nameMap_.at(name)];
  }

  const_reference at(const std::string& name) const {
    return itemMap_[nameMap_.at(name)];
  }


  reference at(unsigned int id) {
     return itemMap_[idMap_.at(id)];
  }

  const_reference at(unsigned int id) const {
    return itemMap_[idMap_.at(id)];
  }

  reference at(EnumType e) {
     return itemMap_[enumMap_.at(e)];
  }

  const_reference at(EnumType e) const {
    return itemMap_[enumMap_.at(e)];
  }

  iterator find(const std::string& name) {
    auto nameIt = nameMap_.find(name);
    if (nameIt == nameMap_.end()) {
      return end();
    }
    return itemMap_.begin() + nameIt->second;
  }

  const_iterator find (const std::string& name) const {
    auto nameIt = nameMap_.find(name);
    if (nameIt == nameMap_.end()) {
      return end();
    }
    return itemMap_.cbegin() + nameIt->second;
  }

  iterator find(unsigned int id) {
    auto idIt = idMap_.find(id);
    if (idIt == idMap_.end()) {
      return end();
    }
    return itemMap_.begin() + idIt->second;
  }

  const_iterator find(unsigned int id) const {
    auto idIt = idMap_.find(id);
    if (idIt == idMap_.end()) {
      return end();
    }
    return itemMap_.cbegin() + idIt->second;
  }

  iterator find(EnumType e) {
    auto enumIt = enumMap_.find(e);
    if (enumIt == enumMap_.end()) {
      return end();
    }
    return itemMap_.begin() + enumIt->second;
  }

  const_iterator find(EnumType e) const {
    auto enumIt = enumMap_.find(e);
    if (enumIt == enumMap_.end()) {
      return end();
    }
    return itemMap_.cbegin() + enumIt->second;
  }

  iterator begin() {
    return itemMap_.begin();
  }

  const_iterator begin() const {
    return itemMap_.begin();
  }

  iterator end() {
    return itemMap_.end();
  }

  const_iterator end() const {
    return itemMap_.end();
  }

  size_type size() const {
    return itemMap_.size();
  }


  void insert(const KeyTuple& key, const_reference data) {
    const std::string& name = std::get<0>(key);
    const unsigned int id = std::get<1>(key);
    const EnumType e = std::get<2>(key);
    typename IdMap::const_iterator idIt = idMap_.find(id);
    if (idIt != idMap_.end() ) {
      std::string message = std::string{"Could not insert data with id "} + std::to_string(id) + std::string{"!"};
      throw std::runtime_error(message);
    }
    typename NameMap::const_iterator nameIt = nameMap_.find(name);
    if (nameIt != nameMap_.end() ) {
       std::string message = std::string{"Could not insert data with name "} + name + std::string{"!"};
       throw std::runtime_error(message);
    }
    typename EnumMap::const_iterator enumIt = enumMap_.find(e);
    if (enumIt != enumMap_.end() ) {
       std::string message = std::string{"Could not insert data with enum "} + std::to_string(static_cast<int>(e)) + std::string{"!"};
       throw std::runtime_error(message);
    }
    idMap_.insert(std::pair<unsigned int, unsigned int>(id, itemMap_.size()));
    nameMap_.insert(std::pair<std::string, unsigned int>(name, itemMap_.size()));
    enumMap_.insert(std::pair<EnumType, unsigned int>(e, itemMap_.size()));
    itemMap_.push_back(data);
    keyMap_.push_back(key);
  }

  size_type erase(const std::string& name) {
    unsigned int id = getId(name);
    const EnumType e = getEnum(name);
    itemMap_.erase(itemMap_.begin()+idMap_.at(id));
    keyMap_.erase(keyMap_.begin()+idMap_.at(id));
    idMap_.erase(id);
    nameMap_.erase(name);
    enumMap_.erase(e);

    return itemMap_.size();
  }

  size_type erase(unsigned int id) {
    const std::string name = getName(id);
    const EnumType e = getEnum(id);
    itemMap_.erase(itemMap_.begin()+idMap_.at(id));
    keyMap_.erase(keyMap_.begin()+idMap_.at(id));
    idMap_.erase(id);
    nameMap_.erase(name);
    enumMap_.erase(e);
    return itemMap_.size();
  }

  size_type erase(const EnumType e) {
    const std::string name = getName(e);
    const unsigned int id = getId(e);
    itemMap_.erase(itemMap_.begin()+idMap_.at(id));
    keyMap_.erase(keyMap_.begin()+idMap_.at(id));
    idMap_.erase(id);
    nameMap_.erase(name);
    enumMap_.erase(e);
    return itemMap_.size();
  }


  void clear() noexcept {
    itemMap_.clear();
    idMap_.clear();
    nameMap_.clear();
    enumMap_.clear();
    keyMap_.clear();
  }

  bool empty() const noexcept {
    return itemMap_.empty();
  }

  EnumType getEnum(unsigned int id) const {
    return std::get<2>(keyMap_[idMap_.at(id)]);
  }

  EnumType getEnum(const std::string& name) const {
    return std::get<2>(keyMap_[nameMap_.at(name)]);
  }

  const std::string& getName(EnumType e) const {
    return std::get<0>(keyMap_[enumMap_.at(e)]);
  }

  const std::string& getName(unsigned int id) const {
    return std::get<0>(keyMap_[idMap_.at(id)]);
  }

  unsigned int getId(EnumType e) const {
    return std::get<1>(keyMap_[enumMap_.at(e)]);
  }

  unsigned int getId(const std::string& name) const {
    return std::get<1>(keyMap_[nameMap_.at(name)]);
  }

  KeyTuple getKey(unsigned int id) const {
    return keyMap_[idMap_.at(id)];
  }

  KeyTuple getKey(EnumType e) const {
    return keyMap_[enumMap_.at(e)];
  }

  KeyTuple getKey(const std::string& name) const {
    return keyMap_[nameMap_.at(name)];
  }

};

template<typename Item_, typename Enum_>
static void createMultiKeyContainerFromKeys(const std::list<std::tuple<std::string, unsigned int, Enum_> >& keys,  MultiKeyContainer<Item_, Enum_>& container, const Item_& defaultItem) {
  container.clear();
  for (auto& key : keys) {
    container.insert(key, Item_(defaultItem));
  }
}

} /* namespace robot_utils */
