/*!
 * @file    EnumContainer.hpp
 * @author  Gabriel Hottiger
 * @date    Oct 24, 2017
 */
#pragma once

// robot utils
#include "std_utils/containers/Key.hpp"
#include "std_utils/containers/EnumArray.hpp"

// STL
#include <array>
#include <algorithm>
#include <type_traits>
#include <stdexcept>

namespace std_utils {

/**
 * @brief Array of elements
 * @tparam Enum_ Type of the enum (starts at 0, consecutive, finishes with SIZE)
 */
template < typename Enum_, std::size_t Size_ = static_cast<size_t>(Enum_::SIZE)>
class KeyArray : public EnumArray<Enum_, std_utils::Key<Enum_>, Size_> {
 public:
  using Item = Key<Enum_>;
  using IdType = typename Item::IdType;

  constexpr KeyArray(std::array<Item, Size_> a) noexcept : EnumArray<Enum_, Item, Size_>(a) { }

  // Name and id getters, not very efficient access
  const Item& atId(IdType id) const {
    auto it = std::find_if(this->begin(), this->end(), [id](const Item & item){ return id == item.getId(); } );
    if(it == this->end()) {
      throw std::out_of_range("[std_utils::KeyArray] No entry with id: " + std::to_string(id) + "!");
    }
    return *it;
  }
  Item& atId(IdType id) {
    return const_cast<Item&>(static_cast<const KeyArray<Enum_, Size_>*>(this)->atId(id));
  }

  const Item& atName(const std::string& name) const {
    auto it = std::find_if(this->begin(), this->end(), [name](const Item & item){ return name == item.getName(); } );
    if(it == this->end()) {
      throw std::out_of_range("[std_utils::KeyArray] No entry with name: " + name + "!");
    }
    return *it;
  }
  Item& atName(const std::string& name) {
    return const_cast<Item&>(static_cast<const KeyArray<Enum_, Size_>*>(this)->atName(name));
  }

  bool containsName(const std::string& name) const {
    auto it = std::find_if(this->begin(), this->end(), [name](const Item & item){ return name == item.getName(); } );
    if(it == this->end()) {
      return false;
    }
    return true;
  }

  bool containsId(IdType id) const {
    auto it = std::find_if(this->begin(), this->end(), [id](const Item & item){ return id == item.getId(); } );
    if(it == this->end()) {
      return false;
    }
    return true;
  }


};

} /* namespace std_utils */
