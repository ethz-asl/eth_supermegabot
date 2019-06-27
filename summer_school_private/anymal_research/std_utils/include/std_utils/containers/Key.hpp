/*!
 * @file    Key.hpp
 * @author  Gabriel Hottiger
 * @date    Oct 24, 2017
 * @version 0.0
 */

#pragma once

// STL
#include <type_traits>

namespace std_utils {
/**
 * @brief Constexpr type that stores relation of name, id and enum
 * @tparam Enum_ Type of the enum
 */
template< typename Enum_>
class Key {

 public:
  using IdType = typename std::underlying_type<Enum_>::type;

  constexpr Key(const Enum_ e, const char * name, const IdType id) noexcept :
    enum_{e},
    name_{name},
    id_{id}
  {
  }

  constexpr Key(const Enum_ e, const char * name) noexcept : Key<Enum_>(e, name, static_cast<IdType>(e) ) { }

  // Constexpr getters
  constexpr const Enum_ getEnum() const { return enum_; }
  constexpr const char * getName() const { return name_; }
  constexpr const IdType getId() const { return id_; }

 private:
  const Enum_ enum_;
  const char * name_;
  const IdType id_;
};

//! C++17 will have template deduction for constructor parameters, until then this helper function is provided
template< typename Enum_>
constexpr Key<Enum_> make_key(const Enum_ e,  const char * name, const typename Key<Enum_>::IdType id) noexcept {
  return Key<Enum_>(e, name, id);
}
template< typename Enum_>
constexpr Key<Enum_> make_key(const Enum_ e,  const char * name) noexcept {
  return make_key(e, name, static_cast<typename Key<Enum_>::IdType>(e));
}

} // namespace std_utils
