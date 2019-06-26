/*!
 * @file	 EnumArray.hpp
 * @author Gabriel Hottiger
 * @date	 Oct 24, 2017
 */

#pragma once

// STL
#include <array>
#include <cassert>
#include <type_traits>

namespace std_utils {

template < typename Enum_, typename Item_, int Size_ = static_cast<int>(Enum_::SIZE)>
class EnumArray : public std::array<Item_, Size_> {
  static_assert(std::is_enum<Enum_>::value, "[std_utils::EnumArray] Enum_ template parameter has to be an enum!");
  static_assert(Size_ >= 0, "[std_utils::EnumArray] Size_ template parameter has to be bigger than or equal to 0!");

 public:
  using Item = Item_;
  using Enum = Enum_;

 protected:
  //! Hide at, [] implementations
  using std::array<Item, Size_>::at;
  using std::array<Item, Size_>::operator[];
  
 public:
  constexpr EnumArray() noexcept : std::array<Item, Size_>{ } {  };
  explicit constexpr EnumArray(std::array<Item, Size_> a) noexcept : std::array<Item, Size_>(std::move(a)) { }
  explicit EnumArray(Item a) noexcept : EnumArray() { std::fill(this->begin(), this->end(), a); }

  //! Provide at, [] implementations for enums
  Item& operator[](Enum e) noexcept { return this->operator[](static_cast<std::size_t>(e)); }
  constexpr const Item& operator[](Enum e) const noexcept { return this->operator[](static_cast<std::size_t>(e)); }
  Item& at(Enum e) { return this->at(static_cast<std::size_t>(e)); }
  constexpr const Item& at(Enum e) const { return this->at(static_cast<std::size_t>(e)); }

  //! Provide get function, this can be used to assure compile time access in C++11
  template<Enum_ E_> Item& get() noexcept { return std::get<static_cast<std::size_t>(E_)>(*this); }
  template<Enum_ E_> constexpr const Item& get() const noexcept{ return std::get<static_cast<std::size_t>(E_)>(*this); }
};


//! is_enum_array false type
template<typename>
struct is_enum_array : std::false_type {};

//! is_enum_array true type
template<typename Enum_, typename Item_, int Size_>
struct is_enum_array<EnumArray<Enum_, Item_, Size_>> : std::true_type {};

//! is_fullsize_enum_array false type
template<typename>
struct is_fullsize_enum_array : std::false_type {};

//! is_fullsize_enum_array true type
template<typename Enum_, typename Item_>
struct is_fullsize_enum_array<EnumArray<Enum_, Item_, static_cast<int>(Enum_::SIZE)>> : std::true_type {};

} // namespace std_utils
