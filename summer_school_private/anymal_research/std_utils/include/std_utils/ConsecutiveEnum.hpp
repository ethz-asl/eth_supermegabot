/*!
 * @file	 ConsecutiveEnum.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */
#pragma once

#include <std_utils/containers/CompileTimeMap.hpp>
#include <iterator>

namespace std_utils {
/**
 * @brief Macro for simplified creation of enums
 * @usage CONSECUTIVE_ENUM(BranchEnum, LF, RF, LH, RH) expands to:
 *
 *        enum class BranchEnum : unsigned int { LF = 0, RF, LH, RH, SIZE };
 *
 *        THIS CREATES THE LAST ELEMENT "SIZE" FOR YOU, NEVER PROVIDE ONE YOURSELF!
 */
#define CONSECUTIVE_ENUM( Name, FirstElement, ... ) enum class Name : unsigned int { FirstElement = 0, ## __VA_ARGS__, SIZE };

#define CONSECUTIVE_ENUM_FROM_LIST( Name, List) CONSECUTIVE_ENUM(Name, List)

#define CONSECUTIVE_ENUM_EMPTY( Name ) enum class Name : unsigned int { SIZE = 0 };


/*
 * Iterator for CONSECUTIVE_ENUM.
 * Original Source: https://github.com/glampert/enum_helpers/blob/master/enum_helpers.hpp
 *
 * Example usage:
 *    CONSECUTIVE_ENUM(BranchEnum, LF, RF, LH, RH)
 *    std_utils::enum_iterator<BranchEnum> iter;
 *    for (auto branch : iter) {
 *      std::cout << "branch = " << static_cast<int>(branch) << "\n";
 *    }
 */
template
< typename EnumType_, EnumType_ last_ = EnumType_::SIZE >
class enum_iterator final : public std::iterator<std::forward_iterator_tag, EnumType_>
{
public:

    using integer_type = typename std::underlying_type<EnumType_>::type;

    constexpr enum_iterator() noexcept
        : id_{ 0 } // Assumes 0 is the first constant
    { }

    constexpr enum_iterator(const EnumType_ index) noexcept
        : id_{ static_cast<integer_type>(index) }
    { }

    enum_iterator operator++() // pre-increment
    {
        id_ += 1;
        return *this;
    }

    enum_iterator operator++(int) // post-increment
    {
        enum_iterator old_val{ *this };
        id_ += 1;
        return old_val;
    }

    EnumType_ operator*() const
    {
        return static_cast<EnumType_>(id_);
    }

    bool operator == (const enum_iterator& other) const noexcept { return id_ == other.id_; }
    bool operator != (const enum_iterator& other) const noexcept { return id_ != other.id_; }
    bool operator <  (const enum_iterator& other) const noexcept { return id_ <  other.id_; }
    bool operator >  (const enum_iterator& other) const noexcept { return id_ >  other.id_; }
    bool operator <= (const enum_iterator& other) const noexcept { return id_ <= other.id_; }
    bool operator >= (const enum_iterator& other) const noexcept { return id_ >= other.id_; }

    enum_iterator begin() const { return enum_iterator{}; }
    enum_iterator end()   const { return enum_iterator{ last_ }; }

private:
    integer_type id_;
};

//! True if dim is a subset of dimVector.
template <typename ArrayType, typename EnumeType>
static inline bool containsEnum(const ArrayType& dimVector, EnumeType dim) {
  return std::find(dimVector.begin(), dimVector.end(), dim) != dimVector.end();
}

//! True if dims is a subset of dimVector.
template <typename ArrayType1, typename ArrayType2>
static bool containsEnums(const ArrayType1& dimVector, const ArrayType2& dims) {
  for (const auto dim : dims) {
    if (!containsEnum(dimVector, dim)) { return false; }
  }
  return true;
}

//! True if dimVector is equal to dims.
template <typename ArrayType1, typename ArrayType2>
static bool consistsOfEnums(const ArrayType1& dimVector, const ArrayType2& dims) {
  if (dimVector.size() != dims.size()) { return false; }
  return containsEnums(dimVector, dims);
}

} /* namespace std_utils */
