/*!
 * @file	EnumMap.hpp
 * @author	Philipp Leemann
 * @date	Sep 2, 2016
 */

#pragma once

#include <type_traits> // conditional
#include <unordered_map>

#include <boost/range/adaptor/map.hpp>

namespace std_utils {

struct EnumClassHash
{
    template <typename T>
    typename std::underlying_type<T>::type operator()(T t) const
    {
        return static_cast<typename std::underlying_type<T>::type>(t);
    }
};

template <typename Key>
using HashType = typename std::conditional<std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;

template <typename Key, typename T>
class EnumMap : public std::unordered_map<Key, T, HashType<Key>> {

 public:
  using std::unordered_map<Key, T, HashType<Key>>::unordered_map;

  auto values() const -> decltype(*this | boost::adaptors::map_values)
  { return *this | boost::adaptors::map_values; }

};

} // namespace std_utils
