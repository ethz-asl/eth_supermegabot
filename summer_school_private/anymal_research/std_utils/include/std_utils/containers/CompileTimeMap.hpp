/**
 * @file    CompileTimeMap.hpp
 * @author  Gabriel Hottiger
 * @date    Apr, 2018
 */

#pragma once

// system includes
#include <cassert>
#include <functional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

// std_utils
#include <std_utils/containers/CompileTimeSet.hpp>

namespace std_utils {

/*! \brief Defines the types (e.g. int, myEnum) for a compile-time map (ctm) key-value pair.
     Functions as a base class to KeyValuePair. Used to check the validity of the key-value pairs of the ctm.
    \tparam KeyType_ Defines the type of key of the key-value pair.
    \tparam ValueType_ Defines the type of value of the key-value pair.
*/
template <typename KeyType_, typename ValueType_>
struct KeyValueType {};

/*! \brief Defines the compile time map (ctm) key-value pairs. Ctm consist of a set of key-value pairs.
    \tparam KeyType_ Defines the type of key of the key value pair.
    \tparam ValueType_ Defines the type of value mapped from the key.
    \tparam Key_ Key of the key value pair.
    \tparam Value_ Value of the key value pair.
*/
template <typename KeyType_, typename ValueType_, KeyType_ Key_, ValueType_ Value_>
struct KeyValuePair : public KeyValueType<KeyType_, ValueType_> {};

/*! \brief Defines the compile time map (ctm) as a variadic template.
    \tparam REST_ parameter pack of the ctm.
*/
template <typename... REST_>
class CompileTimeMap {};

/*! \brief Defines the compile time map (ctm) that maps from KeyType_ to ValueType_ as a variadic template.
    \tparam KeyType_ Defines the type of key of the ctm.
    \tparam ValueType_ Defines the type of value of the ctm.
    \tparam REST_ parameter pack of the ctm.
*/
template <typename KeyType_, typename ValueType_, typename... REST_>
class CompileTimeMap<KeyType_, ValueType_, REST_...> {
  //! Allow private access on all template specializations of CompileTimeMap.
  template <typename... Any>
  friend class CompileTimeMap;

 public:
  //! Expose KeyType and ValueType.
  using KeyType = KeyType_;
  using ValueType = ValueType_;

  //! \brief Default implementation of findAll. Defines an empty CompileTimeSet of keys.
  template <ValueType_ value_>
  using findAll = CompileTimeSet<KeyType_>;

  /*! \brief Default implementation of forEach().
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param args Parameter pack passes to the functor.
      \return true.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEach(Args&&... args) {
    return true;
  }

  /*! \brief Default implementation of forEach().
      \param f function object to call.
      \return true.
  */
  static constexpr bool forEach(std::function<bool(const KeyType_&, const ValueType_&)> f) { return true; }

  /*! \brief Default implementation of at().
      \param key key at which ctm is evaluated.
      \return throw std::out_of_range exception.
  */
  static constexpr ValueType_ at(const KeyType_ key) { return atRecursive(key); }

  /*! \brief Default implementation of at().
      \tparam key_ key at which ctm is evaluated.
      \return throw std::out_of_range exception.
  */
  template <KeyType_ key_>
  static constexpr ValueType_ at() {
    return atRecursive(key_);
  }

  /*! \brief Default implementation of contains().
      \param key key at which ctm is evaluated.
      \return false.
  */
  static constexpr bool contains(const KeyType_) { return false; }

  /*! \brief Default implementation of containsRange().
      \param begin first key of range.
      \param end last key of range.
      \return false.
   */
  static constexpr bool containsRange(const KeyType_ begin, const KeyType_ end) { return false; }

  /*! \brief Default implementation of containsValue().
      \param value value to check for.
      \return false.
  */
  static constexpr bool containsValue(const ValueType_) { return false; }

  /*! \brief Default implementation of allValuesUnique().
      \return true.
  */
  static constexpr bool allValuesUnique() { return true; }

  /*! \brief Default implementation of size().
      \return 0.
  */
  static constexpr std::size_t size() { return 0; }

 private:
  //! \brief Implementation helper for findAll. Defines empty CompileTimeSet of keys.
  template <ValueType_ value>
  using findAllRecursive = CompileTimeSet<KeyType_>;

  //! \brief Implementation helper for findVector. Does nothing.
  static void findVectorRecursive(ValueType_ value, std::vector<KeyType_> & vec) {
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first key of range.
      \param end last key of range.
      \param args Parameter pack passes to the functor.
      \return false.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEachRangePreBegin(KeyType_ begin, KeyType_ end, Args&&... args) {
    return false;
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first key of range.
      \param end last key of range.
      \param f function object to call.
      \return false.
   */
  static constexpr bool forEachRangePreBegin(KeyType_ begin, KeyType_ end,
                                             std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return false;
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first key of range.
      \param end last key of range.
      \param args Parameter pack passes to the functor.
      \return false.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEachRangePostBegin(KeyType_ begin, KeyType_ end, Args&&... args) {
    return false;
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first key of range.
      \param end last key of range.
      \param f function object to call.
      \return false.
   */
  static constexpr bool forEachRangePostBegin(KeyType_ begin, KeyType_ end,
                                              std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return false;
  }

  /*! \brief Implementation helper for containsRange().
      \param begin first key of range.
      \param end last key of range.
      \return false.
  */
  static constexpr bool containsRangePostBegin(const KeyType_ begin, const KeyType_ end) { return false; }

  /*! \brief Default implementation of the atRecursive()
      \param key key at which ctm is evaluated.
      \return throw std::out_of_range exception.
  */
  static constexpr ValueType_ atRecursive(const KeyType_ key) {
    return contains(key)
               ? ValueType_()
               : throw std::out_of_range(std::string{"[CompileTimeMap] Tried to get value for key that is not "
                                                     "contained in the map. Value type name: "} +
                                         std::string{typeid(ValueType_).name()} + std::string{" key type name: "} +
                                         std::string{typeid(KeyType_).name()} + std::string{" key value: "} +
                                         std::to_string(static_cast<int>(key)));
  }

  /*! \brief Default implementation of the findRecursive()
      \param value Value to search in map.
      \return throw std::out_of_range
  */
  static constexpr KeyType_ findRecursive(const ValueType_ value) {
    return containsValue(value)
               ? KeyType_()
               : throw std::out_of_range(std::string{"[CompileTimeMap] Tried to find key for value that is not "
                                                     "contained in the map. Value type name: "} +
                                         std::string{typeid(ValueType_).name()} + std::string{" key type name: "} +
                                         std::string{typeid(KeyType_).name()} + std::string{" value: "} +
                                         std::to_string(static_cast<int>(value)));
  }

  /*! \brief Default implementation of the uniqueKey().
      \param key_ key which is tested for uniqueness.
      \return true.
  */
  static constexpr bool uniqueKey(const KeyType_) { return true; }

  /*! \brief Implementation of the uniqueValue().
      \param value value that is checked for uniqueness
      \return true.
  */
  static constexpr bool uniqueValue(const ValueType_) { return true; }
};

/*! \brief Defines the compile time map (ctm) that maps from KeyType_ to ValueType_ but has wrong key-value pairs.
    \tparam KeyType_ Defines the type of key of the ctm.
    \tparam ValueType_ Defines the type of value of the ctm.
    \tparam KeyValueType_ Type of the key-value pair.
    \tparam REST_ parameter pack of the ctm.
*/
template <typename KeyType_, typename ValueType_, typename KeyValueType_, typename... REST_>
class CompileTimeMap<KeyType_, ValueType_, KeyValueType_, REST_...> {
 public:
  //! Expose KeyType and ValueType.
  using KeyType = KeyType_;
  using ValueType = ValueType_;

  //! Assert the KeyValueType is not the base class
  static_assert(std::is_base_of<KeyValueType<KeyType_, ValueType_>, KeyValueType_>::value,
                "[CompileTimeMap] Wrong key-value types! The KeyValuePair does not inherit from KeyValueType_!");
  //! Assert if type is equal to KeyValueType
  static_assert(!std::is_same<KeyValueType<KeyType_, ValueType_>, KeyValueType_>::value,
                "[CompileTimeMap] KeyValueType_ is not a valid KeyValuePair!");
};

template <typename KeyType_, typename ValueType_, KeyType_ Key_, ValueType_ Value_, typename... REST_>
class CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, REST_...> {
  //! Allow private access on all template specializations of CompileTimeMap.
  template <typename... Any>
  friend class CompileTimeMap;

  //! Assert keys that are not unique
  static_assert(CompileTimeMap<KeyType_, ValueType_, REST_...>::uniqueKey(Key_),
                "[CompileTimeMap] Keys are not unique!");

 public:
  //! Expose KeyType and ValueType.
  using KeyType = KeyType_;
  using ValueType = ValueType_;

  //! \brief Find all entries that map to ValueType_ value_. Defines a compile time set of the found values.
  template <ValueType_ value_>
  using findAll =
      cts_insert_front_if_t<KeyType_,
                            typename CompileTimeMap<KeyType_, ValueType_, REST_...>::template findAllRecursive<value_>,
                            Key_, value_ == Value_>;

  /*!\brief Finds all keys that match a specific value, and returns them as a vector.
   * \param value Value for which matching keys are queried
   */
  static std::vector<KeyType_> findVector(ValueType_ value) {
    std::vector<KeyType_> vec;
    CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, REST_...>::findVectorRecursive(value, vec);
    return vec;
  }

  /*! \brief Implementation of forEach: Call F() on every kv-pair until F returns false.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEach(Args&&... args) {
    return F<Key_, Value_>()(args...) &&
           CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEach<F, Args...>(args...);
  }

  /*! \brief Implementation of forEach: Call f() on every kv-pair until f returns false.
      \param f function to call.
      \return true, iff all f's returned true.
  */
  static constexpr bool forEach(std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return f(Key_, Value_) && CompileTimeMap<KeyType_, ValueType_, REST_...>::forEach(f);
  }

  /*! \brief Implementation of forEachRange: Call F() on every kv-pair in [begin, end] until F returns false.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...)
      \tparam Args Variadic template use for function arguments
      \param begin First element in range (F is called with this element)
      \param end Last element in range (F is called with this element)
      \param args Parameter pack passes to the functor
      \return true, iff all F's returned true.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEachRange(KeyType_ begin, KeyType_ end, Args&&... args) {
    return containsRange(begin, end) &&
           (Key_ == begin
                ? F<Key_, Value_>()(args...) &&
                      ((begin == end)
                           ? true
                           : CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEachRangePostBegin<F, Args...>(
                                 begin, end, args...))
                : CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEachRangePreBegin<F, Args...>(begin, end,
                                                                                                            args...));
  }

  /*! \brief Implementation of forEachRange: Call f() on every kv-pair in [begin, end] until f returns false.
      \param begin First element in range (f is called with this element)
      \param end Last element in range (f is called with this element)
      \param f function to call
      \return true, iff all f's returned true.
  */
  static constexpr bool forEachRange(KeyType_ begin, KeyType_ end,
                                     std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return containsRange(begin, end) &&
           (Key_ == begin ? f(Key_, Value_) &&
                                ((begin == end) ? true
                                                : CompileTimeMap<KeyType_, ValueType_, REST_...>::forEachRangePostBegin(
                                                      begin, end, f))
                          : CompileTimeMap<KeyType_, ValueType_, REST_...>::forEachRangePreBegin(begin, end, f));
  }

  /*! \brief  Returns value at key.
      \param key key at which ctm is evaluated.
      \return The Value_ corresponding to the key, iff key contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr ValueType_ at(const KeyType_ key) {
    return (key == Key_) ? Value_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::atRecursive(key);
  }

  /*! \brief Returns value at key_.
      \tparam key_ key at which ctm is evaluated.
      \return The Value_ corresponding to the key, iff key contained in map.
      Throws std::out_of_range exception otherwise.
  */
  template <KeyType_ key_>
  static constexpr ValueType_ at() {
    // Assert if the key is not contained in the map.
    static_assert(contains(key_), "[CompileTimeMap] The key is not contained in the map!");
    return (key_ == Key_) ? Value_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::atRecursive(key_);
  }

  /*! \brief Return first occurrence of value in map.
      \param value Value to search in map.
      \return Key corresponding to the first occurrence of value, iff value contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr KeyType_ find(const ValueType_ value) {
    return (value == Value_) ? Key_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::findRecursive(value);
  }

  /*! \brief Return first occurrence of value in map.
      \tparam value_ Value to search in map.
      \return Key corresponding to the first occurrence of value, iff value contained in map.
      Throws std::out_of_range exception otherwise.
  */
  template <ValueType_ value_>
  static constexpr KeyType_ find() {
    static_assert(containsValue(value_), "[CompileTimeMap] The value is not contained in the map!");
    return (value_ == Value_) ? Key_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::findRecursive(value_);
  }

  /*! \brief Check if key is contained in map.
      \param key key at which ctm is evaluated.
      \return true iff key is contained in the map. False otherwise.
  */
  static constexpr bool contains(const KeyType_ key) {
    return (key == Key_) ? true : CompileTimeMap<KeyType_, ValueType_, REST_...>::contains(key);
  }

  /*! \brief Check if the range [begin, end] is contained in the map.
      \param begin first key of range.
      \param end last key of range.
      \return true iff range is contained in the map. False otherwise.
   */
  static constexpr bool containsRange(const KeyType_ begin, const KeyType_ end) {
    return (begin == end)
               ? contains(begin)
               : ((begin == Key_) ? CompileTimeMap<KeyType_, ValueType_, REST_...>::containsRangePostBegin(begin, end)
                                  : CompileTimeMap<KeyType_, ValueType_, REST_...>::containsRange(begin, end));
  }

  /*! \brief Check if value is contained in map.
      \param value value to check for.
      \return true iff value is contained in the map. False otherwise.
  */
  static constexpr bool containsValue(const ValueType_ value) {
    return (value == Value_) ? true : CompileTimeMap<KeyType_, ValueType_, REST_...>::containsValue(value);
  }

  /*! \brief Check all values in map for uniqueness.
      \return true iff all values are unique. False otherwise.
  */
  static constexpr bool allValuesUnique() {
    return CompileTimeMap<KeyType_, ValueType_, REST_...>::uniqueValue(Value_) &&
           CompileTimeMap<KeyType_, ValueType_, REST_...>::allValuesUnique();
  }

  /*! \brief Return the size of the map.
      \return size of the map.
  */
  static constexpr std::size_t size() { return 1 + CompileTimeMap<KeyType_, ValueType_, REST_...>::size(); }

 private:
  //! \brief Implementation helper for findAll. Defines empty CompileTimeSet of keys.
  template <ValueType_ value_>
  using findAllRecursive =
      cts_insert_front_if_t<KeyType_,
                            typename CompileTimeMap<KeyType_, ValueType_, REST_...>::template findAllRecursive<value_>,
                            Key_, value_ == Value_>;

  //! \brief Implementation helper for findVector. If value matches adds key to vector.
  static void findVectorRecursive(ValueType_ value, std::vector<KeyType_> & vec) {
    if(value == Value_) {
      vec.push_back(Key_);
    }
    CompileTimeMap<KeyType_, ValueType_, REST_...>::findVectorRecursive(value, vec);
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first key of range.
      \param end last key of range.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEachRangePreBegin(KeyType_ begin, KeyType_ end, Args&&... args) {
    return (Key_ == begin
                ? F<Key_, Value_>()(args...) &&
                      ((begin == end)
                           ? true
                           : CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEachRangePostBegin<F, Args...>(
                                 begin, end, args...))
                : CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEachRangePreBegin<F, Args...>(begin, end,
                                                                                                            args...));
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first key of range.
      \param end last key of range.
      \param f function object to call.
      \return true, iff all f's returned true.
  */
  static constexpr bool forEachRangePreBegin(KeyType_ begin, KeyType_ end,
                                             std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return (Key_ == begin ? f(Key_, Value_) &&
                                ((begin == end) ? true
                                                : CompileTimeMap<KeyType_, ValueType_, REST_...>::forEachRangePostBegin(
                                                      begin, end, f))
                          : CompileTimeMap<KeyType_, ValueType_, REST_...>::forEachRangePreBegin(begin, end, f));
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Key and Value implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first key of range.
      \param end last key of range.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <KeyType_, ValueType_> class F, class... Args>
  static constexpr bool forEachRangePostBegin(KeyType_ begin, KeyType_ end, Args&&... args) {
    return Key_ == end ? F<Key_, Value_>()(args...)
                       : F<Key_, Value_>()(args...) &&
                             CompileTimeMap<KeyType_, ValueType_, REST_...>::template forEachRangePostBegin<F, Args...>(
                                 begin, end, args...);
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first key of range.
      \param end last key of range.
      \param f function object to call.
      \return true, iff all f's returned true.
  */
  static constexpr bool forEachRangePostBegin(KeyType_ begin, KeyType_ end,
                                              std::function<bool(const KeyType_&, const ValueType_&)> f) {
    return Key_ == end ? f(Key_, Value_)
                       : f(Key_, Value_) &&
                             CompileTimeMap<KeyType_, ValueType_, REST_...>::forEachRangePostBegin(begin, end, f);
  }

  /*! \brief Implementation helper for containsRange().
      \param begin first key of range.
      \param end last key of range.
      \return true, iff range is contained in map.
  */
  static constexpr bool containsRangePostBegin(const KeyType_ begin, const KeyType_ end) {
    return (end == Key_) ? true : CompileTimeMap<KeyType_, ValueType_, REST_...>::containsRangePostBegin(begin, end);
  }

  /*! \brief Implementation helper for at().
      \param key key at which ctm is evaluated.
      \return The Value_ corresponding to the key, iff key contained in map. Throws std::out_of_range exception
     otherwise.
  */
  static constexpr ValueType_ atRecursive(const KeyType_ key) {
    return (key == Key_) ? Value_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::atRecursive(key);
  }

  /*! \brief Implementation helper for find().
      \param value value at which ctm is evaluated.
      \return The Key_ corresponding to the value, iff value contained in map. Throws std::out_of_range exception
     otherwise.
  */
  static constexpr KeyType_ findRecursive(const ValueType_ value) {
    return (value == Value_) ? Key_ : CompileTimeMap<KeyType_, ValueType_, REST_...>::findRecursive(value);
  }

  /*! \brief Helper for assertion that only unique keys are allowed in map.
      \param key key that is checked for uniqueness
      \return true iff all keys in the map are unique. False otherwise.
  */
  static constexpr bool uniqueKey(const KeyType_ key) {
    return (key == Key_) ? false : CompileTimeMap<KeyType_, ValueType_, REST_...>::uniqueKey(key);
  }

  /*! \brief Implementation helper for allValuesUnique().
      \param value value that is checked for uniqueness
      \return true iff all values in the map are unique. False otherwise.
  */
  static constexpr bool uniqueValue(const ValueType_ value) {
    return (value == Value_) ? false : CompileTimeMap<KeyType_, ValueType_, REST_...>::uniqueValue(value);
  }
};

//! isCompileTimeMap false type
template <typename... REST_>
struct is_compile_time_map : std::false_type {};

//! isCompileTimeMap true type
template <typename... REST_>
struct is_compile_time_map<CompileTimeMap<REST_...>> : std::true_type {};

/**
 * In the following some useful helper traits to modify maps are provided.
 *
 * IDE Support:
 * To make it easier for the IDE's to deduce the type of the output map, we always pass the KeyType_ and the ValueType_
 * to these traits as template parameters. They could also be auto-deduced. Furthermore in the default implementation of
 * the traits we add a 'type' typedef that defines a default (empty) map.
 *
 * Compiler Output:
 * To increase compiler output quality slightly, static_asserts are added in the default implementation of the traits.
 * They will always fail, since the default implementation is not valid.
 * Still compiler outputs are hard to read for CompileTimeMaps.
 */

/**
 * @brief Default implementation of map concatenation
 * @tparam KeyType_     Key type of the maps
 * @tparam ValueType_   Value type of the maps
 * @tparam Map1_        First Map
 * @tparam Map2_        Second Map
 */
template <typename KeyType_, typename ValueType_, typename Map1_, typename Map2_>
struct ctm_concatenate {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<KeyType_, KeyType_>::value, "[ctm_concatenate] Could not concatenate maps!");
};

//! @brief Template specialization for CompileTimeMaps of same KV-type
template <typename KeyType_, typename ValueType_, typename... Kv1_, typename... Kv2_>
struct ctm_concatenate<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, Kv1_...>,
                       CompileTimeMap<KeyType_, ValueType_, Kv2_...>> {
  using type = CompileTimeMap<KeyType_, ValueType_, Kv1_..., Kv2_...>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map1_, typename Map2_>
using ctm_concatenate_t = typename ctm_concatenate<KeyType_, ValueType_, Map1_, Map2_>::type;

/**
 * @brief Default implementation of back insertion of a kv-pair into a map
 * @tparam KeyType_     Key type of the map
 * @tparam ValueType_   Value type of the map
 * @tparam Map_         Map to back insert kv
 * @tparam Kv_          KeyValuePair to insert
 */
template <typename KeyType_, typename ValueType_, typename Map_, typename Kv_>
struct ctm_insert_back {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_insert_back] Could not back-insert KV into map!");
};

//! @brief Template specialization for CompileTimeMap with same KV Types as KeyValuePair
template <typename KeyType_, typename ValueType_, typename... Kv_, KeyType_ key, ValueType_ value>
struct ctm_insert_back<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, Kv_...>,
                       KeyValuePair<KeyType_, ValueType_, key, value>> {
  using type = CompileTimeMap<KeyType_, ValueType_, Kv_..., KeyValuePair<KeyType_, ValueType_, key, value>>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map_, typename Kv_>
using ctm_insert_back_t = typename ctm_insert_back<KeyType_, ValueType_, Map_, Kv_>::type;

/**
 * @brief Default implementation of front insertion of a kv-pair into a map
 * @tparam KeyType_     Key type of the map
 * @tparam ValueType_   Value type of the map
 * @tparam Map_         Map to front insert kv
 * @tparam Kv_          KeyValuePair to insert
 */
template <typename KeyType_, typename ValueType_, typename Map_, typename Kv_>
struct ctm_insert_front {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_insert_front] Could not front-insert KV into map!");
};

//! @brief Template specialization for CompileTimeMap with same KV Types as KeyValuePair
template <typename KeyType_, typename ValueType_, typename... Kv_, KeyType_ key, ValueType_ value>
struct ctm_insert_front<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, Kv_...>,
                        KeyValuePair<KeyType_, ValueType_, key, value>> {
  using type = CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, key, value>, Kv_...>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map_, typename Kv_>
using ctm_insert_front_t = typename ctm_insert_front<KeyType_, ValueType_, Map_, Kv_>::type;

/**
 * @brief Default implementation of erasing a kv-pair from a map
 * @tparam KeyType_     Key type of the map
 * @tparam ValueType_   Value type of the map
 * @tparam Map_         Map to erase kv from
 * @tparam erase_key    Key corresponding to KV pair to erase
 */
template <typename KeyType_, typename ValueType_, typename Map_, KeyType_ erase_key>
struct ctm_erase {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_erase] Could not erase KV from map!");
};

//! @brief Template specialization for CompileTimeMap with same KeyType as erase_key (recursion termination)
template <typename KeyType_, typename ValueType_, typename... KV_, KeyType_ erase_key>
struct ctm_erase<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, KV_...>, erase_key> {
  using type = CompileTimeMap<KeyType_, ValueType_>;
};

//! @brief Template specialization for CompileTimeMap with same KeyType as erase_key
template <typename KeyType_, typename ValueType_, typename... KV_, KeyType_ key, ValueType_ value, KeyType_ erase_key>
struct ctm_erase<KeyType_, ValueType_,
                 CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, key, value>, KV_...>,
                 erase_key> {
  using type = typename std::conditional<
      key == erase_key, CompileTimeMap<KeyType_, ValueType_, KV_...>,
      ctm_insert_front_t<
          KeyType_, ValueType_,
          typename ctm_erase<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, KV_...>, erase_key>::type,
          KeyValuePair<KeyType_, ValueType_, key, value>>>::type;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map_, KeyType_ erase_key>
using ctm_erase_t = typename ctm_erase<KeyType_, ValueType_, Map_, erase_key>::type;

/**
 * @brief Default implementation of adding a given summand to all values in the map
 * @tparam KeyType_     Key type of the map
 * @tparam ValueType_   Value type of the map
 * @tparam Map_         Map containing values that shall be modified
 * @tparam summand      Value to be added
 */
template <typename KeyType_, typename ValueType_, typename Map_, ValueType_ summand>
struct ctm_add_to_values {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_add_to_values] Could not add summand to values of map!");
};

//! @brief Template specialization for CompileTimeMap with same ValueType as summand (recursion termination)
template <typename KeyType_, typename ValueType_, ValueType_ summand>
struct ctm_add_to_values<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_>, summand> {
  using type = CompileTimeMap<KeyType_, ValueType_>;
};

//! @brief Template specialization for CompileTimeMap with same ValueType as summand
template <typename KeyType_, typename ValueType_, typename... Kv_, KeyType_ Key_, ValueType_ Value_, ValueType_ summand>
struct ctm_add_to_values<KeyType_, ValueType_,
                         CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, Kv_...>,
                         summand> {
  using type = ctm_insert_front_t<
      KeyType_, ValueType_,
      typename ctm_add_to_values<KeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, Kv_...>, summand>::type,
      KeyValuePair<KeyType_, ValueType_, Key_, Value_ + summand>>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map_, ValueType_ summand>
using ctm_add_to_values_t = typename ctm_add_to_values<KeyType_, ValueType_, Map_, summand>::type;

/**
 * @brief  Default implementation of creating a map out of two sets
 * @tparam KeyType_     Key type of the map
 * @tparam ValueType_   Value type of the map
 * @tparam KeySet_      CompileTimeSet of keys
 * @tparam ValueSet_    CompileTimeSet of values
 */
template <typename KeyType_, typename ValueType_, typename KeySet_, typename ValueSet_>
struct ctm_from_sets {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<KeyType_, KeyType_>::value, "[ctm_from_sets] Could not create map from sets!");
};

//! @brief Template specialization for CompileTimeSets with proper KeyType_/ValueType_ (Recursion termination)
template <typename KeyType_, typename ValueType_>
struct ctm_from_sets<KeyType_, ValueType_, CompileTimeSet<KeyType_>, CompileTimeSet<ValueType_>> {
  using type = CompileTimeMap<KeyType_, ValueType_>;
};

//! @brief Template specialization for CompileTimeSets with proper KeyType_/ValueType_
template <typename KeyType_, KeyType_ key, KeyType_... keys, typename ValueType_, ValueType_ value,
          ValueType_... values>
struct ctm_from_sets<KeyType_, ValueType_, CompileTimeSet<KeyType_, key, keys...>,
                     CompileTimeSet<ValueType_, value, values...>> {
  //! Make sure sets have same size
  static_assert(CompileTimeSet<KeyType_, key, keys...>::size() == CompileTimeSet<ValueType_, value, values...>::size(),
                "[ctm_from_sets] Could not create map from sets! Sets have different sizes!");
  using type = ctm_insert_front_t<KeyType_, ValueType_,
                                  typename ctm_from_sets<KeyType_, ValueType_, CompileTimeSet<KeyType_, keys...>,
                                                         CompileTimeSet<ValueType_, values...>>::type,
                                  KeyValuePair<KeyType_, ValueType_, key, value>>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename KeySet_, typename ValueSet_>
using ctm_from_sets_t = typename ctm_from_sets<KeyType_, ValueType_, KeySet_, ValueSet_>::type;

/**
 * @brief  Default implementation of transforming map to different key-value types
 * @tparam TransformedKeyType_     Key type of the new map
 * @tparam TransformedValueType_   Value type of the new map
 * @tparam Map_                    Map before transformation
 * @tparam KeyMap_                 Map that maps the keys to the transformed key type
 * @tparam ValueMap_               Map that maps the values to the transformed value type
 */
template <typename TransformedKeyType_, typename TransformedValueType_, typename Map_, typename KeyMap_,
          typename ValueMap_>
struct ctm_transform {
  //! IDE helper type
  using type = CompileTimeMap<TransformedKeyType_, TransformedValueType_, Map_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_transform] Could not transform map!");
};

//! @brief Template specialization for proper KeyMap_ and ValueMap_ (recursion termination)
template <typename KeyType_, typename ValueType_, typename TransformedKeyType_, typename... Keys_,
          typename TransformedValueType_, typename... Values_>
struct ctm_transform<TransformedKeyType_, TransformedValueType_, CompileTimeMap<KeyType_, ValueType_>,
                     CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>,
                     CompileTimeMap<ValueType_, TransformedValueType_, Values_...>> {
  using type = CompileTimeMap<TransformedKeyType_, TransformedValueType_>;
};

//! @brief Template specialization for proper KeyMap_ and ValueMap_
template <typename KeyType_, typename ValueType_, KeyType_ Key_, ValueType_ Value_, typename... KV_,
          typename TransformedKeyType_, typename... Keys_, typename TransformedValueType_, typename... Values_>
struct ctm_transform<TransformedKeyType_, TransformedValueType_,
                     CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, KV_...>,
                     CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>,
                     CompileTimeMap<ValueType_, TransformedValueType_, Values_...>> {
  using type = ctm_insert_front_t<
      TransformedKeyType_, TransformedValueType_,
      typename ctm_transform<TransformedKeyType_, TransformedValueType_, CompileTimeMap<KeyType_, ValueType_, KV_...>,
                             CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>,
                             CompileTimeMap<ValueType_, TransformedValueType_, Values_...>>::type,
      KeyValuePair<TransformedKeyType_, TransformedValueType_,
                   CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>::at(Key_),
                   CompileTimeMap<ValueType_, TransformedValueType_, Values_...>::at(Value_)>>;
};

//! @brief Template specialization for proper KeyMap_ and empty ValueMap_ (recursion termination)
template <typename KeyType_, typename ValueType_, typename TransformedKeyType_, typename... Keys_>
struct ctm_transform<TransformedKeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_>,
                     CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>, void> {
  using type = CompileTimeMap<TransformedKeyType_, ValueType_>;
};

//! @brief Template specialization for proper KeyMap_ and empty ValueMap_
template <typename KeyType_, typename ValueType_, KeyType_ Key_, ValueType_ Value_, typename... KV_,
          typename TransformedKeyType_, typename... Keys_>
struct ctm_transform<TransformedKeyType_, ValueType_,
                     CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, KV_...>,
                     CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>, void> {
  using type = ctm_insert_front_t<
      TransformedKeyType_, ValueType_,
      typename ctm_transform<TransformedKeyType_, ValueType_, CompileTimeMap<KeyType_, ValueType_, KV_...>,
                             CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>, void>::type,
      KeyValuePair<TransformedKeyType_, ValueType_, CompileTimeMap<KeyType_, TransformedKeyType_, Keys_...>::at(Key_),
                   Value_>>;
};

//! @brief Template specialization for empty KeyMap_ and proper ValueMap_ (recursion termination)
template <typename KeyType_, typename ValueType_, typename TransformedValueType_, typename... Values_>
struct ctm_transform<KeyType_, TransformedValueType_, CompileTimeMap<KeyType_, ValueType_>, void,
                     CompileTimeMap<ValueType_, TransformedValueType_, Values_...>> {
  using type = CompileTimeMap<KeyType_, TransformedValueType_>;
};

//! @brief Template specialization for empty KeyMap_ and proper ValueMap_
template <typename KeyType_, typename ValueType_, KeyType_ Key_, ValueType_ Value_, typename... KV_,
          typename TransformedValueType_, typename... Values_>
struct ctm_transform<KeyType_, TransformedValueType_,
                     CompileTimeMap<KeyType_, ValueType_, KeyValuePair<KeyType_, ValueType_, Key_, Value_>, KV_...>,
                     void, CompileTimeMap<ValueType_, TransformedValueType_, Values_...>> {
  using type = ctm_insert_front_t<
      KeyType_, TransformedValueType_,
      typename ctm_transform<KeyType_, TransformedValueType_, CompileTimeMap<KeyType_, ValueType_, KV_...>, void,
                             CompileTimeMap<ValueType_, TransformedValueType_, Values_...>>::type,
      KeyValuePair<KeyType_, TransformedValueType_, Key_,
                   CompileTimeMap<ValueType_, TransformedValueType_, Values_...>::at(Value_)>>;
};

//! Helper
template <typename TransformedKeyType_, typename TransformedValueType_, typename Map_, typename KeyMap_,
          typename ValueMap_ = void>
using ctm_transform_t =
    typename ctm_transform<TransformedKeyType_, TransformedValueType_, Map_, KeyMap_, ValueMap_>::type;

/**
 * @brief  Default implementation of combining two map of different key-value types
 * @tparam KeyType_     Key type of the new map
 * @tparam ValueType_   Value type of the new map
 * @tparam Map1_        InputMap1 before transformation
 * @tparam Map2_        InputMap2 before transformation
 * @tparam Map1Keys_    Maps keys of Map1_ to KeyType_
 * @tparam Map2Keys_    Maps keys of Map2_ to KeyType_
 * @tparam Map1Values_  Maps values of Map1_ to ValueType_
 * @tparam Map2Values_  Maps values of Map2_ to ValueType_
 */
template <typename KeyType_, typename ValueType_, typename Map1_, typename Map2_, typename Map1Keys_,
          typename Map2Keys_, typename Map1Values_ = void, typename Map2Values_ = void>
struct ctm_combine {
  //! IDE helper type
  using type = CompileTimeMap<KeyType_, ValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<KeyType_, KeyType_>::value, "[ctm_combine] Could not combine maps!");
};

//! @brief Template specialization for proper KeyMaps and ValueMaps
template <typename KeyType_, typename ValueType_, typename KeyType1_, typename ValueType1_, typename KeyType2_,
          typename ValueType2_, typename... KV1_, typename... KVKeys1_, typename... KVValues1_, typename... KV2_,
          typename... KVKeys2_, typename... KVValues2_>
struct ctm_combine<KeyType_, ValueType_, CompileTimeMap<KeyType1_, ValueType1_, KV1_...>,
                   CompileTimeMap<KeyType2_, ValueType2_, KV2_...>, CompileTimeMap<KeyType1_, KeyType_, KVKeys1_...>,
                   CompileTimeMap<KeyType2_, KeyType_, KVKeys2_...>,
                   CompileTimeMap<ValueType1_, ValueType_, KVValues1_...>,
                   CompileTimeMap<ValueType2_, ValueType_, KVValues2_...>> {
  using type = std_utils::ctm_concatenate_t<
      KeyType_, ValueType_,
      ctm_transform_t<KeyType_, ValueType_, CompileTimeMap<KeyType1_, ValueType1_, KV1_...>,
                      CompileTimeMap<KeyType1_, KeyType_, KVKeys1_...>,
                      CompileTimeMap<ValueType1_, ValueType_, KVValues1_...>>,
      ctm_transform_t<KeyType_, ValueType_, CompileTimeMap<KeyType2_, ValueType2_, KV2_...>,
                      CompileTimeMap<KeyType2_, KeyType_, KVKeys2_...>,
                      CompileTimeMap<ValueType2_, ValueType_, KVValues2_...>>>;
};

//! @brief Template specialization for proper KeyMaps and empty ValueMaps
template <typename KeyType_, typename ValueType_, typename KeyType1_, typename KeyType2_, typename... KV1_,
          typename... KVKeys1_, typename... KV2_, typename... KVKeys2_>
struct ctm_combine<KeyType_, ValueType_, CompileTimeMap<KeyType1_, ValueType_, KV1_...>,
                   CompileTimeMap<KeyType2_, ValueType_, KV2_...>, CompileTimeMap<KeyType1_, KeyType_, KVKeys1_...>,
                   CompileTimeMap<KeyType2_, KeyType_, KVKeys2_...>, void, void> {
  using type =
      std_utils::ctm_concatenate_t<KeyType_, ValueType_,
                                   ctm_transform_t<KeyType_, ValueType_, CompileTimeMap<KeyType1_, ValueType_, KV1_...>,
                                                   CompileTimeMap<KeyType1_, KeyType_, KVKeys1_...>, void>,
                                   ctm_transform_t<KeyType_, ValueType_, CompileTimeMap<KeyType2_, ValueType_, KV2_...>,
                                                   CompileTimeMap<KeyType2_, KeyType_, KVKeys2_...>, void>>;
};

//! Helper
template <typename KeyType_, typename ValueType_, typename Map1_, typename Map2_, typename Map1Keys_,
          typename Map2Keys_, typename Map1Values_ = void, typename Map2Values_ = void>
using ctm_combine_t =
    typename ctm_combine<KeyType_, ValueType_, Map1_, Map2_, Map1Keys_, Map2Keys_, Map1Values_, Map2Values_>::type;

/**
 * @brief  Default implementation of combining two map of different key-value types
 * @tparam Enum_        Enum to map to
 * @tparam SubEnum_     Enum from switch to map
 * @tparam Offset_      Offset for static casting (offset of SubEnum_ in new enum Enum_)
 * @tparam Skip_        Skips Skip_ number of elements at the beginning of SubEnum_
 */
template <typename Enum_, typename SubEnum_, std::size_t Offset_ = 0, std::size_t Skip_ = 0>
struct ctm_from_subenum {
  using ctsValues = cts_from_sequence_t<
      Enum_, make_index_sequence_range_t<Offset_, Offset_ - Skip_ + static_cast<std::size_t>(SubEnum_::SIZE)>>;
  using ctsKeys =
      cts_from_sequence_t<SubEnum_, make_index_sequence_range_t<Skip_, static_cast<std::size_t>(SubEnum_::SIZE)>>;
  using type = ctm_from_sets_t<SubEnum_, Enum_, ctsKeys, ctsValues>;
};

//! Helper
template <typename Enum_, typename SubEnum_, std::size_t Offset_ = 0, std::size_t Skip_ = 0>
using ctm_from_subenum_t = typename ctm_from_subenum<Enum_, SubEnum_, Offset_, Skip_>::type;

/**
 * @brief Default implementation of inverting a unique map
 * @tparam InvertedKeyType_     Key type of the inverted map, value type of the map
 * @tparam InvertedValueType_   Value type of the inverted map, key type of the map
 * @tparam Map_         Map containing mappings that shall be inverted
 */
template <typename InvertedKeyType_, typename InvertedValueType_, typename Map_>
struct ctm_invert {
  //! IDE helper type
  using type = CompileTimeMap<InvertedKeyType_, InvertedValueType_>;
  //! Improve compiler output
  static_assert(!std::is_same<Map_, Map_>::value, "[ctm_invert] Could not invert map!");
};

//! @brief Template specialization for CompileTimeMap with proper KeyType_ and ValueType_
template <typename InvertedKeyType_, typename InvertedValueType_>
struct ctm_invert<InvertedKeyType_, InvertedValueType_, CompileTimeMap<InvertedValueType_, InvertedKeyType_>> {
  using type = CompileTimeMap<InvertedKeyType_, InvertedValueType_>;
};

//! @brief Template specialization for CompileTimeMap with proper KeyType_ and ValueType_
template <typename InvertedKeyType_, typename InvertedValueType_, typename... Kv_, InvertedKeyType_ InvertedKey_, InvertedValueType_ InvertedValue_>
struct ctm_invert<InvertedKeyType_, InvertedValueType_,
                  CompileTimeMap<InvertedValueType_, InvertedKeyType_, KeyValuePair<
                      InvertedValueType_, InvertedKeyType_, InvertedValue_, InvertedKey_>, Kv_...>> {
  static_assert(CompileTimeMap<InvertedValueType_, InvertedKeyType_, KeyValuePair<
      InvertedValueType_, InvertedKeyType_, InvertedValue_, InvertedKey_>, Kv_...>::allValuesUnique(),
                "[ctm_invert] Could not invert non-unique map!");
  using type = ctm_insert_front_t<
      InvertedKeyType_, InvertedValueType_,
      typename ctm_invert<InvertedKeyType_, InvertedValueType_, CompileTimeMap<InvertedValueType_, InvertedKeyType_, Kv_...>>::type,
      KeyValuePair<InvertedKeyType_, InvertedValueType_, InvertedKey_, InvertedValue_>>;
};

//! Helper
template <typename InvertedKeyType_, typename InvertedValueType_, typename Map_>
using ctm_invert_t = typename ctm_invert<InvertedKeyType_, InvertedValueType_, Map_>::type;

}  // namespace std_utils
