/**
 * @file    CompileTimeSet.hpp
 * @author  Gabriel Hottiger
 * @date    Apr, 2018
 */

#pragma once

// system includes
#include <cassert>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>

// std_utils
#include "std_utils/integer_sequence.hpp"

namespace std_utils {

/*! \brief Defines the compile time set (cts) as a variadic template.
    \tparam REST_ parameter pack of the cts.
*/
template <typename Type_, Type_... Args>
class CompileTimeSet {};

/*! \brief Defines the compile time set (cts) that contains Type_ as a variadic template.
    \tparam Type_ Defines the type contained in the set.
    \tparam REST_ parameter pack of the cts.
*/
template <typename Type_>
class CompileTimeSet<Type_> {
  //! Allow private access on all template specializations of CompileTimeSet.
  template <typename T_, T_... Args>
  friend class CompileTimeSet;

 public:
  //! Expose Type.
  using Type = Type_;

  /*! \brief Default implementation of forEach().
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param args Parameter pack passes to the functor.
      \return true.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEach(Args&&... args) {
    return true;
  }

  /*! \brief Default implementation of forEach().
      \param f function object to call.
      \return true.
  */
  static constexpr bool forEach(std::function<bool(const Type_&)> f) { return true; }

  /*! \brief Default implementation of contains().
      \param type type at which cts is evaluated.
      \return false
  */
  static constexpr bool contains(const Type_) { return false; }

  /*! \brief Default implementation of containsRange().
      \param begin first type of range.
      \param end last type of range.
      \return false.
   */
  static constexpr bool containsRange(const Type_ begin, const Type_ end) { return false; }

  /*! \brief Default implementation of size().
      \return 0.
  */
  static constexpr std::size_t size() { return 0; }

 private:
  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first type of range.
      \param end last type of range.
      \param args Parameter pack passes to the functor.
      \return false.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEachRangePreBegin(Type_ begin, Type_ end, Args&&... args) {
    return false;
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first type of range.
      \param end last type of range.
      \param f function object to call.
      \return false.
  */
  static constexpr bool forEachRangePreBegin(Type_ begin, Type_ end, std::function<bool(const Type_&)> f) {
    return false;
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first type of range.
      \param end last type of range.
      \param args Parameter pack passes to the functor.
      \return false.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEachRangePostBegin(Type_ begin, Type_ end, Args&&... args) {
    return false;
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first type of range.
      \param end last type of range.
      \param f function object to call.
      \return false.
  */
  static constexpr bool forEachRangePostBegin(Type_ begin, Type_ end, std::function<bool(const Type_&)> f) {
    return false;
  }

  /*! \brief Implementation helper for containsRange().
      \param begin first type of range.
      \param end last type of range.
      \return false.
  */
  static constexpr bool containsRangePostBegin(const Type_ begin, const Type_ end) { return false; }

  /*! \brief Default implementation of the unique().
      \param Type_ type which is tested for uniqueness.
      \return true.
  */
  static constexpr bool unique(const Type_) { return true; }
};

/*! \brief Defines the compile time set (cts) that contains Type_ as a variadic template.
    \tparam Type_ Defines the type contained in the set.
    \tparam REST_ parameter pack of the cts.
*/
template <typename Type_, Type_ Value_, Type_... REST_>
class CompileTimeSet<Type_, Value_, REST_...> {
  template <typename T_, T_... Args>
  friend class CompileTimeSet;

  //! Assert keys that are not unique
  static_assert(CompileTimeSet<Type_, REST_...>::unique(Value_), "Types are not unique!");

 public:
  //! Expose Type
  using Type = Type_;

  /*! \brief Implementation of forEach: Call F() on every type until F returns false.
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEach(Args&&... args) {
    return F<Value_>()(args...) && CompileTimeSet<Type_, REST_...>::template forEach<F, Args...>(args...);
  }

  /*! \brief Implementation of forEach: Call f() on every type until f returns false.
      \param f function to call.
      \return true, iff all f's returned true.
   */
  static constexpr bool forEach(std::function<bool(const Type_&)> f) {
    return f(Value_) && CompileTimeSet<Type_, REST_...>::forEach(f);
  }

  /*! \brief Implementation of forEachRange: Call F() on every type in [begin, end] until F returns false.
      \tparam F Functor type that is templated on Type implements operator()(Args...)
      \tparam Args Variadic template use for function arguments
      \param begin First element in range (F is called with this element)
      \param end Last element in range (F is called with this element)
      \param args Parameter pack passes to the functor
      \return true, iff all F's returned true.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEachRange(Type_ begin, Type_ end, Args&&... args) {
    return containsRange(begin, end) &&
           (Value_ == begin
                ? F<Value_>()(args...) &&
                      ((begin == end) ? true
                                      : CompileTimeSet<Type_, REST_...>::template forEachRangePostBegin<F, Args...>(
                                            begin, end, args...))
                : CompileTimeSet<Type_, REST_...>::template forEachRangePreBegin<F, Args...>(begin, end, args...));
  }

  /*! \brief Implementation of forEachRange: Call f() on every type in [begin, end] until f returns false.
      \param begin First element in range (f is called with this element)
      \param end Last element in range (f is called with this element)
      \param f function to call
      \return true, iff all f's returned true.
   */
  static constexpr bool forEachRange(Type_ begin, Type_ end, std::function<bool(const Type_&)> f) {
    return containsRange(begin, end) &&
           (Value_ == begin
                ? f(Value_) &&
                      ((begin == end) ? true : CompileTimeSet<Type_, REST_...>::forEachRangePostBegin(begin, end, f))
                : CompileTimeSet<Type_, REST_...>::forEachRangePreBegin(begin, end, f));
  }

  /*! \brief Check if type is contained in set.
      \param t type at which cts is evaluated.
      \return true iff type is contained in the set. False otherwise.
  */
  static constexpr bool contains(const Type_ t) {
    return (t == Value_) ? true : CompileTimeSet<Type_, REST_...>::contains(t);
  }

  /*! \brief Check if the range [begin, end] is contained in the set.
      \param begin first type of range.
      \param end last type of range.
      \return true iff range is contained in the set. False otherwise.
   */
  static constexpr bool containsRange(const Type_ begin, const Type_ end) {
    return (begin == end) ? contains(begin)
                          : ((begin == Value_) ? CompileTimeSet<Type_, REST_...>::containsRangePostBegin(begin, end)
                                               : CompileTimeSet<Type_, REST_...>::containsRange(begin, end));
  }

  /*! \brief Return the size of the set.
      \return size of the set.
  */
  static constexpr std::size_t size() { return 1 + CompileTimeSet<Type_, REST_...>::size(); }

 private:
  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first type of range.
      \param end last type of range.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEachRangePreBegin(Type_ begin, Type_ end, Args&&... args) {
    return (Value_ == begin
                ? F<Value_>()(args...) &&
                      ((begin == end) ? true
                                      : CompileTimeSet<Type_, REST_...>::template forEachRangePostBegin<F, Args...>(
                                            begin, end, args...))
                : CompileTimeSet<Type_, REST_...>::template forEachRangePreBegin<F, Args...>(begin, end, args...));
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first type of range.
      \param end last type of range.
      \param f function object to call.
      \return true, iff all f's returned true.
  */
  static constexpr bool forEachRangePreBegin(Type_ begin, Type_ end, std::function<bool(const Type_&)> f) {
    return (Value_ == begin
                ? f(Value_) &&
                      ((begin == end) ? true : CompileTimeSet<Type_, REST_...>::forEachRangePostBegin(begin, end, f))
                : CompileTimeSet<Type_, REST_...>::forEachRangePreBegin(begin, end, f));
  }

  /*! \brief Implementation helper for functor based forEachRange.
      \tparam F Functor type that is templated on Type implements operator()(Args...).
      \tparam Args Variadic template use for function arguments.
      \param begin first type of range.
      \param end last type of range.
      \param args Parameter pack passes to the functor.
      \return true, iff all F's returned true.
  */
  template <template <Type_> class F, class... Args>
  static constexpr bool forEachRangePostBegin(Type_ begin, Type_ end, Args&&... args) {
    return Value_ == end
               ? F<Value_>()(args...)
               : F<Value_>()(args...) &&
                     CompileTimeSet<Type_, REST_...>::template forEachRangePostBegin<F, Args...>(begin, end, args...);
  }

  /*! \brief Implementation helper for std::function based forEachRange.
      \param begin first type of range.
      \param end last type of range.
      \param f function object to call.
      \return true, iff all f's returned true.
  */
  static constexpr bool forEachRangePostBegin(Type_ begin, Type_ end, std::function<bool(const Type_&)> f) {
    return Value_ == end ? f(Value_)
                         : f(Value_) && CompileTimeSet<Type_, REST_...>::forEachRangePostBegin(begin, end, f);
  }

  /*! \brief Implementation helper for containsRange().
      \param begin first type of range.
      \param end last type of range.
      \return true, iff range is contained in set.
  */
  static constexpr bool containsRangePostBegin(const Type_ begin, const Type_ end) {
    return (end == Value_) ? true : CompileTimeSet<Type_, REST_...>::containsRangePostBegin(begin, end);
  }

  /*! \brief Helper for assertion that only unique types are allowed in set.
      \param t type that is checked for uniqueness
      \return true iff all types in the set are unique. False otherwise.
  */
  static constexpr bool unique(const Type_ t) {
    return (t == Value_) ? false : CompileTimeSet<Type_, REST_...>::unique(t);
  }
};

//! is_compile_time_set false type
template <typename... REST_>
struct is_compile_time_set : std::false_type {};

//! is_compile_time_set true type
template <typename... REST_>
struct is_compile_time_set<CompileTimeSet<REST_...>> : std::true_type {};

/**
 * In the following some useful helper traits to modify sets are provided.
 *
 * IDE Support:
 * To make it easier for the IDE's to deduce the type of the output set, we always pass the Type_
 * to these traits as template parameter. It could also be auto-deduced. Furthermore in the default implementation of
 * the traits we add a 'type' typedef that defines a default (empty) set.
 *
 * Compiler Output:
 * To increase compiler output quality slightly, static_asserts are added in the default implementation of the traits.
 * They will always fail, since the default implementation is not valid.
 * Still compiler outputs are hard to read for CompileTimeSets.
 */

/**
 * @brief Default implementation of set concatenation
 * @tparam Type_        Type of the set entries
 * @tparam Set1_        First Map
 * @tparam Set2_        Second Map
 */
template <typename Type_, typename Set1_, typename Set2_>
struct cts_concatenate {
  //! IDE helper type
  using type = CompileTimeSet<Type_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value, "[cts_concatenate] Could not concatenate sets!");
};

//! @brief Template specialization for CompileTimeSets of same type
template <typename Type_, Type_... Values1_, Type_... Values2_>
struct cts_concatenate<Type_, CompileTimeSet<Type_, Values1_...>, CompileTimeSet<Type_, Values2_...>> {
  using type = CompileTimeSet<Type_, Values1_..., Values2_...>;
};

//! Helper
template <typename Type_, typename Set1_, typename Set2_>
using cts_concatenate_t = typename cts_concatenate<Type_, Set1_, Set2_>::type;

/**
 * @brief Default implementation of back insertion of a value into a set
 * @tparam Type_        Type of the set
 * @tparam Set_         Set to back insert value
 * @tparam value        Value to insert
 */
template <typename Type_, typename Set_, Type_ value>
struct cts_insert_back {
  //! IDE helper type
  using type = CompileTimeSet<Type_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value, "[cts_insert_back] Could not back-insert value into set!");
};

//! @brief Template specialization for CompileTimeSet with same Type as Value
template <typename Type_, Type_... Values_, Type_ value>
struct cts_insert_back<Type_, CompileTimeSet<Type_, Values_...>, value> {
  using type = CompileTimeSet<Type_, Values_..., value>;
};

//! Helper
template <typename Type_, typename Set_, Type_ value>
using cts_insert_back_t = typename cts_insert_back<Type_, Set_, value>::type;

/**
 * @brief Default implementation of front insertion of a value into a set
 * @tparam Type_        Type of the set
 * @tparam Set_         Set to front insert value
 * @tparam value        Value to insert
 */
template <typename Type_, typename Set_, Type_ value>
struct cts_insert_front {
  //! IDE helper type
  using type = CompileTimeSet<Type_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value, "[cts_insert_front] Could not front-insert value into set!");
};

//! @brief Template specialization for CompileTimeSet with same Type as Value
template <typename Type_, Type_... Values_, Type_ value>
struct cts_insert_front<Type_, CompileTimeSet<Type_, Values_...>, value> {
  using type = CompileTimeSet<Type_, value, Values_...>;
};

//! Helper
template <typename Type_, typename Set_, Type_ value>
using cts_insert_front_t = typename cts_insert_front<Type_, Set_, value>::type;

/**
 * @brief Default implementation of erasing a value from a set
 * @tparam Type_        Type of the set
 * @tparam Set_         Set to remove erase_type from
 * @tparam erase_type   Type to erase
 */
template <typename Type_, typename Set_, Type_ erase_type>
struct cts_erase {
  //! IDE helper type
  using type = CompileTimeSet<Type_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value, "[cts_erase] Could not erase type from set!");
};

//! @brief Template specialization for CompileTimeSet with same Type as erase_type (recursion termination)
template <typename Type_, Type_... Values_, Type_ erase_type>
struct cts_erase<Type_, CompileTimeSet<Type_, Values_...>, erase_type> {
  using type = CompileTimeSet<Type_>;
};

//! @brief Template specialization for CompileTimeSet with same Type as erase_type
template <typename Type_, Type_... Values_, Type_ t, Type_ erase_type>
struct cts_erase<Type_, CompileTimeSet<Type_, t, Values_...>, erase_type> {
  using type = typename std::conditional<
      t == erase_type, CompileTimeSet<Type_, Values_...>,
      cts_insert_front_t<Type_, typename cts_erase<Type_, CompileTimeSet<Type_, Values_...>, erase_type>::type,
                         t>>::type;
};

//! Helper
template <typename Type_, typename Set_, Type_ erase_type>
using cts_erase_t = typename cts_erase<Type_, Set_, erase_type>::type;

/**
 * @brief Default implementation of conditional front insertion of a value into a set
 * @tparam Type_        Type of the set
 * @tparam Set_         Set to front insert value
 * @tparam value        Value to insert
 * @tparam Condition_   Condition to insert
 */
template <typename Type_, typename Set_, Type_ value, bool Condition_>
struct cts_insert_front_if {
  //! IDE helper type
  using type = CompileTimeSet<Set_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value,
                "[cts_insert_front_if] Could not front-insert value conditionally into set!");
};

//! @brief Template specialization for CompileTimeSet with same Type as Value
template <bool Condition_, typename Type_, Type_... Values_, Type_ value>
struct cts_insert_front_if<Type_, CompileTimeSet<Type_, Values_...>, value, Condition_> {
  using type = typename std::conditional<Condition_, CompileTimeSet<Type_, value, Values_...>,
                                         CompileTimeSet<Type_, Values_...>>::type;
};

//! Helper
template <typename Type_, typename Set_, Type_ value, bool Condition_>
using cts_insert_front_if_t = typename cts_insert_front_if<Type_, Set_, value, Condition_>::type;

/**
 * @brief Default implementation of generation of a set from an index sequence
 * @tparam Type_        Type of the set
 * @tparam Sequence_    Integer sequence
 */
template <typename Type_, typename Sequence_>
struct cts_from_sequence {
  //! IDE helper type
  using type = CompileTimeSet<Type_>;
  //! Improve compiler output
  static_assert(!std::is_same<Type_, Type_>::value, "[cts_from_sequence] Could not generate set from index sequence!");
};

//! @brief Template specialization for CompileTimeSet with same Type as Value
template <typename Type_, std::size_t... Ints>
struct cts_from_sequence<Type_, index_sequence<Ints...>> {
  using type = CompileTimeSet<Type_, static_cast<Type_>(Ints)...>;
};

//! Helper
template <typename Type_, typename Sequence_>
using cts_from_sequence_t = typename cts_from_sequence<Type_, Sequence_>::type;

/**
 * @brief Default implementation of generation of a set from an consecutive enum
 * @tparam Type_        Enum Type of the set
 * @tparam Terminator_  Termiantor of consecutive enum sequence
 */
template <typename Type_, Type_ Terminator_ = Type_::SIZE>
struct cts_from_enum {
  using type = cts_from_sequence_t<Type_, make_index_sequence_t<static_cast<std::size_t>(Terminator_)>>;
};

template <typename Type_, Type_ Terminator_ = Type_::SIZE>
using cts_from_enum_t = typename cts_from_enum<Type_, Terminator_>::type;

}  // namespace std_utils
