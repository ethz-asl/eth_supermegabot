/**
 * @file    CompileTimeKeys.hpp
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

// std_utils
#include <std_utils/containers/CompileTimeString.hpp>
#include <std_utils/containers/Key.hpp>
#include <std_utils/containers/KeyArray.hpp>

namespace std_utils {

/** Compile Time Key implementation
 *  @tparam Enum_   Enum Type
 *  @tparam e_      Key Enum
 *  @tparam Name_   Key Name (as CompileTimeString)
 *  @tparam id_     Key Id (default int value of e_)
 */
template <typename Enum_, Enum_ e_, typename Name_, int id_ = static_cast<int>(e_)>
class CompileTimeKey {
 public:
  using Name = Name_;
  using Enum = Enum_;
  static constexpr Enum_ getEnum() { return e_; }
  static constexpr const char* getName() { return Name_::data(); }
  static constexpr int getId() { return id_; }
};

//! isCompileTimeKey false type
template <typename T>
struct is_compile_time_key : std::false_type {};

//! isCompileTimeKey true type
template <typename Enum_, Enum_ e, typename Name_, int id>
struct is_compile_time_key<CompileTimeKey<Enum_, e, Name_, id>> : std::true_type {};

/*! \brief Defines the compile time keys (ctk) as a variadic template.
    \tparam REST_ parameter pack of the ctk.
*/
template <typename... REST_>
class CompileTimeKeys {};

/*! \brief Defines the compile time keys (ctk) of Enum_ type as a variadic template.
    \tparam Enum_ Defines the type of enum of the ctk.
    \tparam REST_ parameter pack of the ctk.
*/
template <typename Enum_, typename... REST_>
class CompileTimeKeys<Enum_, REST_...> {
  //! Allow private access on all template specializations of CompileTimeKeys.
  template <typename... Any>
  friend class CompileTimeKeys;

 public:
  //! Expose Enum
  using Enum = Enum_;

  //! Get runtime key array
  static constexpr const std_utils::KeyArray<Enum_, 0> getKeysArray() { return std_utils::KeyArray<Enum_, 0>{{}}; }

  /*! \brief Default implementation of mapEnumToId().
      \param e enum for which id is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr int mapEnumToId(const Enum_ e) {
    return containsEnum(e) ? 0
                           : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map enum that is not "
                                                                 "contained in the map to an id. Enum type name: "} +
                                                     std::string{typeid(Enum_).name()} + std::string{" enum value: "} +
                                                     std::to_string(static_cast<int>(e)));
  }

  /*! \brief Default implementation of mapEnumToName().
      \param e enum for which name is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr const char* mapEnumToName(const Enum_ e) {
    return containsEnum(e) ? ""
                           : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map enum that is not "
                                                                 "contained in the map to a name. Enum type name: "} +
                                                     std::string{typeid(Enum_).name()} + std::string{" enum value: "} +
                                                     std::to_string(static_cast<int>(e)));
  }

  /*! \brief Default implementation of mapIdToEnum().
      \param id id for which enum is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr Enum_ mapIdToEnum(const int id) {
    return containsId(id) ? static_cast<Enum_>(0)
                          : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map id that is not "
                                                                "contained in the map to an enum. Enum type name: "} +
                                                    std::string{typeid(Enum_).name()} + std::string{" id value: "} +
                                                    std::to_string(id));
  }

  /*! \brief Default implementation of mapIdToName().
      \param id id for which name is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr const char* mapIdToName(const int id) {
    return containsId(id) ? ""
                          : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map id that is not "
                                                                "contained in the map to a name. Enum type name: "} +
                                                    std::string{typeid(Enum_).name()} + std::string{" id value: "} +
                                                    std::to_string(id));
  }

  /*! \brief Default implementation of mapNameToEnum().
      \param name name for which enum is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr Enum_ mapNameToEnum(const char* name) {
    return containsName(name)
               ? static_cast<Enum_>(0)
               : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map name that is not contained in the "
                                                     "map to an enum. Enum type name: "} +
                                         std::string{typeid(Enum_).name()} + std::string{" name value: "} +
                                         std::string(name));
  }

  /*! \brief Default implementation of mapNameToId().
      \param name name for which id is queried.
      \return throw std::out_of_range exception.
  */
  static constexpr int mapNameToId(const char* name) {
    return containsName(name) ? 0
                              : throw std::out_of_range(std::string{"[CompileTimeKeys] Tried to map name that is not "
                                                                    "contained in the map to an id. Enum type name: "} +
                                                        std::string{typeid(Enum_).name()} +
                                                        std::string{" name value: "} + std::string(name));
  }

  /*! \brief Default implementation of containsEnum().
      \param e enum at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsEnum(const Enum_) { return false; }

  /*! \brief Default implementation of containsName().
      \param name name at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsName(const char*) { return false; }

  /*! \brief Default implementation of containsId().
      \param id id at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsId(const int) { return false; }

  /*! \brief Default implementation of size().
      \return 0.
  */
  static constexpr std::size_t size() { return 0; }

 private:
  /*! \brief Default implementation of the uniqueEnum().
      \param e enum which is tested for uniqueness.
      \return true.
  */
  static constexpr bool uniqueEnum(const Enum_) { return true; }

  /*! \brief Default implementation of the uniqueName().
      \param name name which is tested for uniqueness.
      \return true.
  */
  static constexpr bool uniqueName(const char*) { return true; }

  /*! \brief Default implementation of the uniqueId().
      \param id id which is tested for uniqueness.
      \return true.
  */
  static constexpr bool uniqueId(const int) { return true; }
};

/*! \brief Defines the compile time keys (ctk) of enum type Enum_ but has wrong key entries.
    \tparam Enum_ Defines the enum type of the ctk.
    \tparam KeyType_ Type of the compile time key.
    \tparam REST_ parameter pack of the ctk.
*/
template <typename Enum_, typename KeyType_, typename... REST_>
class CompileTimeKeys<Enum_, KeyType_, REST_...> {
 public:
  //! Expose Enum.
  using Enum = Enum_;

  //! Assert key is of type CompileTimeKey
  static_assert(is_compile_time_key<KeyType_>::value, "[CompileTimeKeys] Wrong key types! Not a CompileTimeKey!");
};

template <typename Enum_, Enum_ e_, typename Name_, int id_, typename... REST_>
class CompileTimeKeys<Enum_, CompileTimeKey<Enum_, e_, Name_, id_>, REST_...> {
  //! Allow private access on all template specializations of CompileTimeKeys.
  template <typename... Any>
  friend class CompileTimeKeys;

  //! Assert keys that are not unique
  static_assert(CompileTimeKeys<Enum_, REST_...>::uniqueEnum(e_), "[CompileTimeKeys] Enums are not unique!");
  static_assert(CompileTimeKeys<Enum_, REST_...>::uniqueName(Name_::data()), "[CompileTimeKeys] Names are not unique!");
  static_assert(CompileTimeKeys<Enum_, REST_...>::uniqueId(id_), "[CompileTimeKeys] Ids are not unique!");

 public:
  //! Expose Enum
  using Enum = Enum_;

  //! Get runtime key array
  static constexpr const std_utils::KeyArray<Enum_, sizeof...(REST_) + 1> getKeysArray() {
    return std_utils::KeyArray<Enum_, sizeof...(REST_) + 1>{
        {std_utils::Key<Enum_>{e_, Name_::data(), id_},
         std_utils::Key<Enum_>{REST_::getEnum(), REST_::getName(), REST_::getId()}...}};
  }

  /*! \brief Default implementation of mapEnumToId().
      \param e enum for which id is queried.
      \return The Id corresponding to the enum, iff enum contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr int mapEnumToId(const Enum_ e) {
    return e == e_ ? id_ : CompileTimeKeys<Enum_, REST_...>::mapEnumToId(e);
  }

  /*! \brief Default implementation of mapEnumToName().
      \param e enum for which name is queried.
      \return The name corresponding to the enum, iff enum contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr const char* mapEnumToName(const Enum_ e) {
    return e == e_ ? Name_::data() : CompileTimeKeys<Enum_, REST_...>::mapEnumToName(e);
  }

  /*! \brief Default implementation of mapIdToEnum().
      \param id id for which enum is queried.
      \return The enum corresponding to the id, iff id contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr Enum_ mapIdToEnum(const int id) {
    return id == id_ ? e_ : CompileTimeKeys<Enum_, REST_...>::mapIdToEnum(id);
  }

  /*! \brief Default implementation of mapIdToName().
      \param id id for which name is queried.
      \return The name corresponding to the id, iff id contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr const char* mapIdToName(const int id) {
    return id == id_ ? Name_::data() : CompileTimeKeys<Enum_, REST_...>::mapIdToName(id);
  }

  /*! \brief Default implementation of mapNameToEnum().
      \param name name for which enum is queried.
      \return The enum corresponding to the name, iff name contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr Enum_ mapNameToEnum(const char* name) {
    return strings_equal(name, Name_::data()) ? e_ : CompileTimeKeys<Enum_, REST_...>::mapNameToEnum(name);
  }

  /*! \brief Default implementation of mapNameToId().
      \param name name for which id is queried.
      \return The id corresponding to the name, iff name contained in map.
      Throws std::out_of_range exception otherwise.
  */
  static constexpr int mapNameToId(const char* name) {
    return strings_equal(name, Name_::data()) ? id_ : CompileTimeKeys<Enum_, REST_...>::mapNameToId(name);
  }

  /*! \brief Default implementation of containsEnum().
      \param e enum at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsEnum(const Enum_ e) {
    return e == e_ ? true : CompileTimeKeys<Enum_, REST_...>::containsEnum(e);
  }

  /*! \brief Default implementation of containsName().
      \param name name at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsName(const char* name) {
    return strings_equal(name, Name_::data()) ? true : CompileTimeKeys<Enum_, REST_...>::containsName(name);
  }

  /*! \brief Default implementation of containsId().
      \param id id at which ctk is evaluated.
      \return false.
  */
  static constexpr bool containsId(const int id) {
    return id == id_ ? true : CompileTimeKeys<Enum_, REST_...>::containsId(id);
  }

  /*! \brief Return the size of the map.
      \return size of the map.
  */
  static constexpr std::size_t size() { return 1 + CompileTimeKeys<Enum_, REST_...>::size(); }

 private:
  /*! \brief Default implementation of the uniqueEnum().
      \param e enum which is tested for uniqueness.
      \return true, iff all enums unique.
  */
  static constexpr bool uniqueEnum(const Enum_ e) {
    return e == e_ ? false : CompileTimeKeys<Enum_, REST_...>::uniqueEnum(e);
  }

  /*! \brief Default implementation of the uniqueName().
      \param name name which is tested for uniqueness.
      \return true, iff all names unique.
  */
  static constexpr bool uniqueName(const char* name) {
    return strings_equal(name, Name_::data()) ? false : CompileTimeKeys<Enum_, REST_...>::uniqueName(name);
  }

  /*! \brief Default implementation of the uniqueId().
      \param id id which is tested for uniqueness.
      \return true, iff all ids unique.
  */
  static constexpr bool uniqueId(const int id) {
    return id == id_ ? false : CompileTimeKeys<Enum_, REST_...>::uniqueId(id);
  }
};

//! isCompileTimeKeys false type
template <typename... REST_>
struct is_compile_time_keys : std::false_type {};

//! isCompileTimeKeys true type
template <typename... REST_>
struct is_compile_time_keys<CompileTimeKeys<REST_...>> : std::true_type {};

/**
 * In the following some useful helper traits to modify keys are provided.
 *
 * IDE Support:
 * To make it easier for the IDE's to deduce the type of the output keys, we always pass the type Enum_
 * to these traits as a template parameter. It could also be auto-deduced. Furthermore in the default implementation of
 * the traits we add a 'type' typedef that defines default (empty) keys.
 *
 * Compiler Output:
 * To increase compiler output quality slightly, static_asserts are added in the default implementation of the traits.
 * They will always fail, since the default implementation is not valid.
 * Still compiler outputs are hard to read for CompileTimeMaps.
 */

/**
 * @brief Default implementation of map concatenation
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys1_       First Keys
 * @tparam Keys2_       Second Keys
 */
template <typename Enum_, typename Keys1_, typename Keys2_>
struct ctk_concatenate {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_concatenate] Could not concatenate keys!");
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type
template <typename Enum_, typename... Keys1_, typename... Keys2_>
struct ctk_concatenate<Enum_, CompileTimeKeys<Enum_, Keys1_...>, CompileTimeKeys<Enum_, Keys2_...>> {
  using type = CompileTimeKeys<Enum_, Keys1_..., Keys2_...>;
};

//! Helper
template <typename Enum_, typename Keys1_, typename Keys2_>
using ctk_concatenate_t = typename ctk_concatenate<Enum_, Keys1_, Keys2_>::type;

/**
 * @brief Default implementation of back insertion of a key into keys
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to back insert Key_ in
 * @tparam Key_         CompileTimeKey to insert
 */
template <typename Enum_, typename Keys_, typename Key_>
struct ctk_insert_back {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_insert_back] Could not back-insert Key into Keys!");
};

//! @brief Template specialization for CompileTimeKey of same Enum-type
template <typename Enum_, typename... Keys_, Enum_ key_enum, typename key_name, int key_id>
struct ctk_insert_back<Enum_, CompileTimeKeys<Enum_, Keys_...>, CompileTimeKey<Enum_, key_enum, key_name, key_id>> {
  using type = CompileTimeKeys<Enum_, Keys_..., CompileTimeKey<Enum_, key_enum, key_name, key_id>>;
};

//! Helper
template <typename Enum_, typename Keys_, typename Key_>
using ctk_insert_back_t = typename ctk_insert_back<Enum_, Keys_, Key_>::type;

/**
 * @brief Default implementation of front insertion of a key into keys
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to front insert Key_ in
 * @tparam Key_         CompileTimeKey to insert
 */
template <typename Enum_, typename Keys_, typename Key_>
struct ctk_insert_front {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_insert_front] Could not front-insert Key into Keys!");
};

//! @brief Template specialization for CompileTimeKey of same Enum-type
template <typename Enum_, typename... Keys_, Enum_ key_enum, typename key_name, int key_id>
struct ctk_insert_front<Enum_, CompileTimeKeys<Enum_, Keys_...>, CompileTimeKey<Enum_, key_enum, key_name, key_id>> {
  using type = CompileTimeKeys<Enum_, CompileTimeKey<Enum_, key_enum, key_name, key_id>, Keys_...>;
};

//! Helper
template <typename Enum_, typename Keys_, typename Key_>
using ctk_insert_front_t = typename ctk_insert_front<Enum_, Keys_, Key_>::type;

/**
 * @brief Default implementation of deletion of a key in keys
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to delete key from
 * @tparam erase_key    Enum coresponding to CompileTimeKey to erase
 */
template <typename Enum_, typename Keys_, Enum_ erase_key>
struct ctk_erase {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_>;
  //! Improve compiler output
  static_assert(!std::is_same<Keys_, Keys_>::value, "[ctk_erase] Could not erase key from keys!");
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type (recursion termination)
template <typename Enum_, typename... Keys_, Enum_ erase_key>
struct ctk_erase<Enum_, CompileTimeKeys<Enum_, Keys_...>, erase_key> {
  using type = CompileTimeKeys<Enum_, Keys_...>;
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type
template <typename Enum_, typename... Keys_, Enum_ key_enum, typename key_name, int key_id, Enum_ erase_key>
struct ctk_erase<Enum_, CompileTimeKeys<Enum_, CompileTimeKey<Enum_, key_enum, key_name, key_id>, Keys_...>,
                 erase_key> {
  using type = typename std::conditional<
      key_enum == erase_key, CompileTimeKeys<Enum_, Keys_...>,
      ctk_insert_front_t<Enum_, typename ctk_erase<Enum_, CompileTimeKeys<Enum_, Keys_...>, erase_key>::type,
                         CompileTimeKey<Enum_, key_enum, key_name, key_id>>>::type;
};

//! Helper
template <typename Enum_, typename Keys_, Enum_ erase_key>
using ctk_erase_t = typename ctk_erase<Enum_, Keys_, erase_key>::type;

/**
 * @brief Default implementation of pop a key from front of keys
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to pop Key_ from front
 */
template <typename Enum_, typename Keys_>
struct ctk_pop_front {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_pop_front] Could not pop-front Key from Keys!");
};

//! @brief Template specialization for CompileTimeKey of same Enum-type
template <typename Enum_, typename... Keys_, Enum_ key_enum, typename key_name, int key_id>
struct ctk_pop_front<Enum_, CompileTimeKeys<Enum_, CompileTimeKey<Enum_, key_enum, key_name, key_id>, Keys_...>> {
  using type = CompileTimeKeys<Enum_, Keys_...>;
};

//! Helper
template <typename Enum_, typename Keys_>
using ctk_pop_front_t = typename ctk_pop_front<Enum_, Keys_>::type;

/**
 * @brief Default implementation of incrementing the ids of keys
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to increment ids from
 * @tparam Increment_   Increment of ids
 */
template <typename Enum_, typename Keys_, int Increment_>
struct ctk_increment_ids {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_, Keys_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_increment_ids] Could not increment ids of keys!");
};

//! @brief Template specialization for CompileTimeKey of same Enum-type (recursion termination)
template <typename Enum_, int Increment_>
struct ctk_increment_ids<Enum_, CompileTimeKeys<Enum_>, Increment_> {
  using type = CompileTimeKeys<Enum_>;
};

//! @brief Template specialization for CompileTimeKey of same Enum-type
template <typename Enum_, typename... Keys_, Enum_ key_enum, typename key_name, int key_id, int Increment_>
struct ctk_increment_ids<Enum_, CompileTimeKeys<Enum_, CompileTimeKey<Enum_, key_enum, key_name, key_id>, Keys_...>,
                         Increment_> {
  using type =
      ctk_insert_front_t<Enum_, typename ctk_increment_ids<Enum_, CompileTimeKeys<Enum_, Keys_...>, Increment_>::type,
                         CompileTimeKey<Enum_, key_enum, key_name, key_id + Increment_>>;
};

//! Helper
template <typename Enum_, typename Keys_, int Increment_>
using ctk_increment_ids_t = typename ctk_increment_ids<Enum_, Keys_, Increment_>::type;

/**
 * @brief Default implementation of popping first element and decrementing the ids of keys by 1
 * @tparam Enum_        Enum type of the keys
 * @tparam Keys_        Keys to pop front and decrement ids from
 */
template <typename Enum_, typename Keys_>
struct ctk_pop_front_and_decrement {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_, Keys_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_pop_front_and_decrement] Could pop and decrement from keys!");
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type
template <typename Enum_, typename... Keys_>
struct ctk_pop_front_and_decrement<Enum_, CompileTimeKeys<Enum_, Keys_...>> {
  using type = ctk_increment_ids_t<Enum_, ctk_pop_front_t<Enum_, CompileTimeKeys<Enum_, Keys_...>>, -1>;
};

//! Helper
template <typename Enum_, typename Keys_>
using ctk_pop_front_and_decrement_t = typename ctk_pop_front_and_decrement<Enum_, Keys_>::type;

/**
 * @brief Default implementation of transforming keys to another enum type
 * @tparam TransformedEnum_        Enum type of the transformed keys
 * @tparam Keys_                   Keys to transform
 * @tparam EnumMap_                CompileTimeMap from Enum_ to TransformedEnum_
 * @tparam IdOffset_               Constant IdOffset of the new keys
 */
template <typename TransformedEnum_, typename Keys_, typename EnumMap_, int IdOffset_ = 0>
struct ctk_transform {
  //! IDE helper type
  using type = CompileTimeKeys<Keys_, EnumMap_>;
  //! Improve compiler output
  static_assert(!std::is_same<TransformedEnum_, TransformedEnum_>::value, "[ctk_transform] Could not transform keys!");
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type (recursion termination)
template <typename TransformedEnum_, typename Enum_, typename... KV_, int IdOffset_>
struct ctk_transform<TransformedEnum_, CompileTimeKeys<Enum_>, CompileTimeMap<Enum_, TransformedEnum_, KV_...>,
                     IdOffset_> {
  using type = CompileTimeKeys<TransformedEnum_>;
};

//! @brief Template specialization for CompileTimeKeys of same Enum-type
template <typename TransformedEnum_, typename Enum_, Enum_ key_enum, typename key_name, int key_id, typename... Keys_,
          typename... KV_, int IdOffset_>
struct ctk_transform<TransformedEnum_,
                     CompileTimeKeys<Enum_, CompileTimeKey<Enum_, key_enum, key_name, key_id>, Keys_...>,
                     CompileTimeMap<Enum_, TransformedEnum_, KV_...>, IdOffset_> {
  using type =
      ctk_insert_front_t<TransformedEnum_,
                         typename ctk_transform<TransformedEnum_, CompileTimeKeys<Enum_, Keys_...>,
                                                CompileTimeMap<Enum_, TransformedEnum_, KV_...>, IdOffset_>::type,
                         CompileTimeKey<TransformedEnum_, CompileTimeMap<Enum_, TransformedEnum_, KV_...>::at(key_enum),
                                        key_name, IdOffset_ + key_id>>;
};

//! Helper
template <typename TransformedEnum_, typename Keys_, typename EnumMap_, int IdOffset_ = 0>
using ctk_transform_t = typename ctk_transform<TransformedEnum_, Keys_, EnumMap_, IdOffset_>::type;

/**
 * @brief Default implementation of transforming keys to another enum type
 * @tparam Enum_                   Enum type of the combined keys
 * @tparam Keys1_                  First Keys
 * @tparam Keys1_                  Second Keys
 * @tparam MapKeys1_               Map enum to first keys to Enum_
 * @tparam MapKeys2_               Map enum to second keys to Enum_
 */
template <typename Enum_, typename Keys1_, typename Keys2_, typename MapKeys1_, typename MapKeys2_>
struct ctk_combine {
  //! IDE helper type
  using type = CompileTimeKeys<Enum_, Keys1_, Keys2_>;
  //! Improve compiler output
  static_assert(!std::is_same<Enum_, Enum_>::value, "[ctk_combine] Could not combine keys!");
};

//! @brief Template specialization for CompileTimeKeys / CompileTimeMaps of matching enums
template <typename Enum1_, typename Enum2_, typename TransformedEnum_, typename... Keys1_, typename... Keys2_,
          typename... KV1_, typename... KV2_>
struct ctk_combine<TransformedEnum_,
                   CompileTimeKeys<Enum1_, Keys1_...>,
                   CompileTimeKeys<Enum2_, Keys2_...>,
                   CompileTimeMap<Enum1_, TransformedEnum_, KV1_...>,
                   CompileTimeMap<Enum2_, TransformedEnum_, KV2_...>> {
  using type = std_utils::ctk_concatenate_t<
      TransformedEnum_,
      std_utils::ctk_transform_t<TransformedEnum_,
                                 CompileTimeKeys<Enum1_, Keys1_...>,
                                 CompileTimeMap<Enum1_, TransformedEnum_, KV1_...>>,
      std_utils::ctk_transform_t<TransformedEnum_,
                                 CompileTimeKeys<Enum2_, Keys2_...>,
                                 CompileTimeMap<Enum2_, TransformedEnum_, KV2_...>,
                                 static_cast<int>(Enum1_::SIZE)>>;
};

//! Helper
template <typename Enum_, typename Keys1_, typename Keys2_, typename MapKeys1_, typename MapKeys2_>
using ctk_combine_t = typename ctk_combine<Enum_, Keys1_, Keys2_, MapKeys1_, MapKeys2_>::type;

}  // namespace std_utils