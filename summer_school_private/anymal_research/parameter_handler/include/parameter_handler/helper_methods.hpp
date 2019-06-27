/*
 * helper_methods.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

// parameter handler
#include "parameter_handler/type_macros.hpp"
#include "parameter_handler/ParameterInterface.hpp"

// tinyxml_tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include "message_logger/message_logger.hpp"

#pragma once

namespace parameter_handler {

template<typename T1>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return false;
  }
}

template <typename T1, typename T2, typename... Tn>
bool isType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    return true;
  } else {
    return isType<T2, Tn...>(param);
  }
}

template<typename >
constexpr bool checkTypeSupport() {
  return false;
}

template <typename TestType, typename T, typename... Tn>
constexpr bool checkTypeSupport() {
  return std::is_same<typename std::decay<TestType>::type, T>::value ? true : checkTypeSupport<TestType, Tn...>();
}

template<typename T>
constexpr bool isTypeSupported() {
  return checkTypeSupport<T, PH_TYPES>();
}

template<typename T1>
void printType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    MELO_INFO_STREAM( "Changed parameter " << param.getName() << " to :" << param.getValue<T1>() );
  }
}

template <typename T1, typename T2, typename... Tn>
void printType(const parameter_handler::ParameterInterface & param) {
  if( param.getType() == typeid(T1) ){
    printType<T1>(param);
  } else {
    printType<T2, Tn...>(param);
  }
}

template<typename T1>
bool storeType(const parameter_handler::ParameterInterface & param, tinyxml_tools::DocumentHandleXML& doc) {
  if( param.getType() == typeid(T1) ){
    return doc.write(param.getName(), param.getValue<T1>());
  }
  return false;
}

template <typename T1, typename T2, typename... Tn>
bool storeType(const parameter_handler::ParameterInterface & param, tinyxml_tools::DocumentHandleXML& doc) {
  if( param.getType() == typeid(T1) ){
    return storeType<T1>(param, doc);
  } else {
    return storeType<T2, Tn...>(param, doc);
  }
}

template<typename T1>
bool loadType(parameter_handler::ParameterInterface & param, const tinyxml_tools::DocumentHandleXML& doc) {
  if( param.getType() == typeid(T1) ){
    T1 v(param.getValue<T1>());
    bool success =  doc.read(param.getName(), v);
    param.setValue<T1>(v);
    return success;
  }
  return false;
}

template <typename T1, typename T2, typename... Tn>
bool loadType(parameter_handler::ParameterInterface & param, const tinyxml_tools::DocumentHandleXML& doc) {
  if( param.getType() == typeid(T1) ){
    return loadType<T1>(param, doc);
  } else {
    return loadType<T2, Tn...>(param, doc);
  }
}

}
