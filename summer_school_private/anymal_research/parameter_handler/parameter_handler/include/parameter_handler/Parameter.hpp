/**
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring, Ralf Kaestner
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * ParameterImpl.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: Dario Bellicoso, Christian Gehring, Ralf Kaestner
 */

#pragma once

#include "parameter_handler/ParameterInterface.hpp"
#include "parameter_handler/ParameterValueInterface.hpp"
#include "parameter_handler/ParameterValue.hpp"

#include "tinyxml_tools/tinyxml_tools.hpp"

#include "message_logger/message_logger.hpp"

namespace parameter_handler {

template<typename ValueType_>
class Parameter : public ParameterInterface
{
public:
  Parameter(const ValueType_& value = ValueType_()) :
    ParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()))
  {
  }

  Parameter(const ValueType_& value, const ValueType_& min, const ValueType_& max) :
    ParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()))
  {
//    value_ = ParameterValuePtr(ValueType_().clone());
    getValuePtr()->setMinValue(min);
    getValuePtr()->setMaxValue(max);
    getValuePtr()->setValue(value);
    getValuePtr()->setDefaultValue(value);
  }

  Parameter(const std::string& name, const ValueType_& value, const ValueType_& min, const ValueType_& max) :
    ParameterInterface(typeid(ValueType_),
                       internal::ParameterValuePtr(new internal::ParameterValue<ValueType_>()),
                       name)
  {
//    value_ = ParameterValuePtr(ValueType_().clone());
    getValuePtr()->setMinValue(min);
    getValuePtr()->setMaxValue(max);
    getValuePtr()->setValue(value);
    getValuePtr()->setDefaultValue(value);
  }


  Parameter(const Parameter<ValueType_>& other) :
    ParameterInterface(other)
  {
  }

  Parameter(const ParameterInterface& other) :
    ParameterInterface(other)
  {
    if (type_ != typeid(ValueType_))
      throw std::runtime_error("Parameter value type mismatch");
  }

  virtual ~Parameter()  { }


  const ValueType_& getValue() const {
    return getValuePtr()->getValue();
  }

  const ValueType_& getDefaultValue() const {
    return getValuePtr()->getDefaultValue();
  }

  const ValueType_& getMinValue() const {
    return getValuePtr()->getMinValue();
  }

  const ValueType_& getMaxValue() const {
    return getValuePtr()->getMaxValue();
  }

  void setValue(const ValueType_& value) {
    getValuePtr()->setValue(value);
    notifyObservers();
  }

  template< typename V_ = ValueType_>
  void setValue(const typename V_::Scalar& value, unsigned int row, unsigned int col = 0,
                typename std::enable_if< std::is_base_of< Eigen::MatrixBase<V_>, V_ >::value>::type* = 0 /* is_eigen */ )
  {
    getValuePtr()->setValue(value, row, col);
    notifyObservers();
  }

  void resetWithDefaultValueAndRelativeBounds(const ValueType_& value, const ValueType_& relativeUpperBound) {
    getValuePtr()->setDefaultValue(value);
    getValuePtr()->setMaxValue(value+relativeUpperBound);
    getValuePtr()->setMinValue(value-relativeUpperBound);
    getValuePtr()->setValue(value);
    notifyObservers();
  }

  void setDefaultValue(const ValueType_& value) {
    getValuePtr()->setDefaultValue(value);
    notifyObservers();
  }

  void setMinValue(const ValueType_& value) {
    getValuePtr()->setMinValue(value);
    notifyObservers();
  }

  void setMaxValue(const ValueType_& value) {
    getValuePtr()->setMaxValue(value);
    notifyObservers();
  }

  void resetToDefault() {
    getValuePtr()->setValue(getValuePtr()->getDefaultValue());
    notifyObservers();
  }

  bool load(const std::string& filename) {
    tinyxml_tools::DocumentHandleXML doc;
    if( doc.create(filename, tinyxml_tools::DocumentMode::READ)) {
      return doc.read(getName(), *this);
    }
    return false;
  }

  bool store(const std::string& filename, bool append = false) const {
    tinyxml_tools::DocumentHandleXML doc;
    auto mode = append ? tinyxml_tools::DocumentMode::APPEND : tinyxml_tools::DocumentMode::WRITE;
    if( doc.create(filename, mode) && doc.write(getName(), *this) ) {
      return doc.save();
    }
    return false;
  }

  bool load(const tinyxml_tools::DocumentHandleXML& doc) {
    return doc.read(getName(), *this);
  }

  bool store(tinyxml_tools::DocumentHandleXML& doc) const {
    return doc.write(getName(), *this);
  }

protected:
  std::shared_ptr<internal::ParameterValue<ValueType_> > getValuePtr() {
    return std::static_pointer_cast<internal::ParameterValue<ValueType_> >(value_);
  }

  std::shared_ptr<const internal::ParameterValue<ValueType_> > getValuePtr() const {
    return std::static_pointer_cast<const internal::ParameterValue<ValueType_> >(value_);
  }

};

template<typename T>
struct is_parameter : std::false_type { };

template<typename U>
struct is_parameter<parameter_handler::Parameter<U>> : std::true_type { };

template<typename T, typename ValueType>
struct is_parameter_of_type : std::false_type { };

template<typename U, typename ValueType>
struct is_parameter_of_type<parameter_handler::Parameter<U>, ValueType> : std::is_same<U, ValueType> { };


}
