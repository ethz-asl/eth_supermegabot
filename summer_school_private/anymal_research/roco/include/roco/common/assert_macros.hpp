/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
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
/*!
* @file     assert_macros.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once

#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "roco/common/source_file_pos.hpp"
#include <iostream>
#include <sstream>
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define ROCO_DEFINE_EXCEPTION(exceptionName, exceptionParent)       \
  class exceptionName : public exceptionParent {            \
  public:                               \
  exceptionName(const char * message) : exceptionParent(message) {}   \
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}                 \
  };


namespace roco {
namespace common {
namespace internal {

template<typename Exception_>
inline void roco_throw_exception(std::string const & exceptionType,
                                 roco::common::internal::source_file_pos sfp,
                                 std::string const & message)
{
  std::stringstream roco_assert_stringstream;
#ifdef _WIN32
  // I have no idea what broke this on Windows but it doesn't work with the << operator.
  roco_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
  roco_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#endif
  throw(Exception_(roco_assert_stringstream.str()));
}

template<typename Exception_>
inline void roco_throw_exception(std::string const & exceptionType,
                                 std::string const & function,
                                 std::string const & file,
                                 int line,
                                 std::string const & message) {
  roco_throw_exception<Exception_>(exceptionType, roco::common::internal::source_file_pos(function,file,line),message);
}

inline std::string roco_string_format(const std::string fmt, ...) {
  int size = ((int)fmt.size()) * 2 + 50;   // use a rubric appropriate for your code
  std::string str;
  va_list ap;
  while (1) {     // maximum 2 passes on a POSIX system...
     str.resize(size);
     va_start(ap, fmt);
     int n = vsnprintf((char *)str.data(), size, fmt.c_str(), ap);
     va_end(ap);
     if (n > -1 && n < size) {  // everything worked
         str.resize(n);
         return str;
     }
     if (n > -1)  // needed size returned
         size = n + 1;   // for null char
     else
         size *= 2;      // guess at a larger size (o/s specific)
  }
  return str;
}

} // namespace internal

template<typename Exception_>
inline void roco_assert_throw(bool assert_condition, std::string message, roco::common::internal::source_file_pos sfp) {
  if(!assert_condition) {
    internal::roco_throw_exception<Exception_>("", sfp,message);
  }
}


} // namespace common
} // namespace rm



#define ROCO_THROW(exceptionType, message) {                \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << message;                  \
    roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
  }


#define ROCO_THROW_SFP(exceptionType, SourceFilePos, message){      \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << message;                  \
    roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, roco_assert_stringstream.str()); \
  }

#define ROCO_ASSERT_TRUE(exceptionType, condition, message)       \
  if(!(condition))                            \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_FALSE(exceptionType, condition, message)        \
  if((condition))                           \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_LT(exceptionType, value, upperBound, message)     \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_GE(exceptionType, value, lowerBound, message)     \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_LE(exceptionType, value, upperBound, message)     \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_GT(exceptionType, value, lowerBound, message)     \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_EQ(exceptionType, value, testValue, message)      \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_NE(exceptionType, value, testValue, message)      \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_OUT(X) std::cout << #X << ": " << (X) << std::endl

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef NDEBUG

#define ROCO_THROW_DBG(exceptionType, message){             \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << message;                  \
    roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
  }



#define ROCO_ASSERT_TRUE_DBG(exceptionType, condition, message)     \
  if(!(condition))                            \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_FALSE_DBG(exceptionType, condition, message)      \
  if((condition))                           \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    }


#define ROCO_ASSERT_DBG_RE( condition, message) ROCO_ASSERT_DBG(std::runtime_error, condition, message)

#define ROCO_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_LT_DBG(exceptionType, value, upperBound, message)   \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)   \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_LE_DBG(exceptionType, value, upperBound, message)   \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }

#define ROCO_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)   \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_EQ_DBG(exceptionType, value, testValue, message)    \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }


#define ROCO_ASSERT_NE_DBG(exceptionType, value, testValue, message)    \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }



#define ROCO_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      roco::common::internal::roco_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,roco_assert_stringstream.str()); \
    }


#define ROCO_OUT_DBG(X) std::cout << #X << ": " << (X) << std::endl

#else

#define ROCO_OUT_DBG(X)
#define ROCO_THROW_DBG(exceptionType, message)
#define ROCO_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define ROCO_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define ROCO_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define ROCO_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define ROCO_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define ROCO_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define ROCO_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define ROCO_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define ROCO_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define ROCO_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)

#endif


