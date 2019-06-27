/*
 * helper_methods.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#include "parameter_handler/helper_methods.hpp"
#include "parameter_handler/type_macros.hpp"
#include "parameter_handler/ParameterInterface.hpp"

#include <ros/ros.h>
#include <Eigen/Core>

#pragma once

namespace parameter_handler_ros {

template <typename ScalarType_, typename MultiArrayMsg_>
bool writeScalarToMessage(const ScalarType_ & scalar, MultiArrayMsg_ & msg) {
  using PrimType = typename MultiArrayMsg_::_data_type::value_type;

  // Add scalar data dimension
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "data";
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = 1;
  msg.data.resize(1);
  msg.data[0] = static_cast<PrimType>(scalar);

  return true;
}

template <typename MatrixType_, typename MultiArrayMsg_>
bool writeMatrixToMessage(const MatrixType_ & matrix, MultiArrayMsg_ & msg) {
  using PrimType = typename MultiArrayMsg_::_data_type::value_type;

  // Add matrix data dimension
  if(matrix.size() > 0) {
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = matrix.rows();
    msg.layout.dim[0].stride = matrix.size();
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = matrix.cols();
    msg.layout.dim[1].stride = matrix.cols();

    msg.data.resize(matrix.size());
    for( int r = 0; r < matrix.rows(); ++r ) {
      for( int c = 0; c < matrix.cols(); ++c ) {
        msg.data[r*matrix.cols() + c] = static_cast<PrimType>(matrix(r,c));
      }
    }
  }
  else {
    ROS_ERROR_STREAM("Matrix size is zero.");
    return false;
  }

  return true;
}

template <typename ScalarParameterMessage_>
bool writeScalarParamToMessage(const parameter_handler::ParameterInterface & param, ScalarParameterMessage_ & msg) {
  return false;
}

template <typename ScalarParameterMessage_, typename T1, typename... Tn>
bool writeScalarParamToMessage(const parameter_handler::ParameterInterface & param, ScalarParameterMessage_ & msg) {
  if(parameter_handler::isType<T1>(param)) {
    msg.name = param.getName();
    if( parameter_handler::isType<PH_SCALAR_TYPES>(param) ) {
      bool success = true;
      success = success && writeScalarToMessage(param.getValue<T1>(), msg.value_current);
      success = success && writeScalarToMessage(param.getMinValue<T1>(), msg.value_min);
      success = success && writeScalarToMessage(param.getMaxValue<T1>(), msg.value_max);
      success = success && writeScalarToMessage(param.getDefaultValue<T1>(), msg.value_default);
      return success;
    }
    return false;
  }
  else {
    return writeScalarParamToMessage<ScalarParameterMessage_, Tn...>(param, msg);
  }
}


template <typename MatrixParameterMessage_>
bool writeMatrixParamToMessage(const parameter_handler::ParameterInterface & param, MatrixParameterMessage_ & msg) {
  return false;
}

template <typename MatrixParameterMessage_, typename T1, typename... Tn>
bool writeMatrixParamToMessage(const parameter_handler::ParameterInterface & param, MatrixParameterMessage_ & msg) {
  if(parameter_handler::isType<T1>(param)) {
    msg.name = param.getName();
    if( parameter_handler::isType<PH_MATRIX_TYPES>(param) ) {
      bool success = true;
      success = success && writeMatrixToMessage(param.getValue<T1>(), msg.value_current);
      success = success && writeMatrixToMessage(param.getMinValue<T1>(), msg.value_min);
      success = success && writeMatrixToMessage(param.getMaxValue<T1>(), msg.value_max);
      success = success && writeMatrixToMessage(param.getDefaultValue<T1>(), msg.value_default);
      return success;
    }
    return false;
  }
  else {
    return writeMatrixParamToMessage<MatrixParameterMessage_, Tn...>(param, msg);
  }
}

template <typename ScalarType_, typename MultiArrayMsg_>
bool readScalarFromMessage(ScalarType_ & scalar, const MultiArrayMsg_ & msg) {
  if( (msg.layout.dim.size() == 1 && msg.layout.dim[0].size == 1) ||
      (msg.layout.dim.size() == 2 && msg.layout.dim[0].size == 1 && msg.layout.dim[1].size == 1 ) ) {
    scalar = static_cast<ScalarType_>(msg.data[0]);
    return true;
  }

  return false;
}

template <typename MatrixType_, typename MultiArrayMsg_>
bool readMatrixFromMessage(MatrixType_ & matrix, const MultiArrayMsg_ & msg) {
  // Handle eigen matrices
  if(msg.layout.dim.size() == 1 || msg.layout.dim.size() == 2) {
    int rows = msg.layout.dim[0].size;
    int cols = ( msg.layout.dim.size() == 2 ) ? msg.layout.dim[1].size : 1;

    // Resize matrix if possible
    if( (matrix.rows() != rows) ) {
      if( MatrixType_::RowsAtCompileTime == Eigen::Dynamic ) {
        matrix.resize(rows, matrix.cols());
      } else {
        MELO_ERROR_STREAM("[PH_ROS]: Wrong number of rows and can not resize matrix.")
        return false;
      }
    }

    if( (matrix.cols() != cols) ) {
      if ( MatrixType_::ColsAtCompileTime == Eigen::Dynamic ) {
        matrix.resize(matrix.rows(), cols);
      } else {
        MELO_ERROR_STREAM("[PH_ROS]: Wrong number of cols and can not resize matrix.")
        return false;
      }
    }

    for( int r = 0; r < rows; ++r ) {
      for( int c = 0; c < cols; ++c ) {
        matrix(r, c) = static_cast<typename MatrixType_::Scalar>(msg.data[r * cols + c]);
      }
    }
    return true;
  } else {
    MELO_ERROR_STREAM("[PH_ROS]: Wrong dimension specification in message, when trying to set matrix parameter.")
  }

  return false;
}

template <typename SetScalarParameterServiceRequest_>
bool readScalarParamFromServiceRequest(parameter_handler::ParameterInterface & param, const SetScalarParameterServiceRequest_ & req) {
  return false;
}

template <typename SetScalarParameterServiceRequest_, typename T1, typename... Tn>
bool readScalarParamFromServiceRequest(parameter_handler::ParameterInterface & param, const SetScalarParameterServiceRequest_ & req) {
  if(parameter_handler::isType<T1>(param)) {
    T1 value = param.getValue<T1>();
    bool success = readScalarFromMessage(value, req.value);
    param.setValue(value);
    return success;
  }
  else {
    return readScalarParamFromServiceRequest<SetScalarParameterServiceRequest_, Tn...>(param, req);
  }
}

template <typename SetMatrixParameterServiceRequest_>
bool readMatrixParamFromServiceRequest(parameter_handler::ParameterInterface & param, const SetMatrixParameterServiceRequest_ & req) {
  return false;
}

template <typename SetMatrixParameterServiceRequest_, typename T1, typename... Tn>
bool readMatrixParamFromServiceRequest(parameter_handler::ParameterInterface & param, const SetMatrixParameterServiceRequest_ & req) {
  if(parameter_handler::isType<T1>(param)) {
    T1 value = param.getValue<T1>();
    bool success = readMatrixFromMessage(value, req.value);
    param.setValue(value);
    return success;
  }
  else {
    return readMatrixParamFromServiceRequest<SetMatrixParameterServiceRequest_, Tn...>(param, req);
  }
}

}
