/*
 * multiarray_helpers.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// rqt_parameters
#include "rqt_parameters/MatrixSpinBox.hpp"

// Qt
#include <QWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>

// ros
#include <ros/ros.h>

// stl
#include <type_traits>

namespace rqt_parameters {

namespace multi_array_helpers {

namespace internal {
  template<typename T> unsigned int setDecimals(T & sb, std::false_type /* is_double_spinbox */ ) { return 0; }
  template<typename T> unsigned int setDecimals(T & sb, std::true_type /* is_double_spinbox */ ) {
    unsigned int decimals = 4;
    sb->setDecimals(decimals);
    return decimals;
  }
}

template <typename MatrixSpinBoxType_, typename GetParamService_>
bool refreshParam( const std::string & paramName,
                   ros::ServiceClient* getParamClient,
                   QWidget * matrix)
{
  using PrimType = typename GetParamService_::Response::_param_type::_value_current_type::_data_type::value_type;

  typename GetParamService_::Request req;
  typename GetParamService_::Response res;

  req.name = paramName;
  if (getParamClient->exists() && getParamClient->call(req, res) && res.success) {
    // Try a dynamic cast to the correct spinbox type
    if(MatrixSpinBox<MatrixSpinBoxType_>* m = dynamic_cast< MatrixSpinBox<MatrixSpinBoxType_>* >(matrix))
    {
      // Check dimensions of the message
      if(res.param.value_current.layout.dim.size() == 1 || res.param.value_current.layout.dim.size() == 2)
      {
        // Get matrix rows/cols and refresh the matrix spinbox to this size
        int rows = res.param.value_current.layout.dim[0].size;
        int cols = (res.param.value_current.layout.dim.size() == 2) ? res.param.value_current.layout.dim[1].size : 1;
        m->refresh(rows, cols);

        // Setup the spinboxes with the data from the message
        for( int r = 0; r < rows; ++r )
        {
          for( int c = 0; c < cols; ++c )
          {
            PrimType val = res.param.value_current.data[r*cols + c];
            PrimType min = res.param.value_min.data[r*cols + c];
            PrimType max = res.param.value_max.data[r*cols + c];
            unsigned int decimals = internal::setDecimals(m->getSpinbox(r,c), std::is_same<MatrixSpinBoxType_, QDoubleSpinBox>() );

            QString tooltip = QString("Min: ") + QString::number(min, 'f', decimals) + QString(" / Max: ") + QString::number(max, 'f', decimals);
            m->getSpinbox(r,c)->setToolTip(tooltip);
            m->getSpinbox(r,c)->setRange(min, max);
            m->getSpinbox(r,c)->setValue(val);
            m->getSpinbox(r,c)->setSingleStep(std::fabs( (max - min) / 10.0) );
          }
        }
      }
      else {
        ROS_ERROR_STREAM("Wrong dimension for matrix.");
        return false;
      }
    }
    else {
      ROS_ERROR_STREAM("MatrixSpinBox has wrong type.");
      return false;
    }
  }
  else {
    ROS_WARN_STREAM("Could not get parameter " << paramName);
    return false;
  }

  return true;
}

template <typename MatrixSpinBoxType_, typename GetParamService_, typename SetParamService_>
bool pushButtonChangeParamPressed(const std::string & paramName,
                                  ros::ServiceClient* getParamClient,
                                  ros::ServiceClient* setParamClient,
                                  QWidget * matrix)
{
  //! Type of the underlying message type ( e.g float64 / int64 )
  using PrimType = typename GetParamService_::Response::_param_type::_value_current_type::_data_type::value_type;

  ROS_INFO_STREAM("Change parameter " << paramName << ".");

  // Fill get paramter request
  typename GetParamService_::Request reqGet;
  typename GetParamService_::Response resGet;

  reqGet.name = paramName;

  // Get parameter from parameter handler ros
  if(!getParamClient->call(reqGet, resGet) || !resGet.success){
    ROS_ERROR_STREAM("Could not get parameter " << paramName);
    return false;
  }

  // Fill set parameter request
  typename SetParamService_::Request req;
  typename SetParamService_::Response res;

  req.name = paramName;


  // Try a dynamic cast to the correct spinbox type
  if(MatrixSpinBox<MatrixSpinBoxType_>* m = dynamic_cast< MatrixSpinBox<MatrixSpinBoxType_>* >(matrix))
  {
    // Add matrix data dimension
    std::size_t rows = m->rows();
    std::size_t cols = m->cols();
    std::size_t size = rows*cols;

    if(size > 0) {
      // Fill layout with matrix dimensions
      req.value.layout.dim.resize(2);
      req.value.layout.dim[0].label = "rows";
      req.value.layout.dim[0].size = rows;
      req.value.layout.dim[0].stride = size;
      req.value.layout.dim[1].label = "cols";
      req.value.layout.dim[1].size = cols;
      req.value.layout.dim[1].stride = cols;

      // Fill data vector
      req.value.data.resize(size);
      for( std::size_t r = 0; r < rows; ++r ) {
        for( std::size_t c = 0; c < cols; ++c ) {
          req.value.data[r*cols + c] = static_cast<PrimType>(m->getSpinbox(r,c)->value());
        }
      }

    }
    else {
      ROS_ERROR_STREAM("Matrix size is zero.");
      return false;
    }
  }
  else {
    ROS_ERROR_STREAM("MatrixSpinBox has wrong type.");
    return false;
  }

  // Set parameter to parameter handler ros
  if(!setParamClient->call(req, res) || !res.success) {
    ROS_ERROR_STREAM("Could not set parameter " << paramName);
    return false;
  }

  return true;
}

}

}
