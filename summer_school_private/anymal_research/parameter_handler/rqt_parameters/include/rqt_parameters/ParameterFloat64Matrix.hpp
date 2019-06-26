/*
 * ParameterInt64Matrix.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// rqt_parameters
#include "rqt_parameters/ParameterBase.hpp"
#include "rqt_parameters/multiarray_helpers.hpp"
#include "rqt_parameters/MatrixSpinBox.hpp"

// parameter_handler_msgs
#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>

// Qt
#include <QDoubleSpinBox>

// ros
#include <ros/ros.h>

namespace rqt_parameters {

//! This class provides implements a float64 multi array parameter type
class ParameterFloat64Matrix: public ParameterBase {
 Q_OBJECT;
 public:
  ParameterFloat64Matrix( const std::string& name,
                        QGridLayout* grid,
                        ros::ServiceClient* getParameterClient,
                        ros::ServiceClient* setParameterClient):
                          ParameterBase(name, grid, getParameterClient, setParameterClient)
  {

  }

  virtual ~ParameterFloat64Matrix() {

  };

  public slots:
    //! React to a change button press
    void pushButtonChangeParamPressed() {
      using parameter_handler_msgs::GetFloatingPointParameter;
      using parameter_handler_msgs::SetFloatingPointParameter;

      multi_array_helpers::pushButtonChangeParamPressed<QDoubleSpinBox, GetFloatingPointParameter, SetFloatingPointParameter>(
          name_, getParameterClient_, setParameterClient_, matrixSpinBoxParamValue);
    }

    //! Refresh the parameter
    void refreshParam() {
      using parameter_handler_msgs::GetFloatingPointParameter;
      multi_array_helpers::refreshParam<QDoubleSpinBox, GetFloatingPointParameter>( name_, getParameterClient_, matrixSpinBoxParamValue);
    }

  protected:
    //! Factory method
    virtual QWidget* createMatrixSpinBox(QWidget* parent) { return new MatrixSpinBox<QDoubleSpinBox>(parent); }

};

} // end namespace
