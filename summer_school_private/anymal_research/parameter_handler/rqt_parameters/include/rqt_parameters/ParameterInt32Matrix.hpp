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
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>

// Qt
#include <QSpinBox>

// ros
#include <ros/ros.h>


namespace rqt_parameters {

//! This class draws and handles multi array parameters
class ParameterInt32Matrix: public ParameterBase {
 Q_OBJECT;
 public:
  //! Constructor
  ParameterInt32Matrix( const std::string& name,
                        QGridLayout* grid,
                        ros::ServiceClient* getParameterClient,
                        ros::ServiceClient* setParameterClient):
                          ParameterBase(name, grid, getParameterClient, setParameterClient)
  {

  }

  //! Destructor
  virtual ~ParameterInt32Matrix() {

  };

  public slots:
    //! React to a change button press
    void pushButtonChangeParamPressed() {
      using parameter_handler_msgs::GetIntegralParameter;
      using parameter_handler_msgs::SetIntegralParameter;

      multi_array_helpers::pushButtonChangeParamPressed<QSpinBox, GetIntegralParameter, SetIntegralParameter>(
          name_, getParameterClient_, setParameterClient_, matrixSpinBoxParamValue);
    }

    //! Refresh the parameter
    void refreshParam() {
      using parameter_handler_msgs::GetIntegralParameter;
      multi_array_helpers::refreshParam<QSpinBox, GetIntegralParameter>( name_, getParameterClient_, matrixSpinBoxParamValue);
    }

  protected:
    //! Factory method
    virtual QWidget* createMatrixSpinBox(QWidget* parent) { return new MatrixSpinBox<QSpinBox>(parent); }

};

} // end namespace
