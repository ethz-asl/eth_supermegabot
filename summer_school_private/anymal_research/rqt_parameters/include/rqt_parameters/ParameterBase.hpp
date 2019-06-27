/*
 * ParameterBase.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring / Gabriel Hottiger
 */

#pragma once

// Qt
#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>

// ros
#include <ros/ros.h>

namespace rqt_parameters {

//! This class draws and handles a double parameter.
class ParameterBase: public QObject {
  Q_OBJECT
 public:
  ParameterBase(const std::string& name,
                QGridLayout* grid,
                ros::ServiceClient* getParameterClient,
                ros::ServiceClient* setParameterClient):
                  labelParamNumber(nullptr),
                  labelParamName(nullptr),
                  matrixSpinBoxParamValue(nullptr),
                  pushButtonChangeParam(nullptr),
                  name_(name),
                  getParameterClient_(getParameterClient),
                  setParameterClient_(setParameterClient),
                  grid_(grid)

  {
  }

  virtual ~ParameterBase() {
    // Disconnect the button signal
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);

    // Remove the widgets from the grid layout
    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(matrixSpinBoxParamValue);
    grid_->removeWidget(pushButtonChangeParam);

    // Delete the widgets
    delete labelParamNumber;
    delete labelParamName;
    delete matrixSpinBoxParamValue;
    delete pushButtonChangeParam;
  };

  void setupGUI(QWidget* widget, size_t maxParamNameWidth) {
    // Get row in layout
    int iRow = grid_->rowCount();

    // Setup the label to the current row number ( e.g. 5) )
    labelParamNumber = new QLabel(widget);
    labelParamNumber->setObjectName(QString::fromStdString(std::string{"labelParamNumber"} + name_));
    labelParamNumber->setText(QString::number(iRow)+QString::fromUtf8(")"));

    // Setup the parameter name label (width is fixed to the longest parameter name)
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(std::string{"labelParamName"} + name_));
    labelParamName->setText(QString::fromStdString(name_));
    labelParamName->setFixedWidth(maxParamNameWidth - 10);

    // Setup the matrix spinbox (create using factory method of subclass)
    matrixSpinBoxParamValue = createMatrixSpinBox(widget);
    matrixSpinBoxParamValue->setObjectName(QString::fromStdString(std::string{"matrixSpinBox"} + name_));
    matrixSpinBoxParamValue->setMinimumSize(QSize(200, 0));

    // Setup 'change' push button (max width of 60)
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(std::string{"pushButtonChange"} + name_));
    pushButtonChangeParam->setText(QString::fromUtf8("change"));
    pushButtonChangeParam->setMaximumWidth(60);

    // Add the widgets to the grid layout and top align them
    grid_->addWidget(labelParamNumber,         iRow, 0, 1, 1, Qt::AlignTop);
    grid_->addWidget(labelParamName,           iRow, 1, 1, 1, Qt::AlignTop);
    grid_->addWidget(matrixSpinBoxParamValue,  iRow, 2, 1, 1, Qt::AlignTop);
    grid_->addWidget(pushButtonChangeParam,    iRow, 3, 1, 1, Qt::AlignTop);

    // Connect signal pressed of change push button
    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

    // Refresh
    refreshParam();
  }

 public slots:
  //! React to press of change push button
  virtual void pushButtonChangeParamPressed() = 0;
  //! Refresh the parameter on command
  virtual void refreshParam() = 0;

 protected:
  /*** Factory method for spin box type
   *   @return base class pointer to the matrix spin box
   */
  virtual QWidget* createMatrixSpinBox(QWidget* parent) = 0;

 public:
  //! Label for the parameter number
  QLabel* labelParamNumber;
  //! Label for the parameter name
  QLabel* labelParamName;
  //! Matrix spinbox
  QWidget* matrixSpinBoxParamValue;
  //! Push button to apply changes
  QPushButton* pushButtonChangeParam;

 protected:
  //! Parameter name
  std::string name_;
  //! ROS service to get the parameter
  ros::ServiceClient* getParameterClient_;
  //! ROS service to set the parameter
  ros::ServiceClient* setParameterClient_;
  //! Grid layout that holds parameter blocks
  QGridLayout* grid_;

};

} // end namespace
