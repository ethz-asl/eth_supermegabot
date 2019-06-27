/*
 * MatrixSpinBox.hpp
 *
 *  Created on: Nov 30, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Qt
#include <QWidget>
#include <QGridLayout>
#include <QAbstractSpinBox>

// Stl
#include <vector>
#include <memory>
#include <type_traits>
#include <assert.h>

namespace rqt_parameters {

//! Class that combines multiple spinboxes representing a matrix
template < typename SpinBoxType_ >
class MatrixSpinBox: public QWidget {
  //! Assertion to check for valid base class
  static_assert(std::is_base_of<QAbstractSpinBox, SpinBoxType_>::value ,"[MatrixSpinBox]: QAbstractSpinBox must be a base of SpinBoxType_!");

 public:

  /*** Constructor
   *   @param parent widget
   */
  MatrixSpinBox(QWidget* parent):
    QWidget(parent),
    grid_(this),
    spinboxes_(),
    rows_(0),
    cols_(0)
  {
    // Setup grid
    grid_.setContentsMargins(0,0,0,0);
    grid_.setVerticalSpacing(0);
    grid_.setHorizontalSpacing(0);
    grid_.setAlignment(Qt::AlignTop);
  }

  //! Default destructor
  ~MatrixSpinBox() {

  }

  /*** If the size of the matrix changed, all spinboxes are deleted and new matrix of spinboxes is built up.
   *   @param rows Number of rows of the matrix
   *   @param cols Number of columns of the matrix
   */
  void refresh(std::size_t rows, std::size_t cols)
  {
    if(rows != rows_ || cols != cols_)
    {

      // Set rows and columns
      rows_ = rows;
      cols_ = cols;

      // Clear grid without deleting objects
      clearLayout(&grid_);

      // Delete old objects and allocate new size
      spinboxes_.clear();
      spinboxes_.resize(rows_ * cols_);

      // Setup spinboxes
      for(auto & sb : spinboxes_) {
        sb.reset(new SpinBoxType_(this));
        sb->setAlignment(Qt::AlignTop);
        sb->setMaximumWidth(100);
      }

      // Add them to the grid
      for(unsigned int r = 0; r<rows_; ++r) {
        for(unsigned int c = 0; c<cols_; ++c) {
          grid_.addWidget(getSpinbox(r,c).get(), r , c, Qt::AlignTop);
        }
      }

    }
  }

  /*** Access a specific spinbox by reference
   *   @param  rows Row of the matrix
   *   @param  cols Column of the matrix
   *   @return      Spinbox
   */
  std::unique_ptr<SpinBoxType_> & getSpinbox(std::size_t row, std::size_t col)
  {
    return spinboxes_.at(row*cols_ + col);
  }

  //! @return current rows of the matrix spinbox
  std::size_t rows() const { return rows_; }

  //! @return current columns of the matrix spinbox
  std::size_t cols() const { return cols_; }

 private:
  /***  Clears a layout without deleting the objects in it!
   *    @param Layout to clear.
   */
  void clearLayout(QLayout* layout)
  {
      while (QLayoutItem* item = layout->takeAt(0))
      {
          if (QLayout* childLayout = item->layout())
              clearLayout(childLayout);
          delete item;
      }
  }

 private:
  //! Grid layout to arrange the matrix of spinboxes
  QGridLayout grid_;
  //! Container for the spinboxes (row-major)
  std::vector< std::unique_ptr<SpinBoxType_> > spinboxes_;
  //! Number of rows of the matrix of spinboxes
  std::size_t rows_;
  //! Number of columns of the matrix of spinboxes
  std::size_t cols_;

};

} // end namespace
