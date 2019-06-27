/*!
* @file     BufferIndicator.cpp
* @author   Gabriel Hottiger
* @date     October 10, 2016
* @brief    Indicator for number of read/unread items.
*/
// rqt_signal_logger
#include "rqt_signal_logger/BufferIndicator.hpp"

// Qt
#include <QPainter>

// Stl
#include <math.h>
#include <iostream>

namespace rqt_signal_logger {

BufferIndicator::BufferIndicator(QWidget* parent) :
  QWidget(parent),
  alignment_(Qt::AlignCenter),
  diameter_(std::min(30, parent->height())),
  nrUnreadElements_(0),
  nrTotalElements_(0),
  bufferSize_(0),
  colorUnread_(QColor(85, 117, 168)),
  colorTotal_(QColor(170, 196, 237,150))
{
  diamX_ = diameter_;
  diamY_ = diameter_;
  update();

  // Enable this to refresh on mouse over
  //  this->setMouseTracking(true);
}

BufferIndicator::~BufferIndicator()
{
}

Qt::Alignment BufferIndicator::alignment() const
{
  return alignment_;
}

void BufferIndicator::setAlignment(Qt::Alignment alignment)
{
  alignment_ = alignment;

  update();
}

int BufferIndicator::heightForWidth(int width) const
{
  return width;
}

QSize BufferIndicator::sizeHint() const
{
  return QSize(diamX_, diamY_);
}

QSize BufferIndicator::minimumSizeHint() const
{
  return QSize(diamX_, diamY_);
}

void BufferIndicator::updateData(const std::size_t nrUnreadElements, std::size_t nrTotalElements, std::size_t bufferSize)
{
  // set sizes
  bufferSize_ = bufferSize;
  nrTotalElements_ = nrTotalElements;
  nrUnreadElements_ = nrUnreadElements;

  // set the tooltip for additional info
  std::string toolTip = std::string{"Buffer size: \t"} + std::to_string(bufferSize_)
  + std::string{"\nTotal items: \t"} + std::to_string(nrTotalElements_)
  + std::string{"\nUnread items: \t"} + std::to_string(nrUnreadElements_);
  this->setToolTip(QString::fromStdString(toolTip));

  // redraw
  update();
}

//bool BufferIndicator::event(QEvent *event){
//  switch(event->type())
//  {
//    case QEvent::Enter:
//      emit refresh();
//      return true;
//      break;
//    default:
//      break;
//  }
//  return QWidget::event(event);
//}

void BufferIndicator::paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event);

  QRect geo = geometry();
  int width = geo.width();
  int height = geo.height();

  // Handle alignement
  int x=0, y=0;
  if ( alignment_ & Qt::AlignLeft )
    x = 0;
  else if ( alignment_ & Qt::AlignRight )
    x = width-diamX_;
  else if ( alignment_ & Qt::AlignHCenter )
    x = (width-diamX_)/2;
  else if ( alignment_ & Qt::AlignJustify )
    x = 0;

  if ( alignment_ & Qt::AlignTop )
    y = 0;
  else if ( alignment_ & Qt::AlignBottom )
    y = height-diamY_;
  else if ( alignment_ & Qt::AlignVCenter )
    y = (height-diamY_)/2;

  // Declare bounding rectangles and center point
  QRectF outerBox = QRectF(x,y,diamX_,diamY_);
  QRectF innerBox = QRectF(x+diamX_/3,y+diamX_/3,diamX_/3,diamY_/3);
  QPointF center(x+diamX_/2, y+diamY_/2);

  // Setup painter
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setPen(Qt::NoPen);

  // Draw white background circle
  p.setBrush(QBrush(QColor("white")));
  p.drawEllipse(outerBox);

  // Draw total elements circle segment
  QPainterPath totalArc;
  totalArc.moveTo(center);
  totalArc.arcTo(outerBox, 90, - (double)nrTotalElements_/(double)bufferSize_ * 360.0);
  p.drawPath(totalArc);
  p.fillPath (totalArc, QBrush (colorTotal_));

  // Draw unread elements circle (always smaller or equal to total elements, equal size use pattern)
  QPainterPath unreadArc;
  unreadArc.moveTo(center);
  unreadArc.arcTo(outerBox, 90, - (double)nrUnreadElements_/(double)bufferSize_ * 360.0);
  QBrush unreadBrush = nrTotalElements_ != nrUnreadElements_? QBrush (colorUnread_): QBrush(QColor(75,0,130));
  p.drawPath(unreadArc);
  p.fillPath (unreadArc, unreadBrush);

  // Draw inner circle with background color
  p.setBrush(QBrush(QWidget::palette().color(QWidget::backgroundRole())));
  p.drawEllipse(innerBox);

}

}
