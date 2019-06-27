 /*!
 * @file     BufferIndicator.hpp
 * @author   Gabriel Hottiger
 * @date     October 10, 2016
 * @brief    Indicator for number of read/unread items.
 */

#pragma once

// Qt
#include <QtDesigner>
#include <QWidget>

namespace rqt_signal_logger {

class QDESIGNER_WIDGET_EXPORT BufferIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(Qt::Alignment alignment READ alignment WRITE setAlignment)

 public:
  explicit BufferIndicator(QWidget* parent=0);
  ~BufferIndicator();

  double diameter() const;
  void setDiameter();

  Qt::Alignment alignment() const;
  void setAlignment(Qt::Alignment alignment);

  int heightForWidth(int width) const;
  QSize sizeHint() const;
  QSize minimumSizeHint() const;

 public slots:
  void updateData(const std::size_t nrUnreadElements, std::size_t nrTotalElements, std::size_t bufferSize);

 signals:
  void refresh();

 protected:
  void paintEvent(QPaintEvent* event);
//  bool event(QEvent *event); // catch mouse hover events

 private:
  Qt::Alignment alignment_;
  double diameter_;
  std::size_t nrUnreadElements_;
  std::size_t nrTotalElements_;
  std::size_t bufferSize_;
  QColor colorUnread_;
  QColor colorTotal_;

  // Pixels per mm for x and y...
  int pixX_, pixY_;
  // Scaled values for x and y diameter.
  int diamX_, diamY_;
};

}
