/*
 * QMtfDialogBox.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfDialogBox_h__
#define __QMtfDialogBox_h__

#include "QMtfControl.h"


class QMtfDialogBox
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QMtfDialogBox ThisClass;
  typedef QDialog Base;

  QMtfDialogBox(QWidget * parent = Q_NULLPTR);

  void setDisplayFunction(QMtfDisplayFunction * displayFunction);
  QMtfDisplayFunction * displayFunction() const;

  void setInputImage(cv::InputArray image, cv::InputArray mask = cv::noArray());
  void setOutputHistogram(const cv::Mat1f & H, double hmin, double hmax);


signals:
  void visibilityChanged(bool visible);

public slots:
  void updateOutputHistogram();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QMtfControl * mtfControl_ = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};

#endif /* __QMtfDialogBox_h__ */
