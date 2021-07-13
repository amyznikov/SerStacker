/*
 * QMtfControlDialogBox.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfControlDialogBox_h__
#define __QMtfControlDialogBox_h__

#include "QMtfControl.h"


class QMtfControlDialogBox
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QMtfControlDialogBox ThisClass;
  typedef QDialog Base;

  QMtfControlDialogBox(QWidget * parent = Q_NULLPTR);

  void setMtf(const c_pixinsight_midtones_transfer_function::ptr & mtf);
  const c_pixinsight_midtones_transfer_function::ptr & mtf() const;

  void setImage(cv::InputArray image);


signals:
  void mtfChanged();
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QMtfControl * imageLevelsConfigWidget_ = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};

#endif /* __QMtfControlDialogBox_h__ */
