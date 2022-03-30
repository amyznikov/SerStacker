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

  void setMtf(const c_pixinsight_mtf::sptr & mtf);
  const c_pixinsight_mtf::sptr & mtf() const;

  void setInputImage(cv::InputArray image, cv::InputArray mask = cv::noArray());
  void setDisplayImage(cv::InputArray image, cv::InputArray mask = cv::noArray());


signals:
  void mtfChanged();
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QMtfControl * mtfControl_ = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};

#endif /* __QMtfControlDialogBox_h__ */
