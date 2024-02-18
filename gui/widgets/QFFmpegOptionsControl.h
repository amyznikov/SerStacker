/*
 * QFFmpegOptionsControl.h
 *
 *  Created on: Feb 18, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFFmpegOptionsControl_h__
#define __QFFmpegOptionsControl_h__

#include "QLineEditBox.h"


class QFFmpegOptionsControl :
    public QLineEditBox
{
  Q_OBJECT;
public:
  typedef QFFmpegOptionsControl ThisClass;
  typedef QLineEditBox Base;

  QFFmpegOptionsControl(QWidget * parent = nullptr);

protected:
  void onMenuButtonClicked();

protected:
  QToolButton * menubutton_ctl = nullptr;
};

#endif /* __QFFmpegOptionsControl_h__ */
