/*
 * QInputSequenceView.h
 *
 *  Created on: Nov 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputSequenceView_h__
#define __QInputSequenceView_h__

#include <gui/qimageview/QImageEditor.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qplaysequencecontrol/QPlaySequenceControl.h>
#include <core/io/c_input_sequence.h>


class QInputSequenceView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QInputSequenceView ThisClass;
  typedef QWidget Base;

  QInputSequenceView(QWidget * parent = nullptr);

  QToolBar * toolbar() const;
  QToolBar * imageViewToolbar() const;
  QToolBar * cloudViewToolbar() const;

protected Q_SLOTS:
  void onStackedWidgetCurrentIndexChanged();
  void onSeek(int pos);

protected:
  void setCurrentToolbar(QToolBar * toolbar);
  void setCurrentStackedWidget(QWidget * w);
  QWidget * currentStackedWidget() const;

protected:
  c_input_sequence::sptr input_sequence_;
  QVBoxLayout * mainLayout_ = nullptr;
  QHBoxLayout * toolbarLayout_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QToolBar * imageViewToolbar_ = nullptr;
  QToolBar * cloudViewToolbar_ = nullptr;
  QStackedWidget * stackWidget_ = nullptr;
  QImageEditor * imageView_ = nullptr;
  QCloudViewer * cloudView_ = nullptr;
  QPlaySequenceControl * playControls_ = nullptr;
};

#endif /* __QInputSequenceView_h__ */
