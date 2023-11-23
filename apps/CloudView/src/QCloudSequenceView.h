/*
 * QCloudSequenceView.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudSequenceView_h__
#define __QCloudSequenceView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qimageview/QImageEditor.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qplaysequencecontrol/QPlaySequenceControl.h>
#include "dataset/c_cloudview_input_source.h"
#include "dataset/c_cloudview_dataframe_processor.h"

namespace cloudview {

class QCloudSequenceView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudSequenceView ThisClass;
  typedef QWidget Base;

  QCloudSequenceView(QWidget * parent = nullptr);

  QCloudSequenceView(QWidget * parent,
      QImageEditor * imageView,
      QCloudViewer * cloudView,
      QTextFileViewer * textView);

  QToolBar * toolbar() const;
  QToolBar * imageViewToolbar() const;
  QToolBar * cloudViewToolbar() const;
  QToolBar * textViewToolbar() const;
  QToolBar * rightToolbar() const;

  QImageEditor * imageView() const;
  QCloudViewer * cloudView() const;
  QTextFileViewer * textView() const;

  void setCurrentView(QWidget * w);
  QWidget * currentView() const;

  void setCurrentToolbar(QToolBar * toolbar);
  QToolBar * currentToolbar() const;

  void setInputSource(const c_cloudview_input_source::sptr & current_source);
  const c_cloudview_input_source::sptr & inputSource() const;

  bool openFile(const QString & abspath);

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentViewChanged();

protected:
  void closeCurrentSource();
  void startDisplay();
  void loadNextFrame();
  void onSeek(int pos);
  void onStackedWidgetCurrentIndexChanged();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  c_cloudview_input_source::sptr currentSource_;
  c_cloudview_dataframe_processor::sptr currentProcessor_;

  QVBoxLayout * mainLayout_ = nullptr;
  QHBoxLayout * toolbarLayout_ = nullptr;
  QToolBar * mainToolbar_ = nullptr;
  QToolBar * imageViewToolbar_ = nullptr;
  QToolBar * cloudViewToolbar_ = nullptr;
  QToolBar * textViewToolbar_ = nullptr;
  QToolBar * rightToolbar_ = nullptr;
  QStackedWidget * stackWidget_ = nullptr;
  QImageEditor * imageView_ = nullptr;
  QCloudViewer * cloudView_ = nullptr;
  QTextFileViewer * textView_ = nullptr;
  QPlaySequenceControl * playControls_ = nullptr;
  QComboBox * viewSelector_ = nullptr;

};

} // namespace cloudview

#endif /* __QCloudSequenceView_h__ */
