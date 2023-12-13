/*
 * QInputSourceView.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputSourceView_h__
#define __QInputSourceView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qimageview/QImageEditor.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qplaysequencecontrol/QPlaySequenceControl.h>

#include "dataset/c_cloudview_dataframe_processor.h"
#include "dataset/c_cloudview_input_source.h"

#include "QImageSourceView.h"
#include "QPointCloudSourceView.h"
#include "QTextSourceView.h"



namespace cloudview {

class QInputSourceView :
    public QWidget,
    public IMtfDisplay,
    public QImageDisplayFunction,
    public QCloudViewDisplayFunction

{
  Q_OBJECT;
  Q_INTERFACES(IMtfDisplay)
public:
  typedef QInputSourceView ThisClass;
  typedef QWidget Base;

  QInputSourceView(QWidget * parent = nullptr);

  QToolBar * toolbar() const;
  QToolBar * imageViewToolbar() const;
  QToolBar * cloudViewToolbar() const;
  QToolBar * textViewToolbar() const;
  QToolBar * rightToolbar() const;

  QImageSourceView * imageView() const;
  QPointCloudSourceView * cloudView() const;
  QTextSourceView * textView() const;

  void setCurrentView(QWidget * w);
  QWidget * currentView() const;

  void setCurrentToolbar(QToolBar * toolbar);
  QToolBar * currentToolbar() const;

  QString currentFileName() const;

  IMtfDisplay * mtfDisplay();
  const IMtfDisplay * mtfDisplay() const;

  void setCurrentProcessor(const c_cloudview_processor::sptr & processor);
  const c_cloudview_processor::sptr & currentProcessor() const;

  void setInputSource(const c_cloudview_input_source::sptr & current_source);
  const c_cloudview_input_source::sptr & inputSource() const;

  bool openFile(const QString & abspath);

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentViewChanged();
  void displayTypeChanged();
  void parameterChanged();
  void displayImageChanged();

protected:
  void setupMainToolbar();
  void setupMtfDisplayFunction();
  void closeCurrentSource();
  void startDisplay();
  void loadNextFrame();
  void processCurrentFrame();
  void onSeek(int pos);
  void onStackedWidgetCurrentIndexChanged();
  void setViewType(ViewType viewType);
  void displayCurrentFrame();

  bool applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U);

  bool applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
      cv::OutputArray colormapImage);

protected: // QWidget
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected: // QImageDisplayFunction
  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::Mat & mtfImage, cv::Mat & displayImage, int ddepth = CV_8U) override;

protected: // QCloudViewDisplayFunction
  void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMask,
      cv::OutputArray displayPoints,
      cv::OutputArray mtfColors,
      cv::OutputArray displayColors) override;

protected: // MTF
  const c_enum_member * displayTypes() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

protected:
  c_cloudview_input_source::sptr currentSource_;
  c_cloudview_data_frame::sptr currentFrame_;
  c_cloudview_processor::sptr currentProcessor_;

  QVBoxLayout * mainLayout_ = nullptr;
  QHBoxLayout * toolbarLayout_ = nullptr;
  QToolBar * mainToolbar_ = nullptr;
  QToolBar * imageViewToolbar_ = nullptr;
  QToolBar * cloudViewToolbar_ = nullptr;
  QToolBar * textViewToolbar_ = nullptr;
  QToolBar * rightToolbar_ = nullptr;

  QStackedWidget * stackWidget_ = nullptr;
  QPlaySequenceControl * playControls_ = nullptr;

  QImageSourceView * imageView_ = nullptr;
  QPointCloudSourceView * cloudView_ = nullptr;
  QTextSourceView * textView_ = nullptr;

  QToolButton * viewSelectionToolbutton_ctl = nullptr;

  std::set<ViewType> supportedViewTypes_;
  std::vector<c_enum_member> displayTypes_;
  ViewType selectedViewType_ = ViewType_Image;
};

} // namespace cloudview

#endif /* __QInputSourceView_h__ */
