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

#include <core/io/c_input_source.h>
#include <core/io/c_input_options.h>
#include <core/dataproc/c_data_frame_processor.h>

#include "QImageSourceView.h"
#include "QPointCloudSourceView.h"
#include "QTextSourceView.h"



namespace serstacker {

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

  void setCurrentProcessor(const c_data_frame_processor::sptr & processor);
  const c_data_frame_processor::sptr & currentProcessor() const;

  //void setInputSource(const c_input_source::sptr & current_source);
  const c_input_source::sptr & inputSource() const;

  const c_input_options * inputOptions() const;
  c_input_options * inputOptions();

  bool openFile(const QString & abspath);

  void reloadCurrentFrame();

  //////////////////////////////////////////
  // TEMPORARY HACK SUUFF
  //
  void setDebayerAlgorithm(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayerAlgorithm() const;

  void setDropBadPixels(bool v);
  bool dropBadPixels() const;

  void setBadPixelsVariationThreshold(double v);
  double badPixelsVariationThreshold() const;

  void setVloDataChannel(VLO_DATA_CHANNEL channel);
  VLO_DATA_CHANNEL vloDataChannel() const;

  c_vlo_processing_options * vlo_processing_options();

  void update_as_vlo_processing_options_chaned();
  //////////////////////////////////////////

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentViewChanged();
  void displayTypeChanged();
  void parameterChanged();
  void displayImageChanged();
  void currentFileNameChanged();
  void currentFrameChanged();

  void debayerAlgorithmChanged();
  void dropBadPixelsChanged();
  void badPixelsVariationThresholdChanged();
  void vloDataChannelChanged();

protected:
  void setupMainToolbar();
  void setupMtfDisplayFunction();
  void closeCurrentSource();
  void startDisplay();
  void loadNextFrame();
  void processCurrentFrame();
  void onSeek(int pos);
  void onStackedWidgetCurrentIndexChanged();
  void setViewType(DataViewType viewType);
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
  c_input_source::sptr currentSource_;
  c_data_frame::sptr currentFrame_;
  c_data_frame_processor::sptr currentProcessor_;
  c_input_options input_options_;

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

  //std::set<DataViewType> supportedViewTypes_;
  std::vector<c_enum_member> displayTypes_;
  DataViewType selectedViewType_ = DataViewType_Image;

  //////////////////////////////////////////
  // TEMPORARY HACK SUUFF
  DEBAYER_ALGORITHM debayerAlgorithm_ = DEBAYER_DEFAULT;
  bool filterBadPixels_ = false;
  double badPixelsVariationThreshold_ = 5;

  c_vlo_processing_options vlo_processing_options_;
  VLO_DATA_CHANNEL vlo_data_channel_ = VLO_DATA_CHANNEL_AMBIENT;
  //////////////////////////////////////////


};

} // namespace serstacker

#endif /* __QInputSourceView_h__ */
