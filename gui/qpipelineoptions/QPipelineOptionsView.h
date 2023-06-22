/*
 * QPipelineOptionsView.h
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineOptionsView_h__
#define __QPipelineOptionsView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qimagestacking/QImageStackingOptions.h>
#include <gui/qcameracalibration/QCameraCalibrationPipelineOptions.h>
#include <gui/qstereocalibration/QStereoCalibrationPipelineOptions.h>
#include <gui/qstereomatcher/QStereoMatcherPipelineOptions.h>
#include <gui/qrstereo/QRStereoOptions.h>
#include <core/pipeline/c_image_processing_pipeline.h>

class QPipelineOptionsView:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QPipelineOptionsView ThisClass;
  typedef QWidget Base;

  QPipelineOptionsView(QWidget * parent = nullptr);

  void set_current_sequence(const c_image_sequence::sptr & image_sequence);
  const c_image_sequence::sptr & current_sequence() const;

  bool cloneCurrentPipeline(const std::vector<c_image_sequence::sptr> & dst);

Q_SIGNALS:
  void parameterChanged();
  void closeWindowRequested();
  void cloneCurrentPipelineRequested();

protected:
  void oncurrentpipelinechanged();
  void enablecontrols(bool v);
  void updateCurrentSettingsWidget(const c_image_processing_pipeline::sptr &pipeline);

protected Q_SLOTS:
  void onPipelineSelectorCurrentIndexChanged(int);
  void onMenuButtonClicked();
  void onAddPipeline();
  void onRemovePipeline();
  void onRenamePipeline();


protected:
  c_image_sequence::sptr current_sequence_;
  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QLabel * sequenceName_lb = nullptr;
  QComboBox * pipelineSelector_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;
  QToolButton * cloneButton_ctl = nullptr;
  QAction * close_ctl = nullptr;

  QScrollArea * scrollArea_ctl = nullptr;
  QImageStackingOptions * imageStackingOptions_ctl = nullptr;
  QCameraCalibrationPipelineOptions * cameraCalibrationOptions_ctl = nullptr;
  QStereoCalibrationPipelineOptions * stereoCalibrationOptions_ctl = nullptr;
  QStereoMatcherPipelineOptions * stereoMatcherOptions_ctl = nullptr;
  QRStereoOptions * rstereoCalibrationOptions_ctl = nullptr;

  QMenu menu_;
  bool updatingControls_ = false;
};

class QAddPipelineDialogBox :
    public QDialog
{
public:
  typedef QAddPipelineDialogBox ThisClass;
  typedef QDialog Base;

  QAddPipelineDialogBox(QWidget * parent = nullptr);

  QString selectedPipelineName() const;
  QString selectedPipelineClass() const;

protected:
  QFormLayout * form_ = nullptr;
  QHBoxLayout * hbox_ = nullptr;
  QLineEditBox * pipelineName_ctl = nullptr;
  QComboBox * pipelineTypeSelector_ctl = nullptr;
  QLabel * pipelineTooltop_ctl = nullptr;
  QPushButton * btnOk_ = nullptr;
  QPushButton * btnCancel_ = nullptr;
};

#endif /* __QPipelineOptionsView_h__ */
