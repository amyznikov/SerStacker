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
#include <gui/qcameracalibration/QCameraCalibrationOptions.h>
#include <gui/qstereocalibration/QStereoCalibrationOptions.h>
#include <gui/qrstereocalibration/QRStereoCalibrationOptions.h>
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

Q_SIGNALS:
  void parameterChanged();
  void closeWindowRequested();

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
  QToolButton * menuButton = nullptr;
  QAction * close_ctl = nullptr;

  QScrollArea * scrollArea_ctl = nullptr;
  QImageStackingOptions * imageStackingOptions_ctl = nullptr;
  QCameraCalibrationOptions * cameraCalibrationOptions_ctl = nullptr;
  QStereoCalibrationOptions * stereoCalibrationOptions_ctl = nullptr;
  QRStereoCalibrationOptions * rstereoCalibrationOptions_ctl = nullptr;

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
