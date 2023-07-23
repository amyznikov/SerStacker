/*
 * QPipelineOptionsCtrl.h
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineOptionsCtrl_h__
#define __QPipelineOptionsCtrl_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/c_image_processing_pipeline_ctrl.h>

class QPipelineSettingsCtrl :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QPipelineSettingsCtrl ThisClass;
  typedef QSettingsWidget Base;

  QPipelineSettingsCtrl(QWidget * parent = nullptr);
  QPipelineSettingsCtrl(const std::vector<c_image_processing_pipeline_ctrl> & ctrls, QWidget * parent = nullptr);

  void setup_controls(const std::vector<c_image_processing_pipeline_ctrl> & ctrls);

  void set_pipeline(c_image_processing_pipeline * pipeline);
  c_image_processing_pipeline * pipeline() const;

protected:
  void onupdatecontrols() override;
  void update_control_states();

protected:
  c_image_processing_pipeline * pipeline_ = nullptr;
  std::map<QWidget*, std::function<bool(const c_image_processing_pipeline*)>> state_ctls_;

};


#endif /* __QPipelineOptionsCtrl_h__ */
