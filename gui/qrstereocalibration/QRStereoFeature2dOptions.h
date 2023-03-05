/*
 * QRStereoFeature2dOptions.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoFeature2dOptions_h___
#define __QRStereoFeature2dOptions_h___

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_rstereo_calibration_pipeline.h>
#include <gui/qfeature2d/QFeature2dOptions.h>


class QRStereoFeature2dOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QRStereoFeature2dOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoFeature2dOptions(QWidget * parent = nullptr);

  void set_feature2d_options(c_rstereo_feature2d_options * options);
  c_rstereo_feature2d_options * feature2d_options() const;

protected:
  void onupdatecontrols() override;
  // void update_controls_state();

protected slots:
  void onDetectorTypeChanged();

protected:
  c_rstereo_feature2d_options * options_ = nullptr;
  QNumberEditBox * scale_ctl = nullptr;
  QSparseFeatureDetectorOptions * sparseFeatureDetectorOptions_ctl = nullptr;
  QSparseFeatureDescriptorOptions * sparseFeatureDescriptorOptions_ctl = nullptr;
  QSparseFeatureMatcherOptions * sparseFeatureMatcherOptions_ctl = nullptr;
  QWidgetList controls;
};



#endif /* __QRStereoFeature2dOptions_h___ */
