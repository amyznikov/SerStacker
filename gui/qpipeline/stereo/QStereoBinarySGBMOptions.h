/*
 * QStereoBinarySGBMOptions.h
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoBinarySGBMOptions_h__
#define __QStereoBinarySGBMOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

class QStereoBinarySGBMOptions :
    public QSettingsWidget
{
public:
  typedef QStereoBinarySGBMOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoBinarySGBMOptions(QWidget * parent = nullptr);

  void set_options(c_cvStereoBinarySGBMOptions * options);
  c_cvStereoBinarySGBMOptions* options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_cvStereoBinarySGBMOptions *options_ = nullptr;

  QNumericBox * minDisparity_ctl = nullptr;
  QNumericBox * numDisparities_ctl = nullptr;
  QNumericBox * blockSize_ctl = nullptr;
  QNumericBox * speckleWindowSize_ctl = nullptr;
  QNumericBox * speckleRange_ctl = nullptr;
  QNumericBox * disp12MaxDiff_ctl = nullptr;

  QNumericBox * preFilterCap_ctl = nullptr;
  QNumericBox * uniquenessRatio_ctl = nullptr;
  QNumericBox * P1_ctl = nullptr;
  QNumericBox * P2_ctl = nullptr;

  QEnumComboBox<StereoBinarySGBMMode> * mode_ctl = nullptr;
  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> * spekleRemovalTechnique_ctl = nullptr;
  QEnumComboBox<StereoBinaryKernelType> * kernelType_ctl = nullptr;
  QEnumComboBox<StereoBinarySubpixelInterpolationMethod> * subPixelInterpolationMethod_ctl = nullptr;

};

#endif /* __QStereoBinarySGBMOptions_h__ */
