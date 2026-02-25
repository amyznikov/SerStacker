/*
 * QStereoBinaryBMOptions.h
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoBinaryBMOptions_h__
#define __QStereoBinaryBMOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

class QStereoBinaryBMOptions :
    public QSettingsWidgetTemplate<c_cvStereoBinaryBMOptions>
{
public:
  typedef QStereoBinaryBMOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_cvStereoBinaryBMOptions> Base;

  QStereoBinaryBMOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * minDisparity_ctl = nullptr;
  QNumericBox * numDisparities_ctl = nullptr;
  QNumericBox * blockSize_ctl = nullptr;
  QNumericBox * speckleWindowSize_ctl = nullptr;
  QNumericBox * speckleRange_ctl = nullptr;
  QNumericBox * disp12MaxDiff_ctl = nullptr;

  QCheckBox * usePrefilter_ctl = nullptr;
  QEnumComboBox<StereoBinaryBMPrefilterType> * preFilterType_ctl = nullptr;
  QNumericBox * preFilterSize_ctl = nullptr;
  QNumericBox * preFilterCap_ctl = nullptr;
  QNumericBox * textureThreshold_ctl = nullptr;
  QNumericBox * uniquenessRatio_ctl = nullptr;
  QNumericBox * smallerBlockSize_ctl = nullptr;
  QNumericBox * scalleFactor_ctl = nullptr;
  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> * spekleRemovalTechnique_ctl = nullptr;
  QEnumComboBox<StereoBinaryKernelType> *  kernelType_ctl = nullptr;
  QNumericBox * agregationWindowSize_ctl = nullptr;
};


#endif /* __QStereoBinaryBMOptions_h__ */
