/*
 * QStereoBinarySGBMOptions.cc
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#include "QStereoBinarySGBMOptions.h"

QStereoBinarySGBMOptions::QStereoBinarySGBMOptions(QWidget * parent) :
    Base("", parent)
{
  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "minDisparity",
          [this](int value) {
            if ( options_ && options_->minDisparity != value ) {
              options_->minDisparity = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->minDisparity;
              return true;
            }
            return false;
          });

  numDisparities_ctl =
      add_numeric_box<int>("numDisparities",
          "numDisparities",
          [this](int value) {
            if ( options_ && options_->numDisparities != value ) {
              options_->numDisparities = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->numDisparities;
              return true;
            }
            return false;
          });

  blockSize_ctl =
      add_numeric_box<int>("blockSize",
          "blockSize",
          [this](int value) {
            if ( options_ && options_->blockSize != value ) {
              options_->blockSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->blockSize;
              return true;
            }
            return false;
          });

  speckleWindowSize_ctl =
      add_numeric_box<int>("speckleWindowSize",
          "speckleWindowSize",
          [this](int value) {
            if ( options_ && options_->speckleWindowSize != value ) {
              options_->speckleWindowSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->speckleWindowSize;
              return true;
            }
            return false;
          });

  speckleRange_ctl =
      add_numeric_box<int>("speckleRange",
          "speckleRange",
          [this](int value) {
            if ( options_ && options_->speckleRange != value ) {
              options_->speckleRange = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->speckleRange;
              return true;
            }
            return false;
          });

  disp12MaxDiff_ctl =
      add_numeric_box<int>("disp12MaxDiff",
          "disp12MaxDiff",
          [this](int value) {
            if ( options_ && options_->disp12MaxDiff != value ) {
              options_->disp12MaxDiff = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->disp12MaxDiff;
              return true;
            }
            return false;
          });

  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "preFilterCap",
          [this](int value) {
            if ( options_ && options_->preFilterCap != value ) {
              options_->preFilterCap = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->preFilterCap;
              return true;
            }
            return false;
          });

  uniquenessRatio_ctl =
      add_numeric_box<int>("uniquenessRatio",
          "uniquenessRatio",
          [this](int value) {
            if ( options_ && options_->uniquenessRatio != value ) {
              options_->uniquenessRatio = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->uniquenessRatio;
              return true;
            }
            return false;
          });

  P1_ctl =
      add_numeric_box<int>("P1",
          "P1",
          [this](int value) {
            if ( options_ && options_->P1 != value ) {
              options_->P1 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->P1;
              return true;
            }
            return false;
          });

  P2_ctl =
      add_numeric_box<int>("P2",
          "P2",
          [this](int value) {
            if ( options_ && options_->P2 != value ) {
              options_->P2 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->P2;
              return true;
            }
            return false;
          });

  //////

  mode_ctl =
      add_enum_combobox<StereoBinarySGBMMode>("mode",
          "mode",
          [this](StereoBinarySGBMMode value) {
            if ( options_ && options_->mode != value ) {
              options_->mode = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySGBMMode * value) {
            if ( options_ ) {
              * value = options_->mode;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> *
  spekleRemovalTechnique_ctl =
      add_enum_combobox<StereoBinarySpeckleRemovalTechnique>("spekleRemovalTechnique",
          "spekleRemovalTechnique",
          [this](StereoBinarySpeckleRemovalTechnique value) {
            if ( options_ && options_->spekleRemovalTechnique != value ) {
              options_->spekleRemovalTechnique = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySpeckleRemovalTechnique * value) {
            if ( options_ ) {
              * value = options_->spekleRemovalTechnique;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinaryKernelType> *
  kernelType_ctl =
      add_enum_combobox<StereoBinaryKernelType>("kernelType",
          "kernelType",
          [this](StereoBinaryKernelType value) {
            if ( options_ && options_->kernelType != value ) {
              options_->kernelType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinaryKernelType * value) {
            if ( options_ ) {
              * value = options_->kernelType;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinarySubpixelInterpolationMethod> *
  subPixelInterpolationMethod_ctl =
      add_enum_combobox<StereoBinarySubpixelInterpolationMethod>("subPixelInterpolation",
          "subPixelInterpolation method",
          [this](StereoBinarySubpixelInterpolationMethod value) {
            if ( options_ && options_->subPixelInterpolationMethod != value ) {
              options_->subPixelInterpolationMethod = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySubpixelInterpolationMethod * value) {
            if ( options_ ) {
              * value = options_->subPixelInterpolationMethod;
              return true;
            }
            return false;
          });

  updateControls();
}

void QStereoBinarySGBMOptions::set_options(c_cvStereoBinarySGBMOptions * options)
{
  options_ = options;
  updateControls();
}

c_cvStereoBinarySGBMOptions* QStereoBinarySGBMOptions::options() const
{
  return options_;
}

void QStereoBinarySGBMOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}
