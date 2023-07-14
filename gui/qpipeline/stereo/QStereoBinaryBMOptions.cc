/*
 * QStereoBinaryBMOptions.cc
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#include "QStereoBinaryBMOptions.h"

QStereoBinaryBMOptions::QStereoBinaryBMOptions(QWidget * parent) :
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

  usePrefilter_ctl =
      add_checkbox("usePrefilter",
          "usePrefilter",
          [this](bool value) {
            if ( options_ && options_->usePrefilter != value ) {
              options_->usePrefilter = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( options_ ) {
              * value = options_->usePrefilter;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinaryBMPrefilterType> *
  preFilterType_ctl =
      add_enum_combobox<StereoBinaryBMPrefilterType>("preFilterType",
          "preFilterType",
          [this](StereoBinaryBMPrefilterType value) {
            if ( options_ && options_->preFilterType != value ) {
              options_->preFilterType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinaryBMPrefilterType * value) {
            if ( options_ ) {
              * value = options_->preFilterType;
              return true;
            }
            return false;
          });

//  QNumericBox *
  preFilterSize_ctl =
      add_numeric_box<int>("preFilterSize",
          "preFilterSize",
          [this](int value) {
            if ( options_ && options_->preFilterSize != value ) {
              options_->preFilterSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->preFilterSize;
              return true;
            }
            return false;
          });

//  QNumericBox *
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

//  QNumericBox *
  textureThreshold_ctl =
      add_numeric_box<int>("textureThreshold",
          "textureThreshold",
          [this](int value) {
            if ( options_ && options_->textureThreshold != value ) {
              options_->textureThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->textureThreshold;
              return true;
            }
            return false;
          });

//  QNumericBox *
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

//  QNumericBox *
  smallerBlockSize_ctl =
      add_numeric_box<int>("smallerBlockSize",
          "smallerBlockSize",
          [this](int value) {
            if ( options_ && options_->smallerBlockSize != value ) {
              options_->smallerBlockSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->smallerBlockSize;
              return true;
            }
            return false;
          });

//  QNumericBox *
  scalleFactor_ctl =
      add_numeric_box<int>("scalleFactor",
          "scalleFactor",
          [this](int value) {
            if ( options_ && options_->scalleFactor != value ) {
              options_->scalleFactor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->scalleFactor;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> *
  spekleRemovalTechnique_ctl =
      add_enum_combobox<StereoBinarySpeckleRemovalTechnique>("spekleRemoval",
          "spekleRemoval Technique",
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

//  QNumericBox *
  agregationWindowSize_ctl =
      add_numeric_box<int>("agregationWindowSize",
          "agregationWindowSize",
          [this](int value) {
            if ( options_ && options_->agregationWindowSize != value ) {
              options_->agregationWindowSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->agregationWindowSize;
              return true;
            }
            return false;
          });

  updateControls();
}

void QStereoBinaryBMOptions::set_options(c_cvStereoBinaryBMOptions * options)
{
  options_ = options;
  updateControls();
}

c_cvStereoBinaryBMOptions* QStereoBinaryBMOptions::options() const
{
  return options_;
}

void QStereoBinaryBMOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}
