/*
 * QStereoBinaryBMOptions.cc
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#include "QStereoBinaryBMOptions.h"

QStereoBinaryBMOptions::QStereoBinaryBMOptions(QWidget * parent) :
    Base(parent)
{
  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "minDisparity",
          [this](int value) {
            if ( _opts && _opts->minDisparity != value ) {
              _opts->minDisparity = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->minDisparity;
              return true;
            }
            return false;
          });

  numDisparities_ctl =
      add_numeric_box<int>("numDisparities",
          "numDisparities",
          [this](int value) {
            if ( _opts && _opts->numDisparities != value ) {
              _opts->numDisparities = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->numDisparities;
              return true;
            }
            return false;
          });

  blockSize_ctl =
      add_numeric_box<int>("blockSize",
          "blockSize",
          [this](int value) {
            if ( _opts && _opts->blockSize != value ) {
              _opts->blockSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->blockSize;
              return true;
            }
            return false;
          });

  speckleWindowSize_ctl =
      add_numeric_box<int>("speckleWindowSize",
          "speckleWindowSize",
          [this](int value) {
            if ( _opts && _opts->speckleWindowSize != value ) {
              _opts->speckleWindowSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->speckleWindowSize;
              return true;
            }
            return false;
          });

  speckleRange_ctl =
      add_numeric_box<int>("speckleRange",
          "speckleRange",
          [this](int value) {
            if ( _opts && _opts->speckleRange != value ) {
              _opts->speckleRange = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->speckleRange;
              return true;
            }
            return false;
          });

  disp12MaxDiff_ctl =
      add_numeric_box<int>("disp12MaxDiff",
          "disp12MaxDiff",
          [this](int value) {
            if ( _opts && _opts->disp12MaxDiff != value ) {
              _opts->disp12MaxDiff = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->disp12MaxDiff;
              return true;
            }
            return false;
          });

  usePrefilter_ctl =
      add_checkbox("usePrefilter",
          "usePrefilter",
          [this](bool value) {
            if ( _opts && _opts->usePrefilter != value ) {
              _opts->usePrefilter = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( _opts ) {
              * value = _opts->usePrefilter;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinaryBMPrefilterType> *
  preFilterType_ctl =
      add_enum_combobox<StereoBinaryBMPrefilterType>("preFilterType",
          "preFilterType",
          [this](StereoBinaryBMPrefilterType value) {
            if ( _opts && _opts->preFilterType != value ) {
              _opts->preFilterType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinaryBMPrefilterType * value) {
            if ( _opts ) {
              * value = _opts->preFilterType;
              return true;
            }
            return false;
          });

//  QNumericBox *
  preFilterSize_ctl =
      add_numeric_box<int>("preFilterSize",
          "preFilterSize",
          [this](int value) {
            if ( _opts && _opts->preFilterSize != value ) {
              _opts->preFilterSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->preFilterSize;
              return true;
            }
            return false;
          });

//  QNumericBox *
  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "preFilterCap",
          [this](int value) {
            if ( _opts && _opts->preFilterCap != value ) {
              _opts->preFilterCap = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->preFilterCap;
              return true;
            }
            return false;
          });

//  QNumericBox *
  textureThreshold_ctl =
      add_numeric_box<int>("textureThreshold",
          "textureThreshold",
          [this](int value) {
            if ( _opts && _opts->textureThreshold != value ) {
              _opts->textureThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->textureThreshold;
              return true;
            }
            return false;
          });

//  QNumericBox *
  uniquenessRatio_ctl =
      add_numeric_box<int>("uniquenessRatio",
          "uniquenessRatio",
          [this](int value) {
            if ( _opts && _opts->uniquenessRatio != value ) {
              _opts->uniquenessRatio = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->uniquenessRatio;
              return true;
            }
            return false;
          });

//  QNumericBox *
  smallerBlockSize_ctl =
      add_numeric_box<int>("smallerBlockSize",
          "smallerBlockSize",
          [this](int value) {
            if ( _opts && _opts->smallerBlockSize != value ) {
              _opts->smallerBlockSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->smallerBlockSize;
              return true;
            }
            return false;
          });

//  QNumericBox *
  scalleFactor_ctl =
      add_numeric_box<int>("scalleFactor",
          "scalleFactor",
          [this](int value) {
            if ( _opts && _opts->scalleFactor != value ) {
              _opts->scalleFactor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->scalleFactor;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> *
  spekleRemovalTechnique_ctl =
      add_enum_combobox<StereoBinarySpeckleRemovalTechnique>("spekleRemoval",
          "spekleRemoval Technique",
          [this](StereoBinarySpeckleRemovalTechnique value) {
            if ( _opts && _opts->spekleRemovalTechnique != value ) {
              _opts->spekleRemovalTechnique = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySpeckleRemovalTechnique * value) {
            if ( _opts ) {
              * value = _opts->spekleRemovalTechnique;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinaryKernelType> *
  kernelType_ctl =
      add_enum_combobox<StereoBinaryKernelType>("kernelType",
          "kernelType",
          [this](StereoBinaryKernelType value) {
            if ( _opts && _opts->kernelType != value ) {
              _opts->kernelType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinaryKernelType * value) {
            if ( _opts ) {
              * value = _opts->kernelType;
              return true;
            }
            return false;
          });

//  QNumericBox *
  agregationWindowSize_ctl =
      add_numeric_box<int>("agregationWindowSize",
          "agregationWindowSize",
          [this](int value) {
            if ( _opts && _opts->agregationWindowSize != value ) {
              _opts->agregationWindowSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->agregationWindowSize;
              return true;
            }
            return false;
          });

  updateControls();
}
