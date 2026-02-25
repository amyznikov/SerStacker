/*
 * QStereoBMOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QStereoBMOptions.h"

QStereoBMOptions::QStereoBMOptions(QWidget * parent) :
    Base(parent)
{
  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "Minimum possible disparity value.\n"
              "Normally, it is zero but sometimes rectification algorithms can shift images,\n"
              "so this parameter needs to be adjusted accordingly",
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
          "The disparity search range.\n"
              "For each pixel algorithm will find the best disparity from 0 (default minimum disparity) to numDisparities.\n"
              "The search range can then be shifted by changing the minimum disparity",
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
          "The linear size of the blocks compared by the algorithm.\n"
              "The size should be odd (as the block is centered at the current pixel).\n"
              "Larger block size implies smoother, though less accurate disparity map.\n"
              "Smaller block size gives more detailed disparity map, but there is higher\n"
              "chance for algorithm to find a wrong correspondence.",
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
          "Maximum size of smooth disparity regions to consider their noise speckles and invalidate.\n"
              "Set it to 0 to disable speckle filtering.\n"
              "Otherwise, set it somewhere in the 50-200 range.",
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
          "Maximum disparity variation within each connected component.\n"
              "If you do speckle  filtering, set the parameter to a positive value, \n"
              "it will be implicitly multiplied by 16.\n"
              "Normally, 1 or 2 is good enough.",
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
          "Maximum allowed difference (in integer pixel units) in the left-right disparity check.\n"
              "Set it to a non-positive value to disable the check.",
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

  preFilterType_ctl =
      add_enum_combobox<StereoBM_PreFilterType>("preFilterType",
          "",
          [this](StereoBM_PreFilterType value) {
            if ( _opts && _opts->preFilterType != value ) {
              _opts->preFilterType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBM_PreFilterType * value) {
            if ( _opts ) {
              * value = _opts->preFilterType;
              return true;
            }
            return false;
          });

  preFilterSize_ctl =
      add_numeric_box<int>("preFilterSize",
          "",
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

  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "",
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

  textureThreshold_ctl =
      add_numeric_box<int>("textureThreshold",
          "",
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

  uniquenessRatio_ctl =
      add_numeric_box<int>("uniquenessRatio",
          "",
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

  smallerBlockSize_ctl =
      add_numeric_box<int>("smallerBlockSize",
          "",
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

  updateControls();
}

//void QStereoBMOptions::set_options(c_cvStereoBMOptions * options)
//{
//  _opts = options;
//  updateControls();
//}
//
//c_cvStereoBMOptions* QStereoBMOptions::options() const
//{
//  return _opts;
//}
//
//void QStereoBMOptions::onupdatecontrols()
//{
//  if ( !_opts ) {
//    setEnabled(false);
//  }
//  else {
//    Base::onupdatecontrols();
//    setEnabled(true);
//  }
//}
