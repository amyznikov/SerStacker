/*
 * QStereoBMOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QStereoBMOptions.h"

QStereoBMOptions::QStereoBMOptions(QWidget * parent) :
    Base("", parent)
{
  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "Minimum possible disparity value. Normally, it is zero but sometimes "
              "rectification algorithms can shift images, so this parameter needs to be adjusted accordingly",
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
          "The disparity search range. For each pixel algorithm will find the best "
              "disparity from 0 (default minimum disparity) to numDisparities. The search range can then be "
              "shifted by changing the minimum disparity",
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
          "the linear size of the blocks compared by the algorithm. The size should be odd "
              "(as the block is centered at the current pixel). Larger block size implies smoother, though less "
              "accurate disparity map. Smaller block size gives more detailed disparity map, but there is higher "
              "chance for algorithm to find a wrong correspondence.",
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
          "",
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
          "",
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
          "",
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

  preFilterType_ctl =
      add_numeric_box<int>("preFilterType",
          "",
          [this](int value) {
            if ( options_ && options_->preFilterType != value ) {
              options_->preFilterType = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->preFilterType;
              return true;
            }
            return false;
          });

  preFilterSize_ctl =
      add_numeric_box<int>("preFilterSize",
          "",
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

  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "",
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

  textureThreshold_ctl =
      add_numeric_box<int>("textureThreshold",
          "",
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

  uniquenessRatio_ctl =
      add_numeric_box<int>("uniquenessRatio",
          "",
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

  smallerBlockSize_ctl =
      add_numeric_box<int>("smallerBlockSize",
          "",
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

  updateControls();
}

void QStereoBMOptions::set_options(c_cvStereoBM_options * options)
{
  options_ = options;
  updateControls();
}

c_cvStereoBM_options* QStereoBMOptions::options() const
{
  return options_;
}

void QStereoBMOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}