/*
 * QStereoSGBMOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QStereoSGBMOptions.h"

QStereoSGBMOptions::QStereoSGBMOptions(QWidget * parent) :
    Base("", parent)
{
  mode_ctl =
      add_enum_combobox<StereoSGBM_Mode>("SGBM Mode:",
          "Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm.\n"
              "It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and \n"
              "huge for HD-size pictures.\nBy default, it is set to false.",
          [this](StereoSGBM_Mode value) {
            if ( options_ && options_->mode != value ) {
              options_->mode = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoSGBM_Mode * value) {
            if ( options_ ) {
              * value = options_->mode;
              return true;
            }
            return false;
          });

  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "Minimum possible disparity value. Normally, it is zero but sometimes\n"
              "rectification algorithms can shift images,\nso this parameter needs to be adjusted accordingly",
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
          "Maximum disparity minus minimum disparity.\nThe value is always greater than zero.\n"
              "In the current implementation, this parameter must be divisible by 16.",
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
          "Matched block size. It must be an odd number >=1.\n"
              "Normally, it should be somewhere in the 3..11 range.",
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
          "Maximum size of smooth disparity regions to consider their noise speckles and invalidate.\n"
          "Set it to 0 to disable speckle filtering.\n"
          "Otherwise, set it somewhere in the 50-200 range.",
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
          "Maximum disparity variation within each connected component.\n"
              "If you do speckle  filtering, set the parameter to a positive value, \n"
              "it will be implicitly multiplied by 16.\n"
              "Normally, 1 or 2 is good enough.",
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
          "Maximum allowed difference (in integer pixel units) in the left-right disparity check.\n"
              "Set it to a non-positive value to disable the check.",
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

  P1_ctl =
      add_numeric_box<int>("P1",
          "The first parameter controlling the disparity smoothness.\n"
              "See P2.",
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
          "The second parameter controlling the disparity smoothness.\n"
              "The larger the values are, the smoother the disparity is.\n"
              "P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.\n"
              "P2 is the penalty on the disparity change by more than 1 between neighbor pixels.\n"
              "The algorithm requires P2 > P1."
              "\nSee stereo_match.cpp sample where some reasonably good P1 and P2 values\n"
              " are shown (like 8*number_of_image_channels*blockSize*blockSize and \n"
              "32*number_of_image_channels*blockSize*blockSize , respectively).",
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

  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "Truncation value for the prefiltered image pixels.\n"
              "The algorithm first computes x-derivative at each pixel \n"
              "and clips its value by [-preFilterCap, preFilterCap] interval.\n"
              "The result values are passed to the Birchfield-Tomasi pixel cost function.",
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
          "Margin in percentage by which the best (minimum) computed cost function value\n"
          "should 'win' the second best value to consider the found match correct.\n"
          "Normally, a value within the 5-15 range is good enough.",
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

  updateControls();
}

void QStereoSGBMOptions::set_options(c_cvStereoSGBMOptions * options)
{
  options_ = options;
  updateControls();
}

c_cvStereoSGBMOptions* QStereoSGBMOptions::options() const
{
  return options_;
}

void QStereoSGBMOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

