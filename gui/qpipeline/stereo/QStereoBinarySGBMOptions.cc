/*
 * QStereoBinarySGBMOptions.cc
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#include "QStereoBinarySGBMOptions.h"

QStereoBinarySGBMOptions::QStereoBinarySGBMOptions(QWidget * parent) :
    Base(parent)
{
  minDisparity_ctl =
      add_numeric_box<int>("minDisparity",
          "Minimum possible disparity value.\n"
              " Normally, it is zero but sometimes rectification algorithms can shift images,\n"
              " so this parameter needs to be adjusted accordingly.\n",
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
          "Maximum disparity minus minimum disparity.\n"
              " The value is always greater than zero. \n"
              " In the current implementation, this parameter must be divisible by 16.",
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
          "Matched block size.\n"
              "It must be an odd number >=1.\n"
              "Normally, it should be somewhere in the 3..11 range.",
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
              " Set it to 0 to disable speckle filtering.\n"
              " Otherwise, set it somewhere in the 50-200 range.",
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
              " If you do speckle filtering, set the parameter to a positive value,\n"
              " it will be implicitly multiplied by 16.\n"
              " Normally, 1 or 2 is good enough.",
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
          "Maximum allowed difference (in pixel units) in the left-right disparity check.\n"
              "Set it to a non-positive value to disable the check",
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

  preFilterCap_ctl =
      add_numeric_box<int>("preFilterCap",
          "Truncation value for the prefiltered image pixels.\n"
              " The algorithm first computes x-derivative at each pixel and \n"
              " clips its value by [-preFilterCap, preFilterCap] interval.\n"
              " The result values are passed to the Birchfield-Tomasi pixel cost function.",
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

  uniquenessRatio_ctl =
      add_numeric_box<int>("uniquenessRatio",
          "Margin in percentage by which the best (minimum) computed cost function\n"
              "should 'win' the second best value to consider the found match correct.\n"
              " Normally, a value within the 5-15 range is good enough.",
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

  P1_ctl =
      add_numeric_box<int>("P1",
          "The first parameter controlling the disparity smoothness.\n"
              "This parameter is used for the case of slanted surfaces (not fronto parallel)",
          [this](int value) {
            if ( _opts && _opts->P1 != value ) {
              _opts->P1 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->P1;
              return true;
            }
            return false;
          });

  P2_ctl =
      add_numeric_box<int>("P2",
          "The second parameter controlling the disparity smoothness.\n"
              "This parameter is used for solving the depth discontinuities problem.\n"
              "The larger the values are, the smoother the disparity is.\n"
              "P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.\n"
              "P2 is the penalty on the disparity change by more than 1 between neighbor pixels.\n"
              "The algorithm requires P2 > P1 .\n"
              "See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown\n"
              " (like 8 *number_of_image_channels * SADWindowSize * SADWindowSize and\n"
              "      32 *number_of_image_channels * SADWindowSize * SADWindowSize, respectively).",
          [this](int value) {
            if ( _opts && _opts->P2 != value ) {
              _opts->P2 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->P2;
              return true;
            }
            return false;
          });

  //////

  mode_ctl =
      add_enum_combobox<StereoBinarySGBMMode>("mode",
          "Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm.\n"
              " It will consume O(W * H * numDisparities) bytes, which is large for 640x480 stereo \n"
              " and huge for HD-size pictures.\n"
              " By default, it is set to false .",
          [this](StereoBinarySGBMMode value) {
            if ( _opts && _opts->mode != value ) {
              _opts->mode = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySGBMMode * value) {
            if ( _opts ) {
              * value = _opts->mode;
              return true;
            }
            return false;
          });

//  QEnumComboBox<StereoBinarySpeckleRemovalTechnique> *
  spekleRemovalTechnique_ctl =
      add_enum_combobox<StereoBinarySpeckleRemovalTechnique>("spekleRemovalTechnique",
          "spekleRemovalTechnique",
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

//  QEnumComboBox<StereoBinarySubpixelInterpolationMethod> *
  subPixelInterpolationMethod_ctl =
      add_enum_combobox<StereoBinarySubpixelInterpolationMethod>("subPixelInterpolation",
          "subPixelInterpolation method",
          [this](StereoBinarySubpixelInterpolationMethod value) {
            if ( _opts && _opts->subPixelInterpolationMethod != value ) {
              _opts->subPixelInterpolationMethod = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](StereoBinarySubpixelInterpolationMethod * value) {
            if ( _opts ) {
              * value = _opts->subPixelInterpolationMethod;
              return true;
            }
            return false;
          });

  updateControls();
}
