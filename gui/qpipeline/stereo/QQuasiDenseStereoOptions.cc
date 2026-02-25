/*
 * QQuasiDenseStereoOptions.cc
 *
 *  Created on: Apr 2, 2023
 *      Author: amyznikov
 */

#include "QQuasiDenseStereoOptions.h"

#if HAVE_OpenCV_stereo

QQuasiDenseStereoOptions::QQuasiDenseStereoOptions(QWidget * parent) :
    Base(parent)
{
  corrWinSize_ctl =
      add_numeric_box<cv::Size>("corrWinSize",
          "similarity window size",
          [this](const cv::Size & value) {
            if ( _opts && (_opts->corrWinSizeX != value.width || _opts->corrWinSizeY != value.height) ) {
              _opts ->corrWinSizeX = value.width;
              _opts ->corrWinSizeY = value.height;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( _opts ) {
              * value = cv::Size(_opts ->corrWinSizeX, _opts ->corrWinSizeY);
              return true;
            }
            return false;
          });

  border_ctl =
      add_numeric_box<cv::Size>("border size",
          "border to ignore",
          [this](const cv::Size & value) {
            if ( _opts && (_opts ->borderX != value.width || _opts ->borderY != value.height) ) {
              _opts ->borderX = value.width;
              _opts ->borderY = value.height;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( _opts ) {
              * value = cv::Size(_opts ->borderX, _opts ->borderY);
              return true;
            }
            return false;
          });

  //matching
  correlationThreshold_ctl =
      add_numeric_box<float>("correlation threshold",
          "correlation threshold",
          [this](float value) {
            if ( _opts && _opts ->correlationThreshold != value ) {
              _opts ->correlationThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( _opts ) {
              * value = _opts ->correlationThreshold;
              return true;
            }
            return false;
          });

  // texture threshold
  textrureThreshold_ctl =
      add_numeric_box<float>("texture threshold",
          "texture threshold",
          [this](float value) {
            if ( _opts && _opts ->textrureThreshold != value ) {
              _opts ->textrureThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( _opts ) {
              * value = _opts ->textrureThreshold;
              return true;
            }
            return false;
          });

  // neighborhood size
  neighborhoodSize_ctl =
      add_numeric_box<int>("neighborhood size",
          "neighborhood size",
          [this](int value) {
            if ( _opts && _opts ->neighborhoodSize != value ) {
              _opts ->neighborhoodSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->neighborhoodSize;
              return true;
            }
            return false;
          });

  // disparity gradient threshold
  disparityGradient_ctl =
      add_numeric_box<int>("disp. gradient",
          "disparity gradient threshold",
          [this](int value) {
            if ( _opts && _opts ->disparityGradient != value ) {
              _opts ->disparityGradient = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->disparityGradient;
              return true;
            }
            return false;
          });

  // Parameters for LK flow algorithm
  lkTemplateSize_ctl =
      add_numeric_box<int>("lkTemplateSize",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( _opts && _opts ->lkTemplateSize != value ) {
              _opts ->lkTemplateSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->lkTemplateSize;
              return true;
            }
            return false;
          });

  lkPyrLvl_ctl =
      add_numeric_box<int>("lkPyrLvl",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( _opts && _opts ->lkPyrLvl != value ) {
              _opts ->lkPyrLvl = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->lkPyrLvl;
              return true;
            }
            return false;
          });

  lkTermParam1_ctl =
      add_numeric_box<int>("lkTermParam1",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( _opts && _opts ->lkTermParam1 != value ) {
              _opts ->lkTermParam1 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->lkTermParam1;
              return true;
            }
            return false;
          });

  lkTermParam2_ctl =
      add_numeric_box<float>("lkTermParam2",
          "Parameters for LK flow algorithm",
          [this](float value) {
            if ( _opts && _opts ->lkTermParam2 != value ) {
              _opts ->lkTermParam2 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( _opts ) {
              * value = _opts ->lkTermParam2;
              return true;
            }
            return false;
          });

  // Parameters for GFT algorithm.
  gftQualityThres_ctl =
      add_numeric_box<float>("gftQualityThres",
          "Parameters for GFT algorithm",
          [this](float value) {
            if ( _opts && _opts ->gftQualityThres != value ) {
              _opts ->gftQualityThres = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( _opts ) {
              * value = _opts ->gftQualityThres;
              return true;
            }
            return false;
          });

  gftMinSeperationDist_ctl =
      add_numeric_box<int>("gftMinSepDist",
          "Parameters for GFT algorithm",
          [this](int value) {
            if ( _opts && _opts ->gftMinSeperationDist != value ) {
              _opts ->gftMinSeperationDist = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->gftMinSeperationDist;
              return true;
            }
            return false;
          });

  gftMaxNumFeatures_ctl =
      add_numeric_box<int>("gftMaxNumFeatures",
          "Parameters for GFT algorithm",
          [this](int value) {
            if ( _opts && _opts ->gftMaxNumFeatures != value ) {
              _opts ->gftMaxNumFeatures = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts ->gftMaxNumFeatures;
              return true;
            }
            return false;
          });

  updateControls();
}

#endif // HAVE_OpenCV_stereo

