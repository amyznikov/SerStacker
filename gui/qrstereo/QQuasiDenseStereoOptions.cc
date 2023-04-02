/*
 * QQuasiDenseStereoOptions.cc
 *
 *  Created on: Apr 2, 2023
 *      Author: amyznikov
 */

#include "QQuasiDenseStereoOptions.h"

#if HAVE_OpenCV_stereo

QQuasiDenseStereoOptions::QQuasiDenseStereoOptions(QWidget * parent) :
    Base("QuasiDenseStereoOptions", parent)
{
  corrWinSize_ctl =
      add_numeric_box<cv::Size>("corrWinSize",
          "similarity window size",
          [this](const cv::Size & value) {
            if ( options_ && (options_->corrWinSizeX != value.width || options_->corrWinSizeY != value.height) ) {
              options_ ->corrWinSizeX = value.width;
              options_ ->corrWinSizeY = value.height;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( options_ ) {
              * value = cv::Size(options_ ->corrWinSizeX, options_ ->corrWinSizeY);
              return true;
            }
            return false;
          });

  border_ctl =
      add_numeric_box<cv::Size>("border size",
          "border to ignore",
          [this](const cv::Size & value) {
            if ( options_ && (options_ ->borderX != value.width || options_ ->borderY != value.height) ) {
              options_ ->borderX = value.width;
              options_ ->borderY = value.height;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( options_ ) {
              * value = cv::Size(options_ ->borderX, options_ ->borderY);
              return true;
            }
            return false;
          });

  //matching
  correlationThreshold_ctl =
      add_numeric_box<float>("correlation threshold",
          "correlation threshold",
          [this](float value) {
            if ( options_ && options_ ->correlationThreshold != value ) {
              options_ ->correlationThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( options_ ) {
              * value = options_ ->correlationThreshold;
              return true;
            }
            return false;
          });

  // texture threshold
  textrureThreshold_ctl =
      add_numeric_box<float>("texture threshold",
          "texture threshold",
          [this](float value) {
            if ( options_ && options_ ->textrureThreshold != value ) {
              options_ ->textrureThreshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( options_ ) {
              * value = options_ ->textrureThreshold;
              return true;
            }
            return false;
          });

  // neighborhood size
  neighborhoodSize_ctl =
      add_numeric_box<int>("neighborhood size",
          "neighborhood size",
          [this](int value) {
            if ( options_ && options_ ->neighborhoodSize != value ) {
              options_ ->neighborhoodSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->neighborhoodSize;
              return true;
            }
            return false;
          });

  // disparity gradient threshold
  disparityGradient_ctl =
      add_numeric_box<int>("disp. gradient",
          "disparity gradient threshold",
          [this](int value) {
            if ( options_ && options_ ->disparityGradient != value ) {
              options_ ->disparityGradient = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->disparityGradient;
              return true;
            }
            return false;
          });

  // Parameters for LK flow algorithm
  lkTemplateSize_ctl =
      add_numeric_box<int>("lkTemplateSize",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( options_ && options_ ->lkTemplateSize != value ) {
              options_ ->lkTemplateSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->lkTemplateSize;
              return true;
            }
            return false;
          });

  lkPyrLvl_ctl =
      add_numeric_box<int>("lkPyrLvl",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( options_ && options_ ->lkPyrLvl != value ) {
              options_ ->lkPyrLvl = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->lkPyrLvl;
              return true;
            }
            return false;
          });

  lkTermParam1_ctl =
      add_numeric_box<int>("lkTermParam1",
          "Parameters for LK flow algorithm",
          [this](int value) {
            if ( options_ && options_ ->lkTermParam1 != value ) {
              options_ ->lkTermParam1 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->lkTermParam1;
              return true;
            }
            return false;
          });

  lkTermParam2_ctl =
      add_numeric_box<float>("lkTermParam2",
          "Parameters for LK flow algorithm",
          [this](float value) {
            if ( options_ && options_ ->lkTermParam2 != value ) {
              options_ ->lkTermParam2 = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( options_ ) {
              * value = options_ ->lkTermParam2;
              return true;
            }
            return false;
          });

  // Parameters for GFT algorithm.
  gftQualityThres_ctl =
      add_numeric_box<float>("gftQualityThres",
          "Parameters for GFT algorithm",
          [this](float value) {
            if ( options_ && options_ ->gftQualityThres != value ) {
              options_ ->gftQualityThres = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](float * value) {
            if ( options_ ) {
              * value = options_ ->gftQualityThres;
              return true;
            }
            return false;
          });

  gftMinSeperationDist_ctl =
      add_numeric_box<int>("gftMinSepDist",
          "Parameters for GFT algorithm",
          [this](int value) {
            if ( options_ && options_ ->gftMinSeperationDist != value ) {
              options_ ->gftMinSeperationDist = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->gftMinSeperationDist;
              return true;
            }
            return false;
          });

  gftMaxNumFeatures_ctl =
      add_numeric_box<int>("gftMaxNumFeatures",
          "Parameters for GFT algorithm",
          [this](int value) {
            if ( options_ && options_ ->gftMaxNumFeatures != value ) {
              options_ ->gftMaxNumFeatures = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_ ->gftMaxNumFeatures;
              return true;
            }
            return false;
          });

  updateControls();
}

void QQuasiDenseStereoOptions::set_options(cv::stereo::PropagationParameters * options)
{
  options_ = options;
  updateControls();
}

cv::stereo::PropagationParameters* QQuasiDenseStereoOptions::options() const
{
  return options_;
}

void QQuasiDenseStereoOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

#endif // HAVE_OpenCV_stereo

