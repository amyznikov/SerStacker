/*
 * QChessboardCornersDetectionOptions.cc
 *
 *  Created on: Feb 27, 2023
 *      Author: amyznikov
 */

#include "QChessboardCornersDetectionOptions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

QFindChessboardCornersOptions::QFindChessboardCornersOptions(QWidget * parent) :
    ThisClass("QFindChessboardCornersOptions", parent)
{
}

QFindChessboardCornersOptions::QFindChessboardCornersOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  max_scales_ctl =
      add_numeric_box<int>("max_scales:",
          [this](int value) {
            if ( options_ ) {
              options_->max_scales = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              *value = options_->max_scales;
            }
            return options_ != nullptr;
          });


  flags_ctl =
      add_flags_editbox<FindChessboardCornersFlags>("flags:",
          [this](int value) {
            if ( options_ ) {
              options_->flags = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              *value = options_->flags;
            }
            return options_ != nullptr;
          });

}

void QFindChessboardCornersOptions::set_options(c_findChessboardCorners_options * options)
{
  options_ = options;
  updateControls();
}

QFindChessboardCornersOptions::c_findChessboardCorners_options* QFindChessboardCornersOptions::options() const
{
  return options_;
}

void QFindChessboardCornersOptions::onupdatecontrols()
{
  setEnabled(options_ != nullptr);
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QFindChessboardCornersSBOptions::QFindChessboardCornersSBOptions(QWidget * parent) :
    ThisClass("QFindChessboardCornersSBOptions", parent)
{
}

QFindChessboardCornersSBOptions::QFindChessboardCornersSBOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  max_scales_ctl =
      add_numeric_box<int>("max_scales:",
          [this](int value) {
            if ( options_ ) {
              options_->max_scales = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              *value = options_->max_scales;
            }
            return options_ != nullptr;
          });

  flags_ctl =
      add_flags_editbox<FindChessboardCornersSBFlags>("flags:",
          [this](int value) {
            if ( options_ ) {
              options_->flags = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              *value = options_->flags;
            }
            return options_ != nullptr;
          });
}

void QFindChessboardCornersSBOptions::set_options(c_findChessboardCornersSB_options * options)
{
  options_ = options;
  updateControls();
}

QFindChessboardCornersSBOptions::c_findChessboardCornersSB_options* QFindChessboardCornersSBOptions::options() const
{
  return options_;
}

void QFindChessboardCornersSBOptions::onupdatecontrols()
{
  setEnabled(options_ != nullptr);
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QCornerSubPixOptions::QCornerSubPixOptions(QWidget * parent) :
    ThisClass("QCornerSubPixOptions", parent)
{
}

QCornerSubPixOptions::QCornerSubPixOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{

  winSize_ctl =
      add_numeric_box<cv::Size>("winSize",
          [this](const cv::Size & value) {
            if ( options_ ) {
              options_->winSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( options_ ) {
              * value = options_->winSize;
              return true;
            }
            return false;
          });

  zeroZone_ctl =
      add_numeric_box<cv::Size>("zeroZone",
          [this](const cv::Size & value) {
            if ( options_ ) {
              options_->zeroZone = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( options_ ) {
              * value = options_->zeroZone;
              return true;
            }
            return false;
          });


  maxIterations_ctl =
      add_numeric_box<int>("MaxIterations:",
          [this](int value) {
            if ( options_ ) {
              if ( ( options_->termCriteria.maxCount = value) > 0 ) {
                options_->termCriteria.type |= cv::TermCriteria::COUNT;
              }
              else {
                options_->termCriteria.type &= ~cv::TermCriteria::COUNT;
              }
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->termCriteria.maxCount;
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("Eps:",
          [this](double value) {
            if ( options_ ) {
              if ( ( options_->termCriteria.epsilon = value) >= 0 ) {
                options_->termCriteria.type |= cv::TermCriteria::EPS;
              }
              else {
                options_->termCriteria.type &= ~cv::TermCriteria::EPS;
              }
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( options_ ) {
              * value = options_->termCriteria.epsilon;
              return true;
            }
            return false;
          });

}

void QCornerSubPixOptions::set_options(c_cornerSubPix_options * options)
{
  options_ = options;
  updateControls();
}

QCornerSubPixOptions::c_cornerSubPix_options* QCornerSubPixOptions::options() const
{
  return options_;
}

void QCornerSubPixOptions::onupdatecontrols()
{
  setEnabled(options_ != nullptr);
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QBilateralFilterOptions::QBilateralFilterOptions(QWidget * parent) :
    ThisClass("QBilateralFilterOptions", parent)
{
}

QBilateralFilterOptions::QBilateralFilterOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  d_ctl =
      add_numeric_box<int>("d:",
          [this](int value) {
            if ( options_ ) {
              options_->d = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->d;
              return true;
            }
            return false;
          });

  sigmaColor_ctl =
      add_numeric_box<double>("SigmaColor:",
          [this](double value) {
            if ( options_ ) {
              options_->sigmaColor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( options_ ) {
              * value = options_->sigmaColor;
              return true;
            }
            return false;
          });

  sigmaSpace_ctl =
      add_numeric_box<double>("SigmaSpace:",
          [this](double value) {
            if ( options_ ) {
              options_->sigmaSpace = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( options_ ) {
              * value = options_->sigmaSpace;
              return true;
            }
            return false;
          });

}

void QBilateralFilterOptions::set_options(c_bilateralFilter_options * options)
{
  options_ = options;
  updateControls();
}

QBilateralFilterOptions::c_bilateralFilter_options* QBilateralFilterOptions::options() const
{
  return options_;
}

void QBilateralFilterOptions::onupdatecontrols()
{
  setEnabled(options_ != nullptr);
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QChessboardCornersDetectionOptions::QChessboardCornersDetectionOptions(QWidget * parent) :
    ThisClass("QChessboardCornersDetectionOptions", parent)
{

  chessboardSize_ctl =
      add_numeric_box<cv::Size>("Chessboard Size:",
          [this](const cv::Size & size) {
            if ( options_ ) {
              options_->chessboard_size = size;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * size) {
            if ( options_ ) {
              *size = options_->chessboard_size;
              return true;
            }
            return false;
          });

  add_numeric_box<cv::Size2f>("Chessboard Cell Size [m]:",
      [this](const cv::Size2f & size) {
        if ( options_ ) {
          options_->chessboard_cell_size = size;
          Q_EMIT parameterChanged();
        }
      },
      [this](cv::Size2f * size) {
        if ( options_ ) {
          *size = options_->chessboard_cell_size;
          return true;
        }
        return false;
      });

  cornersDetectionMethod_ctl =
      add_enum_combobox<FindChessboardCornersMethod>("Chessboard Detection:",
          [this](FindChessboardCornersMethod value) {
            if ( options_ ) {
              options_->method = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](FindChessboardCornersMethod * value) {
            if ( options_ ) {
              *value = options_->method;
            }
            return options_ != nullptr;
          });


  add_expandable_groupbox("findChessboardCorners options",
      findChessboardCornersOptions_ctl = new QFindChessboardCornersOptions(this));
  connect(findChessboardCornersOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  add_expandable_groupbox("findChessboardCornersSB options",
      findChessboardCornersSBOptions_ctl = new QFindChessboardCornersSBOptions(this));
  connect(findChessboardCornersSBOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("CornerSubPix options",
      cornerSubPixOptions_ctl = new QCornerSubPixOptions(this));
  connect(cornerSubPixOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Bilateral Filter options",
      bilateralFilterOptions_ctl = new QBilateralFilterOptions(this));
  connect(bilateralFilterOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

}

QChessboardCornersDetectionOptions::QChessboardCornersDetectionOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
}

void QChessboardCornersDetectionOptions::set_chessboard_corners_detection_options(
    c_chessboard_corners_detection_options * options)
{
  options_ = options;
  updateControls();
}

c_chessboard_corners_detection_options* QChessboardCornersDetectionOptions::chessboard_corners_detection_options() const
{
  return options_;
}

void QChessboardCornersDetectionOptions::onupdatecontrols()
{
  if ( !options_ ) {

    findChessboardCornersOptions_ctl->set_options(nullptr);
    findChessboardCornersSBOptions_ctl->set_options(nullptr);
    cornerSubPixOptions_ctl->set_options(nullptr);
    bilateralFilterOptions_ctl->set_options(nullptr);

    setEnabled(false);
  }
  else {

    findChessboardCornersOptions_ctl->set_options(&options_->findChessboardCorners);
    findChessboardCornersSBOptions_ctl->set_options(&options_->findChessboardCornersSB);
    cornerSubPixOptions_ctl->set_options(&options_->cornerSubPix);
    bilateralFilterOptions_ctl->set_options(&options_->bilateralFilter);

    Base::onupdatecontrols();

    setEnabled(true);
  }

}
