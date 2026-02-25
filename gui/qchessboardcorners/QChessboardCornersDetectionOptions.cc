/*
 * QChessboardCornersDetectionOptions.cc
 *
 *  Created on: Feb 27, 2023
 *      Author: amyznikov
 */

#include "QChessboardCornersDetectionOptions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

QFindChessboardCornersOptions::QFindChessboardCornersOptions(QWidget * parent) :
  Base(parent)
{
  max_scales_ctl =
      add_numeric_box<int>("max_scales:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->max_scales = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              *value = _opts->max_scales;
            }
            return _opts != nullptr;
          });


  flags_ctl =
      add_flags_editbox<FindChessboardCornersFlags>("flags:",
          "flags for cv::findChessboardCorners()",
          [this](int value) {
            if ( _opts ) {
              _opts->flags = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              *value = _opts->flags;
            }
            return _opts != nullptr;
          });

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QFindChessboardCornersSBOptions::QFindChessboardCornersSBOptions(QWidget * parent) :
    Base(parent)
{
  max_scales_ctl =
      add_numeric_box<int>("max_scales:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->max_scales = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              *value = _opts->max_scales;
            }
            return _opts != nullptr;
          });

  flags_ctl =
      add_flags_editbox<FindChessboardCornersSBFlags>("flags:",
          "flags for cv::findChessboardCornersSB()",
          [this](int value) {
            if ( _opts ) {
              _opts->flags = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              *value = _opts->flags;
            }
            return _opts != nullptr;
          });
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QCornerSubPixOptions::QCornerSubPixOptions(QWidget * parent) :
    Base(parent)
{

  winSize_ctl =
      add_numeric_box<cv::Size>("winSize",
          "",
          [this](const cv::Size & value) {
            if ( _opts ) {
              _opts->winSize = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( _opts ) {
              * value = _opts->winSize;
              return true;
            }
            return false;
          });

  zeroZone_ctl =
      add_numeric_box<cv::Size>("zeroZone",
          "",
          [this](const cv::Size & value) {
            if ( _opts ) {
              _opts->zeroZone = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * value) {
            if ( _opts ) {
              * value = _opts->zeroZone;
              return true;
            }
            return false;
          });


  maxIterations_ctl =
      add_numeric_box<int>("MaxIterations:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->max_solver_iterations = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_solver_iterations;
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("Eps:",
          "",
         [this](double value) {
            if ( _opts ) {
              _opts->solver_eps = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->solver_eps;
              return true;
            }
            return false;
          });

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QBilateralFilterOptions::QBilateralFilterOptions(QWidget * parent) :
    Base(parent)
{
  d_ctl =
      add_numeric_box<int>("d:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->d = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->d;
              return true;
            }
            return false;
          });

  sigmaColor_ctl =
      add_numeric_box<double>("SigmaColor:",
          "",
          [this](double value) {
            if ( _opts ) {
              _opts->sigmaColor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->sigmaColor;
              return true;
            }
            return false;
          });

  sigmaSpace_ctl =
      add_numeric_box<double>("SigmaSpace:",
          "",
          [this](double value) {
            if ( _opts ) {
              _opts->sigmaSpace = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->sigmaSpace;
              return true;
            }
            return false;
          });

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QChessboardCornersDetectionOptions::QChessboardCornersDetectionOptions(QWidget * parent) :
    Base(parent)
{
  chessboardSize_ctl =
      add_numeric_box<cv::Size>("Chessboard Size:",
          "",
          [this](const cv::Size & size) {
            if ( _opts ) {
              _opts->chessboard_size = size;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * size) {
            if ( _opts ) {
              *size = _opts->chessboard_size;
              return true;
            }
            return false;
          });

  chessboardCellSize_ctl =
      add_numeric_box<cv::Size2f>("Chessboard Cell Size [m]:",
          "",
          [this](const cv::Size2f & size) {
            if ( _opts ) {
              _opts->chessboard_cell_size = size;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size2f * size) {
            if ( _opts ) {
              *size = _opts->chessboard_cell_size;
              return true;
            }
            return false;
          });

  cornersDetectionMethod_ctl =
      add_enum_combobox<FindChessboardCornersMethod>(
          "Chessboard Detection:",
          "",
          [this](FindChessboardCornersMethod value) {
            if ( _opts ) {
              _opts->method = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](FindChessboardCornersMethod * value) {
            if ( _opts ) {
              *value = _opts->method;
            }
            return _opts != nullptr;
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

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        findChessboardCornersOptions_ctl->setOpts(_opts?  &_opts->findChessboardCorners : nullptr);
        findChessboardCornersSBOptions_ctl->setOpts(_opts?  &_opts->findChessboardCornersSB : nullptr);
        cornerSubPixOptions_ctl->setOpts(_opts?  &_opts->cornerSubPix : nullptr);
        bilateralFilterOptions_ctl->setOpts(_opts?  &_opts->bilateralFilter : nullptr);
    });

}

///////////////////////////////////////////////////////////////////////////////////////////////////
