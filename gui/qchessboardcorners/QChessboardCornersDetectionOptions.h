/*
 * QChessboardCornersDetectionOptions.h
 *
 *  Created on: Feb 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QChessboardCornersDetectionOptions_h___
#define __QChessboardCornersDetectionOptions_h___

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/chessboard/chessboard_detection.h>

class QFindChessboardCornersOptions :
    public QSettingsWidget
{
public:
  typedef QFindChessboardCornersOptions ThisClass;
  typedef QSettingsWidget Base;

  using c_findChessboardCorners_options =
      c_chessboard_corners_detection_options::c_findChessboardCorners_options;

  QFindChessboardCornersOptions(QWidget * parent = nullptr);
  QFindChessboardCornersOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_options(c_findChessboardCorners_options * options);
  c_findChessboardCorners_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_findChessboardCorners_options * options_ = nullptr;

  QNumberEditBox * max_scales_ctl = nullptr;
  QFlagsEditBox<FindChessboardCornersFlags> * flags_ctl = nullptr;
};

class QFindChessboardCornersSBOptions :
    public QSettingsWidget
{
public:
  typedef QFindChessboardCornersSBOptions ThisClass;
  typedef QSettingsWidget Base;

  using c_findChessboardCornersSB_options =
      c_chessboard_corners_detection_options::c_findChessboardCornersSB_options;

  QFindChessboardCornersSBOptions(QWidget * parent = nullptr);
  QFindChessboardCornersSBOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_options(c_findChessboardCornersSB_options * options);
  c_findChessboardCornersSB_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_findChessboardCornersSB_options * options_ = nullptr;

  QNumberEditBox * max_scales_ctl = nullptr;
  QFlagsEditBox<FindChessboardCornersSBFlags> * flags_ctl = nullptr;
};

class QCornerSubPixOptions :
    public QSettingsWidget
{
public:
  typedef QCornerSubPixOptions ThisClass;
  typedef QSettingsWidget Base;

  using c_cornerSubPix_options =
      c_chessboard_corners_detection_options::c_cornerSubPix_options;

  QCornerSubPixOptions(QWidget * parent = nullptr);
  QCornerSubPixOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_options(c_cornerSubPix_options * options);
  c_cornerSubPix_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_cornerSubPix_options * options_ = nullptr;

  QNumberEditBox * winSize_ctl = nullptr;
  QNumberEditBox * zeroZone_ctl = nullptr;
  QNumberEditBox * maxIterations_ctl = nullptr;
  QNumberEditBox * eps_ctl = nullptr;
};

class QBilateralFilterOptions :
    public QSettingsWidget
{
public:
  typedef QBilateralFilterOptions ThisClass;
  typedef QSettingsWidget Base;

  using c_bilateralFilter_options =
      c_chessboard_corners_detection_options::c_bilateralFilter_options;

  QBilateralFilterOptions(QWidget * parent = nullptr);
  QBilateralFilterOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_options(c_bilateralFilter_options * options);
  c_bilateralFilter_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_bilateralFilter_options * options_ = nullptr;

  QNumberEditBox * d_ctl = nullptr;
  QNumberEditBox * sigmaColor_ctl = nullptr;
  QNumberEditBox * sigmaSpace_ctl = nullptr;
};

class QChessboardCornersDetectionOptions :
    public QSettingsWidget
{
public:
  typedef QChessboardCornersDetectionOptions ThisClass;
  typedef QSettingsWidget Base;

  QChessboardCornersDetectionOptions(QWidget * parent = nullptr);
  QChessboardCornersDetectionOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_chessboard_corners_detection_options(c_chessboard_corners_detection_options * options);
  c_chessboard_corners_detection_options * chessboard_corners_detection_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_chessboard_corners_detection_options * options_ = nullptr;

  QNumberEditBox * chessboardSize_ctl = nullptr;
  QNumberEditBox * chessboardCellSize_ctl = nullptr;

  QEnumComboBox<FindChessboardCornersMethod> * cornersDetectionMethod_ctl = nullptr;
  QFindChessboardCornersOptions * findChessboardCornersOptions_ctl = nullptr;
  QFindChessboardCornersSBOptions * findChessboardCornersSBOptions_ctl = nullptr;
  QCornerSubPixOptions * cornerSubPixOptions_ctl = nullptr;
  QBilateralFilterOptions * bilateralFilterOptions_ctl = nullptr;
};

#endif /* __QChessboardCornersDetectionOptions_h___ */
