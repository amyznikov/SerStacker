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
    public QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_findChessboardCorners_options>
{
public:
  typedef QFindChessboardCornersOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_findChessboardCorners_options> Base;

  using c_findChessboardCorners_options =
      c_chessboard_corners_detection_options::c_findChessboardCorners_options;

  QFindChessboardCornersOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * max_scales_ctl = nullptr;
  QFlagsEditBox<FindChessboardCornersFlags> * flags_ctl = nullptr;
};

class QFindChessboardCornersSBOptions :
    public QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_findChessboardCornersSB_options>
{
public:
  typedef QFindChessboardCornersSBOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_findChessboardCornersSB_options> Base;

  using c_findChessboardCornersSB_options =
      c_chessboard_corners_detection_options::c_findChessboardCornersSB_options;

  QFindChessboardCornersSBOptions(QWidget * parent = nullptr);
protected:
  QNumericBox * max_scales_ctl = nullptr;
  QFlagsEditBox<FindChessboardCornersSBFlags> * flags_ctl = nullptr;
};

class QCornerSubPixOptions :
    public QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_cornerSubPix_options>
{
public:
  typedef QCornerSubPixOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_cornerSubPix_options> Base;

  using c_cornerSubPix_options =
      c_chessboard_corners_detection_options::c_cornerSubPix_options;

  QCornerSubPixOptions(QWidget * parent = nullptr);
protected:
  QNumericBox * winSize_ctl = nullptr;
  QNumericBox * zeroZone_ctl = nullptr;
  QNumericBox * maxIterations_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
};

class QBilateralFilterOptions :
    public QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_bilateralFilter_options>
{
public:
  typedef QBilateralFilterOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_chessboard_corners_detection_options::c_bilateralFilter_options> Base;

  using c_bilateralFilter_options =
      c_chessboard_corners_detection_options::c_bilateralFilter_options;

  QBilateralFilterOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * d_ctl = nullptr;
  QNumericBox * sigmaColor_ctl = nullptr;
  QNumericBox * sigmaSpace_ctl = nullptr;
};

class QChessboardCornersDetectionOptions :
    public QSettingsWidgetTemplate<c_chessboard_corners_detection_options>
{
public:
  typedef QChessboardCornersDetectionOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_chessboard_corners_detection_options> Base;

  QChessboardCornersDetectionOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * chessboardSize_ctl = nullptr;
  QNumericBox * chessboardCellSize_ctl = nullptr;
  QEnumComboBox<FindChessboardCornersMethod> * cornersDetectionMethod_ctl = nullptr;
  QFindChessboardCornersOptions * findChessboardCornersOptions_ctl = nullptr;
  QFindChessboardCornersSBOptions * findChessboardCornersSBOptions_ctl = nullptr;
  QCornerSubPixOptions * cornerSubPixOptions_ctl = nullptr;
  QBilateralFilterOptions * bilateralFilterOptions_ctl = nullptr;
};

#endif /* __QChessboardCornersDetectionOptions_h___ */
