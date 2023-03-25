/*
 * chessboard_detection.h
 *
 *  Created on: Jan 20, 2021
 *      Author: amyznikov
 *
 * A few routines for chessboard corners detection on OpenCV images in SerStacker
 */

#pragma once
#ifndef __chessboard_detection_h__
#define __chessboard_detection_h__

#include <opencv2/opencv.hpp>
#include <core/settings.h>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif


// Which method to use for findChessboardCorners
enum FindChessboardCornersMethod {
  cv_findChessboardCorners = 0, // call cv::findChessboardCorners()
  cv_findChessboardCornersSB = 1, // call cv::findChessboardCornersSB()
};

enum FindChessboardCornersFlags
{
  FindChessboardCorners_ADAPTIVE_THRESH = cv::CALIB_CB_ADAPTIVE_THRESH,
  FindChessboardCorners_NORMALIZE_IMAGE = cv::CALIB_CB_NORMALIZE_IMAGE,
  FindChessboardCorners_FILTER_QUADS = cv::CALIB_CB_FILTER_QUADS,
  FindChessboardCorners_FAST_CHECK = cv::CALIB_CB_FAST_CHECK,
  FindChessboardCorners_EXHAUSTIVE = cv::CALIB_CB_EXHAUSTIVE,
  FindChessboardCorners_ACCURACY = cv::CALIB_CB_ACCURACY,
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 3, 0)
  FindChessboardCorners_LARGER = cv::CALIB_CB_LARGER,
  FindChessboardCorners_MARKER = cv::CALIB_CB_MARKER,
#endif
};

enum FindChessboardCornersSBFlags
{
  FindChessboardCornersSB_EXHAUSTIVE = cv::CALIB_CB_EXHAUSTIVE,
  FindChessboardCornersSB_NORMALIZE_IMAGE = cv::CALIB_CB_NORMALIZE_IMAGE,
};


/**
 * Options for find_chessboard_corners()
 */
struct c_chessboard_corners_detection_options
{
  cv::Size chessboard_size = cv::Size(9, 6);
  cv::Size2f chessboard_cell_size = cv::Size2f(0.09, 0.09);

  // Which method to use for findChessboardCorners
  enum FindChessboardCornersMethod method =
      cv_findChessboardCorners;

  // Parameters passed into cv::findChessboardCorners()
  // For details see OpenCV documentation for cv::findChessboardCorners()
  struct c_findChessboardCorners_options
  {
    int max_scales = 2;
    int flags = 0;
  } findChessboardCorners;

  // Parameters passed into cv::findChessboardCornersSB()
  // For details see OpenCV documentation for cv::findChessboardCornersSB()
  struct c_findChessboardCornersSB_options
  {
    int max_scales = 2;
    int flags = 0;
  } findChessboardCornersSB;


  // Parameters passed into cv::cornerSubPix().
  // For details see OpenCV documentation for cv::cornerSubPix() function.
  struct c_cornerSubPix_options
  {

    cv::Size winSize =
        cv::Size(11, 11);

    cv::Size zeroZone =
        cv::Size (-1, -1);

    cv::TermCriteria termCriteria =
        cv::TermCriteria(cv::TermCriteria::COUNT |
            cv::TermCriteria::EPS,
            30,
            0.01);

  } cornerSubPix;

  struct c_bilateralFilter_options
  {
    int d = 3;
    double sigmaColor = 25;
    double sigmaSpace = 1;
  } bilateralFilter;

 };


/**
 * find_chessboard_corners()
 *
 * Detect chessboard pattern of given size on given image,
 * finds the positions of internal corners of the chessboard.
 * */
bool find_chessboard_corners(cv::InputArray src_image, const cv::Size & boardSize,
    /*out*/ std::vector<cv::Point2f> & output_corners,
    const c_chessboard_corners_detection_options & options =
        c_chessboard_corners_detection_options());


/**
 * estimate_chessboard_size()
 *
 * Utility routine: try to detect chessboard internal corners on given
 * chessboard pattern image and estimate chessboard size
 */
cv::Size estimate_chessboard_size(cv::InputArray src_image,
    // optional output images for debug
    /*out, opt */ cv::Mat * output_otsu_image = nullptr,
    /*out, opt */ cv::Mat * output_harris_image = nullptr,
    /*out, opt */ cv::Mat * output_harris_threshold_image = nullptr,
    /*out, opt */ cv::Mat * output_hharris_image = nullptr,
    /*out, opt */ cv::Mat * output_vharris_image = nullptr);


bool save_settings(c_config_setting settings, const c_chessboard_corners_detection_options & options);
bool load_settings(c_config_setting settings, c_chessboard_corners_detection_options * options);

#endif /* __chessboard_detection_h__ */
