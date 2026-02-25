/*
 * chessboard_detection.cc
 *
 *  Created on: Jan 20, 2021
 *      Author: amyznikov
 *
 * A few routines for chessboard corners detection on OpenCV images in SerStacker
 */

#include "chessboard_detection.h"
#include <core/settings/opencv_settings.h>
#include <core/ssprintf.h>
#include <core/debug.h>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

template<>
const c_enum_member * members_of<FindChessboardCornersMethod>()
{
  static const c_enum_member members[] = {
      {cv_findChessboardCorners, "findChessboardCorners", "Calls cv::findChessboardCorners()"},
      {cv_findChessboardCornersSB, "findChessboardCornersSB", "Calls cv::findChessboardCornersSB()"},
      {cv_findChessboardCorners},
  };

  return members;
}

template<>
const c_enum_member* members_of<FindChessboardCornersFlags>()
{
  static const c_enum_member members[] = {
      { FindChessboardCorners_ADAPTIVE_THRESH, "ADAPTIVE_THRESH", "cv::CALIB_CB_ADAPTIVE_THRESH" },
      { FindChessboardCorners_NORMALIZE_IMAGE, "NORMALIZE_IMAGE", "cv::CALIB_CB_NORMALIZE_IMAGE" },
      { FindChessboardCorners_FILTER_QUADS, "FILTER_QUADS", "cv::CALIB_CB_FILTER_QUADS" },
      { FindChessboardCorners_FAST_CHECK, "FAST_CHECK", "cv::CALIB_CB_FAST_CHECK" },
      { FindChessboardCorners_EXHAUSTIVE, "EXHAUSTIVE", "cv::CALIB_CB_EXHAUSTIVE" },
      { FindChessboardCorners_ACCURACY, "ACCURACY", "cv::CALIB_CB_ACCURACY" },
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 3, 0)
      { FindChessboardCorners_LARGER, "LARGER", "cv::CALIB_CB_LARGER" },
      { FindChessboardCorners_MARKER, "MARKER", "cv::CALIB_CB_MARKER" },
#endif
      { 0 }
  };

  return members;
}

template<>
const c_enum_member* members_of<FindChessboardCornersSBFlags>()
{
  static const c_enum_member members[] = {
      { FindChessboardCornersSB_EXHAUSTIVE, "EXHAUSTIVE", "cv::CALIB_CB_EXHAUSTIVE" },
      { FindChessboardCornersSB_NORMALIZE_IMAGE, "NORMALIZE_IMAGE", "cv::CALIB_CB_NORMALIZE_IMAGE" },
      { 0 }
  };

  return members;
}

/**
 * find_chessboard_corners()
 *
 * Detect chessboard pattern of given size on given image,
 * finds the positions of internal corners of the chessboard.
 * */
bool find_chessboard_corners(cv::InputArray src_image, const cv::Size & boardSize,
    /*out*/std::vector<cv::Point2f> & corners,
    const c_chessboard_corners_detection_options & options)
{
  INSTRUMENT_REGION("");

  if ( src_image.empty() ) {
    CF_ERROR("Empty imput image specified");
    return false;
  }

  if ( boardSize.empty() ) {
    CF_ERROR("Empty chessboard pattern size specified");
    return false;
  }

  const int maxScale = std::max(1,
      options.method == cv_findChessboardCorners ?
          options.findChessboardCorners.max_scales :
          options.findChessboardCornersSB.max_scales);

  cv::Mat gray;

  bool found = false;

  if ( src_image.channels() == 1 ) {
    gray = src_image.getMat();
  }
  else {
    cv::cvtColor(src_image, gray,
        cv::COLOR_BGR2GRAY);
  }

  if ( options.bilateralFilter.d > 0 || options.bilateralFilter.sigmaSpace > 0 ) {
    cv::Mat tmp;
    cv::bilateralFilter(gray, tmp, options.bilateralFilter.d,
        options.bilateralFilter.sigmaColor,
        options.bilateralFilter.sigmaSpace);
    gray = std::move(tmp);
  }

  for ( int scale = 1; scale <= maxScale; ++scale ) {

    cv::Mat timg;

    if ( scale != 1 ) {
      timg = gray;
    }
    else {
      cv::resize(gray, timg,
          cv::Size(),
          scale, scale,
          cv::INTER_LINEAR_EXACT);
    }

    if ( options.method == cv_findChessboardCorners ) {

      INSTRUMENT_REGION("cv::findChessboardCorners()");

      found = cv::findChessboardCorners(timg, boardSize, corners,
          options.findChessboardCorners.flags);
    }
    else {

      INSTRUMENT_REGION("cv::findChessboardCornersSB()");

      found = cv::findChessboardCornersSB(timg, boardSize, corners,
          options.findChessboardCornersSB.flags);
    }

    if ( found ) {
      if ( scale > 1 ) {
        cv::Mat(corners) *= 1. / scale;
      }
      break;
    }
  }

  if ( found && !options.cornerSubPix.winSize.empty() ) {

    INSTRUMENT_REGION("cv::cornerSubPix()");

    cv::cornerSubPix(gray,
        corners,
        options.cornerSubPix.winSize,
        options.cornerSubPix.zeroZone,
        cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            options.cornerSubPix.max_solver_iterations,
            options.cornerSubPix.solver_eps
        ));
  }

  return found;
}



/**
 * estimate_chessboard_size()
 *
 * Try to detect chessboard internal corners on given
 * chessboard pattern image and estimate chessboard size
 */
cv::Size estimate_chessboard_size(cv::InputArray src_image,
    /*out, opt */ cv::Mat * output_otsu_image,
    /*out, opt */ cv::Mat * output_harris_image,
    /*out, opt */ cv::Mat * output_harris_threshold_image,
    /*out, opt */ cv::Mat * output_hharris_image,
    /*out, opt */ cv::Mat * output_vharris_image)
{
  cv::Mat gray;
  cv::Mat harris;
  cv::Mat1b hharris;
  cv::Mat1b vharris;
  cv::Size pattern_size;
  int max_frequency;

  const cv::Mat src =
      src_image.getMat();

  // CF_DEBUG("src_image.depth=%d", src_image.depth());

  // convert to CV_8U
  switch ( src.depth() ) {
  case CV_8S :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / INT8_MAX);
    break;
  case CV_8U :
    src.copyTo(gray);
    break;
  case CV_16S :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / INT16_MAX);
    break;
  case CV_16U :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / UINT16_MAX);
    break;
  case CV_32S :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / INT32_MAX);
    break;
  case CV_32F :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / 1.f);
    break;
  case CV_64F :
    src.convertTo(gray, CV_8U, ((double) UINT8_MAX) / 1.0);
    break;
  default :
    CF_ERROR("Not supported input image depth: %d",
        src.depth());

    return cv::Size();
  }

  // convert to grayscale
  if ( gray.channels() != 1 ) {

    cv::cvtColor(gray, gray,
        cv::COLOR_BGR2GRAY);
  }

  // threshold B/W objects (assuming chessboard boxes)
  cv::threshold(gray,
      gray,
      0, 255,
      cv::THRESH_OTSU);

  if ( output_otsu_image ) {
    *output_otsu_image = gray;
  }


  // Apply filter2D to match corner shapes

  static float K1 [10 * 10] = {
      -1, -1, -1, -1, -1, +1, +1, +1, +1, +1,
      -1, -1, -1, -1, -1, +1, +1, +1, +1, +1,
      -1, -1, -1, -1, -1, +1, +1, +1, +1, +1,
      -1, -1, -1, -1, -1, +1, +1, +1, +1, +1,
      -1, -1, -1, -1, -1, +1, +1, +1, +1, +1,
      +1, +1, +1, +1, +1, -1, -1, -1, -1, -1
      +1, +1, +1, +1, +1, -1, -1, -1, -1, -1,
      +1, +1, +1, +1, +1, -1, -1, -1, -1, -1,
      +1, +1, +1, +1, +1, -1, -1, -1, -1, -1,
      +1, +1, +1, +1, +1, -1, -1, -1, -1, -1,
  };

  static const cv::Mat1f K1F =
      cv::Mat1f (10, 10, K1) / (10 * 10 / 2);

  cv::filter2D(gray,
      harris,
      CV_32F,
      K1F,
      cv::Point(K1F.cols / 2, K1F.rows / 2),
      0,
      cv::BORDER_REPLICATE);

  cv::absdiff(harris,
      0,
      harris);

  if( output_harris_image ) {
    harris.copyTo(*output_harris_image);
  }


  // Threshold again to drop non-internal corners
  cv::compare(harris, 120, harris,
        cv::CMP_GT);

  if ( output_harris_threshold_image ) {
    *output_harris_threshold_image = harris;
  }


  // Apply Horizontal and Vertical deriatives, in order to select only corner edge pixels

  static float K2[2] = {
      -1, 1
  };

  static const cv::Mat1f KXF =
      cv::Mat1f (1, 2, K2);

  static const cv::Mat1f KYF =
      cv::Mat1f (2, 1, K2);

  static const cv::Mat1f I =
      cv::Mat1f (1, 1, 1.f);


  cv::sepFilter2D(harris,
      hharris,
      -1,
      KXF,
      I,
      cv::Point(1, 0),
      0,
      cv::BORDER_REPLICATE);

  cv::sepFilter2D(harris,
      vharris,
      -1,
      I,
      KYF,
      cv::Point(0,1),
      0,
      cv::BORDER_REPLICATE);

  if ( output_hharris_image ) {
    hharris.copyTo(*output_hharris_image);
  }

  if ( output_vharris_image ) {
    vharris.copyTo(*output_vharris_image);
  }


  // Scan vertical and horizontal derivatives and count the number of
  // non-zero corner edge pixels (count of transitions from black to white)
  // For better robustness to image noise compute the histogram of counts
  // and select maximally frequent counts.


  cv::rotate(vharris,
      vharris,
      cv::ROTATE_90_CLOCKWISE);


  typedef std::map<int /*count*/, int /*frequency*/>
    transition_map;

  transition_map hmap, vmap;

  static const auto count_transitions =
      [](const cv::Mat1b & image, transition_map & map) {

    for ( int y = 0; y < image.rows; ++y ) {

      const uint8_t * srcp = image[y];
      int c = 0;

      for ( int x = 0; x < image.cols; ++x ) {
        if ( srcp[x] ) {
          ++c;
        }
      }

      if ( c > 0 ) {
        ++map[c];
      }
    }
  };


  count_transitions(hharris, hmap);
  count_transitions(vharris, vmap);

  max_frequency = 0;
  for ( const auto & m : hmap ) {
    const int c = m.first;
    const int f = m.second;

    if ( f > max_frequency ) {
      max_frequency = f;
      pattern_size.width = c;
    }

    // CF_DEBUG("hmap: [%d] %d", c, f);
  }

  max_frequency = 0;
  for ( const auto & m : vmap ) {
    const int c = m.first;
    const int f = m.second;

    if ( f > max_frequency ) {
      max_frequency = f;
      pattern_size.height = c;
    }

    // CF_DEBUG("vmap: [%d] %d", c, f);
  }

  return pattern_size;
}



bool save_settings(c_config_setting settings, const c_chessboard_corners_detection_options & options)
{
  c_config_setting section;

  SAVE_OPTION(settings, options, chessboard_size);
  SAVE_OPTION(settings, options, chessboard_cell_size);
  SAVE_OPTION(settings, options, chessboard_distance);

  SAVE_OPTION(settings, options, method);

  if( (section = settings.add_group("findChessboardCorners")) ) {
    SAVE_OPTION(section, options.findChessboardCorners, max_scales);
    SAVE_OPTION(section, options.findChessboardCorners, flags);
  }

  if( (section = settings.add_group("findChessboardCornersSB")) ) {
    SAVE_OPTION(section, options.findChessboardCornersSB, max_scales);
    SAVE_OPTION(section, options.findChessboardCornersSB, flags);
  }

  if( (section = settings.add_group("cornerSubPix")) ) {
    SAVE_OPTION(section, options.cornerSubPix, winSize);
    SAVE_OPTION(section, options.cornerSubPix, zeroZone);
    SAVE_OPTION(section, options.cornerSubPix, max_solver_iterations);
    SAVE_OPTION(section, options.cornerSubPix, solver_eps);
  }

  if( (section = settings.add_group("bilateralFilter")) ) {
    SAVE_OPTION(section, options.bilateralFilter, d);
    SAVE_OPTION(section, options.bilateralFilter, sigmaColor);
    SAVE_OPTION(section, options.bilateralFilter, sigmaSpace);
  }

  return true;
}

bool load_settings(c_config_setting settings, c_chessboard_corners_detection_options * options)
{
  c_config_setting section;

  LOAD_OPTION(settings, *options, chessboard_size);
  LOAD_OPTION(settings, *options, chessboard_cell_size);
  LOAD_OPTION(settings, *options, chessboard_distance);

  LOAD_OPTION(settings, *options, method);

  if ( (section = settings["findChessboardCorners"]).isGroup() ) {
    bool fOk;
    LOAD_OPTION(section, options->findChessboardCorners, max_scales);
    LOAD_OPTION(section, options->findChessboardCorners, flags);
  }

  if ( (section = settings["findChessboardCornersSB"]).isGroup() ) {
    LOAD_OPTION(section, options->findChessboardCornersSB, max_scales);
    LOAD_OPTION(section, options->findChessboardCornersSB, flags);
  }

  if ( (section = settings["cornerSubPix"]).isGroup() ) {

    LOAD_OPTION(section, options->cornerSubPix, winSize);
    LOAD_OPTION(section, options->cornerSubPix, zeroZone);
    LOAD_OPTION(section, options->cornerSubPix, max_solver_iterations);
    LOAD_OPTION(section, options->cornerSubPix, solver_eps);
  }

  if ( (section = settings["bilateralFilter"]).isGroup() ) {

    LOAD_OPTION(section, options->bilateralFilter, d);
    LOAD_OPTION(section, options->bilateralFilter, sigmaColor);
    LOAD_OPTION(section, options->bilateralFilter, sigmaSpace);
  }

  return true;

}
