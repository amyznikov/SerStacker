/*
 * c_epipolar_matcher.cc
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#include "c_epipolar_matcher.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/stereo/scale_sweep.h>
#include <core/io/save_image.h>
#include <core/debug.h>

#if HAVE_TBB
  #include <tbb/tbb.h>
  using tbb_range = tbb::blocked_range<int>;
#endif


c_epipolar_matcher::c_epipolar_matcher()
{
}

c_epipolar_matcher::c_epipolar_matcher(const c_epipolar_matcher_options & opts) :
    options_(opts)
{
}

bool c_epipolar_matcher::enabled() const
{
  return options_.enabled;
}

void c_epipolar_matcher::set_enabled(bool v)
{
  options_.enabled = v;
}

void c_epipolar_matcher::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string & c_epipolar_matcher::debug_path() const
{
  return debug_path_;
}

void c_epipolar_matcher::set_options(const c_epipolar_matcher_options & opts)
{
  options_ = opts;
}

const c_epipolar_matcher_options & c_epipolar_matcher::options() const
{
  return options_;
}

c_epipolar_matcher_options & c_epipolar_matcher::options()
{
  return options_;
}

const cv::Mat2f & c_epipolar_matcher::matches() const
{
  return matches_;
}

const cv::Mat2i & c_epipolar_matcher::back_matches() const
{
  return back_matches_;
}

const cv::Mat1w & c_epipolar_matcher::costs() const
{
  return costs_;
}


bool c_epipolar_matcher::serialize(c_config_setting settings, bool save)
{
  SERIALIZE_OPTION(settings, save, options_, enabled);
  SERIALIZE_OPTION(settings, save, options_, max_disparity);
  SERIALIZE_OPTION(settings, save, options_, diff_threshold);
  SERIALIZE_OPTION(settings, save, options_, avg_scale);
  SERIALIZE_OPTION(settings, save, options_, enable_debug);

  return true;
}

bool c_epipolar_matcher::match(cv::InputArray _current_image, cv::InputArray _current_mask,
    cv::InputArray _previous_image, cv::InputArray _previous_mask,
    const cv::Matx33f & derotation_homography,
    const cv::Point2d & epipole_location)
{
  INSTRUMENT_REGION("");

  ///////////////////////////////////

  // Check input arguments

  if( _current_image.depth() != CV_8U ) {
    CF_ERROR("Invalid current image depth=%d. Must be CV_8U", _current_image.depth());
    return false;
  }

  if( _previous_image.depth() != CV_8U ) {
    CF_ERROR("Invalid previous image depth=%d. Must be CV_8U", _previous_image.depth());
    return false;
  }

  if( _current_image.size() != _previous_image.size() ) {
    CF_ERROR("Current and Previous image sizes must be equal.\n"
        "current_image: %dx%d previous_image: %dx%d",
        _current_image.cols(), _current_image.rows(),
        _previous_image.cols(), _previous_image.rows());
    return false;
  }

  if( !_current_mask.empty() ) {

    if( _current_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid current mask type=%d. Must be CV_8UC1",
          _current_mask.type());
      return false;
    }

    if( _current_mask.size() != _current_image.size() ) {
      CF_ERROR("current mask size %dx%d is not equal to image size %dx%d",
          _current_mask.cols(), _current_mask.rows(),
          _current_image.cols(), _current_image.rows());
      return false;
    }
  }

  if( !_previous_mask.empty() ) {

    if( _previous_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid previous mask type=%d. Must be CV_8UC1",
          _current_mask.type());
      return false;
    }

    if( _previous_mask.size() != _previous_image.size() ) {
      CF_ERROR("previous mask size %dx%d is not equal to image size %dx%d",
          _previous_mask.cols(), _previous_mask.rows(),
          _previous_image.cols(), _previous_image.rows());
      return false;
    }
  }

  ///////////////////


  cv::Mat current_image, previous_image;
  cv::Mat1b compute_mask;
  cv::Mat absdiff_image;
  cv::Mat remapped_current_image;
  cv::Mat1b avgdiff_image;
  cv::Mat2f cmap;
  std::string debug_filename;

  const cv::Size size =
      _current_image.size();

//  previous_image =
//      _previous_image.getMat();

  cv::warpPerspective(_current_image,
      current_image,
      derotation_homography,
      size,
      cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
      cv::BORDER_CONSTANT);

  if( true ) {

    INSTRUMENT_REGION("compute_mask");

    cv::warpPerspective(_current_mask.empty() ? cv::Mat1b(size, 255) : _current_mask.getMat(),
        compute_mask,
        derotation_homography,
        size,
        cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
        cv::BORDER_CONSTANT);

    cv::compare(compute_mask, 255, compute_mask,
        cv::CMP_GE);

    cv::absdiff(current_image, _previous_image,
        absdiff_image);

    if( absdiff_image.channels() > 1 ) {
      reduce_color_channels(absdiff_image, absdiff_image,
          cv::REDUCE_MAX);
    }
    cv::GaussianBlur(absdiff_image, absdiff_image, cv::Size(), 1, 1,
        cv::BORDER_REPLICATE);

    if( !debug_path_.empty() ) {
      if( !save_image(absdiff_image, debug_filename = ssprintf("%s/absdiff_image.png", debug_path_.c_str())) ) {
        CF_ERROR("save_image('%s') fails", debug_filename.c_str());
        return false;
      }
    }

    cv::compare(absdiff_image, options_.diff_threshold,
        absdiff_image, cv::CMP_GE);

    cv::bitwise_and(absdiff_image, compute_mask,
        compute_mask);

    if( !_previous_mask.empty() ) {
      cv::bitwise_and(_previous_mask, compute_mask,
          compute_mask);
    }
  }

  if( !debug_path_.empty() ) {
    if( !save_image(current_image, debug_filename = ssprintf("%s/current_image.png", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", debug_filename.c_str());
      return false;
    }
    if( !save_image(_previous_image.getMat(), debug_filename = ssprintf("%s/previous_image.png", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", debug_filename.c_str());
      return false;
    }
    if( !save_image(compute_mask, debug_filename = ssprintf("%s/compute_mask.png", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", debug_filename.c_str());
      return false;
    }
  }


  _previous_image.getMat().copyTo(previous_image,
      compute_mask);

  current_image.setTo(0, ~compute_mask);

  /////////////////////////////////////////////////////


  for ( int disparity = 0; disparity < options_.max_disparity; ++disparity ) {

    if ( true ) {

      INSTRUMENT_REGION("iteration");

      if( disparity == 0 ) {
        current_image.copyTo(remapped_current_image);
      }
      else {
        create_scale_compression_remap(disparity, size, epipole_location, cmap, compute_mask);
        cv::remap(current_image, remapped_current_image, cmap, cv::noArray(), cv::INTER_LINEAR);
      }

      cv::absdiff(remapped_current_image, previous_image,
          absdiff_image);

      if( absdiff_image.channels() > 1 ) {
        reduce_color_channels(absdiff_image, absdiff_image,
            cv::REDUCE_MAX);
      }

      const int avg_scale =
          (std::max)(1, options_.avg_scale);

      cv::GaussianBlur(absdiff_image, avgdiff_image, cv::Size(), avg_scale, avg_scale);

//      cv::resize(absdiff_image, avgdiff_image,
//          cv::Size(size.width >> avg_scale, size.height >> avg_scale),
//          0, 0,
//          cv::INTER_AREA);
//
//      cv::resize(avgdiff_image, avgdiff_image,
//          size,
//          0, 0,
//          cv::INTER_AREA);
    }

    if( !debug_path_.empty() ) {
      if( !save_image(remapped_current_image,
          debug_filename = ssprintf("%s/remap/remapped_current_image.%03d.png",
              debug_path_.c_str(), disparity)) ) {
        CF_ERROR("save_image('%s') fails", debug_filename.c_str());
        return false;
      }
      if( !save_image(absdiff_image,
          debug_filename = ssprintf("%s/absdiff/absdiff_image.%03d.png",
              debug_path_.c_str(), disparity)) ) {
        CF_ERROR("save_image('%s') fails", debug_filename.c_str());
        return false;
      }
      if( !save_image(avgdiff_image,
          debug_filename = ssprintf("%s/avgdiff/avgdiff_image.%03d.png",
              debug_path_.c_str(), disparity)) ) {
        CF_ERROR("save_image('%s') fails", debug_filename.c_str());
        return false;
      }
    }
  }

  return true;
}

//bool c_epipolar_matcher::compute_matches(const c_block_array & current_image, const c_block_array & previous_image,
//    const cv::Mat1b & mask, const cv::Point2d & E)
//{
//
//
//
//  const cv::Size size =
//      previous_image.size();
//
//  matches_.create(size);
//  matches_.setTo(-1);
//
//  back_matches_.create(size);
//  back_matches_.setTo(-1);
//
//  costs_.create(size);
//  costs_.setTo(0);
//
//  int max_disparity =
//      options_.max_disparity;
//
//  // loop for reference y
//  for ( int ry = 5;  ry < size.height - 5; ++ry ) {
//    // loop for reference x
//    for ( int rx = 5;  rx < size.width - 5; ++rx ) {
//
//      if ( !mask[ry][rx] ) {
//        continue;
//      }
//
//      int best_cx = rx;
//      int best_cy = ry;
//
//      const c_pixelblock & referennce_image =
//          previous_image[ry][rx];
//
//      uint32_t best_cost =
//          absdiff(referennce_image,
//              current_image[ry][rx]);
//
//      if ( best_cost > 0 ) {
//
//        int incx, incy, cxmax, cymax;
//        double k;
//
//        compute_search_range(size, E, rx, ry, max_disparity,
//            &incx, &incy, &cxmax, &cymax,
//            &k);
//
//        if ( incx ) { // H direction
//
//          for( int cx = rx; cx != cxmax; cx += incx ) {
//
//            const int cy0 = (int)(E.y + k * (cx - E.x));
//            const int cy1 = cy0 + 1;
//            if( cy0 < 5 || cy1 >= size.height - 5 ) {
//              break;
//            }
//
//            if( mask[cy0][cx] ) {
//
//              const uint32_t cost = absdiff(referennce_image,
//                  current_image[cy0][cx]);
//
//              if( cost < best_cost ) {
//                best_cx = cx;
//                best_cy = cy0;
//                if( (best_cost = cost) == 0 ) {
//                  break;
//                }
//              }
//            }
//
//            if( mask[cy1][cx] ) {
//
//              const uint32_t cost = absdiff(referennce_image,
//                  current_image[cy1][cx]);
//
//              if( cost < best_cost ) {
//                best_cx = cx;
//                best_cy = cy1;
//                if( (best_cost = cost) == 0 ) {
//                  break;
//                }
//              }
//            }
//          }
//
//        }
//        else { // V direction
//
//          for( int cy = ry; cy != cymax; cy += incy ) {
//
//            const int cx0 = (int)(E.x + k * (cy - E.y));
//            const int cx1 = cx0 + 1;
//            if( cx0 < 5 || cx1 >= size.width - 5 ) {
//              break;
//            }
//
//            if( mask[cy][cx0] ) {
//
//              const uint32_t cost = absdiff(referennce_image,
//                  current_image[cy][cx0]);
//
//              if( cost < best_cost ) {
//                best_cx = cx0;
//                best_cy = cy;
//                if( (best_cost = cost) == 0 ) {
//                  break;
//                }
//              }
//            }
//
//            if( mask[cy][cx1] ) {
//
//              const uint32_t cost = absdiff(referennce_image,
//                  current_image[cy][cx1]);
//
//              if( cost < best_cost ) {
//                best_cx = cx1;
//                best_cy = cy;
//                if( (best_cost = cost) == 0 ) {
//                  break;
//                }
//              }
//            }
//          }
//
//        }
//      }
//
//      //if ( back_matches_[best_cy][best_cx][0] < 0 )
//      if ( best_cx != rx || best_cy != ry )
//      {
//        matches_[ry][rx][0] = best_cx;
//        matches_[ry][rx][1] = best_cy;
//        back_matches_[best_cy][best_cx][0] = rx;
//        back_matches_[best_cy][best_cx][1] = ry;
//        costs_[ry][rx] = best_cost;
//      }
////      else {
////
////        int brx = back_matches_[best_cy][best_cx][0];
////        int bry = back_matches_[best_cy][best_cx][1];
////
////        if ( best_cost < costs_[brx][brx] ) {
////
////          matches_[bry][brx][0] = -1;
////          matches_[bry][brx][1] = -1;
////          // costs_[bry][brx] = 0;
////
////          matches_[ry][rx][0] = best_cx;
////          matches_[ry][rx][1] = best_cy;
////          back_matches_[best_cy][best_cx][0] = rx;
////          back_matches_[best_cy][best_cx][1] = ry;
////          costs_[ry][rx] = best_cost;
////
////        }
////        else {
////        }
////      }
//    }
//  }
//
//  return true;
//}


cv::Mat1f matchesToEpipolarDisparity(const cv::Mat2f & matches, const cv::Point2d & E)
{
  const cv::Size size =
      matches.size();

  cv::Mat1f disparity(size, -1.f);

  static const auto hyp =
      [](double x, double y) -> double {
          return sqrt(x * x + y * y);
      };

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {
      if( matches[y][x][0] >= 0 && matches[y][x][1] >= 0 ) {

        disparity[y][x] =
            hyp(matches[y][x][0] - E.x, matches[y][x][1] - E.y) -
                hyp(x - E.x, y - E.y);

      }
    }
  }

  return disparity;
}
