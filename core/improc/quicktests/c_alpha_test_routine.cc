/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>


//template<>
//const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
//{
//  static const c_enum_member members[] = {
//      { c_alpha_test_routine::DISPLAY_SRC, "SRC", "" },
//      { c_alpha_test_routine::DISPLAY_G0, "G0", "" },
////      { c_alpha_test_routine::DISPLAY_DIFF, "DIFF", "" },
////      { c_alpha_test_routine::DISPLAY_SUMM, "SUMM", "" },
////      { c_alpha_test_routine::DISPLAY_RATIO, "RATIO", "" },
//      { c_alpha_test_routine::DISPLAY_SRC}
//  };
//  return members;
//}


namespace {

template<class _Tp>
static bool _findNeutralColorBalance(cv::InputArray image, cv::InputArray mask,
    cv::Scalar & outputScales, cv::Scalar & outputShifts,
    int max_iterations )
{
  for ( int i = 0; i < 4; ++i ) {
    outputScales[i] = 1;
    outputShifts[i] = 0;
  }

  if (image.empty() || image.channels() < 2 ) {
    return false;
  }

  if (!mask.empty() && (mask.type() != CV_8UC1 || mask.size() != image.size()) ) {
    return false;
  }


  //
  const cv::Mat src = image.getMat();
  const cv::Size size = image.size();
  const int cn = image.channels();


  // Select reference channel
  int rcn = 0;
  cv::Scalar cmean, cstdev;
  cv::meanStdDev(image, cmean, cstdev, mask);
  for ( int c = 1; c < cn; ++c ) {
    if ( cstdev[c] > cstdev[rcn] ) {
      rcn = c;
    }
  }

  CF_DEBUG("rcn = %d", rcn);


  //

  static const auto computeRegression = [](const cv::Mat & src, const cv::Mat & msk,
      int rcn, c_weighted_line_estimate<double> regs[4]) {

    const cv::Size size = src.size();
    const int cn = src.channels();

    const uint8_t * src_base = src.ptr();
    const size_t src_stride = src.step;
    const uint8_t * mask_base = msk.empty() ? nullptr : msk.ptr();
    const size_t mask_stride = msk.empty() ?  0 : msk.step;

    for( int y = 0; y < size.height; ++y ) {
      const _Tp * srcp = (const _Tp*) (src_base + y * src_stride);
      if ( !mask_base ) {
        for( int x = 0; x < size.width; ++x, srcp += cn ) {
          const double rv = srcp[rcn];
          for( int c = 0; c < cn; ++c ) {
            if( c != rcn ) {
              regs[c].update(rv, srcp[c]);
            }
          }
        }
      }
      else {
        const uint8_t * mskp = mask_base + y * mask_stride;
        for( int x = 0; x < size.width; ++x, ++mskp, srcp += cn ) {
          if( *mskp ) {
            const double rv = srcp[rcn];
            for( int c = 0; c < cn; ++c ) {
              if( c != rcn ) {
                regs[c].update(rv, srcp[c]);
              }
            }
          }
        }
      }
    };
  };

  static const auto maskOutliers = [](const cv::Mat & src, cv::Mat & msk,
      int rcn, const cv::Scalar & shifts, const cv::Scalar & scales,
      const cv::Scalar & thresholds) {

        const cv::Size size = src.size();
        const int cn = src.channels();

        if ( msk.empty() ) {
          msk = cv::Mat1b(src.size(), 255);
        }

        const uint8_t * src_base = src.ptr();
        const size_t src_stride = src.step;
        uint8_t * mask_base = msk.ptr();
        const size_t mask_stride = msk.step;

        int num_outliers = 0;

        for( int y = 0; y < size.height; ++y ) {
          const _Tp * srcp = (const _Tp*) (src_base + y * src_stride);
          uint8_t * __restrict mskp = mask_base + y * mask_stride;
          for( int x = 0; x < size.width; ++x, ++mskp, srcp += cn ) {
            if( *mskp ) {
              const double rv = srcp[rcn];
              for( int c = 0; c < cn; ++c ) {
                if( c != rcn ) {
                  if ( std::abs(shifts[c] + scales[c] * rv - srcp[c]) > thresholds[c] ) {
                    * mskp = 0;
                    ++num_outliers;
                  }
                }
              }
            }
          }
        };

        return num_outliers;
      };

  c_weighted_line_estimate<double> regs[4];
  cv::Scalar thresholds;
  cv::Mat msk;

  for ( int i = 0; i < max_iterations; ++i ) {

    if ( i == 0 && !mask.empty() ) {
      mask.getMat().copyTo(msk);
    }

    for ( int c = 0; c < cn; ++c ) {
      regs[c].reset();
    }

    computeRegression(src, msk, rcn, regs);

    for ( int c = 0; c < cn; ++c ) {
      if ( c != rcn ) {
        regs[c].compute(outputShifts[c], outputScales[c], thresholds[c]);
        thresholds[c] = 3 * std::sqrt(thresholds[c]);
        CF_DEBUG("iteration [%d]: thresholds[c]=%g", i, thresholds[c]);
      }
    }

    if ( i < max_iterations - 1 ) {
      const int num_outliers = maskOutliers(src, msk, rcn, outputShifts, outputScales, thresholds);
      CF_DEBUG("iteration [%d]: num_outliers=%d", i, num_outliers);
      if ( num_outliers < 1 ) {
        break;
      }
    }
  }

  // Convert regression coefficients into direct factors of affine transformation
  // (dst = src * stretch + shift)
  for( int c = 0; c < cn; ++c ) {
    outputScales[c] = 1.0 / outputScales[c];
    outputShifts[c] = -outputShifts[c] * outputScales[c];
  }

  return true;
}


static bool findNeutralColorBalance(cv::InputArray image, cv::InputArray mask,
    cv::Scalar & outputScales, cv::Scalar & outputShifts,
    int max_iterations )
{
  CV_DISPATCH(image.depth(), _findNeutralColorBalance, image, mask,
      outputScales, outputShifts, max_iterations);
  return false;
}


} // namespace

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{

  ctlbind(ctls, "stretch (B;G;R;A)", ctx(&this_class::_stretch), "Scales applied to color channels");
  ctlbind(ctls, "shifts  (B;G;R;A)", ctx(&this_class::_shift), "Offsets added to color channels");
  ctlbind(ctls, "Auto white balance", ctx(&this_class::_auto_white_balance), "Set checked for auto white balance");
  ctlbind(ctls, "Use ROI", ctx(&this_class::_useROI), "Limit auto white balance estimation by user ROI");
  ctlbind(ctls, "max_iterations", ctx(&this_class::_max_iterations), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _stretch);
    SERIALIZE_OPTION(settings, save, *this, _shift);
    SERIALIZE_OPTION(settings, save, *this, _max_iterations);
    SERIALIZE_OPTION(settings, save, *this, _useROI);
    return true;
  }
  return false;
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _auto_white_balance && image.channels() > 1 ) {

    if ( !_useROI ) {
      findNeutralColorBalance(image, mask, _stretch, _shift, _max_iterations);
    }
    else {
      cv::Rect rc (0, 0, image.cols(), image.rows());
      if ( _useROI ) {
        cv::Rect roi;
        if ( ctlbind_get_roi(&roi) ) {
          rc = rc & roi;
        }
      }

      if ( rc.empty() ) {
        CF_ERROR("ROI is empty, can not find auto white balance");
        return false;
      }

      const cv::Mat src = image.getMatRef()(rc);
      const cv::Mat msk = mask.empty() ? cv::Mat() : mask.getMatRef()(rc);
      findNeutralColorBalance(src, msk, _stretch, _shift, _max_iterations);
    }

    set_has_contol_changes(true);
  }

  return applyChannelTransform(image, image, _stretch, _shift);
}
