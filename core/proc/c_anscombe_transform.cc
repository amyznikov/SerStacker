/*
 * c_anscombe_transform.cc
 *
 *  Created on: Oct 25, 2020
 *      Author: amyznikov
 */

#include "c_anscombe_transform.h"
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<anscombe_method>()
{
  static const c_enum_member members[] = {
      { anscombe_none, "none" },
      { anscombe_native, "native", "2 * sqrt(v + 3/8)"},
      { anscombe_generalized, "generalized", "2 * sqrt(g * v + c)"},
      { anscombe_none }  // must  be last
  };

  return members;
}


template<class _Tp>
static bool apply_direct(cv::InputArray _src, cv::OutputArray _dst, double g, double c)
{
  const cv::Size size = _src.size();
  const int type = _src.type();
  const int cn = _src.channels();
  const cv::Mat_<_Tp> src = _src.getMat();

  _dst.create(size, type);
  cv::Mat_<_Tp> dst = _dst.getMatRef();

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &src, &dst](const auto & range) {
        const int nx = size.width * cn;
        const double G = g;
        const double C = c;
        for ( int y = range.start; y < range.end; ++y ) {
          const _Tp * srcp = src[y];
          _Tp * __restrict dstp = dst[y];
          for ( int x = 0; x < nx; ++x) {
            dstp[x] = cv::saturate_cast<_Tp>(2.0 * std::sqrt( G * srcp[x] + C));
          }
        }
      });
  return true;
}

template<class _Tp>
static bool apply_inverse(cv::InputArray _src, cv::OutputArray _dst, double g, double c)
{
  const cv::Size size = _src.size();
  const int type = _src.type();
  const int channels = _src.channels();
  const cv::Mat_<_Tp> src = _src.getMat();

  _dst.create(size, type);
  cv::Mat_<_Tp> dst = _dst.getMatRef();

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &src, &dst](const auto & range) {
        const int nx = size.width * channels;
        const double bias_correction = -c;
        const double inv_g = 1.0 / g;

        for ( int y = range.start; y < range.end; ++y ) {
          const _Tp * srcp = src[y];
          _Tp * __restrict dstp = dst[y];
          for ( int x = 0; x < nx; ++x ) {
            const double v = 0.5 * srcp[x];
            const double vl = std::max(0., (v * v + bias_correction) * inv_g);
            dstp[x] = cv::saturate_cast<_Tp>(vl);
          }
        }
      });

  return true;
}

bool c_anscombe_transform::apply(cv::InputArray _src, cv::OutputArray _dst) const
{
  double g, c;

  switch(_opts.method) {
    case anscombe_none:
      _dst.assign(_src.getMat());
      return true;

    case anscombe_native:
      g = 1;
      c = 3./8.;
      break;
    case anscombe_generalized:
      if ( !_opts.generalized.auto_estimate ) {
        g = _opts.generalized.g;
        c = _opts.generalized.c;
      }
      else {
        cv::Rect brightRoi, darkRoi;
        cv::Mat src;
        if ( _src.channels() == 1 ) {
          src = _src.getMat();
        }
        else {
          // FIXME: this is temporary hack. Normally each channel must be processed separately
          cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
        }

        // ROI size is ~8% of the smaller side of the frame, rounded up to a multiple of 8
        // Not less than 32 pixels (for micro-frames) and no more than 256 (for huge matrices)
        const int minSide = std::min(src.cols, src.rows);
        const int roiSide = std::clamp(((minSide * 8 / 100) + 7) & ~7, 32, 256);
        const cv::Size roiSizeInSrc(roiSide, roiSide);

        // Adjust the scale so that the reduced ROI on the small copy is ~8x8 pixels in size.
        // Use powers of two (2, 4, 8, 16, 32) as they are more efficient for memory alignment.
        int scaleFactor = 1;
        while (scaleFactor * 2 <= 32 && (roiSide / (scaleFactor * 2)) >= 8) {
          scaleFactor *= 2;
        }

        if (!proposeAnscombeRois(src, brightRoi, darkRoi,  scaleFactor, roiSizeInSrc) ) {
          CF_ERROR("proposeAnscombeRois() fails");
          return false;
        }

        if ( !estimateGeneralizedAnscombeParams(src, brightRoi, darkRoi, g, c) ) {
          CF_ERROR("proposeAnscombeRois() fails");
          return false;
        }

        this->auto_estimated_g = g;
        this->auto_estimated_c = c;

        if ( _opts.generalized.dump_estimated_params ) {
          CF_DEBUG("\nANSCOMBE: g = %g c = %g BrightROI = { %d,%d,%dx%d } DarkROI = { %d,%d,%dx%d } ",
              g, c,
              brightRoi.x, brightRoi.y, brightRoi.width, brightRoi.height,
              darkRoi.x, darkRoi.y, darkRoi.width, darkRoi.height);
        }
      }
      break;

    default:
      CF_ERROR("Invalid anscombe method requested: %d", _opts.method);
      return false;
  }


  CV_DISPATCH(_src.depth(), apply_direct, _src, _dst, g, c);

  CF_ERROR("Not supported _src.depth()=%d", _src.depth());
  return false;
}

bool c_anscombe_transform::inverse(cv::InputArray _src, cv::OutputArray _dst) const
{
  double g, c;

  switch(_opts.method) {
    case anscombe_none:
      _dst.assign(_src.getMat());
      return true;

    case anscombe_native:
      g = 1;
      c = 3./8.;
      break;

    case anscombe_generalized:
      if ( !_opts.generalized.auto_estimate ) {
        g = _opts.generalized.g;
        c = _opts.generalized.c;
      }
      else {
        g = this->auto_estimated_g;
        c = this->auto_estimated_c;
      }
      break;

    default:
      CF_ERROR("Invalid anscombe method requested: %d", _opts.method);
      return false;
  }

  if ( _opts.generalized.dump_estimated_params ) {
    CF_DEBUG("\nANSCOMBE_INVERSE: g = %g c = %g", g, c);
  }

  CV_DISPATCH(_src.depth(), apply_inverse, _src, _dst, g, c);

  CF_ERROR("Not supported _src.depth()=%d", _src.depth());
  return false;
}

bool proposeAnscombeRois(cv::InputArray image,
    cv::Rect & outputBrightRoi,
    cv::Rect & outputDarkRoi,
    int scaleFactor,
    cv::Size roiSizeInSrc)
{
  if (image.empty() || image.channels() != 1 ) {
    CF_ERROR("invalid argument: Non-empty single-channel input image expected");
    return false;
  }

  if( scaleFactor <= 1 ) {
    CF_ERROR("invalid argument: scaleFactor=%d must be > 1", scaleFactor);
    return false;
  }

  // Downscale to suppress high frequencies and textures
  const cv::Mat src = image.getMat();

  cv::Mat1f smallImg;
  const int targetCols = src.cols / scaleFactor;
  const int targetRows = src.rows / scaleFactor;
  if( targetCols < 3 || targetRows < 3 ) {
    CF_ERROR("Too small input image size: %dxc%d", src.cols, src.rows);
    return false;
  }

  cv::resize(src, smallImg, cv::Size(targetCols, targetRows),
      0, 0, cv::INTER_AREA);

  // ROI size in coordinates of the downscaled image
  const int smallRoiW = std::max(1, roiSizeInSrc.width / scaleFactor);
  const int smallRoiH = std::max(1, roiSizeInSrc.height / scaleFactor);

  // Apply Box Filter with the reduced ROI size.
  // Now each pixel in blurredImg is equal to the AVERAGE brightness of the entire future ROI,
  // centered at that point. Local craters and harsh shadows will be completely averaged out.
  cv::Mat1f blurredImg;
  cv::boxFilter(smallImg, blurredImg, CV_32F,
      cv::Size(smallRoiW, smallRoiH),
      cv::Point(-1,-1),
      true, cv::BORDER_REPLICATE);

  const int borderX = smallRoiW / 2;
  const int borderY = smallRoiH / 2;

  // Extremes in a blurred image to ensure that the entire area around the extreme point
  // satisfies the condition (darkest/lightest on average)
  const cv::Mat1f searchRoi =
      blurredImg(cv::Rect(borderX, borderY, blurredImg.cols - 2 * borderX,
          blurredImg.rows - 2 * borderY));

  double minVal = 0.0, maxVal = 0.0;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(searchRoi, &minVal, &maxVal, &minLoc, &maxLoc);

  minLoc.x += borderX;
  minLoc.y += borderY;
  maxLoc.x += borderX;
  maxLoc.y += borderY;

  const int srcMinCenterX = minLoc.x * scaleFactor + scaleFactor / 2;
  const int srcMinCenterY = minLoc.y * scaleFactor + scaleFactor / 2;
  const int srcMaxCenterX = maxLoc.x * scaleFactor + scaleFactor / 2;
  const int srcMaxCenterY = maxLoc.y * scaleFactor + scaleFactor / 2;

  outputDarkRoi = cv::Rect(
      srcMinCenterX - roiSizeInSrc.width / 2,
      srcMinCenterY - roiSizeInSrc.height / 2,
      roiSizeInSrc.width,
      roiSizeInSrc.height
  );

  outputBrightRoi = cv::Rect(
      srcMaxCenterX - roiSizeInSrc.width / 2,
      srcMaxCenterY - roiSizeInSrc.height / 2,
      roiSizeInSrc.width,
      roiSizeInSrc.height
  );

  const cv::Rect imgBounds(0, 0, src.cols, src.rows);
  outputDarkRoi = outputDarkRoi & imgBounds;
  outputBrightRoi = outputBrightRoi & imgBounds;

  return (outputDarkRoi.area() == roiSizeInSrc.area() &&
      outputBrightRoi.area() == roiSizeInSrc.area());
}


bool estimateGeneralizedAnscombeParams(const cv::Mat1f & src,
    const cv::Rect & brightRoi,
    const cv::Rect & darkRoi,
    double & g,
    double & c)
{
  if (brightRoi.empty() || darkRoi.empty()) {
    CF_ERROR("Invalid argument: empty image ROI requested");
    return false;
  }

  const cv::Rect imgBounds(0, 0, src.cols, src.rows);
  if ((brightRoi & imgBounds) != brightRoi || (darkRoi & imgBounds) != darkRoi) {
    CF_ERROR("Invalid argument: ROI is out of image");
    return false;
  }

  // Average brightness of the useful signal in the ROI
  const double I_bright = cv::mean(src(brightRoi))[0];
  const double I_dark   = cv::mean(src(darkRoi))[0];

  if (std::abs(I_bright - I_dark) < 1e-6) {
    CF_ERROR("Too little contrast in brightness between regions for analytical separation");
    return false;
  }

  // Immerker noise variance for each ROI ===
  // Immerker Laplacian constant: sqrt(M_PI_2) / 6.0
  constexpr double S = 0.20888568955258338;
  static const float K_data[3 * 3] = {
      +1.f * (float)S, -2.f * (float)S, +1.f * (float)S,
      -2.f * (float)S, +4.f * (float)S, -2.f * (float)S,
      +1.f * (float)S, -2.f * (float)S, +1.f * (float)S
  };
  static const cv::Mat1f K(3, 3, const_cast<float*>(K_data));

  // Local Noise maps for each region to save memory and time
  cv::Mat1f noiseBright, noiseDark;
  cv::filter2D(src(brightRoi), noiseBright, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src(darkRoi), noiseDark, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  // Standard deviation of the Laplacian mask.
  // Since the mean (Mean) of the Laplacian is a priori equal to 0,
  // Stdev^2 is equal to the mean square of the pixels.
  cv::Scalar meanB, stdevB;
  cv::meanStdDev(noiseBright, meanB, stdevB);
  const double sigma2_bright = stdevB[0] * stdevB[0];

  cv::Scalar meanD, stdevD;
  cv::meanStdDev(noiseDark, meanD, stdevD);
  const double sigma2_dark = stdevD[0] * stdevD[0];

  // Linear model: sigma^2 = g * I + sigma_read^2
  // If due to the residual relief texture g goes into minus, force a limit from below
  g = std::max(0., (sigma2_bright - sigma2_dark) / (I_bright - I_dark));

  // Pure sensor reading noise in the dark
  const double sigma2_read = sigma2_dark - g * I_dark;

  // Shift constant for Generalized Anscombe: c = 3/8 * g^2 + sigma_read^2
  // Limit the constant to a minimum positive value to avoid negative numbers under the root
  c = std::max(1e-12, (3.0 / 8.0) * g * g + sigma2_read);

  return true;
}

