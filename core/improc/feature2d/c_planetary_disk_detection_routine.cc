/*
 * c_planetary_disk_detection_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_planetary_disk_detection_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>


template<>
const c_enum_member * members_of<c_planetary_disk_detection_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_planetary_disk_detection_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", "" },
      { c_planetary_disk_detection_routine::DISPLAY_INTENSITY_IMAGE, "INTENSITY_IMAGE", "" },
      { c_planetary_disk_detection_routine::DISPLAY_SPECTRUM_MODULE, "SPECTRUM", "" },
      { c_planetary_disk_detection_routine::DISPLAY_SRC_IMAGE}
  };
  return members;
}

/**
* @brief Adjust rectangular ROI of the planet to an optimal square shape for FFT
**/
static cv::Rect fftGetOptimalSquaredROI(const cv::Size & imageSize, const cv::Rect & rawROI)
{
  const int centerX = rawROI.x + rawROI.width / 2;
  const int centerY = rawROI.y + rawROI.height / 2;
  const int maxSide = std::max(rawROI.width, rawROI.height);

  int optimalSide = cv::getOptimalDFTSize(maxSide);
  if( optimalSide > imageSize.width || optimalSide > imageSize.height ) {
    const int absoluteMaxSide = std::min(imageSize.width, imageSize.height);
    int safeSide = absoluteMaxSide;
    while (safeSide > 0) {
      if( cv::getOptimalDFTSize(safeSide) == safeSide ) {
        break;
      }
      --safeSide;
    }
    optimalSide = safeSide;
  }

  int newX = centerX - optimalSide / 2;
  if (newX + optimalSide > imageSize.width) {
    newX = imageSize.width - optimalSide;
  }
  if (newX < 0) {
    newX = 0;
  }

  int newY = centerY - optimalSide / 2;
  if (newY + optimalSide > imageSize.height) {
    newY = imageSize.height - optimalSide;
  }
  if (newY < 0) {
    newY = 0;
  }

  return cv::Rect(newX, newY, optimalSide, optimalSide);
}

/**
* @brief Apply a smooth circular cosine window to mask the corners of a square.
*/
static cv::Mat1f createCircularApodizationWindow(const cv::Size & size)
{
  cv::Mat1f mask = cv::Mat1f::zeros(size);

  const float r_outer = (size.width / 2.0f) * 0.95f;
  const float r_inner = r_outer * 0.65f;

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &mask](const auto & range) {
        const int cx = size.width / 2;
        const int cy = size.height / 2;

        for( int y = range.start; y < range.end; ++y ) {
          float * __restrict dstp = mask[y];
          for( int x = 0; x < mask.cols; ++x ) {
            const float dx = x - cx;
            const float dy = y - cy;
            const float r = std::sqrt(dx * dx + dy * dy);

            if( r <= r_inner ) {
              dstp[x] = 1.0f;
            }
            else if( r >= r_outer ) {
              dstp[x] = 0.0f;
            }
            else {
              const float fraction = (r - r_inner) / (r_outer - r_inner);
              dstp[x] = 0.5f * (1.0f + std::cos(fraction * CV_PI));
            }
      }
    }
  });


  return mask;
}

/**
* @brief Function for automatically determining the position angle from the FFT
* @param fftSpectrum Cleaned FFT spectrum (after ppsDecomposition and morphological smoothing)
* @return double Polar axis position angle in degrees [0, 180)
*/
// 2022-08-09-2336_8-CapObj-32F
static double estimateRadonOrientation(const cv::Mat1f & fftSpectrum)
{
  const cv::Size fftSize = fftSpectrum.size();
  const int cx = fftSize.width / 2;
  const int cy = fftSize.height / 2;

  // Step over only the very core (15 px) and
  // take the entire beam up to the mid frequencies (150 px)
  const int Rmin = 15;
  const int Rmax = 150;
  const float Rmin2 = Rmin * Rmin;
  const float Rmax2 = Rmax * Rmax;

  // Resolution of Nyquist histogram
  const int num_bins = std::max(180, cvRound(CV_PI * Rmax));
  const double bin_step = 180.0 / num_bins;

  std::vector<float> histogram(num_bins, 0.0f);

  // Scan range
  const int y_start = std::max(0, cy - Rmax);
  const int y_end   = std::min(fftSpectrum.rows, cy + Rmax);
  const int x_start = std::max(0, cx - Rmax);
  const int x_end   = std::min(fftSpectrum.cols, cx + Rmax);

  for (int y = y_start; y < y_end; ++y) {
    const float * srcp = fftSpectrum[y];
    const float dy = y - cy;
    const float dy2 = dy * dy;

    for (int x = x_start; x < x_end; ++x) {
      const float dx = x - cx;
      const float dx2 = dx * dx;
      const float r2 = dx2 + dy2;

      if (r2 >= Rmin2 && r2 <= Rmax2) {

        // Collapse symmetric FFT spectrum into a hemisphere [0, 180)
        double angle = std::atan2(dy, dx) * 180.0 / CV_PI;
        if (angle < 0) {
          angle += 180;
        }

        const int bin_idx = int(angle / bin_step);
        if (bin_idx >= 0 && bin_idx < num_bins) {
          histogram[bin_idx] += srcp[x];
        }
      }
    }
  }

  // Gaussian smooth (kernel size 7 or 9 for smoothness)
  cv::Mat histMat(num_bins, 1, CV_32FC1, histogram.data());
  cv::GaussianBlur(histMat, histMat, cv::Size(1, 9), 0, 0, cv::BORDER_REPLICATE);

  // Search for the peak
  const auto maxpos = std::max_element(histogram.begin(), histogram.end());
  const int best_bin = std::distance(histogram.begin(), maxpos);

  // 3-point cyclic subpixel parabolic interpolation
  // Formula for the parabola vertex: x_opt = x_center + 0.5 * (y1 - y3) / (y1 - 2*y2 + y3)
  const int left_bin  = (best_bin - 1 + num_bins) % num_bins;
  const int right_bin = (best_bin + 1) % num_bins;
  const double y1 = histogram[left_bin];
  const double y2 = histogram[best_bin];
  const double y3 = histogram[right_bin];
  const double denom = y1 - 2.0 * y2 + y3;

  double final_polar_angle = best_bin * bin_step;
  if( std::abs(denom) > 1e-6 ) {
    const double delta_bin = 0.5 * (y1 - y3) / denom;
    double subpixel_bin = best_bin + delta_bin;
    if( subpixel_bin < 0.0 ) {
      subpixel_bin += num_bins;
    }
    final_polar_angle = subpixel_bin * bin_step;
  }

  CF_DEBUG("\n-> [RADON FIXED] Polar Axis Angle: %g° | Bins: %d | Step: %g | R: [%d ... %d]",
      final_polar_angle, num_bins, bin_step, Rmin, Rmax);

  return final_polar_angle - 90.0;
}

void c_planetary_disk_detection_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "intensity_channel", CTL_CONTEXT(ctx, _intensity_channel), "");
  ctlbind(ctls, "gsigma", CTL_CONTEXT(ctx, gsigma), "");
  ctlbind(ctls, "se_radius", CTL_CONTEXT(ctx, se_radius), "");
  ctlbind(ctls, "updateROI", CTL_CONTEXT(ctx, updateROI), "");
}

bool c_planetary_disk_detection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, gsigma);
    SERIALIZE_OPTION(settings, save, *this, se_radius);
    SERIALIZE_OPTION(settings, save, *this, updateROI);
    return true;
  }
  return false;
}

bool c_planetary_disk_detection_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();
  const cv::Size srcSize = src.size();
  const int cn = image.channels();

  cv::Mat grayImage;
  if ( cn != 1  ) {
    extract_channel(src, grayImage, cv::noArray(), cv::noArray(),
        _intensity_channel, CV_32F);
  }
  else if( src.depth() != CV_32F ) {
    src.convertTo(grayImage, CV_32F);
  }
  else {
    grayImage = src;
  }

  cv::Point2f centroid;
  cv::Rect component_rect;
  cv::Mat cmponent_mask;
  cv::Point2f geometrical_center;

  bool fOk =
      simple_planetary_disk_detector(grayImage, mask,
          gsigma, se_radius,
          &centroid,
          &component_rect,
          &cmponent_mask,
          &geometrical_center);

  if ( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }


  CF_DEBUG("\n"
      "centroid: %g;%g rect: %d;%d;%dx%d geometrical_center=%g;%g",
      centroid.x, centroid.y,
      component_rect.x, component_rect.y, component_rect.width, component_rect.height,
      geometrical_center.x, geometrical_center.y);

  if ( updateROI ) {
    ctlbind_update_roi(component_rect);
  }

  const cv::Rect rc = fftGetOptimalSquaredROI(srcSize, component_rect);
  const cv::Size fftSize = rc.size();

  if (ApodizationWindow.size() != fftSize ) {
    ApodizationWindow = createCircularApodizationWindow(fftSize);
  }

  CF_DEBUG("rc={x=%d y=%d w=%d h=%d} ApodizationWindow: %dx%d grayImage: %dx%d",
      rc.x, rc.y, rc.width, rc.height, ApodizationWindow.cols, ApodizationWindow.rows,
      grayImage.cols, grayImage.rows);

  cv::multiply(grayImage(rc), ApodizationWindow, grayImage);

  if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
  }

  fftPPSDecomposition(grayImage, VLAP,
      INTENSITY_P, cv::noArray(),
      true);

  fftSpectrumModule(INTENSITY_P, INTENSITY_Magnitude);

  // Find orientation here
  const double angle =
      estimateRadonOrientation(INTENSITY_Magnitude);

  CF_DEBUG(" -> Detected Polar Axis Position Angle: %g°", angle );

  // Make some simple output debug display here

  static const auto genrgb =
      [](cv::InputArray img, double r, double g, double b) -> cv::Scalar {
        double minv = 0, maxv = 1;
        cv::minMaxLoc(img, &minv, &maxv);
        return CV_RGB(1.05 * r * maxv, 1.05 * g * maxv, 1.05 * b * maxv);
      };

  static const auto drawRotatedRect =
      [](cv::InputOutputArray image, const cv::RotatedRect & rc,
      const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
  {
    cv::Point2f pts[4];
    rc.points(pts);

    for( int i = 0; i < 4; i++ ) {
      cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
    }

    cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
    cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
  };

  if ( _display == DISPLAY_SRC_IMAGE ) {
    cv::RotatedRect rrc;
    rrc.size.width = 2 * fftSize.width / 3;
    rrc.size.height = 2 * fftSize.width / 6;
    rrc.angle = angle;
    rrc.center.x = rc.x + rc.width / 2;
    rrc.center.y = rc.y + rc.height / 2;

    if( image.channels() != 3 ) {
      cv::cvtColor(image.getMat(), image, cv::COLOR_GRAY2BGR);
    }

    drawRotatedRect(image, rrc, genrgb(image, 0.8, 0, 0), 1);
    mask.move(cmponent_mask);
    return true;
  }

  if ( _display == DISPLAY_INTENSITY_IMAGE ) {
    grayImage.copyTo(image);
    mask.release();
    return true;
  }

  if ( _display == DISPLAY_SPECTRUM_MODULE ) {
    INTENSITY_Magnitude.copyTo(image);
    mask.release();
    return true;
  }

  CF_DEBUG("H");
  return fOk;
}

