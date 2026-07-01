/*
 * c_planetary_disk_detection_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_planetary_disk_detection_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/gradient.h>
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

template<>
const c_enum_member * members_of<c_planetary_disk_detection_routine::ORIENTATION_METHOD>()
{
  static const c_enum_member members[] = {
      { c_planetary_disk_detection_routine::ORIENTATION_RADON_FFT, "RADON_FFT", "" },
      { c_planetary_disk_detection_routine::ORIENTATION_RADON_STENSOR, "RADON_STENSOR", "" },
      { c_planetary_disk_detection_routine::ORIENTATION_RADON_FFT}
  };
  return members;
}


void c_planetary_disk_detection_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "intensity_channel", CTL_CONTEXT(ctx, _intensity_channel), "");
  ctlbind(ctls, "orientation_method", CTL_CONTEXT(ctx, _orientation_method), "");
  ctlbind(ctls, "updateROI", CTL_CONTEXT(ctx, updateROI), "");
  ctlbind_expandable_group(ctls, "planetary disk detection",
      [&, ctx = CTL_CONTEXT(ctx, _planetary_disk_opts)]() {
        ctlbind(ctls, ctx);
      });
}

bool c_planetary_disk_detection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _orientation_method);
    SERIALIZE_OPTION(settings, save, *this, updateROI);
    if ( auto group = SERIALIZE_GROUP(settings, save, "planetary_disk_detection") ) {
      serialize_base_planetary_disk_detector_options(group, save, _planetary_disk_opts);
    }
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
          _planetary_disk_opts,
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
    ApodizationWindow = fftCreateCircularApodizationWindow(fftSize);
  }

  CF_DEBUG("rc={x=%d y=%d w=%d h=%d} ApodizationWindow: %dx%d grayImage: %dx%d",
      rc.x, rc.y, rc.width, rc.height, ApodizationWindow.cols, ApodizationWindow.rows,
      grayImage.cols, grayImage.rows);

  cv::multiply(grayImage(rc), ApodizationWindow, grayImage);

  double orientation_angle = 0;

  if ( _orientation_method == ORIENTATION_RADON_STENSOR ) {

    orientation_angle =
        gradientEstimateRadonOrientation(grayImage);
  }
  else {

    if( VLAP.size() != fftSize ) {
      VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
    }

    fftPPSDecomposition(grayImage, VLAP,
        INTENSITY_P, cv::noArray(),
        true);

    fftSpectrumModule(INTENSITY_P, INTENSITY_Magnitude);

    orientation_angle =
        fftEstimateRadonOrientation(INTENSITY_Magnitude);
  }

  if( orientation_angle > 90 ) {
    orientation_angle -= 180;
  }
  else if( orientation_angle < -90 ) {
    orientation_angle += 180;
  }

  CF_DEBUG("\n -> Detected orientation angle: %g°", orientation_angle );

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
    rrc.size.width = fftSize.width;
    rrc.size.height = fftSize.height / 2;
    rrc.angle = orientation_angle;
    rrc.center.x = geometrical_center.x;
    rrc.center.y = geometrical_center.y;

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

  return fOk;
}

