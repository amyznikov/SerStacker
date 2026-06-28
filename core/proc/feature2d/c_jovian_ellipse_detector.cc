/*
 * c_jovian_ellipse_detector.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector.h"
#include <core/proc/autoclip.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/pyrscale.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/pyrscale.h>
#include <core/proc/gradient.h>
#include <core/proc/project_to_radius_vector.h>
#include <core/proc/fitEllipseLM.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/settings/opencv_settings.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<JOVIAN_ELLIPSE_DETECTION_METHOD>()
{
  static const c_enum_member members[] = {
      { JOVIAN_ELLIPSE_DETECTION_RADON_FFT, "RADON_FFT", "" },
      { JOVIAN_ELLIPSE_DETECTION_STENSOR, "STENSOR", "" },
      { JOVIAN_ELLIPSE_DETECTION_RADON_FFT }
  };

  return members;
}

static void pnormalize(cv::InputArray _src, cv::OutputArray _dst, int lvl)
{
  const cv::Mat src = _src.getMat();
  const cv::Size src_size = _src.size();

  cv::Mat m;
  pyramid_downscale(src, m, lvl, cv::BORDER_REPLICATE);
  pyramid_upscale(m, src_size);
  cv::subtract(src, m, _dst, cv::noArray(), CV_32F);
}

// 2022-08-09-2336_8-CapObj-32F
// Compute first-order derivatives
static void differentiate(cv::InputArray _src, cv::OutputArray gx, cv::OutputArray gy, double gsigma = 0)
{
  static float deriv_kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(deriv_kernel);
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(deriv_kernel);

  if( !(gsigma > 0) ) {
    cv::filter2D(_src, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(_src, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(_src, tmp, cv::Size(0, 0), gsigma, gsigma, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
}

static void extract_points(const cv::Mat1f & image, const cv::Mat1b & mask,
    std::vector<cv::Point3f> & pts, double threshold,  bool weighted = true)
{
  const int cx = image.cols / 2;
  const int cy = image.rows / 2;

  for( int y = 0; y < image.rows; ++y ) {
    const float * srcp = image[y];
    const uint8_t * mp = mask[y];
    for( int x = 0; x < image.cols; ++x ) {
      if( mp[x] ) {
        const double w = srcp[x] - threshold;
        if( w > 0 ) {
          const double r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
          pts.emplace_back(x, y, r2 * ( weighted ? w * w : w) );
        }
      }
    }
  }
}

void c_jovian_ellipse_detector::clear()
{
  _grayscaleInputImage.release();
  _grayscaleImageCrop.release();
  _gx.release();
  _gy.release();
  _g.release();
  _gr.release();
  _grth.release();
  _planetaryDiskMask.release();
  _skirtMask.release();
  _edge_points.clear();
  _final_planetary_disk_mask.release();
  _apodizationWindow.release();
  _radonMagnitude.release();
  VLAP.release();
}


bool c_jovian_ellipse_detector::detect(cv::InputArray inputImage, cv::InputArray inputMask)
{
  INSTRUMENT_REGION("");

  //
  // Convert input image to CV_32F gray scale

  const cv::Mat src = inputImage.getMat();
  const cv::Size srcSize = src.size();

  if ( src.type() == CV_32FC1 ) {
    _grayscaleInputImage = src;
  }
  else if( src.channels() == 1 ) {
    src.convertTo(_grayscaleInputImage, CV_32F);
  }
  else {
    cv::cvtColor(src, _grayscaleInputImage, cv::COLOR_BGR2GRAY);
    _grayscaleInputImage.convertTo(_grayscaleInputImage, CV_32F);
  }


  //
  // Detect planetary disk and extract pixel mask covering planetary disk shape

  const bool planetaryDiskDetected =
      simple_planetary_disk_detector(_grayscaleInputImage, inputMask,
          _opts.planetary_disk_detector_options,
          &_planetaryDiskCentroid,
          &_planetaryDiskROI,
          &_planetaryDiskMask,
          &_planetaryDiskGeometricalCenter);

  if( !planetaryDiskDetected ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  //
  // Crop planetary disk ROI into separate array for apodization
  // The same crop size is used for all of the detection methods
  // Prepare also skirt mask and radial gradient to include the ellipse edge only

  _cropRC = fftGetOptimalSquaredROI(srcSize,  _planetaryDiskROI);
  const cv::Size cropSize = _cropRC.size();

  CF_DEBUG("CROP = {x=%d y=%d w=%d h=%d}", _cropRC.x, _cropRC.y, _cropRC.width, _cropRC.height);

  if (_apodizationWindow.size() != cropSize ) {
    _apodizationWindow = fftGenerateButterworthFilter(cropSize, 0.325 * CV_2PI, 10);
  }

  _grayscaleInputImage(_cropRC).copyTo(_grayscaleImageCrop);
  differentiate(_grayscaleImageCrop, _gx, _gy, _opts.sigma_contour);

  const cv::Point skirtCenter(cropSize.width / 2, cropSize.height / 2);
  static const cv::Mat1b THSE(5, 5, uint8_t(255));
  project_to_radius_vector(skirtCenter, _gx, _gy, _gr, cv::noArray(), -1e4);
  cv::max(_gr, 0.f, _grth);
  cv::morphologyEx(_grth, _grth, cv::MORPH_TOPHAT, THSE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  cv::multiply(_grayscaleImageCrop, _apodizationWindow, _grayscaleImageCrop);


  //
  // Detect Jovian orientation using one from available methods

  double ellipse_angle_deg = 0;

  switch (_opts.method) {
    case JOVIAN_ELLIPSE_DETECTION_RADON_FFT:
      ellipse_angle_deg = compute_jovian_orientation_radon_fft();
      break;
    case JOVIAN_ELLIPSE_DETECTION_STENSOR:
      ellipse_angle_deg = compute_jovian_orientation_stensor();
      break;
    default:
      CF_ERROR("APP BUG: Not supported method %d requested", _opts.method);
      return false;
  }
  if( ellipse_angle_deg > 90 ) {
    ellipse_angle_deg -= 180;
  }
  else if( ellipse_angle_deg < -90 ) {
    ellipse_angle_deg += 180;
  }

  CF_DEBUG("\nORIENTATION: %s angle = %g deg", toCString(_opts.method), ellipse_angle_deg);


  // Use orientation from above to improve jovian disk fit
  // based on extracted planetary disk edge

  constexpr double axis_ratio = k_jovian_axis_ratio;
  const double tilt_to_earth = _opts.planetary_disk_tilt * CV_PI / 180;
  const double cos_tilt = cos(tilt_to_earth);
  const double sin_tilt = sin(tilt_to_earth);
  const double fixed_axis_ratio = std::sqrt(axis_ratio * axis_ratio * cos_tilt * cos_tilt + sin_tilt * sin_tilt);



  double skirtThreshold = 0;

  for ( int iteration = 0; iteration < _opts.skirt_iterations; ++iteration ) {
    int skirtSize, skirtRadius;

    _skirtMask.create(cropSize), _skirtMask.setTo(0);

    if ( iteration == 0 ) {
      skirtSize = 2 * std::max(11, 2 * (cropSize.width / 32) + 1) + 1;
      skirtRadius = cropSize.width / 2 - skirtSize / 2;
      cv::circle(_skirtMask, skirtCenter, skirtRadius, cv::Scalar::all(255), skirtSize, cv::LINE_8);

      cv::Scalar mv, sv;
      cv::meanStdDev(_grth, mv, sv, _skirtMask);
      skirtThreshold = mv[0];
      CF_DEBUG("skirt: mean=%g stdev=%g", mv[0], sv[0]);
    }
    else {
      skirtSize = 31;
      cv::ellipse(_skirtMask, _final_planetary_disk_ellipse, cv::Scalar::all(255), skirtSize, cv::LINE_8);
    }

    _edge_points.clear();
    extract_points(_grth, _skirtMask, _edge_points, skirtThreshold, _opts.lmweighted);
    CF_DEBUG("[iteration %d] edge_points.size=%zu skirtSize=%d skirtRadius=%d", iteration, _edge_points.size(), skirtSize, skirtRadius);

    fitEllipseLMW(_edge_points,
        fixed_axis_ratio, // b / a
        ellipse_angle_deg * CV_PI / 180, // radians
        &_final_planetary_disk_ellipse);

  }

  // Add optional offset if requested by user
  _final_planetary_disk_ellipse.center.x += _cropRC.x + _opts.offset.x;
  _final_planetary_disk_ellipse.center.y += _cropRC.y + _opts.offset.y;

  // Create also filed binary ellipse mask
  draw_ellipse_mask(_final_planetary_disk_mask, srcSize, _final_planetary_disk_ellipse);

  const double A = _final_planetary_disk_ellipse.size.width / 2;
  const double B = _final_planetary_disk_ellipse.size.width * axis_ratio / 2;
  const double C = _final_planetary_disk_ellipse.size.width / 2;
  _center = _final_planetary_disk_ellipse.center;
  _axes = cv::Vec3d(A, B, C);
  _pose = build_ellipsoid_pose(0., _opts.planetary_disk_tilt * CV_PI / 180,
      _final_planetary_disk_ellipse.angle * CV_PI / 180);

  if( true ) {
    const std::string s = serialize_ellipsoid_to_string(_center, _axes, _pose * 180 / CV_PI);
    CF_DEBUG("\nfitEllipseLMW:\n" "%s\n", s.c_str());
  }

  return true;
}

double c_jovian_ellipse_detector::compute_jovian_orientation_radon_fft()
{
  cv::Mat2f INTENSITY_P;

  const cv::Size cropSize = _cropRC.size();
  if( VLAP.size() != cropSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(cropSize, true);
  }

  fftPPSDecomposition(_grayscaleImageCrop, VLAP,
      INTENSITY_P, cv::noArray(),
      true);

  fftSpectrumModule(INTENSITY_P, _radonMagnitude);

  const double angle =
      fftEstimateRadonOrientation(_radonMagnitude);

  CF_DEBUG(" -> Detected Polar Axis Position Angle: %g°", angle );

  return angle;
}


double c_jovian_ellipse_detector::compute_jovian_orientation_stensor()
{
  // Tensor components:Jxx = gx*gx, Jyy = gy*gy, Jxy = gx*gy
  // Analytical computation of the angle of least change (direction ALONG the stripes).
  // The formula calculates the angle of the leading eigenvector (normal),
  // so to rotate along the equator, we add 90 degrees.

  cv::Mat1f jxx, jyy, jxy;
  cv::Mat1f scale_map;
  cv::Scalar mean_g, stdev_g;

  if ( _opts.nscale > 0 ) {
    pnormalize(_grayscaleImageCrop, _grayscaleImageCrop, _opts.nscale);
  }

  differentiate(_grayscaleImageCrop, _gx, _gy, _opts.sigma_clouds);

  cv::magnitude(_gx, _gy, _g);
  cv::meanStdDev(_g, mean_g, stdev_g);
  cv::min(_g, mean_g[0] + 3 * stdev_g[0], scale_map);
  cv::divide(scale_map, _g + 1e-3 * stdev_g[0], scale_map);

  cv::multiply(_gx, scale_map, _gx);
  cv::multiply(_gy, scale_map, _gy);
  cv::multiply(_g, scale_map, _g);

  if( _opts.gweighted ) {
    cv::multiply(_gx, _g, _gx);
    cv::multiply(_gy, _g, _gy);
  }

  cv::multiply(_gx, _gx, jxx);
  cv::multiply(_gy, _gy, jyy);
  cv::multiply(_gx, _gy, jxy);

  const double Jxx = cv::sum(jxx)[0];
  const double Jyy = cv::sum(jyy)[0];
  const double Jxy = cv::sum(jxy)[0];

  double angle = 0.5 * std::atan2(2.0 * Jxy, Jxx - Jyy) * 180.0 / CV_PI + 90.0;
  if( angle > 180.0 ) {
    angle -= 360.0;
  }
  if( angle < -180.0 ) {
    angle += 360.0;
  }

  return angle;
}

bool serialize_base_jovian_ellipse_detector_options(c_config_setting section, bool save,
    c_jovian_ellipse_detector_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, sigma_contour);
  SERIALIZE_OPTION(section, save, opts, sigma_clouds);
  SERIALIZE_OPTION(section, save, opts, nscale);
  SERIALIZE_OPTION(section, save, opts, skirt_iterations);
  SERIALIZE_OPTION(section, save, opts, neps);
  SERIALIZE_OPTION(section, save, opts, gweighted);
  SERIALIZE_OPTION(section, save, opts, lmweighted);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_tilt);
  SERIALIZE_OPTION(section, save, opts, offset);
  serialize_base_planetary_disk_detector_options(section, save, opts.planetary_disk_detector_options);
  return true;
}
