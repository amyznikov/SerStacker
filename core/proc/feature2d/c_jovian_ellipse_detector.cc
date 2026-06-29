/*
 * c_jovian_ellipse_detector.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector.h"
#include <core/proc/fft.h>
#include <core/proc/morphology.h>
#include <core/proc/pyrscale.h>
#include <core/proc/project_to_radius_vector.h>
#include <core/proc/feature2d/ellipsoid.h>
#include <core/proc/fitEllipseLM.h>
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

    const int border = cvRound(gsigma * 4.0) + 2;
    const cv::Rect rc(border, border, _src.cols(), _src.rows());

    cv::copyMakeBorder(_src, tmp, border, border, border, border, cv::BORDER_REPLICATE);
    cv::GaussianBlur(tmp, tmp, cv::Size(0, 0), gsigma, gsigma, cv::BORDER_REPLICATE);
    cv::filter2D(tmp(rc), gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(tmp(rc), gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
}

static void skirtToPolar(cv::InputArray srcImage, cv::OutputArray dstImage, cv::OutputArray outputMask2D,
    const cv::RotatedRect & baseEllipse,
    int skirtIndentInside,              // Indentation INSIDE the outline
    int skirtIndentOutside )            // Indentation OUTWARD the outline
{
  const cv::Mat src = srcImage.getMat();
  if( src.empty() ) {
    dstImage.release();
    outputMask2D.release();
    return;
  }

  const cv::Point2f center = baseEllipse.center;
  const double A_base = baseEllipse.size.width / 2.0;
  const double B_base = baseEllipse.size.height / 2.0;
  const double axisRatio = (A_base > 1e-4) ? (B_base / A_base) : 1.0;
  const double angle_rad = baseEllipse.angle * CV_PI / 180.0;
  const double cos_alpha = std::cos(angle_rad);
  const double sin_alpha = std::sin(angle_rad);

  // azimuths along rows, distances along columns
  const int polarWidth = std::max(1, skirtIndentOutside - skirtIndentInside + 1);
  const double max_A = A_base + skirtIndentOutside;
  const int polarHeight = cvRound(2.0 * CV_PI * max_A);

  dstImage.create(polarHeight, polarWidth, src.type());
  cv::Mat1f dst = dstImage.getMat();

  outputMask2D.create(src.size(), CV_8UC1);
  cv::Mat1b mask2D = outputMask2D.getMat();
  mask2D.setTo(0);

  const double dw = double(skirtIndentOutside - skirtIndentInside) / std::max(1, polarWidth - 1);
  const double dtheta = 2.0 * CV_PI / polarHeight;

  cv::parallel_for_(cv::Range(0, polarHeight),
      [=, &dst, &src, &mask2D](const cv::Range & range) {

        std::vector<float> mapX(polarWidth);
        std::vector<float> mapY(polarWidth);

        for (int y = range.start; y < range.end; ++y) {
          const double theta = y * dtheta;
          const double cos_t = std::cos(theta);
          const double sin_t = std::sin(theta);
          const double r_base = (A_base * axisRatio) / std::sqrt(axisRatio * axisRatio * cos_t * cos_t + sin_t * sin_t);

          for (int x = 0; x < polarWidth; ++x) {
            const double w = skirtIndentInside + x * dw;
            const double r_current = r_base + w;

            const double local_x = r_current * cos_t;
            const double local_y = r_current * sin_t;

            const float px = float(center.x + local_x * cos_alpha - local_y * sin_alpha);
            const float py = float(center.y + local_x * sin_alpha + local_y * cos_alpha);

            mapX[x] = px;
            mapY[x] = py;

            const int ix = cvRound(px);
            const int iy = cvRound(py);
            if (ix >= 0 && ix < mask2D.cols && iy >= 0 && iy < mask2D.rows) {
              mask2D(iy, ix) = 255;
            }
          }

          cv::Mat mapX_row(1, polarWidth, CV_32FC1, mapX.data());
          cv::Mat mapY_row(1, polarWidth, CV_32FC1, mapY.data());
          cv::Mat dst_row = dst.row(y);

          cv::remap(src, dst_row, mapX_row, mapY_row, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        }
      });
}

static void edgePointsFromPolar(const cv::Mat1f & polarSkirtProfile,
    const cv::RotatedRect & baseEllipse,
    int skirtIndentInside,              // Indentation INSIDE the outline
    int skirtIndentOutside,             // Indentation OUTWARD the outline
    std::vector<cv::Point3f> & pts)
{
  if( polarSkirtProfile.empty() ) {
    return;
  }

  const int width = polarSkirtProfile.cols;
  const int height = polarSkirtProfile.rows;

  const cv::Point2f center = baseEllipse.center;
  const double A_base = baseEllipse.size.width / 2.0;
  const double B_base = baseEllipse.size.height / 2.0;
  const double axisRatio = (A_base > 1e-4) ? (B_base / A_base) : 1.0;

  const double angle_rad = baseEllipse.angle * CV_PI / 180.0;
  const double cos_alpha = std::cos(angle_rad);
  const double sin_alpha = std::sin(angle_rad);

  const double dw = double(skirtIndentOutside - skirtIndentInside) / std::max(1, width - 1);
  const double dtheta = 2.0 * CV_PI / height;

  // Fixed size because for parallel execution
  pts.resize(height);

  // Search for the global maximum in a ROW (along distances)
  // azimuths along rows, distances along columns
  cv::parallel_for_(cv::Range(0, height),
      [=, &polarSkirtProfile, &pts](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {

          const float* srcp = polarSkirtProfile[y];

          float max_val = 0;
          int best_x = -1;
          for (int x = 0; x < width; ++x) {
            const float val = srcp[x];
            if (val > max_val ) {
              max_val = val;
              best_x = x;
            }
          }

          // If the maximum is on the edge or is not valid, we write zero weight
          if (best_x < 0 || best_x >= width ) {
            pts[y] = cv::Point3f(0,0,0);
            continue;
          }

          // Subpixel parabolic interpolation along the line (along the X coordinate)
          const float alpha = srcp[best_x - 1];
          const float beta = srcp[best_x];
          const float gamma = srcp[best_x + 1];

          double sub_x = best_x;
          if (beta > alpha && beta > gamma) {
            double denominator = 2.0 * (alpha - 2.0 * beta + gamma);
            if (std::abs(denominator) > 1e-6) {
              sub_x += (alpha - gamma) / denominator;
            }
          }

          const double w_current = skirtIndentInside + sub_x * dw;
          const double theta = y * dtheta;
          const double cos_t = std::cos(theta);
          const double sin_t = std::sin(theta);
          const double r_base = (A_base * axisRatio) / std::sqrt(axisRatio * axisRatio * cos_t * cos_t + sin_t * sin_t);
          const double r_current = r_base + w_current;
          const double local_x = r_current * cos_t;
          const double local_y = r_current * sin_t;
          const float real_x = float(center.x + local_x * cos_alpha - local_y * sin_alpha);
          const float real_y = float(center.y + local_x * sin_alpha + local_y * cos_alpha);

          pts[y] = cv::Point3f(real_x, real_y, max_val * max_val);
        }
      });
}

static void edgePointsFromPolar2(const cv::Mat1f & polarSkirtProfile,
    const cv::RotatedRect & baseEllipse,
    int skirtIndentInside,              // Indentation INSIDE the outline
    int skirtIndentOutside,             // Indentation OUTWARD the outline
    std::vector<cv::Point3f> & pts)
{
  if (polarSkirtProfile.empty()) {
    return;
  }

  // azimuths along rows, distances along columns

  const int width = polarSkirtProfile.cols;
  const int height = polarSkirtProfile.rows;
  const cv::Point2f center = baseEllipse.center;
  const double A_base = baseEllipse.size.width / 2.0;
  const double B_base = baseEllipse.size.height / 2.0;
  const double axisRatio = (A_base > 1e-4) ? (B_base / A_base) : 1.0;

  const double angle_rad = baseEllipse.angle * CV_PI / 180.0;
  const double cos_alpha = std::cos(angle_rad);
  const double sin_alpha = std::sin(angle_rad);

  const double dw = double(skirtIndentOutside - skirtIndentInside) / std::max(1, width - 1);
  const double dtheta = 2.0 * CV_PI / height;

  pts.clear();
  pts.reserve(height * width / 2);

  for (int y = 0; y < height; ++y) {
    const double theta = y * dtheta;
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    const double r_base = (A_base * axisRatio) / std::sqrt(axisRatio * axisRatio * cos_t * cos_t + sin_t * sin_t);

    const float* srcp = polarSkirtProfile[y];

    for (int x = 0; x < width; ++x) {
      const float w = srcp[x];
      if (w > 0 ) {
        const double w_current = skirtIndentInside + x * dw;
        const double r_current = r_base + w_current;
        const double local_x = r_current * cos_t;
        const double local_y = r_current * sin_t;

        const float real_x = float(center.x + local_x * cos_alpha - local_y * sin_alpha);
        const float real_y = float(center.y + local_x * sin_alpha + local_y * cos_alpha);
        pts.emplace_back(real_x, real_y, w * w);
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
  _finalPlanetaryDiskMask.release();
  _apodizationWindow.release();
  _radonMagnitude.release();
  VLAP.release();
  _skirtPolar.release();
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
  const cv::Point2f coarseSkirtCenter(cropSize.width / 2.f, cropSize.height / 2.f);

  _grayscaleInputImage(_cropRC).copyTo(_grayscaleImageCrop);

  CF_DEBUG("CROP = {x=%d y=%d w=%d h=%d}", _cropRC.x, _cropRC.y, _cropRC.width, _cropRC.height);

  //
  // Prepare skirt data for contour detection.
  // This must be done before applying the apodization to _grayscaleImageCrop

  differentiate(_grayscaleImageCrop, _gx, _gy, 0);
  project_to_radius_vector(coarseSkirtCenter, _gx, _gy, _gr, cv::noArray(), -1e4);

  if( _enableDebugImages ) {
    static const cv::Mat1b THSE(5, 5, uint8_t(255));
    cv::GaussianBlur(_gr, _grth, cv::Size(0, 0), 5, 5, cv::BORDER_REPLICATE);
    cv::max(_grth, 0.f, _grth);
    cv::morphologyEx(_grth, _grth, cv::MORPH_TOPHAT, THSE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  }

  // Apply apodization window for orientation detection
  if (_apodizationWindow.size() != cropSize ) {
    _apodizationWindow = fftGenerateButterworthFilter(cropSize, 0.325 * CV_2PI, 10);
  }
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

  //
  // Use orientation and iniutial skirt from above to improve jovian disk fit
  // using points from extracted planetary disk edge
  CF_DEBUG("\nORIENTATION: %s angle = %g deg", toCString(_opts.method), ellipse_angle_deg);

  constexpr double axis_ratio = k_jovian_axis_ratio;
  const double tilt_to_earth = _opts.planetary_disk_tilt * CV_PI / 180;
  const double cos_tilt = cos(tilt_to_earth);
  const double sin_tilt = sin(tilt_to_earth);
  const double fixed_axis_ratio = std::sqrt(axis_ratio * axis_ratio * cos_tilt * cos_tilt + sin_tilt * sin_tilt);

  const double R_estimated = 0.45 * cropSize.width;
  const int coarseSkirtIndentInside = -cvRound(R_estimated * 0.2);
  const int coarseSkirtIndentOutside =  cvRound(R_estimated * 0.02);
  const cv::RotatedRect coarseSkirtEllipse(coarseSkirtCenter, cropSize, 0.0f);

  skirtToPolar(_gr, _skirtPolar, _skirtMask, coarseSkirtEllipse,
      coarseSkirtIndentInside, coarseSkirtIndentOutside);

  cv::GaussianBlur(_skirtPolar, _skirtPolar, cv::Size(5, 15),
      0, 0, cv::BORDER_DEFAULT);

  edgePointsFromPolar(_skirtPolar, coarseSkirtEllipse,
      coarseSkirtIndentInside, coarseSkirtIndentOutside,
      _edge_points);

  fitEllipseLMW(_edge_points,
      fixed_axis_ratio, // b / a
      ellipse_angle_deg * CV_PI / 180, // radians
      &_finalPlanetaryDiskEllipse);

  if ( _opts.skirt_iterations > 1 ) {

    static const cv::Mat1b TOPHAT_SE(3, 7, uint8_t(255));

    const int fineSkirtIndentInside = -9;
    const int fineSkirtIndentOutside = +9;
    const cv::RotatedRect fineSkirtEllipse = _finalPlanetaryDiskEllipse;

    skirtToPolar(_gr, _skirtPolar, _skirtMask, fineSkirtEllipse,
        fineSkirtIndentInside, fineSkirtIndentOutside);

    cv::GaussianBlur(_skirtPolar, _skirtPolar, cv::Size(3, 11),
        0, 0, cv::BORDER_DEFAULT);

    cv::max(_skirtPolar, 0.f, _skirtPolar);
    cv::morphologyEx(_skirtPolar, _skirtPolar, cv::MORPH_TOPHAT, TOPHAT_SE,
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);

    edgePointsFromPolar2(_skirtPolar, fineSkirtEllipse,
        fineSkirtIndentInside, fineSkirtIndentOutside,
        _edge_points);

    fitEllipseLMW(_edge_points,
        fixed_axis_ratio, // b / a
        ellipse_angle_deg * CV_PI / 180, // radians
        &_finalPlanetaryDiskEllipse);
  }

  // Add optional offset if requested by user
  _finalPlanetaryDiskEllipse.center.x += _cropRC.x + _opts.offset.x;
  _finalPlanetaryDiskEllipse.center.y += _cropRC.y + _opts.offset.y;

  // Create also filed binary ellipse mask
  draw_ellipse_mask(_finalPlanetaryDiskMask, srcSize, _finalPlanetaryDiskEllipse);

  const double A = _finalPlanetaryDiskEllipse.size.width / 2;
  const double B = _finalPlanetaryDiskEllipse.size.width * axis_ratio / 2;
  const double C = _finalPlanetaryDiskEllipse.size.width / 2;
  _center = _finalPlanetaryDiskEllipse.center;
  _axes = cv::Vec3d(A, B, C);
  _pose = build_ellipsoid_pose(0., _opts.planetary_disk_tilt * CV_PI / 180,
      _finalPlanetaryDiskEllipse.angle * CV_PI / 180);

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

  if ( _opts.nscale > 0 ) {
    pnormalize(_grayscaleImageCrop, _grayscaleImageCrop, _opts.nscale);
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
  cv::multiply(_gx, _g, _gx);
  cv::multiply(_gy, _g, _gy);

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
  SERIALIZE_OPTION(section, save, opts, sigma_clouds);
  SERIALIZE_OPTION(section, save, opts, nscale);
  SERIALIZE_OPTION(section, save, opts, skirt_iterations);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_tilt);
  SERIALIZE_OPTION(section, save, opts, offset);
  serialize_base_planetary_disk_detector_options(section, save, opts.planetary_disk_detector_options);
  return true;
}
