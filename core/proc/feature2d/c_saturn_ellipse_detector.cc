/*
 * c_saturn_ellipse_detector.cc
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#include "c_saturn_ellipse_detector.h"
#include <core/proc/pixtype.h>
#include <core/proc/feature2d/ellipsoid.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/project_to_radius_vector.h>
#include <core/proc/fitEllipseLM.h>
#include <core/settings/opencv_settings.h>
#include <core/debug.h>


namespace {

// (Weighted PCA)
class PCAW
{
public:
  cv::Point2d mean;           // Weighted center of mass
  cv::Vec2d eigenvalues;      // Eigenvalues ​​(0 - max, 1 - min)
  cv::Matx22d eigenvectors;   // The rows of the matrix are the eigenvectors (0 - along the rings, 1 - across)

  PCAW() : mean(0, 0), eigenvalues(0, 0), eigenvectors(cv::Matx22d::zeros())
  {
  }

  /**
   * @brief Accumulate weighted moments and compute the PCAW basis
   * */
  template<typename T>
  bool compute(const cv::Mat_<T> & image, const cv::Mat1b & mask, const cv::Rect & roi)
  {
    const cv::Rect safe_roi = roi & cv::Rect(0, 0, image.cols, image.rows);
    if( safe_roi.empty() ) {
      return false;
    }

    double sum_w = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    // Find the weighted barycenter (mean)
    for( int y = safe_roi.y; y < safe_roi.y + safe_roi.height; ++y ) {
      const T * srcp = image[y];
      const uint8_t * mskp = mask[y];
      for( int x = safe_roi.x; x < safe_roi.x + safe_roi.width; ++x ) {
        if( mskp[x] ) {
          double w = srcp[x];
          sum_w += w;
          sum_x += w * x;
          sum_y += w * y;
        }
      }
    }

    if( sum_w < FLT_EPSILON ) {
      return false;
    }

    mean.x = sum_x / sum_w;
    mean.y = sum_y / sum_w;

    // Central weighted moments (covariance)
    double mu20 = 0.0;
    double mu02 = 0.0;
    double mu11 = 0.0;

    for( int y = safe_roi.y; y < safe_roi.y + safe_roi.height; ++y ) {
      const T * srcp = image[y];
      const uint8_t * mskp = mask[y];
      for( int x = safe_roi.x; x < safe_roi.x + safe_roi.width; ++x ) {
        if( mskp[x] ) {
          double w = srcp[x];
          double dx = x - mean.x;
          double dy = y - mean.y;
          mu20 += w * dx * dx;
          mu02 += w * dy * dy;
          mu11 += w * dx * dy;
        }
      }
    }

    mu20 /= sum_w;
    mu02 /= sum_w;
    mu11 /= sum_w;

    // Analytical solution of the characteristic equation (SVD/Eigen 2x2)
    // Trace and determinant of the covariance matrix
    const double trace = mu20 + mu02;
    const double det = mu20 * mu02 - mu11 * mu11;
    const double disc = std::sqrt(std::max(0.0, trace * trace - 4.0 * det));

    // eigenvalues ​​(algebraic dispersion along the axes)
    eigenvalues[0] = 0.5 * (trace + disc); // Maximum (along the major axis)
    eigenvalues[1] = 0.5 * (trace - disc); // Minimum (across the major axis)

    // Slope angle of the first principal component
    const double angle = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);

    // Orthonormal eigenvectors (as strings, similar to cv::PCA)
    eigenvectors(0, 0) = std::cos(angle);  // ev0.x
    eigenvectors(0, 1) = std::sin(angle);  // ev0.y
    eigenvectors(1, 0) = -std::sin(angle); // ev1.x (perpendicular)
    eigenvectors(1, 1) = std::cos(angle);  // ev1.y

    return true;
  }
};
}

template<class _Tp>
static bool _detect_primary_orientation_pcaw(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi,
    cv::RotatedRect * outrc)
{
  const cv::Mat_<_Tp> src = image.getMat();
  const cv::Mat1b msk = mask.getMat();

  PCAW pcaw;
  if (!pcaw.compute(src, msk, roi)) {
      return false;
  }

  // Angle in degrees from the first eigenvector (line 0)
  double angle_deg = std::atan2(pcaw.eigenvectors(0, 1), pcaw.eigenvectors(0, 0)) * 180.0 / CV_PI;

  // Dimensions (Width and Height) by projecting the mask geometry onto a new basis.
  // One quick pass over the mask.
  double max_dim1 = -DBL_MAX, min_dim1 = DBL_MAX;
  double max_dim2 = -DBL_MAX, min_dim2 = DBL_MAX;

  const cv::Rect safe_roi = roi & cv::Rect(0, 0, msk.cols, msk.rows);
  for (int y = safe_roi.y; y < safe_roi.y + safe_roi.height; ++y) {
    const uint8_t * mskp = msk[y];
    for (int x = safe_roi.x; x < safe_roi.x + safe_roi.width; ++x) {
      if (mskp[x]) {
        double dx = x - pcaw.mean.x;
        double dy = y - pcaw.mean.y;

        // Projection onto the first axis (along the rings, line 0)
        const double proj1 = dx * pcaw.eigenvectors(0, 0) + dy * pcaw.eigenvectors(0, 1);
        if (proj1 > max_dim1) {
          max_dim1 = proj1;
        }
        if (proj1 < min_dim1) {
          min_dim1 = proj1;
        }

        // Projection onto the second axis (across the rings, line 1)
        const double proj2 = dx * pcaw.eigenvectors(1, 0) + dy * pcaw.eigenvectors(1, 1);
        if (proj2 > max_dim2) {
          max_dim2 = proj2;
        }
        if (proj2 < min_dim2) {
          min_dim2 = proj2;
        }
      }
    }
  }

  if (outrc) {
    outrc->center = cv::Point2f(static_cast<float>(pcaw.mean.x), static_cast<float>(pcaw.mean.y));
    outrc->size.width  = static_cast<float>(max_dim1 - min_dim1);
    outrc->size.height = static_cast<float>(max_dim2 - min_dim2);
    outrc->angle = static_cast<float>(angle_deg);
  }

  return true;
}

static bool detect_primary_orientation_pcaw(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi,
    cv::RotatedRect * outrc)
{
  CV_DISPATCH(image.depth(), _detect_primary_orientation_pcaw, image, mask, roi, outrc);
  return true;
}

// Compute first-order derivatives
static void differentiate(cv::InputArray _src, cv::OutputArray gx, cv::OutputArray gy, double gbsigma = 0)
{
  static float deriv_kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(deriv_kernel);
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(deriv_kernel);

  if( !(gbsigma > 0) ) {
    cv::filter2D(_src, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(_src, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(_src, tmp, cv::Size(0, 0), gbsigma, gbsigma, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
}

static bool extract_points(const cv::Mat1f & gr_image, const cv::Mat1b & mask,
    const cv::RotatedRect & roi, std::vector<cv::Point3f> & pts, bool weighted = true)
{
  pts.clear();

  const cv::Rect safe_roi = roi.boundingRect() & cv::Rect(0, 0, gr_image.cols, gr_image.rows);
  if( safe_roi.empty() ) {
    return false;
  }

  const double xc = roi.center.x;
  const double yc = roi.center.y;
  const double alpha = roi.angle * CV_PI / 180.0;
  const double sa = std::sin(alpha);
  const double ca = std::cos(alpha);
  const double max_xr = roi.size.width / 2.0 + 1.0;
  const double max_yr = roi.size.height / 2.0 + 1.0;
  const int npts = cv::countNonZero(mask(safe_roi));
  pts.reserve(npts);

  // Pixels inside the ROI
  // Transform current point into the local coordinate system of the oriented ROI
  // Double geometric filter: the point must be inside the red rectangle
  for( int y = safe_roi.y; y < safe_roi.y + safe_roi.height; ++y ) {
    const float * grp = gr_image[y];
    const uint8_t * mskp = mask[y];
    const double dy = y - yc;
    for( int x = safe_roi.x; x < safe_roi.x + safe_roi.width; ++x ) {
      if( mskp[x] ) {
        const double dx = x - xc;
        const double xr = dx * ca + dy * sa;
        const double yr = -dx * sa + dy * ca;
        if( std::abs(xr) <= max_xr && std::abs(yr) <= max_yr ) {
          pts.emplace_back(x,y, weighted ? grp[x] : 1.f);
        }
      }
    }
  }

  return pts.size() > 5;
}

void c_saturn_ellipse_detector::set_options(const c_saturn_ellipse_detector_options & opts)
{
  _opts = opts;
}

const c_saturn_ellipse_detector_options& c_saturn_ellipse_detector::options() const
{
  return _opts;
}

const cv::Mat& c_saturn_ellipse_detector::grayscale_image() const
{
  return _grayscale_image;
}

const cv::Mat1b& c_saturn_ellipse_detector::initial_mask() const
{
  return _initial_mask;
}

const cv::Mat1b& c_saturn_ellipse_detector::pca_mask() const
{
  return _pca_mask;
}

const cv::RotatedRect & c_saturn_ellipse_detector::pca_rect() const
{
  return _pca_rect;
}

const cv::Mat1b & c_saturn_ellipse_detector::skirt_mask() const
{
  return _skirt_mask;
}

const cv::Mat& c_saturn_ellipse_detector::gradient_image() const
{
  return _gradient_image;
}

const cv::Mat1f & c_saturn_ellipse_detector::gx_image() const
{
  return _gx;
}
const cv::Mat1f & c_saturn_ellipse_detector::gy_image() const
{
  return _gy;
}
const cv::Mat1f & c_saturn_ellipse_detector::gr_image() const
{
  return _gr;
}
const cv::Mat1f & c_saturn_ellipse_detector::grth_image() const
{
  return _grth;
}

const cv::RotatedRect & c_saturn_ellipse_detector::skirt_roi() const
{
  return _skirt_roi;
}

bool c_saturn_ellipse_detector::detect(cv::InputArray image, cv::InputArray mask)
{
  INSTRUMENT_REGION("");

  if ( !detect_initial_mask(image, mask) ) {
    CF_ERROR("detect_total_mask() fails");
    return false;
  }

  if ( !detect_pca_rect() ) {
    CF_ERROR("detect_pca_rect() fails");
    return false;
  }

  CF_DEBUG("\n"
      "PCA RECT: {\n"
      " center = %g; %g\n"
      " axes = %g; %g\n"
      " angle = %g deg\n"
      "}",
      _pca_rect.center.x, _pca_rect.center.y,
      _pca_rect.size.width, _pca_rect.size.height,
      _pca_rect.angle);

  if ( !compute_radial_gradient(image, mask) ) {
    CF_ERROR("compute_radial_gradient() fails");
    return false;
  }

  // Use orientation from PCA above to compute
  // planetary disk fit based on extracted planetary disk edge points

  std::vector<cv::Point3f> edge_points;
  if ( !extract_points(_grth, _skirt_mask, _skirt_roi, edge_points, _opts.lmweighted) ) {
    CF_ERROR("extract edge points fails");
    return false;
  }


  constexpr double axis_ratio = k_saturn_axis_ratio;
  const double tilt_to_earth = _opts.planetary_disk_tilt * CV_PI / 180;
  const double cos_tilt = cos(tilt_to_earth);
  const double sin_tilt = sin(tilt_to_earth);
  const double fixed_axis_ratio = std::sqrt(axis_ratio * axis_ratio * cos_tilt * cos_tilt + sin_tilt * sin_tilt);

  fitEllipseLMW(edge_points,
      fixed_axis_ratio, // b / a
      _skirt_roi.angle * CV_PI / 180, // radians
      &_final_planetary_disk_ellipse);

  // Add optional offset if requested by user
  _final_planetary_disk_ellipse.center += _opts.offset;

  // Create also filed binary ellipse mask
  //draw_ellipse_mask(_final_planetary_disk_mask, _image.size(), _final_planetary_disk_ellipse);

  ////////////////
  const double A = _final_planetary_disk_ellipse.size.width / 2.0;
  const double B = _final_planetary_disk_ellipse.size.width * axis_ratio / 2.0;
  const double C = _final_planetary_disk_ellipse.size.width / 2.0;

  _center = _final_planetary_disk_ellipse.center;
  _axes = cv::Vec3d(A, B, C);
  _pose = build_ellipsoid_pose(0., _opts.planetary_disk_tilt * CV_PI / 180, _final_planetary_disk_ellipse.angle * CV_PI / 180);

  if( true ) {
    const std::string s = serialize_ellipsoid_to_string(_center, _axes, _pose * 180 / CV_PI);
    CF_DEBUG("\nfitEllipseLMW:\n" "%s\n", s.c_str());
  }

  return true;
}

bool c_saturn_ellipse_detector::detect_initial_mask(cv::InputArray _input_image, cv::InputArray _input_mask)
{
  // Convert input image to gray scale
  if( _input_image.channels() == 1 ) {
    _input_image.getMat().copyTo(_grayscale_image);
  }
  else {
    cv::cvtColor(_input_image, _grayscale_image, cv::COLOR_BGR2GRAY);
  }

  // Detect planetary disk and extract pixel mask covering planetary disk shape
  bool fOk =
      simple_planetary_disk_detector(_grayscale_image, _input_mask,
          _opts.planetary_disk_detector_options,
          &_detected_component_centroid,
          &_detected_component_roi,
          &_initial_mask);
  if( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  const int pca_mask_dilate_size = 2 * 7 + 1;

  cv::dilate(_initial_mask, _pca_mask,
      cv::Mat1b(pca_mask_dilate_size, pca_mask_dilate_size, 255),
      cv::Point(-1, -1), 1,
      cv::BORDER_CONSTANT);

  _pca_roi = cv::Rect(0, 0, _input_image.cols(), _input_image.rows()) &
      cv::Rect(_detected_component_roi.x - pca_mask_dilate_size / 2,
          _detected_component_roi.y - pca_mask_dilate_size / 2,
          _detected_component_roi.width + pca_mask_dilate_size,
          _detected_component_roi.height + pca_mask_dilate_size);

  return true;
}

bool c_saturn_ellipse_detector::detect_pca_rect()
{
  return detect_primary_orientation_pcaw(_grayscale_image, _pca_mask, _pca_roi, &_pca_rect);
}

bool c_saturn_ellipse_detector::compute_radial_gradient(cv::InputArray _input_image, cv::InputArray _input_mask)
{
  // Prepare mask for radial gradient based planetary edge detection
  const cv::Size size(_pca_rect.size.width, _pca_rect.size.height);
  const int max_size = std::max(size.width, size.height);
  const int min_size = std::max(size.width, size.height);
  const int _skirt_size = std::max(11, 2 * (min_size / 48) + 1);

  morphological_gradient(_initial_mask, _skirt_mask,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_skirt_size, _skirt_size)),
      cv::BORDER_REPLICATE);


  // Prepare gr and grth images for contour detection
  static const cv::Mat1b THSE(5, 5, uint8_t(255));
  extract_channel(_input_image, _gradient_image, cv::noArray(), cv::noArray(), _opts.gradient_channel, CV_32F);
  differentiate(_gradient_image, _gx, _gy, _opts.sigma_contour);
  project_to_radius_vector(_detected_component_centroid, _gx, _gy, _gr, cv::noArray());
  cv::multiply(_gr, cv::Scalar::all(-1e4), _gr);
  cv::max(_gr, 0.f, _grth);
  cv::morphologyEx(_grth, _grth, cv::MORPH_TOPHAT, THSE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  _gr.setTo(0, ~_skirt_mask);
  //cv::sqrt(_gr, _gr);

  _skirt_roi  = _pca_rect;
  _skirt_roi.size.width = _skirt_roi.size.height / k_saturn_axis_ratio;
  return true;
}

bool serialize_base_saturn_ellipse_detector_options(c_config_setting section, bool save,
    c_saturn_ellipse_detector_options & opts)
{
  //SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, gradient_channel);
  SERIALIZE_OPTION(section, save, opts, sigma_contour);
  //SERIALIZE_OPTION(section, save, opts, sigma_clouds);
  SERIALIZE_OPTION(section, save, opts, nscale);
  SERIALIZE_OPTION(section, save, opts, neps);
  //SERIALIZE_OPTION(section, save, opts, gweighted);
  SERIALIZE_OPTION(section, save, opts, lmweighted);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_tilt);
  SERIALIZE_OPTION(section, save, opts, offset);
  serialize_base_planetary_disk_detector_options(section, save, opts.planetary_disk_detector_options);
  return true;
}


