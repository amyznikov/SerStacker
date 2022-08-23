/*
 * jupiter.cc
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#include "jupiter.h"
#include "morphology.h"
#include "geo-reconstruction.h"
#include "eccalign.h"
#include "planetary-disk-detection.h"
#include <tbb/tbb.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) ((a)<<16 | (b)<<8 | (c))
#endif

#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

// Debug message macros
#ifndef CF_DEBUG
  #define CF_DEBUG(...) \
    fprintf(stderr, "%s() : %d ", __FUNCTION__, __LINE__), \
    fprintf(stderr, __VA_ARGS__), \
    fprintf(stderr, "\n")
#endif

#ifndef CF_ERROR
# define CF_ERROR  CF_DEBUG
#endif

#ifndef CF_FATAL
# define CF_FATAL CF_ERROR
#endif

// Prevent conflicts with MS VS min() and max() macros
#ifdef _WIN32
# ifdef min
#   undef min
# endif
# ifdef max
#  undef max
# endif
#endif


static inline double square(double x)
{
  return x * x;
}


#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(3,4,0) )
namespace cv {
namespace hal {
void filter2D(int stype, int dtype, int kernel_type,
              uchar * src_data, size_t src_step,
              uchar * dst_data, size_t dst_step,
              int width, int height,
              int full_width, int full_height,
              int offset_x, int offset_y,
              uchar * kernel_data, size_t kernel_step,
              int kernel_width, int kernel_height,
              int anchor_x, int anchor_y,
              double delta, int borderType,
              bool isSubmatrix);
}}
#endif

static inline void doFilter2D(const cv::Mat & src, cv::Mat & dst, int ddepth,
    cv::InputArray kernel, const cv::Point & anchor,
    double delta = 0, int borderType = cv::BORDER_REFLECT)
{
#if ( CV_VERSION_CURRRENT < CV_VERSION_INT(3,4,0) )
  cv::filter2D(src, dst, ddepth, kernel, anchor, delta, borderType);
#else

  if ( src.data == dst.data ) { // in-place filtering is not supported by this routine
    CF_FATAL("APP BUG: Inplace doFilter2D() is not supported by this function. "
        "Fix your source code.");
    exit (1);
  }


  if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));

  const cv::Point ofs(0, 0);
  const cv::Size wsz(src.cols, src.rows);
  const cv::Mat K = kernel.getMat();

  cv::hal::filter2D(src.type(), dst.type(), K.type(),
      src.data, src.step, dst.data, dst.step,
      dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
      K.data, K.step, K.cols, K.rows,
      anchor.x, anchor.y,
      delta, borderType, src.isSubmatrix());

#endif
}

/*
 * Five-point approximation to first order image derivative.
 *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
 * */
static void differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth = -1)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
        0.f,
      (+8.f / 12),
      (-1.f / 12));

  if ( ddepth < 0 ) {
    ddepth = std::max(_src.depth(), CV_32F);
  }

  const cv::Mat & src = _src.getMat();
  doFilter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  doFilter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
}

static void gradient_magnitude(cv::InputArray src, cv::OutputArray dst, double sigma = 0)
{
  cv::Mat g, gx, gy;
  if ( sigma > 0 ) {
    cv::GaussianBlur(src, g, cv::Size(), sigma, sigma, cv::BORDER_REFLECT101);
  }
  else {
    g = src.getMat();
  }
  differentiate(g, gx, gy);
  //cv::add(gx.mul(gx), gy.mul(gy), dst);
  cv::magnitude(gx, gy, dst);
}

static void draw_artifical_jupiter(bool filled_ellipse, cv::Mat & image, const cv::RotatedRect & rc,
    const cv::Scalar & color,
    int thickness = 1,
    int line_type = cv::LINE_AA,
    const std::vector<float> * hlines = nullptr)
{


  static const auto draw_hlines =
      [](cv::Mat & _image, const cv::RotatedRect & _rc, const std::vector<float> & _hlines,
          double A, double B, double angle,
          const cv::Scalar & _color,
          int _thickness,
          int _line_type) {
        for ( uint i = 0, n = _hlines.size(); i < n; ++i ) {

          const double y = _hlines[i] * B;

          const double x1 = +0.75 * A * sqrt(1. - square(_hlines[i]));
          const double x2 = -0.75 * A * sqrt(1. - square(_hlines[i]));

          const double xp1 = x1 * cos(angle) - y * sin(angle);
          const double yp1 = x1 * sin(angle) + y * cos(angle);

          const double xp2 = x2 * cos(angle) - y * sin(angle);
          const double yp2 = x2 * sin(angle) + y * cos(angle);

          cv::line(_image,
            _rc.center + cv::Point2f( xp1, yp1),
            _rc.center + cv::Point2f( xp2, yp2),
            _color,
            _thickness,
            _line_type);
        }
    };

  const double A = 0.5 * rc.size.width;
  const double B = 0.5 * rc.size.height;
  const double angle = rc.angle * CV_PI / 180;

  if ( filled_ellipse ) {
    thickness = -1;
  }

  cv::ellipse(image, rc, color, thickness, line_type);

  if ( hlines && !hlines->empty() ) {
    if ( filled_ellipse ) {
      draw_hlines(image, rc, *hlines, A, B, angle, 0, 2, line_type);
    }
    else {
      draw_hlines(image, rc, *hlines, A, B, angle, color, 2, line_type);
    }
  }
}

void create_jovian_rotation_remap(double rotation_angle,
    const cv::RotatedRect & E,
    const cv::Matx23d & R2I,
    const cv::Size & size,
    cv::Mat2f & rmap,
    cv::Mat1f & wmask)
{
  INSTRUMENT_REGION("");

  double sa, ca;
  sincos(E.angle * CV_PI / 180, &sa, &ca);

  rmap.create(size);
  wmask.create(size);

  typedef tbb::blocked_range<int> tbb_range;

  tbb::parallel_for(tbb_range(0, size.height, 64),
      [E, R2I, sa, ca, rotation_angle, &rmap, &wmask ](const tbb_range & r ) {

        const double A = 0.5 * E.size.width;
        const double B = 0.5 * E.size.height;
        const double x0 = E.center.x;
        const double y0 = E.center.y;

        for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
          for ( int x = 0; x < rmap.cols; ++x ) {

            double xe = ((x - x0) * ca + (y - y0) * sa) / A;
            double ye = (-(x - x0) * sa + (y - y0) * ca) / B;

            if ( xe * xe + ye * ye >= 1 ) {
              rmap[y][x][0] = -1;
              rmap[y][x][1] = -1;
              wmask[y][x] = 0;
              continue;
            }

            const double q = sqrt(1. - ye * ye);
            const double phi = asin(xe / q) + rotation_angle;

            if ( (phi <= -CV_PI / 2) || (phi >= CV_PI / 2) ) {
              rmap[y][x][0] = -1;
              rmap[y][x][1] = -1;
              wmask[y][x] = 0;
              continue;
            }

            xe = A * q * sin(phi);
            ye = B * ye;

            const double xx = xe * ca - ye * sa + x0;
            const double yy = xe * sa + ye * ca + y0;

            rmap[y][x][0] = R2I(0, 0) * xx + R2I(0, 1) * yy + R2I(0, 2);
            rmap[y][x][1] = R2I(1, 0) * xx + R2I(1, 1) * yy + R2I(1, 2);
            wmask[y][x] = cos(phi);
          }
        }
      });

}

void get_jovian_ellipse_bounding_box(const cv::Size & image_size, const cv::RotatedRect & ellipse, cv::Rect * bbox)
{
  cv::Rect boundingRect = ellipse.boundingRect();
  if ( boundingRect.x < 0 ) {
    boundingRect.x = 0;
  }
  if ( boundingRect.y < 0 ) {
    boundingRect.y = 0;
  }
  if ( boundingRect.x + boundingRect.width >= image_size.width ) {
    boundingRect.width = image_size.width - boundingRect.x;
  }
  if ( boundingRect.y + boundingRect.height >= image_size.height ) {
    boundingRect.height = image_size.height - boundingRect.y;
  }

  * bbox = boundingRect;
}



// set mininum rotation angle for best rotation search range.
void c_jovian_derotation::set_min_rotation(double v)
{
  min_rotation_ = v;
}

double c_jovian_derotation::min_rotation() const
{
  return min_rotation_;
}

// set maximum rotation angle for best rotation search range.
void c_jovian_derotation::set_max_rotation(double v)
{
  max_rotation_ = v;
}

double c_jovian_derotation::max_rotation() const
{
  return max_rotation_;
}

void c_jovian_derotation::set_normalization_scale(int v)
{
  planetary_detector_.set_normalization_scale(v);
}

int c_jovian_derotation::normalization_scale() const
{
  return planetary_detector_.normalization_scale();
}

void c_jovian_derotation::set_normalization_blur(double v)
{
  planetary_detector_.set_normalization_blur(v);
}

double c_jovian_derotation::normalization_blur() const
{
  return planetary_detector_.normalization_blur();
}

void c_jovian_derotation::set_eccflow_support_scale(int v)
{
  eccflow_support_scale_ = v;
}

int c_jovian_derotation::eccflow_support_scale() const
{
  return eccflow_support_scale_;
}

void c_jovian_derotation::set_eccflow_normalization_scale(int v)
{
  eccflow_normalization_scale_ = v;
}

int c_jovian_derotation::eccflow_normalization_scale() const
{
  return eccflow_normalization_scale_;
}


void c_jovian_derotation::set_eccflow_max_pyramid_level(int v)
{
  eccflow_max_pyramid_level_ = v;
}

int c_jovian_derotation::max_eccflow_pyramid_level(int v)
{
  return eccflow_max_pyramid_level_;
}

void c_jovian_derotation::set_hlines(const std::vector<float> & hlines)
{
  planetary_detector_.set_hlines(hlines);
}

const std::vector<float> & c_jovian_derotation::hlines() const
{
  return planetary_detector_.hlines();
}

const cv::RotatedRect & c_jovian_derotation::reference_ellipse() const
{
  return reference_ellipse_;
}

const cv::RotatedRect & c_jovian_derotation::current_ellipse() const
{
  return current_ellipse_;
}

const cv::Rect & c_jovian_derotation::reference_bounding_box() const
{
  return reference_bounding_box_;
}

const cv::Rect & c_jovian_derotation::current_bounding_box() const
{
  return current_bounding_box_;
}

const cv::Mat & c_jovian_derotation::reference_uncropped_planetary_disk_mask() const
{
  return reference_uncropped_planetary_disk_mask_;
}

const cv::Mat & c_jovian_derotation::current_uncropped_planetary_disk_mask() const
{
  return current_uncropped_planetary_disk_mask_;
}

const cv::Mat2f & c_jovian_derotation::current_cropped_derotation_remap() const
{
  return current_cropped_remap_;
}

const cv::Mat1f & c_jovian_derotation::current_cropped_wmask() const
{
  return current_cropped_wmask_;
}

double c_jovian_derotation::compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
    const cv::Mat1f & rotated_component_image,
    const cv::Mat1f & rotation_mask,
    cv::Mat1f * output_difference_image /* optional for debug */)
{
  INSTRUMENT_REGION("");

  double total_cost = 0;
  double total_weight = 0;

  if ( output_difference_image ) {
    output_difference_image->create(rotation_mask.size());
    output_difference_image->setTo(0);
  }

  for ( int y = 0; y < rotation_mask.rows; ++y ) {
    for ( int x = 0; x < rotation_mask.cols; ++x ) {
      if ( rotation_mask[y][x] > 0 ) {

        const double difference =
            rotation_mask[y][x] * abs(reference_component_image[y][x] - rotated_component_image[y][x]);

        total_cost += difference;
        total_weight += rotation_mask[y][x];

        if ( output_difference_image ) {
          (*output_difference_image)[y][x] = difference;
        }
      }
    }
  }

  return total_cost / total_weight;
}

bool c_jovian_derotation::setup_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if ( !planetary_detector_.detect_planetary_disk(reference_image, reference_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(reference_image) fails");
    return false;
  }

  reference_ellipse_ = planetary_detector_.planetary_disk_ellipse();
  reference_bounding_box_ = planetary_detector_.crop_bounding_box();
  planetary_detector_.uncropped_planetary_disk_mask().copyTo(reference_uncropped_planetary_disk_mask_);
  planetary_detector_.cropped_normalized_image().copyTo(reference_cropped_normalized_image_);
  planetary_detector_.cropped_gray_image().copyTo(reference_cropped_gray_image_);

  return true;
}



bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");

  if ( !planetary_detector_.detect_planetary_disk(current_image, current_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(current_image) fails");
    return false;
  }

  current_ellipse_ = planetary_detector_.planetary_disk_ellipse();
  current_bounding_box_ = planetary_detector_.crop_bounding_box();
  planetary_detector_.uncropped_planetary_disk_mask().copyTo(current_uncropped_planetary_disk_mask_);
  planetary_detector_.cropped_normalized_image().copyTo(current_cropped_normalized_image_);
  planetary_detector_.cropped_gray_image().copyTo(current_cropped_gray_image_);

  if ( !debug_path_.empty() ) {
    save_image(current_uncropped_planetary_disk_mask_, ssprintf("%s/current_uncropped_planetary_disk_mask_.tiff", debug_path_.c_str()));
    save_image(current_cropped_normalized_image_, ssprintf("%s/current_cropped_normalized_image_.tiff", debug_path_.c_str()));
    save_image(current_cropped_gray_image_, ssprintf("%s/current_cropped_gray_image_.tiff", debug_path_.c_str()));

    save_image(reference_uncropped_planetary_disk_mask_, ssprintf("%s/reference_uncropped_planetary_disk_mask_.tiff", debug_path_.c_str()));
    save_image(reference_cropped_normalized_image_, ssprintf("%s/reference_cropped_normalized_image_.tiff", debug_path_.c_str()));
    save_image(reference_cropped_gray_image_, ssprintf("%s/reference_cropped_gray_image_.tiff", debug_path_.c_str()));
  }

  //
  // Align current jovian component image to the reference component image
  //

  ecc_.set_motion_type(ECC_MOTION_EUCLIDEAN_SCALED);
  ecc_.set_eps(0.1);
  ecc_.set_min_rho(0.4);

  cv::Matx23d T =
      createEyeTransform(ecc_.motion_type());

  bool fOk =
      ecch_.align(&ecc_,
          current_cropped_gray_image_,
          reference_cropped_gray_image_,
          T);

  if ( !fOk ) {
    CF_ERROR("ecch.align(current_component_image_->reference_component_image_) fails");
    return false;
  }

  if ( !debug_path_.empty() ) {
    cv::Mat tmp;
    cv::remap(current_cropped_gray_image_, tmp, ecc_.current_remap(), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    save_image(tmp, ssprintf("%s/current_cropped_gray_image_aligned.tiff", debug_path_.c_str()));
  }


  //
  // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
  // Select the best candidate.
  //

  const int num_rotations =
        reference_ellipse_.size.width * (max_rotation_ - min_rotation_) / CV_PI;

  const double rotation_step =
    (max_rotation_ - min_rotation_) / num_rotations;

  double current_cost, best_cost;
  double best_rotation;
  int best_rotation_index;

  CF_DEBUG("c_jovian_derotation:  use num_rotations=%d rotation_step=%g deg",
      num_rotations,
      rotation_step * 180 / CV_PI);

  cv::Mat current_rotated_normalized_image;

  cv::RotatedRect cropped_reference_ellipse =
      reference_ellipse_;

  cropped_reference_ellipse.center.x -=
      reference_bounding_box_.x;

  cropped_reference_ellipse.center.y -=
      reference_bounding_box_.y;

  for ( int i = 0; i < num_rotations; ++i ) {

    INSTRUMENT_REGION("derotation_body");


    const double l =
          min_rotation_ + i * rotation_step;

    create_jovian_rotation_remap(l,
        cropped_reference_ellipse,
        T,
        reference_cropped_gray_image_.size(),
        current_cropped_remap_,
        current_cropped_wmask_);

    cv::remap(current_cropped_normalized_image_, current_rotated_normalized_image,
        current_cropped_remap_, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);

    current_cost =
        compute_jovian_derotation_cost(
            reference_cropped_normalized_image_,
            current_rotated_normalized_image,
            current_cropped_wmask_,
            nullptr/*debug_path_.empty() ? nullptr :
                &current_difference_image*/);


    if ( !debug_path_.empty() ) {
      // CF_DEBUG("R %6d: l=%+.3f cost=%.6f", i, l * 180 / CV_PI, current_cost);
    }


    if ( i == 0 || current_cost < best_cost ) {
      best_cost = current_cost;
      best_rotation = l;
      best_rotation_index = i;
    }
  }

  CF_DEBUG("BEST cost: %g at rotation %d = %g deg",
      best_cost, best_rotation_index, best_rotation * 180 / CV_PI);

  create_jovian_rotation_remap(best_rotation,
    cropped_reference_ellipse,
    T,
    reference_cropped_gray_image_.size(),
    current_cropped_remap_,
    current_cropped_wmask_);

  if ( !debug_path_.empty() ) {
    cv::remap(current_cropped_normalized_image_, current_rotated_normalized_image, current_cropped_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    save_image(current_rotated_normalized_image, ssprintf("%s/current_rotated_normalized_image.tiff", debug_path_.c_str()));
    save_image(current_cropped_wmask_, ssprintf("%s/current_cropped_wmask_.tiff", debug_path_.c_str()));
  }

  //
  // Use ecc optical flow to perfectly align rotated current component image
  //

  if ( eccflow_support_scale_ > 1 ) {

    INSTRUMENT_REGION("eccflow_.compute()");

    eccflow_.set_max_pyramid_level(eccflow_max_pyramid_level_);
    eccflow_.set_support_scale(eccflow_support_scale_);
    eccflow_.set_normalization_scale(eccflow_normalization_scale_);

    fOk = eccflow_.compute(current_cropped_normalized_image_,
        reference_cropped_normalized_image_,
        current_cropped_remap_);

    if ( !fOk ) {
      CF_ERROR("eccflow_.compute() fails");
      return false;
    }

    if ( !debug_path_.empty() ) {
      cv::Mat tmp;
      cv::remap(current_cropped_normalized_image_, tmp, current_cropped_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      save_image(tmp, ssprintf("%s/current_rotated_normalized_ecc_aligned_image.tiff", debug_path_.c_str()));
    }
  }

  return true;
}

void c_jovian_derotation::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string & c_jovian_derotation::debug_path() const
{
  return debug_path_;
}

//////////////////////////


void c_jovian_ellipse_detector::set_hlines(const std::vector<float> & v)
{
  options_.hlines = v;
}

const std::vector<float> & c_jovian_ellipse_detector::hlines() const
{
  return options_.hlines;
}

void c_jovian_ellipse_detector::set_normalization_scale(int v)
{
  options_.normalization_scale = v;
}

int c_jovian_ellipse_detector::normalization_scale() const
{
  return options_.normalization_scale;
}

int c_jovian_ellipse_detector::actual_normalization_scale() const
{
  return actual_normalization_scale_;
}

void c_jovian_ellipse_detector::set_normalization_blur(double v)
{
  options_.normalization_blur = v;
}

double c_jovian_ellipse_detector::normalization_blur() const
{
  return options_.normalization_blur;
}

void c_jovian_ellipse_detector::set_options(const c_jovian_ellipse_detector_options & v)
{
  options_ = v;
}

c_jovian_ellipse_detector_options * c_jovian_ellipse_detector::options()
{
  return &options_;
}

const cv::Mat & c_jovian_ellipse_detector::uncropped_planetary_disk_mask() const
{
  return uncropped_planetary_disk_mask_;
}

//cv::Mat c_jovian_ellipse_detector::component_mask(const cv::Rect & crop) const
//{
//  return uncropped_planetary_disk_mask_(crop);
//}

const cv::Mat & c_jovian_ellipse_detector::cropped_gray_image() const
{
  return cropped_gray_image_;
}

const cv::Mat & c_jovian_ellipse_detector::cropped_normalized_image() const
{
  return cropped_normalized_image_;
}

const cv::Mat & c_jovian_ellipse_detector::cropped_gradient_image() const
{
  return cropped_gradient_image_;
}

const cv::Mat & c_jovian_ellipse_detector::initial_artificial_ellipse() const
{
  return cropped_artificial_ellipse_;
}

const cv::Mat & c_jovian_ellipse_detector::initial_artificial_ellipse_fit() const
{
  return cropped_initial_ellipse_fit_;
}

const cv::Mat & c_jovian_ellipse_detector::cropped_final_ellipse_fit() const
{
  return cropped_final_ellipse_fit_;
}

const cv::Rect & c_jovian_ellipse_detector::crop_bounding_box() const
{
  return crop_bounding_box_;
}

const cv::RotatedRect & c_jovian_ellipse_detector::planetary_disk_ellipse() const
{
  return ellipse_;
}

bool c_jovian_ellipse_detector::detect_planetary_disk(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  cv::Mat gray_image, edge_mask;
  std::vector<cv::Point2f> component_edge_points;
  cv::Scalar m, s;

  /////////////////////////////////////////////////////////////////////////////////////////
  // Convert input image to gray scale

  if ( _image.channels() == 1 ) {
    _image.getMat().copyTo(gray_image);
  }
  else {
    cv::cvtColor(_image, gray_image, cv::COLOR_BGR2GRAY);
  }


  /////////////////////////////////////////////////////////////////////////////////////////
  // Detect planetary disk mask and use fitEllipseAMS() on planetary disk edge
  // as initial ellipse estimate

  if( !simple_planetary_disk_detector(gray_image, _mask, nullptr, 1, &crop_bounding_box_, &uncropped_planetary_disk_mask_) ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }

  cv::meanStdDev(gray_image, m, s, uncropped_planetary_disk_mask_);
  cv::bitwise_and(uncropped_planetary_disk_mask_, gray_image > s[0] / 2, uncropped_planetary_disk_mask_);
  morphological_smooth_close(uncropped_planetary_disk_mask_, uncropped_planetary_disk_mask_, cv::Mat1b(3, 3, 255));
  geo_fill_holes(uncropped_planetary_disk_mask_, uncropped_planetary_disk_mask_, 8);

  morphological_gradient(uncropped_planetary_disk_mask_, edge_mask, cv::Mat1b(3, 3, 255), cv::BORDER_CONSTANT);
  cv::findNonZero(edge_mask, component_edge_points);

  ellipse_ = cv::fitEllipseAMS(component_edge_points);
  if ( ellipse_.size.width < ellipse_.size.height ) {
    std::swap(ellipse_.size.width, ellipse_.size.height);
    ellipse_.angle -= 90;
  }

  CF_DEBUG("fitEllipseAMS: angle=%g width=%g height=%g", ellipse_.angle, ellipse_.size.width, ellipse_.size.height);


  /////////////////////////////////////////////////////////////////////////////////////////
  // Crop planetary disk into separate sub-image with some margins,
  // compute the image gradient to enhance the disk edge
  // draw artificial solid ellipse
  // and finally use ECC to improve initial ellipse estimate

  int margin = std::max(16, (std::max(crop_bounding_box_.width, crop_bounding_box_.height)/8) & ~0x1);
  int l = std::max(0, crop_bounding_box_.x - margin);
  int t = std::max(0, crop_bounding_box_.y - margin);
  int r = std::min(gray_image.cols - 1, crop_bounding_box_.x + crop_bounding_box_.width + margin - 1);
  int b = std::min(gray_image.rows - 1, crop_bounding_box_.y + crop_bounding_box_.height + margin - 1);

  crop_bounding_box_.x = l;
  crop_bounding_box_.y = t;
  crop_bounding_box_.width = r - l + 1;
  crop_bounding_box_.height = b - t + 1;

  gray_image(crop_bounding_box_).copyTo(cropped_gray_image_, uncropped_planetary_disk_mask_(crop_bounding_box_));
  gradient_magnitude(cropped_gray_image_, cropped_gradient_image_, std::max(1., crop_bounding_box_.width / 300.) );

  static constexpr double jovian_polar_to_equatorial_axis_ratio = 66.854 / 71.492;

  double A = 0.5 * ellipse_.size.width;
  double B = A * jovian_polar_to_equatorial_axis_ratio;
  const cv::Point2f C = ellipse_.center - cv::Point2f(crop_bounding_box_.x, crop_bounding_box_.y);

  if ( true ) { // for debug visualization

    cv::RotatedRect rc = ellipse_;
    rc.center.x -= crop_bounding_box_.x;
    rc.center.y -= crop_bounding_box_.y;

    cv::normalize(cropped_gradient_image_, cropped_gradient_image_, 0, 1, cv::NORM_MINMAX);
    cv::cvtColor(cropped_gradient_image_, cropped_initial_ellipse_fit_, cv::COLOR_GRAY2BGR);
    draw_artifical_jupiter(false, cropped_initial_ellipse_fit_, rc, CV_RGB(0, 1, 0), 1, cv::LINE_AA, &options_.hlines);
  }


  cropped_artificial_ellipse_.create(crop_bounding_box_.size(), CV_32FC1);
  cropped_artificial_ellipse_.setTo(0);
  draw_artifical_jupiter(true, cropped_artificial_ellipse_, cv::RotatedRect(C, cv::Size2f(2 * A, 2 * B), 0), 1, 2, cv::LINE_AA, &options_.hlines);
  cv::GaussianBlur(cropped_artificial_ellipse_, cropped_artificial_ellipse_, cv::Size(), 2);

  c_ecc_forward_additive ecc(ECC_MOTION_EUCLIDEAN_SCALED);

  cv::Matx23f T =
      createEuclideanTransform(C.x, C.y,
          C.x, C.y,
          1.0,
          ellipse_.angle * CV_PI / 180);

  if ( true ) {
    CF_DEBUG("T BEFORE: {\n"
        "A = %g B = %g angle=%g center = (%g %g)\n"
        "}\n",
        A, B, ellipse_.angle, ellipse_.center.x, ellipse_.center.y);
  }


  ecc.set_eps(0.2);
  ecc.set_min_rho(0.25);
  ecc.set_max_iterations(100);

  c_ecch ecch(&ecc);
  if ( !ecch.align(cropped_artificial_ellipse_, cropped_gradient_image_, T) ) {
    CF_ERROR("ecch.align(artifical_ellipse) fails");
    return false;
  }
//  if ( !ecc.align(initial_artifical_ellipse_, component_image_, T) ) {
//    CF_ERROR("ecc.align(artifical_ellipse) fails");
//    return false;
//  }


  /////////////////////////////////////////////////////////////////////////////////////////
  // Extract best fit ellipse parameters and return to caller

  cv::Vec2f CC;
  double Tx, Ty, scale, angle;

  cv::invertAffineTransform(T, T);
  getEuclideanComponents(T, &Tx, &Ty, &scale, &angle);

  CC = T * cv::Vec3f(C.x, C.y, 1);
  A *= scale;
  B *= scale;

  ellipse_.size.width = 2 * A;
  ellipse_.size.height = 2 * B;
  ellipse_.center.x = CC(0) + crop_bounding_box_.x;
  ellipse_.center.y = CC(1) + crop_bounding_box_.y;
  ellipse_.angle = -angle * 180 / M_PI;

  if ( true ) {
    CF_DEBUG("T AFTER: {\n"
        "A = %g B = %g angle=%g center = (%g %g)\n"
        "}\n",
        A, B, ellipse_.angle, ellipse_.center.x, ellipse_.center.y);
  }

  CF_DEBUG("ELLIPSE: center=(%g %g) size=(%g x %g) angle=%g",
      ellipse_.center.x, ellipse_.center.y,
      ellipse_.size.width, ellipse_.size.height,
      ellipse_.angle);

  /////////////////////////////////////////////////////////////////////////////////////////
  // Update component bounding box, image and mask using improved ellipse fit
  cv::Rect bbox;
  get_jovian_ellipse_bounding_box(_image.size(), ellipse_, &bbox);
  margin = std::max(16, (std::max(bbox.width, bbox.height)/16) & ~0x1);
  l = std::max(0, bbox.x - margin);
  t = std::max(0, bbox.y - margin);
  r = std::min(gray_image.cols - 1, bbox.x + bbox.width + margin - 1);
  b = std::min(gray_image.rows - 1, bbox.y + bbox.height + margin - 1);

  crop_bounding_box_.x = l;
  crop_bounding_box_.y = t;
  crop_bounding_box_.width = r - l + 1;
  crop_bounding_box_.height = b - t + 1;

  gray_image(crop_bounding_box_).copyTo(cropped_gray_image_);
  gradient_magnitude(cropped_gray_image_, cropped_gradient_image_, std::max(1., crop_bounding_box_.width / 300.) );
  uncropped_planetary_disk_mask_.create(gray_image.size(), CV_8U);
  uncropped_planetary_disk_mask_.setTo(0);
  cv::ellipse(uncropped_planetary_disk_mask_, ellipse_, 255, -1, cv::LINE_8);

  // create also normalized image for jovian derotation loop

  if ( options_.normalization_scale < 0 ) {
    cropped_gray_image_.copyTo(cropped_normalized_image_);
  }
  else  {
    cv::Mat mean;

    cv::meanStdDev(cropped_gray_image_, m, s/*, mask*/);

    if ( options_.normalization_scale <= 0 ) {
      cv::subtract(cropped_gray_image_, m, mean);
    }
    else {
      ecc_downscale(cropped_gray_image_, mean, options_.normalization_scale, cv::BORDER_REPLICATE);
      ecc_upscale(mean, cropped_gray_image_.size());
      cv::subtract(cropped_gray_image_, /*mv*/mean, mean);
    }

    cv::multiply(mean, 1. / s[0], cropped_normalized_image_);
  }

  if( options_.normalization_blur > 0 ) {
    cv::GaussianBlur(cropped_normalized_image_,
        cropped_normalized_image_,
        cv::Size(),
        options_.normalization_blur,
        options_.normalization_blur);
  }

  // create also artificial reference ellipse mask

  if ( true ) { // for debug visualization

    cv::RotatedRect rc2 = ellipse_;
    rc2.center.x -= crop_bounding_box_.x;
    rc2.center.y -= crop_bounding_box_.y;

    cv::normalize(cropped_gradient_image_, cropped_gradient_image_, 0, 1, cv::NORM_MINMAX);
    cv::cvtColor(cropped_gradient_image_, cropped_final_ellipse_fit_, cv::COLOR_GRAY2BGR);
    draw_artifical_jupiter(false, cropped_final_ellipse_fit_, rc2, CV_RGB(0, 1, 0), 1, cv::LINE_AA, &options_.hlines);
  }

  return true;
}


