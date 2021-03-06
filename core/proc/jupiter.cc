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
  cv::magnitude(gx, gy, dst);
}

static void draw_artifical_jupiter(cv::Mat & image, const cv::RotatedRect & rc,
    const cv::Scalar & color,
    int thickness = 1,
    int line_type = cv::LINE_AA)
{
  const double A = 0.5 * rc.size.width;
  const double B = 0.5 * rc.size.height;
  const double angle = rc.angle * CV_PI / 180;

  static const double ys[] = {
      +85.0/251,
      +128.0/251,
      -42.0/251,
      -97.0/251,
  };

  cv::ellipse(image, rc, color, thickness, line_type);

  for ( uint i = 0; i < sizeof(ys)/sizeof(ys[0]); ++i ) {

    const double y = ys[i] * B;

    const double x1 = +0.9 * A * sqrt(1. - square(ys[i]));
    const double x2 = -0.9 * A * sqrt(1. - square(ys[i]));

    const double xp1 = x1 * cos(angle) - y * sin(angle);
    const double yp1 = x1 * sin(angle) + y * cos(angle);

    const double xp2 = x2 * cos(angle) - y * sin(angle);
    const double yp2 = x2 * sin(angle) + y * cos(angle);

    cv::line(image,
        rc.center + cv::Point2f( xp1, yp1),
        rc.center + cv::Point2f( xp2, yp2),
        color,
        thickness,
        line_type);
  }
}

// TODO: add horizontal lines to artifical ellipse in appropriate zones
// for better rotation fitting (assuming that jupiter usually shows zonal gradients)
bool detect_jovian_ellipse(cv::InputArray _image, cv::RotatedRect * rc, const std::string & dbgpath)
{
  cv::Mat gray_image;
  cv::Rect component_rect;
  cv::Mat component_mask;
  cv::Mat component_edge;
  cv::Mat component_image;
  cv::Mat artifical_ellipse;
  cv::Mat debug_image;
  cv::Mat * pdbg = nullptr;
  std::vector<cv::Point2f> component_edge_points;

  static constexpr double jovian_polar_to_equatorial_axis_ratio = 66.854 / 71.492;

  if ( _image.channels() == 1 ) {
    gray_image = _image.getMat();
  }
  else {
    cv::cvtColor(_image, gray_image, cv::COLOR_BGR2GRAY);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Detect planetary disk mask and use fitEllipseAMS() as initial ellipse estimate

  if ( !dbgpath.empty() ) {
    save_image(gray_image, ssprintf("%s/gray_image.tiff", dbgpath.c_str()) );
    pdbg = &debug_image;
  }

  if( !simple_planetary_disk_detector(gray_image, cv::noArray(), nullptr, 1, &component_rect, &component_mask, nullptr, pdbg) ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    if ( !dbgpath.empty() ) {
      save_image(debug_image, ssprintf("%s/component_debug_image.tiff", dbgpath.c_str()) );
    }
    return false;
  }

  if ( !dbgpath.empty() ) {
    save_image(component_mask, ssprintf("%s/initial_component_mask.tiff", dbgpath.c_str()) );
    save_image(debug_image, ssprintf("%s/component_debug_image.tiff", dbgpath.c_str()) );
  }

  geo_fill_holes(component_mask, component_mask, 8);
  if ( !dbgpath.empty() ) {
    save_image(component_mask, ssprintf("%s/component_mask.tiff", dbgpath.c_str()) );
  }
  morphological_gradient(component_mask, component_edge, cv::Mat1b(3, 3, 255), cv::BORDER_CONSTANT);

  cv::findNonZero(component_edge, component_edge_points);
  *rc = cv::fitEllipseAMS(component_edge_points);
  rc->angle -= 90;


  const int margin = std::max(16, (std::max(component_rect.width, component_rect.height)/8) & ~0x1);
  const int l = std::max(0, component_rect.x - margin);
  const int t = std::max(0, component_rect.y - margin);
  const int r = std::min(gray_image.cols - 1, component_rect.x + component_rect.width + margin - 1);
  const int b = std::min(gray_image.rows - 1, component_rect.y + component_rect.height + margin - 1);

  component_rect.x = l;
  component_rect.y = t;
  component_rect.width = r - l + 1;
  component_rect.height = b - t + 1;

  /////////////////////////////////////////////////////////////////////////////////////////
  // Copy planetary disk into separate sub-image,
  // compute the image gradient to enchange the disk edge

  gray_image(component_rect).copyTo(component_image, component_mask(component_rect));
  gradient_magnitude(component_image, component_image, std::max(1., component_rect.width/250.) );

  if ( !dbgpath.empty() ) {
    save_image(component_image, ssprintf("%s/gradient_magnitude.tiff", dbgpath.c_str()) );
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Draw artifical jovian ellipse and use of ECC to fit it into planetary disk gradient image

  const double A = 0.5 * std::max(rc->size.width, rc->size.height);
  const double B = A * jovian_polar_to_equatorial_axis_ratio;
  const cv::Point2f C = rc->center - cv::Point2f(component_rect.x, component_rect.y);

  artifical_ellipse.create(component_rect.size(), CV_32FC1);
  artifical_ellipse.setTo(0);
  draw_artifical_jupiter(artifical_ellipse, cv::RotatedRect(C, cv::Size2f(2 * A, 2 * B), 0), 1, 2);
  cv::GaussianBlur(artifical_ellipse, artifical_ellipse, cv::Size(), 3);

  if ( !dbgpath.empty() ) {
    save_image(artifical_ellipse, ssprintf("%s/initial_artifical_ellipse.tiff", dbgpath.c_str()) );
  }

  c_ecc_forward_additive ecc(ECC_MOTION_EUCLIDEAN_SCALED);
  c_ecch ecch(&ecc);

  cv::Matx23f T =
      createEuclideanTransform(C.x, C.y,
          C.x, C.y,
          1.0,
          rc->angle * CV_PI / 180);

  if ( false ) {
    CF_DEBUG("T BEFORE: {\n"
        "  %+15.6f %+15.6f %+15.6f\n"
        "  %+15.6f %+15.6f %+15.6f\n"
        "}\n",
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2));
  }


  ecc.set_min_rho(0.2);
  if ( !ecch.align(artifical_ellipse, component_image, T) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }


  if ( false ) {
    CF_DEBUG("T AFTER: {\n"
        "  %+15.6f %+15.6f %+15.6f\n"
        "  %+15.6f %+15.6f %+15.6f\n"
        "}\n",
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2));
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Extract best fit ellipse parameters and return to caller

  cv::Vec2f CC;
  double Tx, Ty, scale, angle;

  cv::invertAffineTransform(T, T);
  getEuclideanComponents(T, &Tx, &Ty, &scale, &angle);

  CC = T * cv::Vec3f(C.x, C.y, 1);

  //  CF_DEBUG("Tx=%g Ty=%g scale=%g angle=%g delta_angle=%g",
  //      Tx, Ty, scale,
  //      angle * 180 / M_PI,
  //      angle * 180 / M_PI - rc->angle);

  rc->size.width = 2 * A * scale;
  rc->size.height = 2 * B * scale;
  rc->center.x = CC(0) + component_rect.x;
  rc->center.y = CC(1) + component_rect.y;
  rc->angle = -angle * 180 / M_PI;

  //////////////////

  CF_DEBUG("ELLIPSE: center=(%g %g) size=(%g x %g) angle=%g",
      rc->center.x, rc->center.y,
      rc->size.width, rc->size.height,
      rc->angle);

  if ( !dbgpath.empty() ) {

    cv::RotatedRect rc2 = *rc;
    rc2.center.x -= component_rect.x;
    rc2.center.y -= component_rect.y;

    cv::normalize(component_image, component_image, 0, 1, cv::NORM_MINMAX);
    cv::cvtColor(component_image, artifical_ellipse, cv::COLOR_GRAY2BGR);
    draw_artifical_jupiter(artifical_ellipse, rc2, CV_RGB(0, 1, 0), 1);

    save_image(artifical_ellipse, ssprintf("%s/fitted_artifical_ellipse.tiff", dbgpath.c_str()) );
  }

  return true;
}




void create_jovian_rotation_remap(double rotation_angle,
    const cv::RotatedRect & E,
    const cv::Matx23d & R2I,
    const cv::Size & size,
    cv::Mat2f & rmap,
    cv::Mat1f & wmask)
{
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

//void c_jovian_derotation::set_align_jovian_disk_horizontally(bool v)
//{
//  align_jovian_disk_horizontally_ = v;
//}
//
//bool c_jovian_derotation::align_jovian_disk_horizontally() const
//{
//  return align_jovian_disk_horizontally_;
//}

const cv::RotatedRect & c_jovian_derotation::reference_ellipse() const
{
  return reference_ellipse_;
}

const cv::RotatedRect & c_jovian_derotation::current_ellipse() const
{
  return current_ellipse_;
}

const cv::Rect & c_jovian_derotation::reference_boundig_box() const
{
  return reference_boundig_box_;
}

const cv::Rect & c_jovian_derotation::current_boundig_box() const
{
  return current_boundig_box_;
}

const cv::Mat1b & c_jovian_derotation::reference_ellipse_mask() const
{
  return reference_ellipse_mask_;
}

const cv::Mat1b & c_jovian_derotation::current_ellipse_mask() const
{
  return current_ellipse_mask_;
}

const cv::Mat2f & c_jovian_derotation::current_total_remap() const
{
  return current_total_remap_;
}

const cv::Mat1f & c_jovian_derotation::current_total_mask() const
{
  return current_total_mask_;
}

const cv::Mat1b & c_jovian_derotation::current_total_binary_mask() const
{
  return current_total_binary_mask_;
}

void c_jovian_derotation::normalize_jovian_image(cv::InputArray _src, cv::InputArray mask, cv::OutputArray dst, int normalization_scale)
{
  cv::Scalar mv, sv;
  cv::Mat src, mean;

  src = _src.getMat();

  cv::meanStdDev(src, mv, sv/*, mask*/);

  ecc_downscale(src, mean, normalization_scale, cv::BORDER_REPLICATE);
  ecc_upscale(mean, src.size());
  cv::subtract(src, mean, mean);
  cv::multiply(mean, 1./sv[0], dst);
}


double c_jovian_derotation::compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
    const cv::Mat1f & rotated_component_image,
    const cv::Mat1f & rotation_mask,
    cv::Mat1f * output_difference_image /* optional for debug */)
{
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

bool c_jovian_derotation::extract_jovian_image(cv::InputArray src_image, cv::InputArray src_mask,
    cv::RotatedRect * output_ellipse,
    cv::Rect * output_ellipse_boundig_box,
    cv::Mat * output_component_image,
    cv::Mat * output_component_mask,
    cv::Mat1b * output_ellipse_mask) const
{
  if ( !detect_jovian_ellipse(src_image, output_ellipse, enable_debug_ ? debug_path_ : "") ) {
    CF_ERROR("detect_jovian_ellipse() fails");
    return false;
  }

  get_jovian_ellipse_bounding_box(src_image.size(), *output_ellipse,
      output_ellipse_boundig_box);

  output_ellipse->center.x -= output_ellipse_boundig_box->x;
  output_ellipse->center.y -= output_ellipse_boundig_box->y;

  if ( src_image.channels() == 1 ) {
    src_image.getMat()(*output_ellipse_boundig_box).copyTo(*output_component_image);
  }
  else {
    cv::cvtColor(src_image.getMat()(*output_ellipse_boundig_box),
        *output_component_image, cv::COLOR_BGR2GRAY);
  }

  if ( src_mask.empty() ) {
    output_component_mask->release();
  }
  else {
    src_mask.getMat()(*output_ellipse_boundig_box).copyTo(*output_component_mask);
  }

  // create also artificaly drawn reference ellipse mask
  output_ellipse_mask->create(output_component_image->size());
  output_ellipse_mask->setTo(0);
  cv::ellipse(*output_ellipse_mask, *output_ellipse, 255, -1, cv::LINE_8);

  return true;
}

bool c_jovian_derotation::setup_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  // Just extract and save reference jovian image

  bool fOk =
      extract_jovian_image(reference_image, reference_mask,
          &reference_ellipse_,
          &reference_boundig_box_,
          &reference_component_image_,
          &reference_component_mask_,
          &reference_ellipse_mask_);

  if ( !fOk) {
    CF_ERROR("extract_jovian_image(reference_image) fails");
    return false;
  }

  // precompute normalization scale and normalized reference image

  normalization_scale_ =
      std::max(3, eccflow_support_scale_);

  normalize_jovian_image(reference_component_image_,
      reference_component_mask_,
      reference_normalized_image_,
      normalization_scale_);

  return true;
}



bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
{

  //
  // Extract current jovian image
  //

  bool fOk =
      extract_jovian_image(current_image, current_mask,
          &current_ellipse_,
          &current_boundig_box_,
          &current_component_image_,
          &current_component_mask_,
          &current_ellipse_mask_);

  if ( !fOk) {
    CF_ERROR("extract_jovian_image(current_image) fails");
    return false;
  }

  normalize_jovian_image(current_component_image_,
      current_component_mask_,
      current_normalized_image_,
      normalization_scale_);


  if ( enable_debug_ && !debug_path_.empty() ) {
    save_image(current_component_image_, ssprintf("%s/current_component_image_.tiff", debug_path_.c_str()));
    save_image(reference_component_image_, ssprintf("%s/reference_component_image_.tiff", debug_path_.c_str()));
    save_image(current_component_mask_, ssprintf("%s/current_component_mask_.tiff", debug_path_.c_str()));
    save_image(reference_component_mask_, ssprintf("%s/reference_component_mask_.tiff", debug_path_.c_str()));
    save_image(current_normalized_image_, ssprintf("%s/current_normalized_image_.tiff", debug_path_.c_str()));
    save_image(reference_normalized_image_, ssprintf("%s/reference_normalized_image_.tiff", debug_path_.c_str()));
    save_image(current_ellipse_mask_, ssprintf("%s/current_ellipse_mask_.tiff", debug_path_.c_str()));
    save_image(reference_ellipse_mask_, ssprintf("%s/reference_ellipse_mask_.tiff", debug_path_.c_str()));
  }

  //
  // Align current jovian component image to the reference component image
  //

  ecc_.set_motion_type(ECC_MOTION_EUCLIDEAN_SCALED);
  ecc_.set_eps(0.1);
  ecc_.set_min_rho(0.4);

  cv::Matx23d T =
      createEyeTransform(ecc_.motion_type());



  fOk =
      ecch_.align(&ecc_,
          current_component_image_,
          reference_component_image_,
          T,
          current_component_mask_,
          reference_component_mask_);

  if ( !fOk ) {
    CF_ERROR("ecch.align(current_component_image_->reference_component_image_) fails");
    return false;
  }



  //
  // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
  // Select the best candidate.
  //


  const double rotation_step =
      eccflow_support_scale_ > 1 ?
          CV_PI / reference_ellipse_.size.width :
          0.25 * CV_PI / reference_ellipse_.size.width;


  const int num_rotations =
      (int)((max_rotation_ - min_rotation_) / rotation_step);

  double current_cost, best_cost;
  double best_rotation;
  int best_rotation_index;

  CF_DEBUG("c_jovian_derotation: use rotation_step=%g deg",
      rotation_step * 180/ CV_PI);

  cv::Mat2f rotation_remap;
  cv::Mat rotated_current_image;

  for ( int i = 0; i < num_rotations; ++i ) {

    const double l =
          min_rotation_ + i * rotation_step;

    create_jovian_rotation_remap(l,
        reference_ellipse_,
        T,
        reference_component_image_.size(),
        rotation_remap,
        current_total_mask_);

    cv::remap(current_normalized_image_, rotated_current_image,
        rotation_remap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);

    current_cost =
        compute_jovian_derotation_cost(
            reference_normalized_image_,
            rotated_current_image,
            current_total_mask_);

    // CF_DEBUG("R %6d: l=%+.3f cost=%.6f", i, l * 180 / CV_PI, current_cost);

    if ( i == 0 || current_cost < best_cost ) {
      best_cost = current_cost;
      best_rotation = l;
      best_rotation_index = i;
    }
  }

  CF_DEBUG("BEST cost: %g at rotation %d = %g deg",
      best_cost, best_rotation_index, best_rotation * 180 / CV_PI);

  //
  // Use ecc optical flow to perfectly align rotated current component image
  //

  create_jovian_rotation_remap(best_rotation,
      reference_ellipse_,
      T,
      reference_component_image_.size(),
      rotation_remap,
      current_total_mask_);


  cv::compare(current_total_mask_, 0,
      current_total_binary_mask_,
      cv::CMP_GT);

  if ( enable_debug_ && !debug_path_.empty() ) {
    save_image(current_total_mask_, ssprintf("%s/current_rotation_mask_.tiff", debug_path_.c_str()));
    save_image(current_total_binary_mask_, ssprintf("%s/current_total_binary_mask_.tiff", debug_path_.c_str()));
  }

  if ( eccflow_support_scale_ > 1 ) {

    eccflow_.set_max_pyramid_level(eccflow_max_pyramid_level_);
    eccflow_.set_support_scale(eccflow_support_scale_);
    eccflow_.set_normalization_scale(eccflow_normalization_scale_);

    fOk = eccflow_.compute(current_normalized_image_,
        reference_normalized_image_,
        rotation_remap,
        current_ellipse_mask_,
        current_total_binary_mask_);

    if ( !fOk ) {
      CF_ERROR("eccflow_.compute(current_component_image_->reference_component_image_) fails");
      return false;
    }
  }
  else if (false) {

    // FIXME: not sure yet if it is a good idea.
    //  optimize image normalization and update rotation_remap after ecc_.align() to allow this code work

    ecc_.set_motion_type(ECC_MOTION_AFFINE);
    ecc_.set_eps(0.1);
    ecc_.set_min_rho(0.5);


    cv::remap(current_normalized_image_, current_normalized_image_,
        rotation_remap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);

    cv::remap(current_ellipse_mask_, current_ellipse_mask_,
        rotation_remap, cv::noArray(),
        cv::INTER_NEAREST,
        cv::BORDER_REPLICATE);

    cv::Matx23d T2 =
        createEyeTransform(ecc_.motion_type());

    fOk = ecc_.align(current_normalized_image_, reference_normalized_image_, T2,
        current_ellipse_mask_, current_total_binary_mask_);

    CF_DEBUG("ecc_.align(): fOk = %d eps=%g rho=%g iterations=%d",
        fOk, ecc_.current_eps(), ecc_.rho(), ecc_.num_iterations());

  }

  current_total_remap_.create(rotation_remap.size());
  current_total_remap_.setTo(-1);

  cv::add(rotation_remap,
      cv::Scalar(current_boundig_box_.x, current_boundig_box_.y),
      current_total_remap_,
      reference_ellipse_mask_);


//  if ( align_jovian_disk_horizontally_ ) {
//
//    cv::Matx23f T =
//        createEuclideanTransform(reference_ellipse_.center.x, reference_ellipse_.center.y,
//            reference_ellipse_.center.x, reference_ellipse_.center.y,
//            1.0,
//            -reference_ellipse_.angle * CV_PI / 180);
//
//    addRemap(createRemap(ECC_MOTION_EUCLIDEAN, T, current_total_remap_.size(), current_total_remap_.depth()),
//        current_total_remap_);
//
//  }

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

void c_jovian_derotation::set_enable_debug(bool v)
{
  enable_debug_ = v;
}

bool c_jovian_derotation::enable_debug() const
{
  return enable_debug_;
}


