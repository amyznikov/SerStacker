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

static void gradient_magnitude(cv::InputArray src, cv::OutputArray dst)
{
  cv::Mat gx, gy;
  differentiate(src, gx, gy);
  cv::magnitude(gx, gy, dst);
}

// TODO: add horizontal lines to artifical ellipse in appropriate zones
// for better rotation fitting (assuming that jupiter usually shows zonal gradients)
bool detect_jovian_ellipse(cv::InputArray _image, cv::RotatedRect * rc)
{
  cv::Mat gray_image;
  cv::Rect component_rect;
  cv::Mat component_mask;
  cv::Mat component_edge;
  cv::Mat component_image;
  cv::Mat artifical_ellipse;
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


  if( !simple_planetary_disk_detector(gray_image, cv::noArray(), nullptr, 1, &component_rect, &component_mask) ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }

  geo_fill_holes(component_mask, component_mask, 8);
  morphological_gradient(component_mask, component_edge, cv::Mat1b(3, 3, 255), cv::BORDER_CONSTANT);
  cv::findNonZero(component_edge, component_edge_points);
  *rc = cv::fitEllipseAMS(component_edge_points);
  rc->angle -= 90;


  const int l = std::max(0, component_rect.x - 16);
  const int t = std::max(0, component_rect.y - 16);
  const int r = std::min(gray_image.cols - 1, component_rect.x + component_rect.width + 16 - 1);
  const int b = std::min(gray_image.rows - 1, component_rect.y + component_rect.height + 16 - 1);

  component_rect.x = l;
  component_rect.y = t;
  component_rect.width = r - l + 1;
  component_rect.height = b - t + 1;

  /////////////////////////////////////////////////////////////////////////////////////////
  // Copy planetary disk into separate sub-image,
  // compute the image gradient to enchange the disk edge

  gray_image(component_rect).copyTo(component_image, component_mask(component_rect));
  gradient_magnitude(component_image, component_image);

  /////////////////////////////////////////////////////////////////////////////////////////
  // Draw artifical jovian ellipse

  const double A = 0.5 * std::max(rc->size.width, rc->size.height);
  const double B = A * jovian_polar_to_equatorial_axis_ratio;
  const cv::Point2f C = rc->center - cv::Point2f(component_rect.x, component_rect.y);

  artifical_ellipse.create(component_rect.size(), CV_32F);
  artifical_ellipse.setTo(0);
  cv::ellipse(artifical_ellipse, C, cv::Size(A, B), 0, 0, 360, 1.0, 2.0, cv::LINE_AA);
  cv::GaussianBlur(artifical_ellipse, artifical_ellipse, cv::Size(), 2);

  /////////////////////////////////////////////////////////////////////////////////////////
  // Use of ECC to fit artifical jovian ellipse into planetary disk gradient image

  c_ecc_forward_additive ecc(ECC_MOTION_EUCLIDEAN_SCALED);
  c_ecc_pyramide_align ecch(&ecc);

  cv::Matx23f T =
      createEuclideanTransform(C.x, C.y, C.x, C.y, 1.0, rc->angle * CV_PI / 180);

  if ( true ) {
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

  if ( true ) {
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

  CF_DEBUG("Tx=%g Ty=%g scale=%g angle=%g delta_angle=%g",
      Tx, Ty, scale,
      angle * 180 / M_PI,
      angle * 180 / M_PI - rc->angle);

  rc->size.width = 2 * A * scale;
  rc->size.height = 2 * B * scale;
  rc->center.x = CC(0) + component_rect.x;
  rc->center.y = CC(1) + component_rect.y;
  rc->angle = -angle * 180 / M_PI;

  //////////////////

  CF_DEBUG("ERC: center=(%g %g) size=(%g x %g) angle=%g",
      rc->center.x, rc->center.y,
      rc->size.width, rc->size.height,
      rc->angle);

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



const cv::RotatedRect & c_jovian_derotation::reference_ellipse() const
{
  return reference_ellipse_;
}

const cv::Rect & c_jovian_derotation::reference_ellipse_boudig_box() const
{
  return reference_ellipse_boudig_box_;
}

bool c_jovian_derotation::setup_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  if( !detect_jovian_ellipse(reference_image, &reference_ellipse_) ) {
    CF_ERROR("detect_jovian_ellipse(reference_image) fails");
    return false;
  }

  get_jovian_ellipse_bounding_box(reference_image.size(), reference_ellipse_,
      &reference_ellipse_boudig_box_);

  if( reference_image.channels() == 1 ) {
    reference_image.getMat()(reference_ellipse_boudig_box_).copyTo(reference_image_);
  }
  else {
    cv::cvtColor(reference_image.getMat()(reference_ellipse_boudig_box_),
        reference_image_, cv::COLOR_BGR2GRAY);
  }

  if ( reference_mask.empty() ) {
    reference_mask_.release();
  }
  else {
    reference_mask.getMat()(reference_ellipse_boudig_box_).copyTo(reference_mask_);
  }

  return true;
}

bool c_jovian_derotation::compute(cv::InputArray input_image, cv::Mat2f & rmap, cv::InputArray input_mask)
{

  return false;
}

