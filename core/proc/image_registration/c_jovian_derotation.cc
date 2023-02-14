/*
 * c_jovian_derotation.cc
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#include "c_jovian_derotation.h"
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#include <tbb/tbb.h>

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

}
}


void create_jovian_rotation_remap(double l,
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
      [E, R2I, sa, ca, l, &rmap, &wmask](const tbb_range & r) {

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
            const double phi = asin(xe / q) + l;

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
  if( boundingRect.x < 0 ) {
    boundingRect.x = 0;
  }
  if( boundingRect.y < 0 ) {
    boundingRect.y = 0;
  }
  if( boundingRect.x + boundingRect.width >= image_size.width ) {
    boundingRect.width = image_size.width - boundingRect.x;
  }
  if( boundingRect.y + boundingRect.height >= image_size.height ) {
    boundingRect.height = image_size.height - boundingRect.y;
  }

  *bbox = boundingRect;
}


void c_jovian_derotation::set_jovian_detector_options(const c_jovian_ellipse_detector_options & v)
{
  planetary_detector_.set_options(v);
}

const c_jovian_ellipse_detector_options & c_jovian_derotation::jovian_detector_options() const
{
  return planetary_detector_.options();
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

void c_jovian_derotation::set_num_orientations(int v)
{
  num_orientations_ = v;
}

int c_jovian_derotation::num_orientations() const
{
  return num_orientations_;
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


void c_jovian_derotation::set_force_reference_ellipse(bool v)
{
  force_reference_ellipse_ = v;
}

bool c_jovian_derotation::force_reference_ellipse() const
{
  return force_reference_ellipse_;
}

const cv::RotatedRect& c_jovian_derotation::reference_ellipse() const
{
  return reference_ellipse_;
}

const cv::RotatedRect& c_jovian_derotation::current_ellipse() const
{
  return current_ellipse_;
}

const cv::Mat1b & c_jovian_derotation::reference_ellipse_mask() const
{
  return reference_ellipse_mask_;
}

const cv::Mat1b & c_jovian_derotation::current_ellipse_mask() const
{
  return current_ellipse_mask_;
}

const cv::Mat2f& c_jovian_derotation::current_derotation_remap() const
{
  return current_remap_;
}

const cv::Mat1f& c_jovian_derotation::current_wmask() const
{
  return current_wmask_;
}

double c_jovian_derotation::compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
    const cv::Mat1f & rotated_component_image,
    const cv::Mat1f & wmask,
    cv::Mat1f * output_difference_image /* optional for debug */)
{
  INSTRUMENT_REGION("");

  double total_cost = 0;
  double total_weight = 0;

  if( output_difference_image ) {
    output_difference_image->create(wmask.size());
    output_difference_image->setTo(0);
  }

  for( int y = 0; y < wmask.rows; ++y ) {
    for( int x = 0; x < wmask.cols; ++x ) {
      if( wmask[y][x] > 0 ) {

        const double difference =
            wmask[y][x] * abs(reference_component_image[y][x] - rotated_component_image[y][x]);

        total_cost += difference;
        total_weight += wmask[y][x];

        if( output_difference_image ) {
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

  if( !planetary_detector_.detect_jovian_disk(reference_image, reference_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(reference_image) fails");
    return false;
  }

  reference_ellipse_ = planetary_detector_.planetary_disk_ellipse();
  planetary_detector_.gray_image().copyTo(reference_gray_image_);
  planetary_detector_.normalized_image().copyTo(reference_normalized_image_);
  planetary_detector_.aligned_artifial_ellipse_mask().copyTo(reference_ellipse_mask_);

  return true;
}

bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
{

  INSTRUMENT_REGION("");

  if ( !planetary_detector_.detect_jovian_disk(current_image, current_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(current_image) fails");
    return false;
  }

  current_ellipse_ = planetary_detector_.planetary_disk_ellipse();
  planetary_detector_.gray_image().copyTo(current_gray_image_);
  planetary_detector_.normalized_image().copyTo(current_normalized_image_);
  planetary_detector_.aligned_artifial_ellipse_mask().copyTo(current_ellipse_mask_);


//
//  if ( force_reference_ellipse_ ) {
//    current_ellipse_ = reference_ellipse_;
//    current_bounding_box_ = reference_bounding_box_;
//  }
//
  if ( !debug_path_.empty() ) {
    save_image(current_gray_image_, ssprintf("%s/current_gray_image_.tiff", debug_path_.c_str()));
    save_image(current_normalized_image_, ssprintf("%s/current_normalized_image_.tiff", debug_path_.c_str()));
    save_image(current_ellipse_mask_, ssprintf("%s/current_ellipse_mask_.tiff", debug_path_.c_str()));

    save_image(reference_gray_image_, ssprintf("%s/reference_gray_image_.tiff", debug_path_.c_str()));
    save_image(reference_normalized_image_, ssprintf("%s/reference_normalized_image_.tiff", debug_path_.c_str()));
    save_image(reference_ellipse_mask_, ssprintf("%s/reference_ellipse_mask_.tiff", debug_path_.c_str()));


    cv::Mat tmp;
    current_normalized_image_.copyTo(tmp);
    cv::ellipse(tmp, current_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/orig_current_normalized_image_.tiff", debug_path_.c_str()));

    reference_normalized_image_.copyTo(tmp);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/orig_reference_normalized_image_.tiff", debug_path_.c_str()));
  }




  //
  // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
  // Select the best candidate.
  //

  const int num_rotations =
        reference_ellipse_.size.width * (max_rotation_ - min_rotation_) / CV_PI;

  const double rotation_step =
    (max_rotation_ - min_rotation_) / num_rotations;

  const int num_orientations = std::max(1, std::min(15, num_orientations_));
  const double orientation_step = // radian
      0.25 * num_orientations / reference_ellipse_.size.width;

  double current_cost, best_cost;
  double best_rotation;
  double best_orientation;
  int best_rotation_index;

  CF_DEBUG("c_jovian_derotation:\n"
      "use num_rotations=%3d rotation_step=%g deg\n"
      "num_orientations =%3d orientation_step=%g deg",
      num_rotations,
      rotation_step * 180 / CV_PI,
      num_orientations,
      orientation_step * 180 / CV_PI);

  // current -> reference ellipse transform

  double initial_orientation =
      (reference_ellipse_.angle - current_ellipse_.angle) * CV_PI / 180;


  if( !debug_path_.empty() ) {

    c_euclidean_image_transform transform(
        cv::Vec2f(reference_ellipse_.center.x, reference_ellipse_.center.y),
        cv::Vec2f(current_ellipse_.center.x, current_ellipse_.center.y),
        -initial_orientation,
        current_ellipse_.size.width / reference_ellipse_.size.width);

    cv::Mat tmp;
    cv::Mat2f rmap;
    transform.create_remap(rmap, reference_gray_image_.size());

    cv::remap(current_normalized_image_, tmp, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/rmap_current_normalized_image_.tiff", debug_path_.c_str()));

    cv::remap(current_gray_image_, tmp, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/rmap_current_gray_image_.tiff", debug_path_.c_str()));
  }


  cv::Mat1f current_rotated_normalized_image;
  cv::Mat1f current_difference_image;

  for( int i = 0; i < num_rotations; ++i ) {

    INSTRUMENT_REGION("derotation_body");

    const double l =
        min_rotation_ + i * rotation_step;

    for( int j = 0; j < num_orientations; ++j ) {

      const double current_orientation =
          initial_orientation + (j - num_orientations / 2) * orientation_step;

      const cv::Matx23f Tj =
          create_euclidean_transform(
              cv::Vec2f(reference_ellipse_.center.x, reference_ellipse_.center.y),
              cv::Vec2f(current_ellipse_.center.x, current_ellipse_.center.y),
              -current_orientation,
              current_ellipse_.size.width / reference_ellipse_.size.width);

      create_jovian_rotation_remap(l,
          reference_ellipse_,
          Tj,
          reference_gray_image_.size(),
          current_remap_,
          current_wmask_);

      cv::remap(current_normalized_image_, current_rotated_normalized_image,
          current_remap_, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REPLICATE);

      if( !debug_path_.empty() ) {

        save_image(current_rotated_normalized_image,
            ssprintf("%s/rotations/current_rotated_normalized_image.%03d.%03d.tiff",
                debug_path_.c_str(),
                i, j));
      }

      current_cost =
          compute_jovian_derotation_cost(
              reference_normalized_image_,
              current_rotated_normalized_image,
              current_wmask_,
              debug_path_.empty() ?
                  nullptr :
                  &current_difference_image);

      if( !debug_path_.empty() ) {

        CF_DEBUG("R %6d: l=%+.3f o=%+.3f cost=%.6f", i,
            l * 180 / CV_PI,
            current_orientation * 180 / CV_PI,
            current_cost);

        save_image(current_difference_image,
            ssprintf("%s/diffs/current_difference_image.%03d.%03d.tiff",
                debug_path_.c_str(),
                i, j));
      }


      if( (i == 0 && j == 0) || current_cost < best_cost ) {
        best_cost = current_cost;
        best_rotation_index = i;
        best_rotation = l;
        best_orientation = current_orientation;
      }
    }
  }

  CF_DEBUG("BEST cost: %g at rotation %d: %g deg orientation: %g deg",
      best_cost, best_rotation_index, best_rotation * 180 / CV_PI,
      best_orientation * 180 / CV_PI);


  cv::Matx23f T =
      create_euclidean_transform(
          cv::Vec2f(reference_ellipse_.center.x, reference_ellipse_.center.y),
          cv::Vec2f(current_ellipse_.center.x, current_ellipse_.center.y),
          -best_orientation,
          current_ellipse_.size.width / reference_ellipse_.size.width);

  create_jovian_rotation_remap(best_rotation,
      reference_ellipse_,
      T,
      reference_gray_image_.size(),
      current_remap_,
      current_wmask_);

  if( !debug_path_.empty() ) {

    cv::Mat tmp;
    cv::remap(current_gray_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/current_best_rotated_gray_image.tiff", debug_path_.c_str()));

    reference_gray_image_.copyTo(tmp);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/reference_best_rotated_gray_image.tiff", debug_path_.c_str()));

    save_image(current_wmask_, ssprintf("%s/current_wmask_.tiff", debug_path_.c_str()));
  }

  //
  // Use ecc optical flow to perfectly align rotated current component image
  //

  if( eccflow_support_scale_ > 1 ) {

    INSTRUMENT_REGION("eccflow_.compute()");

    eccflow_.set_max_pyramid_level(eccflow_max_pyramid_level_);
    eccflow_.set_support_scale(eccflow_support_scale_);
    eccflow_.set_normalization_scale(eccflow_normalization_scale_);

    bool fOk =
        eccflow_.compute(current_normalized_image_,
            reference_normalized_image_,
            current_remap_,
            current_ellipse_mask_,
            reference_ellipse_mask_);

    if( !fOk ) {
      CF_ERROR("eccflow_.compute() fails");
      return false;
    }

    if( !debug_path_.empty() ) {
      cv::Mat tmp;
      cv::remap(current_gray_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
      save_image(tmp, ssprintf("%s/current_eccflow_image.tiff", debug_path_.c_str()));
    }
  }

  return true;
}

void c_jovian_derotation::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string& c_jovian_derotation::debug_path() const
{
  return debug_path_;
}

//////////////////////////

