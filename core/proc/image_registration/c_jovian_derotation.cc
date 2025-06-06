/*
 * c_jovian_derotation.cc
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#include "c_jovian_derotation.h"
#include <core/proc/unsharp_mask.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TBB && !defined(Q_MOC_RUN)
# include <tbb/tbb.h>
#endif

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

void create_jovian_rotation_remap(double l,
    const cv::RotatedRect & current_ellipse,
    const cv::Matx23d & R2I,
    const cv::Size & size,
    cv::Mat2f & rmap,
    cv::Mat1f & wmask)
{
  INSTRUMENT_REGION("");

  const cv::RotatedRect &E =
      current_ellipse;

  const float ca =
      std::cos(E.angle * CV_PI / 180);

  const float sa =
      std::sin(E.angle * CV_PI / 180);

  const float A = E.size.width / 2;
  const float B = E.size.height / 2;
  const float x0 = E.center.x;
  const float y0 = E.center.y;

  rmap.create(size);
  wmask.create(size);

#if 0 // HAVE_TBB && !defined(Q_MOC_RUN)
  typedef tbb::blocked_range<int> tbb_range;
  tbb::parallel_for(tbb_range(0, size.height, 64),
      [A, B, x0, y0, R2I, ca, sa, l, &rmap, &wmask](const tbb_range & r) {
        for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
#else
  for( int y = 0; y < size.height; ++y ) {
#endif

    for( int x = 0; x < size.width; ++x ) {

      // reference -> current image coordinates
      const float xi = R2I(0, 0) * x + R2I(0, 1) * y + R2I(0, 2);
      const float yi = R2I(1, 0) * x + R2I(1, 1) * y + R2I(1, 2);

      float xe = ((xi - x0) * ca + (yi - y0) * sa) / A;
      float ye = (-(xi - x0) * sa + (yi - y0) * ca) / B;

      if( xe * xe + ye * ye >= 1 ) {
        rmap[y][x][0] = -1;
        rmap[y][x][1] = -1;
        wmask[y][x] = 0;
        continue;
      }

      const float q = sqrt(1. - ye * ye);
      const float phi = asin(xe / q) + l;

      if( (phi <= -CV_PI / 2) || (phi >= CV_PI / 2) ) {
        rmap[y][x][0] = -1;
        rmap[y][x][1] = -1;
        wmask[y][x] = 0;
        continue;
      }

      xe = A * q * sin(phi);
      ye = B * ye;

      const float xx = xe * ca - ye * sa + x0;
      const float yy = xe * sa + ye * ca + y0;

      rmap[y][x][0] = xx;
      rmap[y][x][1] = yy;
      wmask[y][x] = cos(phi);
    }
  }

#if 0 // HAVE_TBB && !defined(Q_MOC_RUN)
      });
#endif

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

static void normalize_jovian_image(const cv::Mat & src, cv::Mat & dst, const int ellipse_width)
{
  unsharp_mask(src, cv::noArray(), dst, 5, 0.95, -1, -1);

//  cv::Mat tmp;
//  cv::Scalar m, s;
//
//  const double sigma = ellipse_width / 10.;
//  const int ksize = 2 * (int) ((std::max)(1., 4 * sigma)) + 1;
//
//  cv::GaussianBlur(src, tmp, cv::Size(), sigma, sigma);
//  cv::subtract(src, tmp, tmp);
//
//  cv::meanStdDev(tmp, m, s);
//  cv::multiply(tmp, 1. / s[0], dst);
}

static void drawRotatedRectange(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{

//  if ( 1 ) {
//    cv::rectangle(image, compute_ellipse_bounding_box(rc),
//        cv::Scalar(0, 0, 1),
//        3);
//  }


  cv::Point2f pts[4];
  rc.points(pts);

  cv::ellipse(image, rc, color, thickness, lineType);

  for( int i = 0; i < 4; i++ ) {
    cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
  }

  cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
  cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
}

void c_jovian_derotation::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string& c_jovian_derotation::debug_path() const
{
  return debug_path_;
}

void c_jovian_derotation::set_detector_options(const c_jovian_ellipse_detector_options & v)
{
  jovian_detector_.set_options(v);
}

const c_jovian_ellipse_detector_options & c_jovian_derotation::detector_options() const
{
  return jovian_detector_.options();
}

void c_jovian_derotation::set_max_pyramid_level(int v)
{
  max_pyramid_level_ = v;
}

int c_jovian_derotation::max_pyramid_level() const
{
  return max_pyramid_level_;
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

const cv::RotatedRect& c_jovian_derotation::planetary_disk_ellipse() const
{
  return jovian_ellipse_;
}

const cv::Mat1b & c_jovian_derotation::planetary_disk_ellipse_mask() const
{
  return jovian_ellipse_mask_;
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

int c_jovian_derotation::estimate_max_pyramid_level(const cv::RotatedRect & ellipse, int maxlevel = -1)
{
  const int min_size = 64;

  cv::Size size =
      ellipse.boundingRect().size();

  int l = 0;

  while ((maxlevel < 0 || l < maxlevel) && std::min(size.width, size.height) > min_size) {

    size.width = (size.width + 1) / 2;
    size.height = (size.height + 1) / 2;

    ++l;
  }

  return l;
}


bool c_jovian_derotation::detect_jovian_ellipse(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");


  if( !jovian_detector_.detect_jovian_disk(reference_image, reference_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(reference_image) fails");
    return false;
  }

  jovian_ellipse_mask_ = jovian_detector_.final_planetary_disk_mask();
  jovian_ellipse_ = jovian_detector_.final_planetary_disk_ellipse();
  jovian_crop_ = compute_ellipse_crop_box(jovian_ellipse_, reference_image.size());

//  CF_DEBUG("jovian_crop_: x=%d y=%d w=%d h=%d",
//      jovian_crop_.x, jovian_crop_.y,
//      jovian_crop_.width, jovian_crop_.height);
//
//  return false;
//

  CF_DEBUG("debug_path_: %s", debug_path_.c_str());

  if ( !debug_path_.empty() ) {

    std::string fname;
    cv::Mat tmp;

    cv::cvtColor(jovian_detector_.gray_image(), tmp, cv::COLOR_GRAY2BGR);
    drawRotatedRectange(tmp, jovian_ellipse_, CV_RGB(0, 1, 0));

    if( !save_image(tmp, reference_mask_,
        fname = ssprintf("%s/reference_ellipse.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", fname.c_str());
      return false;
    }

    CF_DEBUG("debug images saved to '%s'", fname.c_str());
  }

  return true;
}


bool c_jovian_derotation::setup_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  CF_DEBUG("enter c_jovian_derotation::setup_reference_image()");

  if ( reference_image.channels() == 1 ) {
    reference_image.getMat().copyTo(reference_image_);
  }
  else {
    cv::cvtColor(reference_image, reference_image_, cv::COLOR_BGR2GRAY);
  }

  if ( reference_mask.empty() ) {
    reference_mask_.release();
  }
  else {
    reference_mask.getMat().copyTo(reference_mask_);
  }

  if ( !debug_path_.empty() ) {

    std::string fname;
    cv::Mat tmp;

    if ( !save_image(reference_image_, reference_mask_,
        fname = ssprintf("%s/reference_image.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", fname.c_str());
      return false;
    }


    cv::cvtColor(reference_image_, tmp, cv::COLOR_GRAY2BGR);
    drawRotatedRectange(tmp, jovian_ellipse_, CV_RGB(0, 1, 0));

    if( !save_image(tmp, reference_mask_,
        fname = ssprintf("%s/reference_ellipse.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", fname.c_str());
      return false;
    }

  }

  CF_DEBUG("leave c_jovian_derotation::setup_reference_image()");

  return true;
}


bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
{

  INSTRUMENT_REGION("");

  CF_DEBUG("enter c_jovian_derotation::compute()");

  if ( current_image.channels() == 1 ) {
    current_image.getMat().copyTo(current_image_);
  }
  else {
    cv::cvtColor(current_image, current_image_, cv::COLOR_BGR2GRAY);
  }

  if ( current_mask.empty() ) {
    current_mask_.release();
  }
  else {
    current_mask.getMat().copyTo(current_mask_);
  }

  if ( !debug_path_.empty() ) {

    std::string fname;
    cv::Mat tmp;

    if ( !save_image(current_image_, current_mask_,
        fname = ssprintf("%s/current_image.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", fname.c_str());
      return false;
    }


    cv::cvtColor(current_image_, tmp, cv::COLOR_GRAY2BGR);
    drawRotatedRectange(tmp, jovian_ellipse_, CV_RGB(0, 1, 0));

    if( !save_image(tmp, current_mask_,
        fname = ssprintf("%s/current_ellipse.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", fname.c_str());
      return false;
    }
  }


  // crop and scale jovian images

  std::vector<cv::Mat> reference_pyramid;
  std::vector<cv::Mat> current_pyramid;

  const int num_orientations =
      std::max(1, std::min(15, num_orientations_));

  const float orientation_step = // radian
      0.25f * num_orientations / jovian_ellipse_.size.width;

  const int maxlevel =
      (std::min)(estimate_max_pyramid_level(jovian_ellipse_, max_pyramid_level_),
          estimate_max_pyramid_level(jovian_ellipse_, max_pyramid_level_));

  CF_DEBUG("maxlevel=%d", maxlevel);

  CF_DEBUG("jovian_crop_: x=%d y=%d w=%d h=%d r=%d b=%d",
      jovian_crop_.x, jovian_crop_.y,
      jovian_crop_.width, jovian_crop_.height,
      jovian_crop_.x + jovian_crop_.width, jovian_crop_.y +jovian_crop_.height);

  CF_DEBUG("reference_image_: %dx%d",
      reference_image_.cols, reference_image_.rows);
  CF_DEBUG("current_image_: %dx%d",
      current_image_.cols, current_image_.rows);

  if ( maxlevel < 1 ) {

    reference_pyramid.emplace_back();
    normalize_jovian_image(reference_image_(jovian_crop_),
        reference_pyramid.back(),
        jovian_ellipse_.size.width);

    current_pyramid.emplace_back();
    normalize_jovian_image(current_image_(jovian_crop_),
        current_pyramid.back(),
        jovian_ellipse_.size.width);

  }
  else {

    cv::buildPyramid(reference_image_(jovian_crop_).clone(),
        reference_pyramid,
        maxlevel);

    cv::buildPyramid(current_image_(jovian_crop_).clone(),
        current_pyramid,
        maxlevel);

    for( int i = 0, n = current_pyramid.size(); i < n; ++i ) {

      const double scale_factor =
          1. / (1 << i);

      normalize_jovian_image(reference_pyramid[i],
          reference_pyramid[i],
          jovian_ellipse_.size.width * scale_factor);

      normalize_jovian_image(current_pyramid[i],
          current_pyramid[i],
          jovian_ellipse_.size.width * scale_factor);
    }
  }


  const double initial_orientation = 0;
      //(reference_ellipse_.angle - current_ellipse_.angle) * CV_PI / 180;

  double best_orientation =
      initial_orientation;

  double min_rotation =
      this->min_rotation_;

  double max_rotation =
      this->max_rotation_;

  double best_rotation = 0;

  for( int scale = current_pyramid.size() - 1; scale >= 0; --scale ) {

    const cv::Mat & reference_scaled_image =
        reference_pyramid[scale];

    const cv::Mat & current_scaled_image =
        current_pyramid[scale];

    const float scale_ratio =
        1. / (1 << scale);

    CF_DEBUG("reference_scaled_image.size=%dx%d", reference_scaled_image.cols, reference_scaled_image.rows);
    CF_DEBUG("current_scaled_image.size=%dx%d", current_scaled_image.cols, current_scaled_image.rows);

    cv::RotatedRect scaled_ellipse(
        cv::Point2f(jovian_ellipse_.center.x - jovian_crop_.x,
            jovian_ellipse_.center.y - jovian_crop_.y)  * scale_ratio,
            jovian_ellipse_.size * scale_ratio,
            jovian_ellipse_.angle);

    //
    // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
    // Select the best candidate.
    //

    const int num_rotations =
        std::max(5, (int) (scaled_ellipse.size.width *
            (max_rotation - min_rotation) / CV_PI));

    const float rotation_step =
        (max_rotation - min_rotation) / (num_rotations);

    double current_cost, best_cost;

    for( int i = 0; i <= num_rotations; ++i ) {

      const double l =
          min_rotation + i * rotation_step;

      for( int j = 0; j < num_orientations; ++j ) {

        const double current_orientation =
            best_orientation + (j - num_orientations / 2) * orientation_step;

        const cv::Matx23f Tj =
            create_euclidean_transform(cv::Vec2f(scaled_ellipse.center.x, scaled_ellipse.center.y),
                cv::Vec2f(scaled_ellipse.center.x, scaled_ellipse.center.y),
                -current_orientation,
                1);

        create_jovian_rotation_remap(l,
            scaled_ellipse,
            Tj,
            reference_scaled_image.size(),
            current_remap_,
            current_wmask_);

        cv::Mat1f current_rotated_normalized_image;

        cv::remap(current_scaled_image, current_rotated_normalized_image,
            current_remap_, cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REPLICATE);


        if( !debug_path_.empty() ) {

          save_image(current_rotated_normalized_image,
              ssprintf("%s/rotations/current_rotated_normalized_image.%03d.%03d.%03d.tiff",
                  debug_path_.c_str(),
                  scale, i, j));
        }

        current_cost =
            compute_jovian_derotation_cost(
                reference_scaled_image,
                current_rotated_normalized_image,
                current_wmask_);

        if( (i == 0 && j == 0) || current_cost < best_cost ) {
          best_cost = current_cost;
          best_rotation = l;
          best_orientation = current_orientation;
        }
      }
    }

    min_rotation = best_rotation - rotation_step;
    max_rotation = best_rotation + rotation_step;

    CF_DEBUG("[scale %d] BEST rotation %g deg orientation: %g deg", scale,
        best_rotation * 180 / CV_PI,
        best_orientation * 180 / CV_PI);
  }


  CF_DEBUG("\n\n XXXX: FINAL BEST rotation=%g deg orientation=%g deg",
      best_rotation * 180 / CV_PI,
      best_orientation * 180 / CV_PI);

  cv::Matx23f T =
      create_euclidean_transform(
          cv::Vec2f(jovian_ellipse_.center.x, jovian_ellipse_.center.y),
          cv::Vec2f(jovian_ellipse_.center.x, jovian_ellipse_.center.y),
          -best_orientation,
          1);

  create_jovian_rotation_remap(best_rotation,
      jovian_ellipse_,
      T,
      reference_image_.size(),
      current_remap_,
      current_wmask_);

  if( !debug_path_.empty() ) {

    cv::Mat tmp;
    cv::remap(current_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::ellipse(tmp, jovian_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/current_best_rotated_gray_image.tiff", debug_path_.c_str()));

    reference_image_.copyTo(tmp);
    cv::ellipse(tmp, jovian_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/reference_best_rotated_gray_image.tiff", debug_path_.c_str()));

    save_image(current_wmask_, ssprintf("%s/current_wmask_.tiff", debug_path_.c_str()));
  }

  //
  // Use ecc optical flow to perfectly align rotated current component image
  //

//  if( eccflow_support_scale_ > 1 ) {
//
//    INSTRUMENT_REGION("eccflow_.compute()");
//
//    eccflow_.set_max_pyramid_level(eccflow_max_pyramid_level_);
//    eccflow_.set_support_scale(eccflow_support_scale_);
//    eccflow_.set_normalization_scale(eccflow_normalization_scale_);
//
//    const cv::Mat & current_normalized_image =
//        current_pyramid.front();
//
//    const cv::Mat & reference_normalized_image =
//        reference_pyramid.front();
//
//    CF_DEBUG("current_normalized_image: %dx%d\n"
//        "reference_normalized_image: %dx%d\n"
//        "current_remap_: %dx%d\n"
//        "current_ellipse_mask_: %dx%d\n"
//        "reference_ellipse_mask_: %dx%d\n",
//        current_normalized_image.cols, current_normalized_image.rows,
//        reference_normalized_image.cols, reference_normalized_image.rows,
//        current_remap_.cols, current_remap_.rows,
//        current_ellipse_mask_.cols, current_ellipse_mask_.rows,
//        reference_ellipse_mask_.cols, reference_ellipse_mask_.rows);
//
//    bool fOk =
//        eccflow_.compute(current_normalized_image,
//            reference_normalized_image,
//            current_remap_,
//            current_ellipse_mask_,
//            reference_ellipse_mask_);
//
//    if( !fOk ) {
//      CF_ERROR("eccflow_.compute() fails");
//      return false;
//    }
//
//    if( !debug_path_.empty() ) {
//      cv::Mat tmp;
//      cv::remap(current_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//      cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
//      save_image(tmp, ssprintf("%s/current_eccflow_image.tiff", debug_path_.c_str()));
//    }
//  }

  CF_DEBUG("leave c_jovian_derotation::compute()");

  return true;
}

bool c_jovian_derotation::compute(double current_tstamp_sec, double target_tstamp_sec)
{
  // Jupiter daily rotation period is 9h 55m 30s.

  static constexpr double rotation_period_sec =
      9. * 3660 + 55. * 60 + 30.;

  const double rotation_angle_deg =
      360 * (current_tstamp_sec - target_tstamp_sec) / rotation_period_sec;

  return compute(rotation_angle_deg);
}


bool c_jovian_derotation::compute(double zrotation_deg)
{
  return false;
}


