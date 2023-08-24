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
  cv::Mat tmp;
  cv::Scalar m, s;

  const double sigma = ellipse_width / 10.;
  const int ksize = 2 * (int) ((std::max)(1., 4 * sigma)) + 1;

  cv::GaussianBlur(src, tmp, cv::Size(), sigma, sigma);
  cv::subtract(src, tmp, tmp);

  cv::meanStdDev(tmp, m, s);
  cv::multiply(tmp, 1. / s[0], dst);
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

void c_jovian_derotation::set_jovian_detector_options(const c_jovian_ellipse_detector_options & v)
{
  planetary_detector_.set_options(v);
}

const c_jovian_ellipse_detector_options & c_jovian_derotation::jovian_detector_options() const
{
  return planetary_detector_.options();
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

bool c_jovian_derotation::setup_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if( !planetary_detector_.detect_jovian_disk(reference_image, reference_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(reference_image) fails");
    return false;
  }

  planetary_detector_.gray_image().copyTo(reference_gray_image_);
  planetary_detector_.final_planetary_disk_mask().copyTo(reference_ellipse_mask_);
  reference_ellipse_ = planetary_detector_.final_planetary_disk_ellipse();

  CF_DEBUG("debug_path_='%s'", debug_path_.c_str());

  if ( !debug_path_.empty() ) {

    cv::Mat tmp;
    std::string filename;

    reference_gray_image_.copyTo(tmp);

    drawRotatedRectange(tmp, reference_ellipse_, cv::Scalar::all(1), 1, cv::LINE_8);

    if( !save_image(tmp, reference_ellipse_mask_,
        filename = ssprintf("%s/reference_gray_image_.tiff", debug_path_.c_str())) ) {
      CF_ERROR("save_image('%s') fails", filename.c_str());
      return false;
    }

  }

  return true;
}


bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
{

  INSTRUMENT_REGION("");

  if ( !planetary_detector_.detect_jovian_disk(current_image, current_mask) ) {
    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(current_image) fails");
    return false;
  }

  current_gray_image_ = planetary_detector_.gray_image();
  current_ellipse_mask_ = planetary_detector_.final_planetary_disk_mask();
  current_ellipse_ = planetary_detector_.final_planetary_disk_ellipse();


  if ( !debug_path_.empty() ) {
    save_image(current_gray_image_, ssprintf("%s/current_gray_image_.tiff", debug_path_.c_str()));
    save_image(current_ellipse_mask_, ssprintf("%s/current_ellipse_mask_.tiff", debug_path_.c_str()));

    save_image(reference_gray_image_, ssprintf("%s/reference_gray_image_.tiff", debug_path_.c_str()));
    save_image(reference_ellipse_mask_, ssprintf("%s/reference_ellipse_mask_.tiff", debug_path_.c_str()));


    cv::Mat tmp;
    current_gray_image_.copyTo(tmp);
    cv::ellipse(tmp, current_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/current_gray_image_edge.tiff", debug_path_.c_str()));

    reference_gray_image_.copyTo(tmp);
    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
    save_image(tmp, ssprintf("%s/reference_gray_image_edge.tiff", debug_path_.c_str()));
  }


  // crop and scale jovian images

  std::vector<cv::Mat> reference_pyramid;
  std::vector<cv::Mat> current_pyramid;

  const cv::Rect reference_crop_rect =
      compute_ellipse_crop_box(reference_ellipse_, reference_gray_image_.size());

  const cv::Rect current_crop_rect =
      compute_ellipse_crop_box(current_ellipse_, current_gray_image_.size());

  const int num_orientations =
      std::max(1, std::min(15, num_orientations_));

  const float orientation_step = // radian
      0.25f * num_orientations / reference_ellipse_.size.width;

  const int maxlevel =
      (std::min)(estimate_max_pyramid_level(reference_ellipse_, max_pyramid_level_),
          estimate_max_pyramid_level(current_ellipse_, max_pyramid_level_));

  if ( maxlevel < 1 ) {

    reference_pyramid.emplace_back();
    normalize_jovian_image(reference_gray_image_(reference_crop_rect),
        reference_pyramid.back(),
        reference_ellipse_.size.width);

    current_pyramid.emplace_back();
    normalize_jovian_image(current_gray_image_(current_crop_rect),
        current_pyramid.back(),
        current_ellipse_.size.width);

  }
  else {

    cv::buildPyramid(reference_gray_image_(reference_crop_rect).clone(),
        reference_pyramid,
        maxlevel);

    cv::buildPyramid(current_gray_image_(current_crop_rect).clone(),
        current_pyramid,
        maxlevel);

    for( int i = 0, n = current_pyramid.size(); i < n; ++i ) {

      const double scale_factor =
          1. / (1 << i);

      normalize_jovian_image(reference_pyramid[i],
          reference_pyramid[i],
          reference_ellipse_.size.width * scale_factor);

      normalize_jovian_image(current_pyramid[i],
          current_pyramid[i],
          current_ellipse_.size.width * scale_factor);
    }
  }


  const double initial_orientation =
      (reference_ellipse_.angle - current_ellipse_.angle) * CV_PI / 180;

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

    cv::RotatedRect scaled_reference_ellipse(
        cv::Point2f(reference_ellipse_.center.x - reference_crop_rect.x,
            reference_ellipse_.center.y - reference_crop_rect.y)  * scale_ratio,
        reference_ellipse_.size * scale_ratio,
        reference_ellipse_.angle);

    cv::RotatedRect scaled_current_ellipse(
        cv::Point2f(current_ellipse_.center.x - current_crop_rect.x,
            current_ellipse_.center.y - current_crop_rect.y)  * scale_ratio,
        current_ellipse_.size * scale_ratio,
        current_ellipse_.angle);

    //
    // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
    // Select the best candidate.
    //

    const int num_rotations =
        std::max(5, (int) (scaled_reference_ellipse.size.width *
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
            create_euclidean_transform(cv::Vec2f(scaled_reference_ellipse.center.x, scaled_reference_ellipse.center.y),
                cv::Vec2f(scaled_current_ellipse.center.x, scaled_current_ellipse.center.y),
                -current_orientation,
                scaled_current_ellipse.size.width / scaled_reference_ellipse.size.width);

        create_jovian_rotation_remap(l,
            scaled_current_ellipse,
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


  CF_DEBUG("BEST rotation=%g deg orientation=%g deg",
      best_rotation * 180 / CV_PI,
      best_orientation * 180 / CV_PI);


  cv::Matx23f T =
      create_euclidean_transform(
          cv::Vec2f(reference_ellipse_.center.x, reference_ellipse_.center.y),
          cv::Vec2f(current_ellipse_.center.x, current_ellipse_.center.y),
          -best_orientation,
          current_ellipse_.size.width / reference_ellipse_.size.width);

  create_jovian_rotation_remap(best_rotation,
      current_ellipse_,
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

    const cv::Mat & current_normalized_image =
        current_pyramid.front();

    const cv::Mat & reference_normalized_image =
        reference_pyramid.front();

    CF_DEBUG("current_normalized_image: %dx%d\n"
        "reference_normalized_image: %dx%d\n"
        "current_remap_: %dx%d\n"
        "current_ellipse_mask_: %dx%d\n"
        "reference_ellipse_mask_: %dx%d\n",
        current_normalized_image.cols, current_normalized_image.rows,
        reference_normalized_image.cols, reference_normalized_image.rows,
        current_remap_.cols, current_remap_.rows,
        current_ellipse_mask_.cols, current_ellipse_mask_.rows,
        reference_ellipse_mask_.cols, reference_ellipse_mask_.rows);

    bool fOk =
        eccflow_.compute(current_normalized_image,
            reference_normalized_image,
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


//
//bool c_jovian_derotation::compute(cv::InputArray current_image, cv::InputArray current_mask)
//{
//
//  INSTRUMENT_REGION("");
//
//  if ( !planetary_detector_.detect_jovian_disk(current_image, current_mask) ) {
//    CF_ERROR("ellipse_detector_.detect_jovian_ellipse(current_image) fails");
//    return false;
//  }
//
//  current_ellipse_ = planetary_detector_.planetary_disk_ellipse();
//  current_gray_image_ = planetary_detector_.gray_image();
//  current_ellipse_mask_ = planetary_detector_.aligned_artifial_ellipse_mask();
//
//
//  if ( !debug_path_.empty() ) {
//    save_image(current_gray_image_, ssprintf("%s/current_gray_image_.tiff", debug_path_.c_str()));
//    save_image(current_ellipse_mask_, ssprintf("%s/current_ellipse_mask_.tiff", debug_path_.c_str()));
//
//    save_image(reference_gray_image_, ssprintf("%s/reference_gray_image_.tiff", debug_path_.c_str()));
//    save_image(reference_ellipse_mask_, ssprintf("%s/reference_ellipse_mask_.tiff", debug_path_.c_str()));
//
//
//    cv::Mat tmp;
//    current_gray_image_.copyTo(tmp);
//    cv::ellipse(tmp, current_ellipse_, cv::Scalar::all(1));
//    save_image(tmp, ssprintf("%s/current_gray_image_edge.tiff", debug_path_.c_str()));
//
//    reference_gray_image_.copyTo(tmp);
//    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
//    save_image(tmp, ssprintf("%s/reference_gray_image_edge.tiff", debug_path_.c_str()));
//  }
//
//
//  // crop and scale jovian images
//
//  std::vector<cv::Mat> reference_pyramid;
//  std::vector<cv::Mat> current_pyramid;
//
//  const cv::Rect reference_crop_rect =
//      compute_ellipse_crop_box(reference_ellipse_, reference_gray_image_.size());
//
//  const cv::Rect current_crop_rect =
//      compute_ellipse_crop_box(current_ellipse_, current_gray_image_.size());
//
//  const int num_orientations =
//      std::max(1, std::min(15, num_orientations_));
//
//  const float orientation_step = // radian
//      0.25f * num_orientations / reference_ellipse_.size.width;
//
//  const int maxlevel =
//      (std::min)(estimate_max_pyramid_level(reference_ellipse_, max_pyramid_level_),
//          estimate_max_pyramid_level(current_ellipse_, max_pyramid_level_));
//
//  if ( maxlevel < 1 ) {
//
//    reference_pyramid.emplace_back();
//    normalize_jovian_image(reference_gray_image_(reference_crop_rect),
//        reference_pyramid.back(),
//        reference_ellipse_.size.width);
//
//    current_pyramid.emplace_back();
//    normalize_jovian_image(current_gray_image_(current_crop_rect),
//        current_pyramid.back(),
//        current_ellipse_.size.width);
//
//  }
//  else {
//
//    cv::buildPyramid(reference_gray_image_(reference_crop_rect).clone(),
//        reference_pyramid,
//        maxlevel);
//
//    cv::buildPyramid(current_gray_image_(current_crop_rect).clone(),
//        current_pyramid,
//        maxlevel);
//
//    for( int i = 0, n = current_pyramid.size(); i < n; ++i ) {
//
//      const double scale_factor =
//          1. / (1 << i);
//
//      normalize_jovian_image(reference_pyramid[i],
//          reference_pyramid[i],
//          reference_ellipse_.size.width * scale_factor);
//
//      normalize_jovian_image(current_pyramid[i],
//          current_pyramid[i],
//          current_ellipse_.size.width * scale_factor);
//    }
//  }
//
//
//  const double initial_orientation =
//      (reference_ellipse_.angle - current_ellipse_.angle) * CV_PI / 180;
//
//  double best_orientation =
//      initial_orientation;
//
//  double min_rotation =
//      this->min_rotation_;
//
//  double max_rotation =
//      this->max_rotation_;
//
//  double best_rotation = 0;
//
//  for( int scale = current_pyramid.size() - 1; scale >= 0; --scale ) {
//
//    const cv::Mat & reference_scaled_image =
//        reference_pyramid[scale];
//
//    const cv::Mat & current_scaled_image =
//        current_pyramid[scale];
//
//    const float scale_ratio =
//        1. / (1 << scale);
//
//    CF_DEBUG("reference_scaled_image.size=%dx%d", reference_scaled_image.cols, reference_scaled_image.rows);
//    CF_DEBUG("current_scaled_image.size=%dx%d", current_scaled_image.cols, current_scaled_image.rows);
//
//    cv::RotatedRect scaled_reference_ellipse(
//        cv::Point2f(reference_ellipse_.center.x - reference_crop_rect.x,
//            reference_ellipse_.center.y - reference_crop_rect.y)  * scale_ratio,
//        reference_ellipse_.size * scale_ratio,
//        reference_ellipse_.angle);
//
//    cv::RotatedRect scaled_current_ellipse(
//        cv::Point2f(current_ellipse_.center.x - current_crop_rect.x,
//            current_ellipse_.center.y - current_crop_rect.y)  * scale_ratio,
//        current_ellipse_.size * scale_ratio,
//        current_ellipse_.angle);
//
//    //
//    // Loop over the sequence of rotation angles and compute derotation cost for each rotation angle
//    // Select the best candidate.
//    //
//
//    const int num_rotations =
//        std::max(5, (int) (scaled_reference_ellipse.size.width *
//            (max_rotation - min_rotation) / CV_PI));
//
//    const float rotation_step =
//        (max_rotation - min_rotation) / (num_rotations);
//
//    double current_cost, best_cost;
//
//    for( int i = 0; i <= num_rotations; ++i ) {
//
//      const double l =
//          min_rotation + i * rotation_step;
//
//      for( int j = 0; j < num_orientations; ++j ) {
//
//        const double current_orientation =
//            best_orientation + (j - num_orientations / 2) * orientation_step;
//
//        const cv::Matx23f Tj =
//            create_euclidean_transform(cv::Vec2f(scaled_reference_ellipse.center.x, scaled_reference_ellipse.center.y),
//                cv::Vec2f(scaled_current_ellipse.center.x, scaled_current_ellipse.center.y),
//                -current_orientation,
//                scaled_current_ellipse.size.width / scaled_reference_ellipse.size.width);
//
//        create_jovian_rotation_remap(l,
//            scaled_current_ellipse,
//            Tj,
//            reference_scaled_image.size(),
//            current_remap_,
//            current_wmask_);
//
//        cv::Mat1f current_rotated_normalized_image;
//
//        cv::remap(current_scaled_image, current_rotated_normalized_image,
//            current_remap_, cv::noArray(),
//            cv::INTER_LINEAR,
//            cv::BORDER_REPLICATE);
//
//
//        if( !debug_path_.empty() ) {
//
//          save_image(current_rotated_normalized_image,
//              ssprintf("%s/rotations/current_rotated_normalized_image.%03d.%03d.%03d.tiff",
//                  debug_path_.c_str(),
//                  scale, i, j));
//        }
//
//        current_cost =
//            compute_jovian_derotation_cost(
//                reference_scaled_image,
//                current_rotated_normalized_image,
//                current_wmask_);
//
//        if( (i == 0 && j == 0) || current_cost < best_cost ) {
//          best_cost = current_cost;
//          best_rotation = l;
//          best_orientation = current_orientation;
//        }
//      }
//    }
//
//    min_rotation = best_rotation - rotation_step;
//    max_rotation = best_rotation + rotation_step;
//
//    CF_DEBUG("[scale %d] BEST rotation %g deg orientation: %g deg", scale,
//        best_rotation * 180 / CV_PI,
//        best_orientation * 180 / CV_PI);
//  }
//
//
//  CF_DEBUG("BEST rotation=%g deg orientation=%g deg",
//      best_rotation * 180 / CV_PI,
//      best_orientation * 180 / CV_PI);
//
//
//  cv::Matx23f T =
//      create_euclidean_transform(
//          cv::Vec2f(reference_ellipse_.center.x, reference_ellipse_.center.y),
//          cv::Vec2f(current_ellipse_.center.x, current_ellipse_.center.y),
//          -best_orientation,
//          current_ellipse_.size.width / reference_ellipse_.size.width);
//
//  create_jovian_rotation_remap(best_rotation,
//      current_ellipse_,
//      T,
//      reference_gray_image_.size(),
//      current_remap_,
//      current_wmask_);
//
//  if( !debug_path_.empty() ) {
//
//    cv::Mat tmp;
//    cv::remap(current_gray_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
//    save_image(tmp, ssprintf("%s/current_best_rotated_gray_image.tiff", debug_path_.c_str()));
//
//    reference_gray_image_.copyTo(tmp);
//    cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
//    save_image(tmp, ssprintf("%s/reference_best_rotated_gray_image.tiff", debug_path_.c_str()));
//
//    save_image(current_wmask_, ssprintf("%s/current_wmask_.tiff", debug_path_.c_str()));
//  }
//
//  //
//  // Use ecc optical flow to perfectly align rotated current component image
//  //
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
//      cv::remap(current_gray_image_, tmp, current_remap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//      cv::ellipse(tmp, reference_ellipse_, cv::Scalar::all(1));
//      save_image(tmp, ssprintf("%s/current_eccflow_image.tiff", debug_path_.c_str()));
//    }
//  }
//
//  return true;
//}

void c_jovian_derotation::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string& c_jovian_derotation::debug_path() const
{
  return debug_path_;
}

