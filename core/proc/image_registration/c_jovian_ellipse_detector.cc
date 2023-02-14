/*
 * c_jovian_ellipse_detector.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector.h"
#include "image_transform.h"
#include "ecc_motion_model.h"
#include "ecc2.h"

#include <core/proc/autoclip.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/pyrscale.h>
#include <core/debug.h>


namespace {

/**
 * Given binary mask src this routine extracts the list of connected components,
 *  selects maximal one by area and returns its bounding rectangle and mask
 *  */
bool get_maximal_connected_component(const cv::Mat1b & src,
    cv::Rect * bounding_rect,
    cv::Mat * component_mask = nullptr)
{
  cv::Mat1i labels, stats;
  cv::Mat1d centroids;
  int N;

  if ( (N = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, labels.type())) < 2 ) {
    return false;
  }

  struct ss {
    int label, area;
  };

  std::vector<ss> cstats;

  for ( int i = 1; i < N; ++i ) {
    cstats.emplace_back();
    cstats.back().label = i;
    cstats.back().area = stats[i][cv::CC_STAT_AREA];
  }

  if ( cstats.size() > 1 ) {
    std::sort(cstats.begin(), cstats.end(),
        [](const ss & p, const ss & n) {
          return p.area > n.area;
        });
  }

  if ( cstats[0].area < 4 ) {
    CF_DEBUG("Small area: %d", cstats[0].area);
    return false;
  }

  bounding_rect->x = stats[cstats[0].label][cv::CC_STAT_LEFT];
  bounding_rect->y = stats[cstats[0].label][cv::CC_STAT_TOP];
  bounding_rect->width = stats[cstats[0].label][cv::CC_STAT_WIDTH];
  bounding_rect->height = stats[cstats[0].label][cv::CC_STAT_HEIGHT];

  if( component_mask ) {
    cv::compare(labels, cstats[0].label,
        *component_mask,
        cv::CMP_EQ);
  }


  return true;
}

/**
 * Threshold given grayscale input image and extract maximal area
 * connected component mask assuming it is planetary disk on a frame
 */
bool detect_planetary_disk(cv::InputArray input_image, cv::InputArray input_mask,
    double gbsigma,
    double stdev_factor,
    cv::Mat * output_component_mask)
{
  cv::Mat src, gray;
  cv::Mat1b comp;
  double noise;
  double threshold;

  cv::Rect rc, rcc;

  if ( (src = input_image.getMat()).empty() ) {
    return false;
  }

  if ( src.channels() == 1 ) {
    src.copyTo(gray);
  }
  else if ( src.channels() == 4 ) {
    cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if ( gbsigma > 0 ) {
    GaussianBlur(gray, gray, cv::Size(0, 0), gbsigma);
  }

  autoclip(gray, input_mask, gray, 1, 99, 0, 255);
  gray.convertTo(gray, CV_8U);

  cv::threshold(gray, comp, 0, 255, cv::THRESH_TRIANGLE);

  morphological_smooth_close(comp, comp,
      cv::Mat1b(5, 5, 255),
      cv::BORDER_CONSTANT);

  if ( !input_mask.empty() ) {
    comp.setTo(0, ~input_mask.getMat());
  }

  if ( !get_maximal_connected_component(comp, &rc, &comp) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
    return false;
  }

  if( stdev_factor > 0 ) {

    cv::Scalar m, s;
    double min = 0, max = 0;

    cv::meanStdDev(gray, m, s, input_mask);
    cv::minMaxLoc(gray, &min, &max, nullptr, nullptr, input_mask);

    const double threshold = s[0] * stdev_factor;

    // CF_DEBUG("s[0]=%g threshold=%g min=%g max=%g", s[0], threshold,  min, max);

    cv::bitwise_and(comp, gray > threshold, comp);
  }

  morphological_smooth_close(comp, comp, cv::Mat1b(3, 3, 255));
  geo_fill_holes(comp, comp, 8);

  if ( !get_maximal_connected_component(comp, &rc, output_component_mask) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
    return false;
  }

  return true;
}



template<class ImageTransformType>
bool align_images_ecc(ImageTransformType * transform, cv::InputArray input_image, cv::InputArray reference_image)
{
  typename c_ecc_motion<ImageTransformType>::motion_model
    motion_model(transform);

  ecc2::c_ecc_forward_additive ecc(&motion_model);

  ecc.set_max_eps(0.1);
  ecc.set_min_rho(0.5);
  ecc.set_max_iterations(30);
  ecc.set_input_smooth_sigma(1);
  ecc.set_reference_smooth_sigma(1);

  if( !ecc.align(input_image, reference_image) ) {
    CF_ERROR("ecc.align() fails: failed: %d rho=%g eps=%g num_iterations=%d",
        ecc.failed(),
        ecc.rho(),
        ecc.eps(),
        ecc.num_iterations());

    return false;
  }

  return true;
}



} // namespace


void c_jovian_ellipse_detector::set_normalization_scale(int v)
{
  options_.normalization_scale = v;
}

int c_jovian_ellipse_detector::normalization_scale() const
{
  return options_.normalization_scale;
}

void c_jovian_ellipse_detector::set_stdev_factor(double v)
{
  options_.stdev_factor = v;
}

double c_jovian_ellipse_detector::stdev_factor() const
{
  return options_.stdev_factor;
}

void c_jovian_ellipse_detector::set_normalization_blur(double v)
{
  options_.normalization_blur = v;
}

double c_jovian_ellipse_detector::normalization_blur() const
{
  return options_.normalization_blur;
}

void c_jovian_ellipse_detector::set_gradient_blur(double v)
{
  options_.gradient_blur = v;
}

double c_jovian_ellipse_detector::gradient_blur() const
{
  return options_.gradient_blur;
}

void c_jovian_ellipse_detector::set_enable_debug_images(bool v)
{
  enable_debug_images_ = v;
}

bool c_jovian_ellipse_detector::enable_debug_images() const
{
  return enable_debug_images_;
}

void c_jovian_ellipse_detector::set_options(const c_jovian_ellipse_detector_options & v)
{
  options_ = v;
}

const c_jovian_ellipse_detector_options & c_jovian_ellipse_detector::options() const
{
  return options_;
}

c_jovian_ellipse_detector_options & c_jovian_ellipse_detector::options()
{
  return options_;
}

const cv::Mat& c_jovian_ellipse_detector::detected_planetary_disk_mask() const
{
  return detected_planetary_disk_mask_;
}

const cv::Mat& c_jovian_ellipse_detector::detected_planetary_disk_edge() const
{
  return detected_planetary_disk_edge_;
}

const cv::RotatedRect& c_jovian_ellipse_detector::ellipseAMS() const
{
  return ellipseAMS_;
}

const cv::Mat& c_jovian_ellipse_detector::initial_artifial_ellipse_edge() const
{
  return initial_artifial_ellipse_edge_;
}

const cv::Mat& c_jovian_ellipse_detector::remapped_artifial_ellipse_edge() const
{
  return remapped_artifial_ellipse_edge_;
}

const cv::Mat& c_jovian_ellipse_detector::aligned_artifial_ellipse_edge() const
{
  return aligned_artifial_ellipse_edge_;
}

const cv::Mat1b& c_jovian_ellipse_detector::aligned_artifial_ellipse_edge_mask() const
{
  return aligned_artifial_ellipse_edge_mask_;
}

const cv::Mat1b& c_jovian_ellipse_detector::aligned_artifial_ellipse_mask() const
{
  return aligned_artifial_ellipse_mask_;
}

const cv::RotatedRect& c_jovian_ellipse_detector::ellipseAMS2() const
{
  return ellipseAMS2_;
}

const cv::RotatedRect& c_jovian_ellipse_detector::planetary_disk_ellipse() const
{
  return ellipse_;
}

const cv::Mat& c_jovian_ellipse_detector::gray_image() const
{
  return gray_image_;
}

const cv::Mat& c_jovian_ellipse_detector::normalized_image() const
{
  return normalized_image_;
}

bool c_jovian_ellipse_detector::detect_jovian_disk(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  cv::Mat gray_image;
  std::vector<cv::Point2f> component_edge_points;
  cv::Scalar m, s;

  /////////////////////////////////////////////////////////////////////////////////////////
  // Convert input image to gray scale

  if( _image.channels() == 1 ) {
    _image.getMat().copyTo(gray_image);
  }
  else {
    cv::cvtColor(_image, gray_image, cv::COLOR_BGR2GRAY);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Detect planetary disk mask and use fitEllipseAMS() on planetary disk edge
  // as initial ellipse estimate

  bool fOk =
      detect_planetary_disk(gray_image, _mask,
          1,
          options_.stdev_factor,
          &detected_planetary_disk_mask_);

  if( !fOk ) {
    CF_ERROR("detect_planetary_disk() fails");
    return false;
  }

  morphological_gradient(detected_planetary_disk_mask_,
      detected_planetary_disk_edge_,
      cv::Mat1b(3, 3, 255),
      cv::BORDER_CONSTANT);

  cv::findNonZero(detected_planetary_disk_edge_,
      component_edge_points);

  ellipse_ =
      cv::fitEllipseAMS(component_edge_points);

  if( ellipse_.size.width < ellipse_.size.height ) {
    std::swap(ellipse_.size.width, ellipse_.size.height);
    ellipse_.angle -= 90;
  }

  ellipseAMS_ = ellipse_;
  CF_DEBUG("INITIAL ELLIPSE: width=%g height=%g angle=%g",
      ellipse_.size.width, ellipse_.size.height, ellipse_.angle);

  /////////////////////////////////////////////////////////////////////////////////////////
  // Try to fit artificial jovian ellipse

  static constexpr double jovian_polar_to_equatorial_axis_ratio = 66.854 / 71.492;

  initial_artifial_ellipse_edge_.create(_image.size());
  initial_artifial_ellipse_edge_.setTo(0);

  double A = 0.5 * ellipseAMS_.size.width;
  double B = A * jovian_polar_to_equatorial_axis_ratio;

  cv::RotatedRect rc(ellipseAMS_.center, cv::Size(2 * A, 2 * B), 0);

  cv::ellipse(initial_artifial_ellipse_edge_, rc, 1, 1, cv::LINE_AA);
  cv::GaussianBlur(initial_artifial_ellipse_edge_, initial_artifial_ellipse_edge_, cv::Size(), 1, 1);


  c_euclidean_image_transform transform(cv::Vec2f(ellipseAMS_.center.x, ellipseAMS_.center.y),
      cv::Vec2f(ellipseAMS_.center.x, ellipseAMS_.center.y),
      -ellipseAMS_.angle * CV_PI / 180,
      1);

  cv::Mat2f rmap;

  if ( enable_debug_images_ ) {
    // for debug

    transform.create_remap(rmap,
        initial_artifial_ellipse_edge_.size());

    cv::remap(initial_artifial_ellipse_edge_,
        remapped_artifial_ellipse_edge_,
        rmap,
        cv::noArray(),
        cv::INTER_LINEAR);
  }

  if( !align_images_ecc(&transform, initial_artifial_ellipse_edge_, detected_planetary_disk_edge_) ) {
    CF_ERROR("align_images_ecc(artifical_ellipse) fails");
    return false;
  }

  transform.create_remap(rmap,
      initial_artifial_ellipse_edge_.size());

  cv::remap(initial_artifial_ellipse_edge_,
      aligned_artifial_ellipse_edge_,
      rmap,
      cv::noArray(),
      cv::INTER_LINEAR);

  cv::compare(aligned_artifial_ellipse_edge_, 0.1,
      aligned_artifial_ellipse_edge_mask_,
      cv::CMP_GT);

  component_edge_points.clear();
  cv::findNonZero(aligned_artifial_ellipse_edge_mask_, component_edge_points);

  ellipse_ = cv::fitEllipse(component_edge_points);
  if( ellipse_.size.width < ellipse_.size.height ) {
    std::swap(ellipse_.size.width, ellipse_.size.height);
    ellipse_.angle -= 90;
  }
  ellipseAMS2_ = ellipse_;

  CF_DEBUG("FINAL ELLIPSE: width=%g height=%g angle=%g", ellipse_.size.width, ellipse_.size.height, ellipse_.angle);


  // create also filed binary ellipse mask
  aligned_artifial_ellipse_mask_.create(_image.size());
  aligned_artifial_ellipse_mask_.setTo(0);
  cv::ellipse(aligned_artifial_ellipse_mask_, ellipse_, 255, -1, cv::LINE_8);
  cv::dilate(aligned_artifial_ellipse_mask_, aligned_artifial_ellipse_mask_, cv::Mat1b(5, 5, 255));

  // create also normalized image for jovian derotation loop

  if( _image.channels() == 1 ) {
    _image.copyTo(gray_image_);
  }
  else {
    cv::cvtColor(_image, gray_image_, cv::COLOR_BGR2GRAY);
  }

  if( options_.normalization_scale < 0 ) {
    gray_image_.copyTo(normalized_image_);
  }
  else {
    cv::Mat mean;

    cv::meanStdDev(gray_image_, m, s,
        aligned_artifial_ellipse_mask_);

    if( options_.normalization_scale == 0 ) {
      cv::subtract(gray_image_, m, mean);
    }
    else {
      pyramid_downscale(gray_image_, mean, options_.normalization_scale, cv::BORDER_REPLICATE);
      pyramid_upscale(mean, gray_image_.size());
      cv::subtract(gray_image_, mean, mean);
    }

    cv::multiply(mean, 1. / s[0], normalized_image_);
  }

  if( options_.normalization_blur > 0 ) {
    cv::GaussianBlur(normalized_image_,
        normalized_image_,
        cv::Size(),
        options_.normalization_blur,
        options_.normalization_blur);
  }

  return true;
}
