/*
 * c_jovian_ellipse_detector.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector.h"
#include "image_transform.h"
#include "ecc2.h"

#include <core/proc/autoclip.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/pyrscale.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/gradient.h>
#include <core/proc/fitEllipseLM.h>

#include <core/debug.h>

cv::Rect compute_ellipse_bounding_box(const cv::RotatedRect & rc)
{
  const float a =
      rc.angle * CV_PI / 180;

  const float ca =
      std::cos(a);

  const float sa =
      std::sin(a);

  const float ux = rc.size.width * ca / 2;
  const float uy = -rc.size.width * sa / 2;
  const float vx = rc.size.height * sa / 2;
  const float vy = rc.size.height * ca / 2;

  const float halfwidth = sqrt(ux * ux + vx * vx);
  const float halfheight = sqrt(uy * uy + vy * vy);

  float left = rc.center.x - halfwidth;
  float top = rc.center.y - halfheight;

  return cv::Rect(left, top,
      2 * halfwidth,
      2 * halfheight);
}

cv::Rect compute_ellipse_crop_box(const cv::RotatedRect & ellipse, const cv::Size & image_size, int margin)
{
  cv::Rect rc =
      compute_ellipse_bounding_box(ellipse);

  if( margin < 0 ) {
    margin =
        std::max(16, (int) (ellipse.size.width / 5));
  }

//  CF_DEBUG("ellipse_bounding_box: x=%d y=%d w=%d h=%d margin=%d image_size=%dx%d",
//      rc.x, rc.y, rc.width, rc.height, margin, image_size.width, image_size.height);

  rc.x -= margin;
  rc.y -= margin;
  rc.width += 2 * margin;
  rc.height += 2 * margin;

  if( rc.x < 0 ) {
    rc.x = 0;
  }
  if( rc.y < 0 ) {
    rc.y = 0;
  }
  if( rc.x + rc.width >= image_size.width ) {
    rc.width = image_size.width - rc.x;
  }
  if( rc.y + rc.height >= image_size.height ) {
    rc.height = image_size.height - rc.y;
  }

//  CF_DEBUG("final rc: x=%d y=%d w=%d h=%d margin=%d image_size=%dx%d",
//      rc.x, rc.y, rc.width, rc.height, margin, image_size.width, image_size.height);

  return rc;
}

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

  if( (N = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, labels.type())) < 2 ) {
    return false;
  }

  struct ss
  {
    int label, area;
  };

  std::vector<ss> cstats;

  for( int i = 1; i < N; ++i ) {
    cstats.emplace_back();
    cstats.back().label = i;
    cstats.back().area = stats[i][cv::CC_STAT_AREA];
  }

  if( cstats.size() > 1 ) {
    std::sort(cstats.begin(), cstats.end(),
        [](const ss & p, const ss & n) {
          return p.area > n.area;
        });
  }

  if( cstats[0].area < 4 ) {
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
    cv::Mat * output_component_mask,
    cv::Rect * output_component_rect = nullptr)
{
  cv::Mat src, gray;
  cv::Mat1b comp;
  double noise;
  double threshold;

  cv::Rect rc, rcc;

  if( (src = input_image.getMat()).empty() ) {
    return false;
  }

  if( src.channels() == 1 ) {
    src.copyTo(gray);
  }
  else if( src.channels() == 4 ) {
    cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if( gbsigma > 0 ) {
    GaussianBlur(gray, gray, cv::Size(0, 0), gbsigma);
  }

  autoclip(gray, input_mask, gray, 1, 99, 0, 255);
  gray.convertTo(gray, CV_8U);

  cv::threshold(gray, comp, 0, 255, cv::THRESH_TRIANGLE);

  morphological_smooth_close(comp, comp,
      cv::Mat1b(5, 5, 255),
      cv::BORDER_CONSTANT);

  if( !input_mask.empty() ) {
    comp.setTo(0, ~input_mask.getMat());
  }

  if( !get_maximal_connected_component(comp, &rc, &comp) ) {
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

  if( !get_maximal_connected_component(comp, &rc, output_component_mask) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
    return false;
  }

  if( output_component_rect ) {
    *output_component_rect = rc;
  }

  return true;
}

//template<class ImageTransformType>
//bool align_images_ecc(ImageTransformType * transform, cv::InputArray input_image, cv::InputArray reference_image)
//{
////  typename c_ecc_motion<ImageTransformType>::motion_model
////    motion_model(transform);
//
//  c_ecc_forward_additive ecc(transform);
//
//  ecc.set_max_eps(0.1);
//  ecc.set_min_rho(0.5);
//  ecc.set_max_iterations(30);
//  ecc.set_input_smooth_sigma(1);
//  ecc.set_reference_smooth_sigma(1);
//
//  if( !ecc.align(input_image, reference_image) ) {
//    CF_ERROR("ecc.align() fails: failed: %d rho=%g eps=%g num_iterations=%d",
//        ecc.failed(),
//        ecc.rho(),
//        ecc.eps(),
//        ecc.num_iterations());
//
//    return false;
//  }
//
//  return true;
//}
//

}// namespace

void c_jovian_ellipse_detector::set_stdev_factor(double v)
{
  options_.stdev_factor = v;
}

double c_jovian_ellipse_detector::stdev_factor() const
{
  return options_.stdev_factor;
}

void c_jovian_ellipse_detector::set_pca_blur(double sigma)
{
  options_.pca_blur = sigma;
}

double c_jovian_ellipse_detector::pca_blur() const
{
  return options_.pca_blur;
}

void c_jovian_ellipse_detector::set_offset(const cv::Point2f & v)
{
  options_.offset = v;
}

const cv::Point2f& c_jovian_ellipse_detector::offset() const
{
  return options_.offset;
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

const c_jovian_ellipse_detector_options& c_jovian_ellipse_detector::options() const
{
  return options_;
}

c_jovian_ellipse_detector_options& c_jovian_ellipse_detector::options()
{
  return options_;
}

const cv::Mat1b& c_jovian_ellipse_detector::final_planetary_disk_mask() const
{
  return final_planetary_disk_mask_;
}

const cv::Mat1b& c_jovian_ellipse_detector::detected_planetary_disk_mask() const
{
  return detected_planetary_disk_mask_;
}

const cv::Mat1b& c_jovian_ellipse_detector::detected_planetary_disk_edge() const
{
  return detected_planetary_disk_edge_;
}

//const cv::RotatedRect& c_jovian_ellipse_detector::ellipseAMS() const
//{
//  return ellipseAMS_;
//}

const cv::RotatedRect& c_jovian_ellipse_detector::final_planetary_disk_ellipse() const
{
  return final_planetary_disk_ellipse_;
}

const cv::Mat& c_jovian_ellipse_detector::gray_image() const
{
  return gray_image_;
}

const cv::Mat1f c_jovian_ellipse_detector::pca_gx() const
{
  return pca_gx_;
}

const cv::Mat1f c_jovian_ellipse_detector::pca_gy() const
{
  return pca_gy_;
}

double c_jovian_ellipse_detector::compute_jovian_orientation_pca(const cv::Mat & gray_image,
    const cv::Mat & planetary_disk_mask, const cv::Rect & component_rect)
{
  static float kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(kernel); // / 1.5;
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(kernel); // / 1.5;

  const cv::Size size = component_rect.size();

  cv::Mat g;

  if( options_.pca_blur <= 0 ) {
    g = gray_image(component_rect);
  }
  else {
    cv::GaussianBlur(gray_image(component_rect), g, cv::Size(-1, -1),
        options_.pca_blur, options_.pca_blur,
        cv::BORDER_REPLICATE);
  }

  cv::filter2D(g, pca_gx_, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(g, pca_gy_, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  const int erode_size = std::max(3, 2 * (int) (0.075 *
      std::min(size.width, size.height)) + 1);

  cv::Mat1b mask;

  cv::erode(planetary_disk_mask(component_rect), mask,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_size, erode_size)),
      cv::Point(-1, -1),
      1,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  const int npts =
      cv::countNonZero(mask);

  cv::Mat1f data_pts(npts, 2);

  int ii = 0;
  for( int y = 0; y < mask.rows; ++y ) {
    for( int x = 0; x < mask.cols; ++x ) {
      if( mask[y][x] ) {
        data_pts[ii][0] = pca_gx_[y][x];
        data_pts[ii][1] = pca_gy_[y][x];
        ++ii;
      }
    }
  }

  cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Select second eigen vector for ellipse orientation
  const cv::Vec2f eigen_vec(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));
  const double angle = atan2(eigen_vec[1], eigen_vec[0]) * 180 / CV_PI;

  return angle;
}

bool c_jovian_ellipse_detector::detect_jovian_disk(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  std::vector<cv::Point2f> component_edge_points;
  cv::Rect detected_component_rect;
  cv::RotatedRect ellipse;

  /////////////////////////////////////////////////////////////////////////////////////////
  // Convert input image to gray scale

  if( _image.channels() == 1 ) {
    _image.getMat().copyTo(gray_image_);
  }
  else {
    cv::cvtColor(_image, gray_image_, cv::COLOR_BGR2GRAY);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Detect planetary disk mask and use fitEllipseAMS() on planetary disk edge
  // as initial ellipse estimate

  bool fOk =
      detect_planetary_disk(gray_image_, _mask,
          1,
          options_.stdev_factor,
          &detected_planetary_disk_mask_,
          &detected_component_rect);

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

  // Use PCA on image gradients to compute jovian disk orientation
  ellipse.angle =
      compute_jovian_orientation_pca(gray_image_,
          detected_planetary_disk_mask_,
          detected_component_rect);

  if( ellipse.angle > 90 ) {
    ellipse.angle -= 180;
  }
  else if( ellipse.angle < -90 ) {
    ellipse.angle += 180;
  }

  static constexpr double jovian_axis_ratio =
      0.93512560845968779724;

  fitEllipseLM1(component_edge_points,
      jovian_axis_ratio, // b / a
      ellipse.angle * CV_PI / 180, // radians
      &final_planetary_disk_ellipse_);

  final_planetary_disk_ellipse_.center +=
      options_.offset;

  //final_planetary_disk_ellipse_ = ellipse;

  CF_DEBUG("\n"
      "fitEllipseLM1: width=%g height=%g angle=%g",
      final_planetary_disk_ellipse_.size.width,
      final_planetary_disk_ellipse_.size.height,
      final_planetary_disk_ellipse_.angle);

  // create also filed binary ellipse mask
  final_planetary_disk_mask_.create(_image.size());
  final_planetary_disk_mask_.setTo(0);
  cv::ellipse(final_planetary_disk_mask_, final_planetary_disk_ellipse_, 255, -1, cv::LINE_8);
  cv::erode(final_planetary_disk_mask_, final_planetary_disk_mask_, cv::Mat1b(3, 3, 255));

  return true;
}

