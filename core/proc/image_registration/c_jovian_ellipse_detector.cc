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
#include <core/proc/unsharp_mask.h>
#include <core/proc/gradient.h>

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
    cv::Mat * output_component_mask,
    cv::Rect * output_component_rect = nullptr)
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

  if( output_component_rect ) {
    *output_component_rect = rc;
  }

  return true;
}



template<class ImageTransformType>
bool align_images_ecc(ImageTransformType * transform, cv::InputArray input_image, cv::InputArray reference_image)
{
  typename c_ecc_motion<ImageTransformType>::motion_model
    motion_model(transform);

  c_ecc_forward_additive ecc(&motion_model);

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


void c_jovian_ellipse_detector::set_stdev_factor(double v)
{
  options_.stdev_factor = v;
}

double c_jovian_ellipse_detector::stdev_factor() const
{
  return options_.stdev_factor;
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

bool c_jovian_ellipse_detector::detect_jovian_disk(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  cv::Mat gray_image;
  std::vector<cv::Point2f> component_edge_points;
  cv::Scalar m, s;
  cv::Rect detected_component_rect;

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

  ellipse_ =
      cv::fitEllipseAMS(component_edge_points);

  if( ellipse_.size.width < ellipse_.size.height ) {
    std::swap(ellipse_.size.width, ellipse_.size.height);
    ellipse_.angle -= 90;
  }

  if ( false ) {

    cv::Mat1f gx, gy;

    gray_image(detected_component_rect).copyTo(gradient_test_image_);

    if( !compute_gradient(gradient_test_image_, gx, 1, 0, 3, CV_32F) ) {
      CF_ERROR("compute_gradient(gx) fails");
    }
    if( !compute_gradient(gradient_test_image_, gy, 0, 1, 3, CV_32F) ) {
      CF_ERROR("compute_gradient(gx) fails");
    }
    //cv::absdiff(gx, 0, gx);
    //cv::absdiff(gy, 0, gy);
    //cv::phase(gx, gy, gradient_test_image_, true);


    const cv::Size size = gx.size();

    CF_DEBUG("gx.size=%dx%d", gx.cols, gx.rows);
    CF_DEBUG("gy.size=%dx%d", gy.cols, gy.rows);

    cv::Mat1f data_pts((3 * size.height / 4 - size.height / 4) * (3 * size.width / 4 - size.width / 4), 2);
    cv::Mat1f g = gradient_test_image_;


    CF_DEBUG("data_pts.rows=%d", data_pts.rows);

    int ii = 0;
    for( int y = size.height / 4; y < 3 * size.height / 4; ++y ) {
      for( int x = size.width / 4; x < 3 * size.width / 4; ++x ) {
        data_pts[ii][0] = gx[y][x];
        data_pts[ii][1] = gy[y][x];
        g[y][x] += 0.5;
        ++ii;
      }
    }

    CF_DEBUG("ii=%d data_pts.rows=%d", ii, data_pts.rows);

    cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    //Store the center of the object
    cv::Point2f cntr = cv::Point2f(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2f> eigen_vecs(2);
    std::vector<float> eigen_val(2);
    for( int i = 0; i < 2; i++ ) {
      eigen_vecs[i] = cv::Point2f(pca.eigenvectors.at<float>(i, 0), pca.eigenvectors.at<float>(i, 1));
      eigen_val[i] = pca.eigenvalues.at<float>(i);
    }

    // orientation
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x) * 180 / CV_PI;

    CF_DEBUG("eigen_vecs: { %g (%g %g)  %g (%g %g) } \n"
        "angle=%g deg ",
        eigen_val[0], eigen_vecs[0].x, eigen_vecs[0].y,
        eigen_val[1], eigen_vecs[1].x, eigen_vecs[1].y,
        angle);


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

  return true;
}
