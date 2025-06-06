/*
 * c_desaturate_edges_routine.cc
 *
 *  Created on: Oct 23, 2022
 *      Author: amyznikov
 */

#include "c_desaturate_edges_routine.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/debug.h>

void c_desaturate_edges_routine::set_alpha(double v)
{
  alpha_ = v;
}

double c_desaturate_edges_routine::alpha() const
{
  return alpha_;
}

void c_desaturate_edges_routine::set_gbsigma(double v)
{
  gbsigma_ = v;
}

double c_desaturate_edges_routine::gbsigma() const
{
  return gbsigma_;
}

void c_desaturate_edges_routine::set_stdev_factor(double v)
{
  stdev_factor_ = v;
}

double c_desaturate_edges_routine::stdev_factor() const
{
  return stdev_factor_;
}

void c_desaturate_edges_routine::set_se_close_radius(int v)
{
  se_close_radius_ = v;
}

int c_desaturate_edges_routine::se_close_radius() const
{
  return se_close_radius_;
}

void c_desaturate_edges_routine::set_show_weights(bool v)
{
  show_weights_ = v;
}

bool c_desaturate_edges_routine::show_weights() const
{
  return show_weights_;
}

void c_desaturate_edges_routine::set_blur_radius(double v)
{
  blur_radius_ = v;
}

double c_desaturate_edges_routine::blur_radius() const
{
  return blur_radius_;
}

void c_desaturate_edges_routine::set_l1norm(bool v)
{
  l1norm_ = v;
}

bool c_desaturate_edges_routine::l1norm() const
{
  return l1norm_;
}

bool c_desaturate_edges_routine::compute_planetary_disk_weights(const cv::Mat & src_image, const cv::Mat & src_mask,
    cv::Mat1f & dst_image) const
{
  cv::Mat planetary_disk_mask;

  bool fOk =
      simple_planetary_disk_detector(src_image, src_mask,
          gbsigma_,
          stdev_factor_,
          se_close_radius_,
          nullptr,
          nullptr,
          &planetary_disk_mask);

  if( !fOk ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }


  cv::distanceTransform(planetary_disk_mask, planetary_disk_mask,
      cv::DIST_L2,
      cv::DIST_MASK_PRECISE,
      CV_32F);

  if ( blur_radius_ > 0 ) {
    cv::GaussianBlur(planetary_disk_mask, planetary_disk_mask,
        cv::Size(),
        blur_radius_);
  }

  double min, max;
  cv::minMaxLoc(planetary_disk_mask,
      &min, &max);

  cv::multiply(planetary_disk_mask, 1. / max,
      planetary_disk_mask);

  if ( l1norm_ ) {
    cv::sqrt(planetary_disk_mask,
        planetary_disk_mask);
  }

  dst_image =
      std::move(planetary_disk_mask);

  return true;
}

template<class T>
static void combine_images(const cv::Mat_<cv::Vec<T, 3>> & color_image, const cv::Mat_<cv::Vec<T, 3>> & gray_image,
    cv::Mat_<cv::Vec<T, 3>> & dst_image,
    const cv::Mat1f & weights, double alpha )
{

  dst_image.create(color_image.size());

  for ( int y = 0; y < color_image.rows; ++y ) {
    for ( int x = 0; x < color_image.cols; ++x ) {

      if ( !(weights[y][x] > 0) ) {
        dst_image[y][x] = color_image[y][x];
      }
      else {

        const double w1 = std::min(1., alpha + weights[y][x]);
        const double w2 = 1 - w1;

        for ( int i = 0; i < 3; ++i ) {

          dst_image[y][x][i] =
              cv::saturate_cast<T>( w1 * color_image[y][x][i] + w2 * gray_image[y][x][i]);
        }


      }
    }
  }
}

void c_desaturate_edges_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, alpha, "");
  BIND_PCTRL(ctls, gbsigma, "");
  BIND_PCTRL(ctls, stdev_factor, "");
  BIND_PCTRL(ctls, blur_radius, "");
  BIND_PCTRL(ctls, l1norm, "");
  BIND_PCTRL(ctls, show_weights, "");
}

bool c_desaturate_edges_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, alpha);
    SERIALIZE_PROPERTY(settings, save, *this, gbsigma);
    SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
    SERIALIZE_PROPERTY(settings, save, *this, blur_radius);
    SERIALIZE_PROPERTY(settings, save, *this, l1norm);
    SERIALIZE_PROPERTY(settings, save, *this, show_weights);
    return true;
  }
  return false;
}

bool c_desaturate_edges_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() == 3 ) {

    cv::Mat gray_image;
    cv::Mat1f weights;

    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    if( compute_planetary_disk_weights(gray_image, mask.getMat(), weights) ) {

      if ( show_weights_ ) {
        weights.copyTo(image);
      }
      else {

        cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);

        switch (image.depth()) {

        case CV_8U: {
          cv::Mat3b combined_image;
          combine_images(cv::Mat3b(image.getMat()), cv::Mat3b(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_8S: {
          typedef cv::Mat_<cv::Vec<int8_t, 3>> Mat3h;
          Mat3h combined_image;
          combine_images(Mat3h(image.getMat()), Mat3h(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_16U: {
          cv::Mat3w combined_image;
          combine_images(cv::Mat3w(image.getMat()), cv::Mat3w(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_16S: {
          cv::Mat3s combined_image;
          combine_images(cv::Mat3s(image.getMat()), cv::Mat3s(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_32S: {
          cv::Mat3i combined_image;
          combine_images(cv::Mat3i(image.getMat()), cv::Mat3i(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_32F: {
          cv::Mat3f combined_image;
          combine_images(cv::Mat3f(image.getMat()), cv::Mat3f(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }

        case CV_64F: {
          cv::Mat3d combined_image;
          combine_images(cv::Mat3d(image.getMat()), cv::Mat3d(gray_image), combined_image, weights, alpha_);
          combined_image.copyTo(image);
          break;
        }
        }
      }
    }
  }

  return true;
}
