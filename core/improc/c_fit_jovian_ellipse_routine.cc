/*
 * c_fit_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#include "c_fit_jovian_ellipse_routine.h"
#include <core/proc/cdiffs.h>
#include <core/ssprintf.h>

c_fit_jovian_ellipse_routine::c_class_factory c_fit_jovian_ellipse_routine::class_factory;

template<>
const c_enum_member * members_of<c_fit_jovian_ellipse_routine::display_type>()
{
  static constexpr c_enum_member members[] = {
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_mask, "planetary_disk_mask", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_edge, "planetary_disk_edge", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_ellipse, "planetary_disk_ellipse", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_ellipseAMS, "planetary_disk_ellipseAMS", },
      { c_fit_jovian_ellipse_routine::display_initial_uncropped_artifial_ellipse, "initial_uncropped_artifial_ellipse", },
      { c_fit_jovian_ellipse_routine::display_aligned_uncropped_artifial_ellipse, "aligned_uncropped_artifial_ellipse", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_ellipseAMS2, "planetary_disk_ellipseAMS2", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_eigen2d_mu, "eigen2d_mu", },
      { c_fit_jovian_ellipse_routine::display_uncropped_planetary_disk_eigen2d_N, "eigen2d_N", },





      { c_fit_jovian_ellipse_routine::display_cropped_gray_image, "cropped_gray_image", },
      { c_fit_jovian_ellipse_routine::display_cropped_component_mask, "cropped_component_mask", },
      { c_fit_jovian_ellipse_routine::display_cropped_gradient_image, "cropped_gradient_image", },
      { c_fit_jovian_ellipse_routine::display_cropped_normalized_image, "cropped_normalized_image", },
      { c_fit_jovian_ellipse_routine::display_initial_artificial_ellipse, "initial_artifical_ellipse", },
      { c_fit_jovian_ellipse_routine::display_initial_ellipse_fit, "initial_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, nullptr, },
  };

  return members;
}

c_fit_jovian_ellipse_routine::c_fit_jovian_ellipse_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_fit_jovian_ellipse_routine::ptr c_fit_jovian_ellipse_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_fit_jovian_ellipse_routine::set_display(display_type v)
{
  display_type_ = v;
}

c_fit_jovian_ellipse_routine::display_type c_fit_jovian_ellipse_routine::display() const
{
  return display_type_;
}

void c_fit_jovian_ellipse_routine::set_hlines(const std::vector<float> & hlines)
{
  detector_.set_hlines(hlines);
}

const std::vector<float> & c_fit_jovian_ellipse_routine::hlines() const
{
  return detector_.hlines();
}

void c_fit_jovian_ellipse_routine::set_normalization_scale(int v)
{
  return detector_.set_normalization_scale(v);
}

int c_fit_jovian_ellipse_routine::normalization_scale() const
{
  return detector_.normalization_scale();
}

void c_fit_jovian_ellipse_routine::set_normalization_blur(double v)
{
  return detector_.set_normalization_blur(v);
}

double c_fit_jovian_ellipse_routine::normalization_blur() const
{
  return detector_.normalization_blur();
}

void c_fit_jovian_ellipse_routine::set_gradient_blur(double v)
{
  return detector_.set_gradient_blur(v);
}

double c_fit_jovian_ellipse_routine::gradient_blur() const
{
  return detector_.gradient_blur();
}

c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector()
{
  return &detector_;
}

const c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector() const
{
  return &detector_;
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, hlines);
  SAVE_PROPERTY(settings, *this, normalization_scale);
  SAVE_PROPERTY(settings, *this, normalization_blur);
  SAVE_PROPERTY(settings, *this, gradient_blur);
  SAVE_PROPERTY(settings, *this, display);

  return true;
}

bool c_fit_jovian_ellipse_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, hlines);
  LOAD_PROPERTY(settings, this, normalization_scale);
  LOAD_PROPERTY(settings, this, normalization_blur);
  LOAD_PROPERTY(settings, this, gradient_blur);
  LOAD_PROPERTY(settings, this, display);

  return true;
}

//
///*
// * Five-point approximation to first order image derivative.
// *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
// * */
//static void differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth)
//{
//  static thread_local const cv::Matx<float, 1, 5> K(
//      (+1.f / 12),
//      (-8.f / 12),
//        0.f,
//      (+8.f / 12),
//      (-1.f / 12));
//
//  if ( ddepth < 0 ) {
//    ddepth = std::max(_src.depth(), CV_32F);
//  }
//
//  const cv::Mat & src = _src.getMat();
//  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//}

static void rotatedRectange(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{
  cv::Point2f pts[4];
  rc.points(pts);

  for (int i = 0; i < 4; i++) {
   cv::line(image, pts[i], pts[(i+1)%4], color, thickness, lineType, shift);
  }
}

bool c_fit_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  static const auto atan2 =
      [](const cv::Mat1f & Nx, const cv::Mat1f & Ny, cv::Mat1f & angle) {

        angle.create(Nx.size());
        for ( int y = 0; y < Nx.rows; ++y ) {
          for ( int x = 0; x < Nx.cols; ++x ) {
            angle[y][x] = std::atan2 (Ny[y][x], Nx[y][x]) * 180 / CV_PI;
          }
        }

      };


  detector_.detect_planetary_disk(image, mask);

  switch (display_type_) {
  case display_uncropped_planetary_disk_eigen2d_mu:
  case display_uncropped_planetary_disk_eigen2d_N: {

    cv::Mat gray;
    cv::Mat mu1, mu2, mu;
    cv::Mat1f Nx, Ny, angle;
    bool fixNormals = false;

    if ( image.channels() == 3 ) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
      gray = image.getMat();
    }

    if( detector_.gradient_blur() > 0 ) {
      cv::GaussianBlur(gray, gray, cv::Size(), detector_.gradient_blur());
    }

    eigen2d(gray,
        &mu1,
        &mu2,
        &Nx,
        &Ny,
        fixNormals);

    //    cv::absdiff(Nx, 0, Nx);
    //    cv::absdiff(Ny, 0, Ny);

    cv::Mat1b msk;
    cv::erode(detector_.uncropped_planetary_disk_mask(), msk,
        cv::Mat1b(63,63, 255));

    switch (display_type_) {
    case display_uncropped_planetary_disk_eigen2d_mu:
      //cv::magnitude(mu1, mu2, mu);
      cv::add(mu1.mul(mu1), mu2.mul(mu2), mu);
      mu.setTo(0, ~msk);
      mu.copyTo(image);
      break;
    case display_uncropped_planetary_disk_eigen2d_N:
      atan2(Nx, Ny, angle);
      angle.setTo(0, ~msk);
      angle.copyTo(image);

      if ( true ) {
        //cv::magnitude(mu1, mu2, mu);
        cv::add(mu1.mul(mu1), mu2.mul(mu2), mu);
        mu.setTo(0, ~msk);

        cv::multiply(mu, Nx, Nx);
        cv::multiply(mu, Ny, Ny);

        //    cv::absdiff(Nx, 0, Nx);
        //    cv::absdiff(Ny, 0, Ny);

        double avgMu = cv::mean(mu, msk)[0];
        double NX = cv::mean(Nx, msk)[0] / avgMu;
        double NY = cv::mean(Ny, msk)[0] / avgMu;
        double avgA = std::atan2(NY, NX);

        CF_DEBUG("avgA=%g", avgA * 180 / CV_PI);

        cv::cvtColor(image,  image, cv::COLOR_GRAY2BGR);

        cv::line(image,
            detector_.ellipseAMS2().center,
            detector_.ellipseAMS2().center + 200 * cv::Point2f(cos(avgA), sin(avgA)),
            CV_RGB(255, 100, 0),
            2,
            cv::LINE_AA);

      }

      break;
    }


    break;
  }

  case display_uncropped_planetary_disk_mask:
    detector_.uncropped_planetary_disk_mask().copyTo(image);
    break;
  case display_uncropped_planetary_disk_edge:
    image.setTo(1, detector_.uncropped_planetary_disk_edge());
    break;
  case display_uncropped_planetary_disk_ellipseAMS:
    rotatedRectange(image, detector_.ellipseAMS(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS(), CV_RGB(0, 0, 1), 1);
    break;
  case display_initial_uncropped_artifial_ellipse:
    detector_.initial_uncropped_artifial_ellipse().copyTo(image);
    break;
  case display_aligned_uncropped_artifial_ellipse:
    detector_.aligned_uncropped_artifial_ellipse().copyTo(image);
    break;
  case display_uncropped_planetary_disk_ellipseAMS2:
    rotatedRectange(image, detector_.ellipseAMS2(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS2(), CV_RGB(0, 0, 1), 1);
    break;
  case display_uncropped_planetary_disk_ellipse:
    rotatedRectange(image, detector_.planetary_disk_ellipse(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.planetary_disk_ellipse(), 1, 1);
    break;
  case display_cropped_gray_image:
    detector_.cropped_gray_image().copyTo(image);
    break;
  case display_cropped_component_mask:
    detector_.uncropped_planetary_disk_mask()(detector_.crop_bounding_box()).copyTo(image);
    break;
  case display_cropped_gradient_image:
    detector_.cropped_gradient_image().copyTo(image);
    break;
  case display_cropped_normalized_image:
    detector_.cropped_normalized_image().copyTo(image);
    break;
  case display_initial_artificial_ellipse:
    detector_.initial_artificial_ellipse().copyTo(image);
    break;
  case display_initial_ellipse_fit:
    detector_.initial_artificial_ellipse_fit().copyTo(image);
    break;
  case display_final_ellipse_fit:
    detector_.cropped_final_ellipse_fit().copyTo(image);
    break;
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}