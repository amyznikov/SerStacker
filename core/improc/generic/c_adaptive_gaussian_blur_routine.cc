/*
 * c_adaptive_gaussian_blur_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_adaptive_gaussian_blur_routine.h"

#include <opencv2/ximgproc.hpp>
#include <core/proc/morphology.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_adaptive_gaussian_blur_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayFiltered, "Filtered" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayRamp, "Ramp" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayTex, "Tex" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayEdges, "Edges" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayLpass, "Lpass" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayHpass, "Hpass" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayDetail, "Detail" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayFiltered}  // must  be last
  };

  return members;
}

void c_adaptive_gaussian_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "seRadius", ctx(&this_class::seRadius), "Radius of morphologcal SE element used to compute rampLee and texLee maps");
  ctlbind(ctls, "mkRadius", ctx(&this_class::mkRadius), "Optional medianBlur kernel radius for lpass image");
  ctlbind(ctls, "gfSigma", ctx(&this_class::gfSigma), "Gaussian filter sigma for lpass image");
  ctlbind(ctls, "hBoost", ctx(&this_class::hBoost), "");
  ctlbind_spinbox(ctls, "sn", ctx(&this_class::sn), 1, 12, 1, "Steepness of the S-curve slope (typically 2..8)");
  ctlbind_spinbox(ctls, "qmin", ctx(&this_class::qmin), 0, 1, 1e-4, "Quantile for lower bound of the edge map");
  ctlbind_spinbox(ctls, "smin", ctx(&this_class::smin), 0, 1, 1e-3, "Output  sharpness  for qmin");
  ctlbind_spinbox(ctls, "qmax", ctx(&this_class::qmax), 0, 1, 1e-4, "Quantile for upper bound of the edge map");
  ctlbind_spinbox(ctls, "smax", ctx(&this_class::smax), 0, 1, 1e-3, "Output sharpness for qmax");
  ctlbind(ctls, "display", ctx(&this_class::displayType), "Select output image");
}

bool c_adaptive_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, displayType);
    SERIALIZE_OPTION(settings, save, *this, seRadius);
    SERIALIZE_OPTION(settings, save, *this, sn);
    SERIALIZE_OPTION(settings, save, *this, qmin);
    SERIALIZE_OPTION(settings, save, *this, qmax);
    SERIALIZE_OPTION(settings, save, *this, smin);
    SERIALIZE_OPTION(settings, save, *this, smax);
    SERIALIZE_OPTION(settings, save, *this, mkRadius);
    SERIALIZE_OPTION(settings, save, *this, gfSigma);
    SERIALIZE_OPTION(settings, save, *this, hBoost);
    return true;
  }
  return false;
}

static void calculateSBounds(const cv::Mat1f & src, double * xmin, double * xmax, double qmin = 0.05, double qmax = 0.95)
{
  std::vector<float> samples;

  if( src.isContinuous() ) {
    samples.assign(src.begin(), src.end());
  }
  else {
    for( int i = 0; i < src.rows; ++i ) {
      samples.insert(samples.end(), src.ptr<float>(i), src.ptr<float>(i) + src.cols);
    }
  }

  auto it_min = samples.begin() + static_cast<int>(samples.size() * std::max(0., std::min(1., qmin)));
  if( it_min == samples.end() ) {
    it_min = samples.begin() + samples.size() - 1U;
  }

  auto it_max = samples.begin() + static_cast<int>(samples.size() * std::max(0., std::min(1., qmax)));
  if( it_max == samples.end() ) {
    it_max = samples.begin() + samples.size() - 1U;
  }

  std::nth_element(samples.begin(), it_min, samples.end());
  *xmin = it_min == samples.end() ? samples.back() : *it_min;

  std::nth_element(it_min, it_max, samples.end());
  *xmax = it_max == samples.end() ? samples.back() : *it_max;
}

/**
* S-shaped mapping to the interval [0..1] with a guaranteed leading zero.
* @param _image Input/Output single-channel matrix of type CV_32FC1
* @param n Steepness of the slope (typically 2..8)
* @param xmin Lower bound of the linear region (quantile qmin)
* @param xmax Upper bound of the linear region (quantile qmax)
* @param smin Output range min
* @param smax Output range max
*/
void mapPixelsSShaped(cv::InputOutputArray _image, int n, double xmin, double xmax, double smin, double smax)
{
  if( n < 1 ) {
    n = 1;
  }

  if( xmax <= xmin ) {
    xmax = xmin + 1e-5f;
  }

  const double xmid = (xmax + xmin) / 2.0f;
  const double w = (xmax - xmin) / 2.0f;
  const double fn = static_cast<float>(n);
  const double wn = std::pow(w, fn);
  const double inv_n = 1.0f / fn;

  const auto s_curve = [&](double v) -> double {
    const double diff = v - xmid;
    const double abs_diff_n = std::pow(std::abs(diff), fn);
    return 0.5 + diff / (2.0 * std::pow(abs_diff_n + wn, inv_n));
  };

  const double f0 = s_curve(0.0);
  const double s1 = (smax - smin) / (1.0 - f0);
  const double s0 = smin - f0* s1;

  cv::Mat1f image = _image.getMatRef();

  image.forEach([=](float & pixel, const int position[]) -> void {
    pixel = static_cast<float>(s0 + s1 * s_curve(pixel) );
  });
}

bool c_adaptive_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();

  if( displayType == DisplayRamp || displayType == DisplayTex || displayType == DisplayEdges ||
      displayType == DisplayDetail || displayType == DisplayFiltered ) {


    const int seSize = 2 * seRadius + 1;

    if ( src.channels() == 1 ) {
      ramptexLee(src, ramp, tex, cv::Mat1b(seSize, seSize, 255), cv::BORDER_REFLECT101);
    }
    else {
      cv::Mat tmp;
      cv::cvtColor(src, tmp, cv::COLOR_BGR2GRAY);
      ramptexLee(tmp, ramp, tex, cv::Mat1b(seSize, seSize, 255), cv::BORDER_REFLECT101);
    }

    const cv::Mat1f G1 = cv::getGaussianKernel(2 * seSize + 1, 0, CV_32F);
    cv::sepFilter2D(ramp, ramp, CV_32F, G1, G1, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    if( displayType == DisplayRamp ) {
      ramp.copyTo(image);
      return true;
    }

    cv::add(tex, std::max(1e-5, 0.1 * cv::mean(tex)[0]), tex);
    fast_gaussian_blur(tex, cv::noArray(), tex, 15);
    if( displayType == DisplayTex ) {
      tex.copyTo(image);
      return true;
    }

    cv::divide(ramp, tex, edges);

    double xmin, xmax;
    calculateSBounds(edges, &xmin, &xmax, qmin, qmax);
    mapPixelsSShaped(edges, sn, xmin, xmax, smin, smax);

    if( displayType == DisplayEdges ) {
      edges.copyTo(image);
      return true;
    }
  }

  if( displayType == DisplayLpass || displayType == DisplayHpass || displayType == DisplayDetail || displayType == DisplayFiltered ) {

    if ( mkRadius < 1 ) {
      fast_gaussian_blur(src, mask, lpass, gfSigma, cv::BORDER_DEFAULT, CV_32F);
    }
    else {
      cv::medianBlur(src, lpass, 2 * mkRadius + 1);
      fast_gaussian_blur(lpass, mask, lpass, gfSigma, cv::BORDER_DEFAULT, CV_32F);
    }

    if( displayType == DisplayLpass ) {
      lpass.copyTo(image);
      return true;
    }
  }

  if( displayType == DisplayHpass || displayType == DisplayDetail || displayType == DisplayFiltered ) {

    cv::subtract(src, lpass, hpass, cv::noArray(), CV_32F);

    if( displayType == DisplayHpass ) {
      hpass.copyTo(image);
      return true;
    }
  }

  if( displayType == DisplayDetail || displayType == DisplayFiltered ) {

    if ( src.channels() > 1 ) {
      std::vector<cv::Mat> ec(src.channels(), edges);
      cv::merge(ec, edges);
    }

    cv::multiply(edges, hpass, detail, hBoost, CV_32F);

    if( displayType == DisplayDetail) {
      detail.copyTo(image);
      return true;
    }
  }

  cv::add(lpass, detail, image);

  return true;
}
