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
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayWeights, "Weights" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayDetail, "Detail" },
      //{ c_adaptive_blend_filter_routine::DisplayType::DisplayAlpha, "Alpha" },
      { c_adaptive_gaussian_blur_routine::DisplayType::DisplayFiltered}  // must  be last
  };

  return members;
}

void c_adaptive_gaussian_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "seRadius", ctx(&this_class::seRadius), "Morphologcal SE radius");
  ctlbind_spinbox(ctls, "edgeLimit", ctx(&this_class::edgeLimit), 0.0, 100.0, 0.01, "");
  ctlbind(ctls, "mkRadius", ctx(&this_class::mkRadius), "medianBlur kernel radius for guide image");
  ctlbind(ctls, "gfSigma", ctx(&this_class::gfSigma), "Gaussian filter sigma");
  ctlbind(ctls, "hBoost", ctx(&this_class::hBoost), "");
  ctlbind_spinbox(ctls, "x1", ctx(&this_class::x1), 0.0, 0.99, 0.01, "");
  ctlbind_spinbox(ctls, "a1", ctx(&this_class::a1), 0.0, 0.999, 0.001, "");
  ctlbind_spinbox(ctls, "x2", ctx(&this_class::x2), 0.0, 0.99, 0.01, "");
  ctlbind_spinbox(ctls, "a2", ctx(&this_class::a2), 0.0, 0.999, 0.001, "");
  ctlbind(ctls, "display", ctx(&this_class::displayType), "Select output image");
}

bool c_adaptive_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, displayType);
    SERIALIZE_OPTION(settings, save, *this, seRadius);
    SERIALIZE_OPTION(settings, save, *this, edgeLimit);
//    SERIALIZE_OPTION(settings, save, *this, lpgP);
//    SERIALIZE_OPTION(settings, save, *this, lpgDscale);
//    SERIALIZE_OPTION(settings, save, *this, lpgUscale);
    SERIALIZE_OPTION(settings, save, *this, mkRadius);
    SERIALIZE_OPTION(settings, save, *this, gfSigma);
    SERIALIZE_OPTION(settings, save, *this, hBoost);
    SERIALIZE_OPTION(settings, save, *this, x1);
    SERIALIZE_OPTION(settings, save, *this, a1);
    SERIALIZE_OPTION(settings, save, *this, x2);
    SERIALIZE_OPTION(settings, save, *this, a2);
    return true;
  }
  return false;
}

static double calculateXMax(const cv::Mat1f & src, double percentile = 0.95)
{
  std::vector<float> samples;
  if( src.isContinuous() ) {
    samples.assign(src.ptr<float>(0), src.ptr<float>(0) + src.total());
  }
  else {
    for( int i = 0; i < src.rows; ++i ) {
      samples.insert(samples.end(), src.ptr<float>(i), src.ptr<float>(i) + src.cols);
    }
  }

  const auto it = samples.begin() + static_cast<int>((samples.size() - 1) * std::max(0.0, std::min(percentile, 1.0)));
  std::nth_element(samples.begin(), it, samples.end());

  return static_cast<double>(*it);
}

static void mapPixels(cv::InputArray _src, cv::OutputArray _dst, double xmax)
{
  const cv::Mat1f src = _src.getMat();
  _dst.create(src.size(), src.type());
  cv::Mat1f dst = _dst.getMatRef();

  const float n = 6.0f;
  const float xmax_n = std::pow(static_cast<float>(xmax), n);
  const float inv_n = 1.0f / n;

  dst.forEach([&](float & pixel, const int position[]) -> void {
    const float x = src(position[0], position[1]);
    if (x <= 0.0f) {
      pixel = 0.0f;
    }
    else { // pixel = x / (x^n + xmax^n)^(1/n)
      float x_n = std::pow(x, n);
      pixel = x / std::pow(x_n + xmax_n, inv_n);
    }
  });
}

bool c_adaptive_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();

  if( displayType == DisplayRamp || displayType == DisplayTex || displayType == DisplayEdges ||
      displayType == DisplayWeights || displayType == DisplayDetail || displayType == DisplayFiltered ) {


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

    //    const cv::Mat1f G2 = cv::getGaussianKernel(4 * seSize + 1, 0, CV_32F);
    //    cv::sepFilter2D(tex, tex, CV_32F, G2, G2, cv::Point(-1, -1), 0.1 * cv::mean(tex)[0], cv::BORDER_DEFAULT);
    cv::add(tex, 0.1 * cv::mean(tex)[0], tex);
    fast_gaussian_blur(tex, cv::noArray(), tex, 15);
    if( displayType == DisplayTex ) {
      tex.copyTo(image);
      return true;
    }



    cv::divide(ramp, tex, edges);
    mapPixels(edges, edges, calculateXMax(edges, 0.01 * edgeLimit));

    if( displayType == DisplayEdges ) {
      edges.copyTo(image);
      return true;
    }
  }

  if( displayType == DisplayWeights || displayType == DisplayDetail || displayType == DisplayFiltered ) {

    const float L1 = std::log((1.f - a1) / a1);
    const float L2 = std::log((1.f - a2) / a2);
    const float beta = (L1 - L2) / std::max(x2 - x1, 1e-6f);
    const float x0 = (L1 / beta) + x1;

    CF_DEBUG("L1=%g L2=%g a1=%g a2=$g x0=%g beta=%g", L1, L2, a1, a2, x0, beta);

    cv::exp(beta * (x0 - edges), weights);
    weights = 1.0f / (1.0f + weights);

    if( displayType == DisplayWeights ) {
      weights.copyTo(image);
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
      std::vector<cv::Mat> wc(src.channels(), weights);
      cv::merge(wc, weights);
    }

    cv::multiply(weights, hpass, detail, hBoost, CV_32F);

    if( displayType == DisplayDetail) {
      detail.copyTo(image);
      return true;
    }
  }

  cv::add(lpass, detail, image);

  return true;
}
