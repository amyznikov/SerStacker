/*
 * c_dog_routine.cc
 *
 *  Created on: Jul 31, 2026
 *      Author: amyznikov
 */

#include "c_dog_routine.h"

static void computeDoG(cv::InputArray _src, cv::OutputArray dst, double sigma1, double sigma2,
    cv::BorderTypes borderType, const cv::Scalar & borderValue)
{
  cv::Mat src, g1, g2;
  cv::Rect roi;

  const int clearBorderType = (borderType & ~cv::BORDER_ISOLATED);
  const bool needBorder = clearBorderType == cv::BORDER_WRAP
      || clearBorderType == cv::BORDER_CONSTANT
      || clearBorderType == cv::BORDER_TRANSPARENT;

  if( !needBorder ) {
    src = _src.getMat();
  }
  else {
    const double maxSigma = std::max(sigma1, sigma2);
    const int borderSize = static_cast<int>(std::ceil(3.0 * maxSigma));
    cv::copyMakeBorder(_src, src, borderSize, borderSize, borderSize, borderSize,
        borderType, borderValue);
    roi = cv::Rect(borderSize, borderSize, _src.cols(), _src.rows());
  }

  if( sigma1 <= 0 ) {
    g1 = src;
  }
  else {
    const int ksize = 2 * std::max(1, (int)(sigma1 * 4)) + 1;
    const cv::Mat1f G = cv::getGaussianKernel(ksize, sigma1, CV_32F);
    cv::sepFilter2D(src, g1, -1, G, G, cv::Point(-1,-1), 0, borderType);
  }

  if( sigma2 <= 0 ) {
    g2 = src;
  }
  else {
    const int ksize = 2 * std::max(1, (int)(sigma2 * 4)) + 1;
    const cv::Mat1f G = cv::getGaussianKernel(ksize, sigma2, CV_32F);
    cv::sepFilter2D(src, g2, -1, G, G, cv::Point(-1,-1), 0, borderType);
  }

  const int ddepth =
      dst.fixedType() ? dst.depth() :
          std::max(_src.depth(), CV_32F);

  if ( roi.empty() ) {
    cv::subtract(g1, g2, dst, cv::noArray(), ddepth);
  }
  else {
    cv::subtract(g1(roi), g2(roi), dst, cv::noArray(), ddepth);
  }
}

void c_dog_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "sigma1", ctx(&this_class::sigma1), "Gaussian Sigma1 ");
  ctlbind(ctls, "sigma2", ctx(&this_class::sigma2), "Gaussian Sigma2");
  ctlbind(ctls, "border_type", ctx(&this_class::borderType), "");
  ctlbind(ctls, "border_value", ctx(&this_class::borderValue), "");
}

bool c_dog_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, sigma1);
    SERIALIZE_OPTION(settings, save, *this, sigma2);
    SERIALIZE_OPTION(settings, save, *this, borderType);
    SERIALIZE_OPTION(settings, save, *this, borderValue);
    return true;
  }
  return false;
}

bool c_dog_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  computeDoG(image.getMat(), image, sigma1, sigma2, borderType, borderValue);

  return true;
}

