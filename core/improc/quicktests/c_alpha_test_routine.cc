/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC_IMAGE, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_MEAN_IMAGE, "MEAN", "" },
      { c_alpha_test_routine::DISPLAY_STDEV_IMAGE, "STDEV", "" },
      { c_alpha_test_routine::DISPLAY_MEANSTDEV_IMAGE, "MEANSTDEV", "" },
      { c_alpha_test_routine::DISPLAY_TOPHAT_IMAGE, "TOPHAT", "" },
      { c_alpha_test_routine::DISPLAY_BGMAP_IMAGE, "BGMAP", "" },
      { c_alpha_test_routine::DISPLAY_MEANSTDEV_IMAGE}
  };
  return members;
}

static void create_meansdtdev_map(cv::InputArray _src, cv::OutputArray _dst,
    double gsigma, double kmean, double kstdev)
{
  const cv::Mat src = _src.getMat();
  cv::Mat m, s;
  fast_gaussian_blur(src, cv::noArray(), m, gsigma, cv::BORDER_DEFAULT, CV_32F);
  fast_gaussian_blur(src.mul(src), cv::noArray(), s, gsigma, cv::BORDER_DEFAULT, CV_32F);
  cv::absdiff(s, m.mul(m), s);
  cv::sqrt(s,s);
  cv::addWeighted(m, kmean, s, kstdev, 0, _dst);
}

static void create_median_background_map(cv::InputArray _src, cv::OutputArray _dst, int lvls)
{
  cv::Mat src;
  std::vector<cv::Size> sizes;

  sizes.emplace_back(_src.size());
  cv::medianBlur(_src, src, 5);
  cv::pyrDown(src, src);
  for ( int i = 1; i < lvls; ++i ) {
    sizes.emplace_back(src.size());
    cv::medianBlur(src, src, 5);
    cv::pyrDown(src, src);
  }

  for ( int i = sizes.size()-1; i >= 0; --i ) {
    cv::pyrUp(src, src, sizes[i]);
  }

  _dst.move(src);
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "gsigma", CTL_CONTEXT(ctx, _gsigma), "");
  ctlbind(ctls, "kmean", CTL_CONTEXT(ctx, _kmean), "");
  ctlbind(ctls, "kstdev", CTL_CONTEXT(ctx, _kstdev), "");
  ctlbind(ctls, "bgmaplvls", CTL_CONTEXT(ctx, _bgmaplvls), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _kmean);
    SERIALIZE_OPTION(settings, save, *this, _kstdev);
    SERIALIZE_OPTION(settings, save, *this, _bgmaplvls);
    return true;
  }
  return false;
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  if ( _display == DISPLAY_BGMAP_IMAGE ) {
    create_median_background_map(image.getMat(), image, _bgmaplvls);
    return true;
  }


  const int cn = image.channels();
  cv::Mat src, blur1, blur2, detail, gray, texture, K, filtered;

  image.getMat().convertTo(src, CV_32F);

  const double kmean = _display == DISPLAY_STDEV_IMAGE ? 0 : _kmean;
  const double kstdev = _display == DISPLAY_MEAN_IMAGE ? 0 : _kstdev;

  create_meansdtdev_map(src, image, _gsigma, kmean, kstdev);

  if ( _display == DISPLAY_TOPHAT_IMAGE ) {
    cv::subtract(src, image, image);
  }

  return true;
}

