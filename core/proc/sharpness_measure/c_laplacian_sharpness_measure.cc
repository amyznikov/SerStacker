/*
 * c_laplacian_sharpness_measure.cc
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#include "c_laplacian_sharpness_measure.h"
#include <core/proc/morphology.h>
#include <core/proc/pyrscale.h>

c_laplacian_sharpness_measure::c_laplacian_sharpness_measure()
{

}

c_laplacian_sharpness_measure::c_laplacian_sharpness_measure(int dscale, const cv::Size & se_size)
{

  _opts.dscale = dscale;
  _opts.se_size = se_size;
}

void c_laplacian_sharpness_measure::set_dscale(int v)
{
  _opts.dscale = v;
}

int c_laplacian_sharpness_measure::dscale() const
{
  return _opts.dscale;
}

void c_laplacian_sharpness_measure::set_se_size(const cv::Size & v)
{
  _opts.se_size = v;
}

const cv::Size & c_laplacian_sharpness_measure::se_size() const
{
  return _opts.se_size;
}

bool c_laplacian_sharpness_measure::create_map(cv::InputArray image,
    int dscale, const cv::Size & se_size,
    cv::OutputArray output_map)
{
  cv::Mat img;

  if( image.channels() == 1 ) {
    if( dscale > 0 ) {
      pyramid_downscale(image, img, dscale);
    }
  }
  else {
    cv::cvtColor(image, img, cv::COLOR_BGR2GRAY);
    if( dscale > 0 ) {
      pyramid_downscale(img, img, dscale);
    }
  }

  if( dscale > 0 ) {
    pyramid_downscale(img, img, dscale);
  }

  static thread_local cv::Mat1b SE;

  if( SE.size() != se_size ) {
    SE.create(se_size);
    SE.setTo(255);
  }

  morphological_laplacian_abs(img, output_map, SE, cv::BORDER_REPLICATE);

  return  true;
}

bool c_laplacian_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return create_map(image, _opts.dscale, _opts.se_size, output_map);
}

cv::Scalar c_laplacian_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Scalar v;
  compute(image, mask, _opts.dscale,  _opts.se_size, &v);
  return v;
}

bool c_laplacian_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask,
    int dscale, const cv::Size & se_size,
    cv::Scalar * output_sharpness_measure)
{
  cv::Mat lap, msk;

  create_map(image, dscale, se_size, lap);

  if( !mask.empty() ) {
    if( mask.size() == lap.size() ) {
      msk = mask.getMat();
    }
    else {
      cv::resize(mask, msk, lap.size(), 0, 0, cv::INTER_NEAREST);
    }
  }

  *output_sharpness_measure =
      cv::mean(lap, msk);

  return true;
}
