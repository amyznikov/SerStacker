/*
 * c_mgmap_sharpness_measure.cc
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#include "c_local_variance_sharpness_measure.h"

#include <core/proc/pixtype.h>

static bool pdownscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  if( std::min(src.cols(), src.rows()) < 4 ) {
    src.copyTo(dst);
  }
  else {

    cv::pyrDown(src, dst, cv::Size(), border_mode);

    if( std::min(dst.cols, dst.rows) >= 4 ) {

      for( int l = 1; l < level; ++l ) {

        cv::pyrDown(dst, dst, cv::Size(), border_mode);

        if( std::min(dst.cols, dst.rows) < 4 ) {
          break;
        }
      }
    }
  }

  return true;
}

static bool pupscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((sizes.back().width + 1) / 2, (sizes.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}

static inline void computeMorphGradient(cv::InputArray src, cv::OutputArray dst, int kradius)
{
}

static void downscaleLocalVariance(cv::InputArray src, cv::OutputArray outputVarianceMap,
    int downscale_levels, int morph_gradient_radius)
{
  const cv::Mat1b SE(2 * morph_gradient_radius + 1,
      2 * morph_gradient_radius + 1, 255);

  if( downscale_levels <= 0 ) {
    cv::morphologyEx(src, outputVarianceMap, cv::MORPH_GRADIENT,
        SE, cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
    return;
  }

  cv::Mat current_img = src.getMat();
  cv::Mat current_grad;
  cv::Mat accumulated_variance;

  for( int level = 0; level < downscale_levels; ++level ) {

    cv::morphologyEx(current_img, current_grad, cv::MORPH_GRADIENT,
        SE, cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);

    if( accumulated_variance.empty() ) {
      current_grad.copyTo(accumulated_variance);
    }
    else {
      cv::pyrDown(accumulated_variance, accumulated_variance, current_grad.size());
      cv::add(current_grad, accumulated_variance, accumulated_variance);
    }

    if( level < downscale_levels - 1 ) {
      cv::pyrDown(current_img, current_img);
    }
  }

  outputVarianceMap.move(accumulated_variance);
}

bool create_morph_gradient_map(cv::InputArray _src, cv::OutputArray dst,
    const c_local_variance_map_options & opts)
{
  cv::Mat src, g;

  if ( _src.channels() == 1  ) {
    src = _src.getMat();
  }
  else {
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
  }

  downscaleLocalVariance(src, g, opts.dscale, std::max(1, opts.kradius));

  if( opts.uscale > 0 && opts.uscale > opts.dscale ) {
    pdownscale(g, g, opts.uscale - opts.dscale);
  }

  if( opts.p == 2 ) {
    cv::multiply(g, g, g);
  }
  else if ( opts.p != 0 && opts.p != 1 ) {
    cv::pow(g, opts.p, g);
  }

  if( g.size() != src.size() ) {
    pupscale(g, src.size());
  }

  dst.move(g);

  return true;
}

bool create_local_variance_map(cv::InputArray _src, cv::OutputArray dst, const c_local_variance_map_options & opts)
{
  cv::Mat src;
  cv::Mat m, s;

  if (_src.channels() == 1) {
    _src.getMat().convertTo(src, CV_32F);
  }
  else {
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
    src.convertTo(src, CV_32F);
  }

  if ( opts.dscale > 0 ) {
    pdownscale(src, src, opts.dscale);
  }

  const int winSize = 2 * std::max(1, opts.kradius) + 1;
  const cv::Size ksize(winSize, winSize);

  cv::boxFilter(src, m, CV_32F, ksize, cv::Point(-1,-1), false, cv::BORDER_REPLICATE);
  cv::boxFilter(src.mul(src), s, CV_32F, ksize, cv::Point(-1,-1), false, cv::BORDER_REPLICATE);
  cv::absdiff(s, m.mul(m), s);

  if( opts.uscale > 0 && opts.uscale > opts.dscale ) {
    pdownscale(s, s, opts.uscale - opts.dscale);
  }

  if( opts.p == 2 ) {
    cv::multiply(s, s, s);
  }
  else if( opts.p != 0 && opts.p != 1 ) {
    cv::pow(s, opts.p, s);
  }

  if( s.size() != _src.size() ) {
    pupscale(s, _src.size());
  }

  dst.move(s);
  return true;
}

bool c_local_variance_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map,
    const c_local_variance_map_options & opts)
{
  return create_morph_gradient_map(image, output_map, opts);
}

bool c_local_variance_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return create_map(image, output_map, _opts);
}

cv::Scalar c_local_variance_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Mat map;
  create_map(image, map);
  return cv::mean(map, mask);
}

bool serialize_local_variance_map_options(c_config_setting section, bool save, c_local_variance_map_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, p);
  SERIALIZE_OPTION(section, save, opts, kradius);
  SERIALIZE_OPTION(section, save, opts, dscale);
  SERIALIZE_OPTION(section, save, opts, uscale);
  return true;
}
