/*
 * c_desaturate_shadows_routine.cc
 *
 *  Created on: May 31, 2025
 *      Author: amyznikov
 */

#include "c_desaturate_shadows_routine.h"
#include <core/proc/color_saturation.h>
#include <core/proc/autoclip.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

template<class T>
static void combine_images(cv::InputArray _color_image, cv::InputArray _gray_image,const cv::Mat1f & weights,
    cv::OutputArray _combined_image,
    double wmin, double wmax)
{

  const cv::Mat_<cv::Vec<T, 3>> color_image =
      _color_image.getMat();

  const cv::Mat_<T> gray_image =
      _gray_image.getMat();

  _combined_image.create(_color_image.size(),
      _color_image.type());

  cv::Mat_<cv::Vec<T, 3>> combined_image =
      _combined_image.getMatRef();

  const double strecth =
      1 / (wmax - wmin);

  for( int y = 0; y < color_image.rows; ++y ) {
    for( int x = 0; x < color_image.cols; ++x ) {

      const double w1 = std::max(0., std::min(1., (weights[y][x] - wmin) * strecth));
      const double w2 = 1 - w1;
      const double g = gray_image[y][x];

      for( int i = 0; i < 3; ++i ) {

        combined_image[y][x][i] =
            cv::saturate_cast<T>(w1 * color_image[y][x][i] + w2 * g);
      }

    }
  }
}

static void combine_images(cv::InputArray color_image, cv::InputArray gray_image, const cv::Mat1f & weights,
    cv::OutputArray combined_image, double wmin, double wmax)
{

  switch (color_image.depth()) {
    case CV_8U:
      combine_images<uint8_t>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_8S:
      combine_images<int8_t>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_16U:
      combine_images<uint16_t>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_16S:
      combine_images<int16_t>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_32S:
      combine_images<int32_t>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_32F:
      combine_images<float>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    case CV_64F:
      combine_images<double>(color_image, gray_image, weights, combined_image, wmin, wmax);
      break;
    default:
      break;
  }

}

void c_desaturate_shadows_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, wmin, "");
  BIND_PCTRL(ctls, wmax, "");
  BIND_PCTRL(ctls, mblur, "");
}

bool c_desaturate_shadows_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, wmin);
    SERIALIZE_PROPERTY(settings, save, *this, wmax);
    SERIALIZE_PROPERTY(settings, save, *this, mblur);
    return true;
  }
  return false;
}

bool c_desaturate_shadows_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() != 3 ) {
    return true;
  }

  color_saturation_hls(image, _gray, 0);

  cv::cvtColor(_gray, _gray, cv::COLOR_BGR2GRAY);

  convert_depth(_gray, CV_32F, _weights);

  if( _mblur ) {
    fast_gaussian_blur(_weights, cv::noArray(), _weights, _mblur);
  }

  combine_images(image.getMat(), _gray, _weights, image, _wmin, _wmax);

  return true;
}
