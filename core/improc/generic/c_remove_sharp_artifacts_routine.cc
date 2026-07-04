/*
 * c_remove_sharp_artifacts_routine.cc
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#include "c_remove_sharp_artifacts_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>


template<class T>
static bool _wadd(cv::InputOutputArray image, cv::InputArray _blured_image, const cv::Mat1f & w)
{
  cv::Mat_<T> img = image.getMat();
  const cv::Mat_<T> & blured = _blured_image.getMat();

  const int rows = image.rows();
  const int cols = image.cols();
  const int channels = image.channels();

  cv::parallel_for_(cv::Range(0, rows),
      [&, cols, channels](const auto & range) {

        for( int y = range.start; y < range.end; ++y ) {
          const T * blurp = blured[y];
          const float *wp = w[y];

          T * dstp = img[y];

          for( int x = 0; x < cols; ++x ) {

            const double alpha = wp[x];

            if( alpha > FLT_EPSILON ) {
              for( int c = 0; c < channels; ++c ) {
                dstp[x * channels + c] = cv::saturate_cast<T>(
                    alpha * dstp[x * channels + c] + (1 - alpha) * blurp[x * channels + c]);
              }
            }
          }
        }
      });

  return true;
}

static bool wadd(cv::InputOutputArray image, cv::InputArray blured_image, const cv::Mat1f & w)
{
  CV_DISPATCH(image.depth(), _wadd, image, blured_image, w);
  return false;
}

void c_remove_sharp_artifacts_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "mask_erode_radius", ctx(&this_class::_mask_erode_radius), "SE radius for erode planetary disk mask");
   ctlbind(ctls, "mask_blur_radius", ctx(&this_class::_mask_blur_radius), "GaussianBlur sigma used to blur mask");
   ctlbind(ctls, "edge_blur_radius", ctx(&this_class::_edge_blur_radius), "GaussianBlur sigma used to blur image edge");
   ctlbind(ctls, "show_mask", ctx(&this_class::_show_mask), "show objects mask instead of processing");
   ctlbind(ctls, "show_blured_image", ctx(&this_class::_show_blured_image), "show blured image instead of processing");
}

bool c_remove_sharp_artifacts_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _mask_erode_radius);
    SERIALIZE_OPTION(settings, save, *this, _mask_blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _edge_blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _show_mask);
    SERIALIZE_OPTION(settings, save, *this, _show_blured_image);
    return true;
  }
  return false;
}

bool c_remove_sharp_artifacts_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.empty() ) {
    return true;

  }
  cv::Mat planetary_disk_mask;

  const bool planetaryDiskDetected =
      simple_planetary_disk_detector(image, mask,
          1, 5,
          nullptr,
          nullptr,
          &planetary_disk_mask,
          nullptr);

  if( !planetaryDiskDetected ) {
    CF_ERROR("simple_planetary_disk_detector fails");
    return false;
  }

  if( _mask_erode_radius.width > 0 || _mask_erode_radius.height > 0 ) {
    cv::Mat1b se(2 * _mask_erode_radius.height + 1, 2 * _mask_erode_radius.width + 1, 255);
    cv::erode(planetary_disk_mask, planetary_disk_mask, se);
  }

  if( _mask_blur_radius > 0 ) {
    cv::GaussianBlur(planetary_disk_mask, planetary_disk_mask, cv::Size(),
        _mask_blur_radius, _mask_blur_radius,
        cv::BORDER_DEFAULT);
  }

  if( _show_mask ) {
    planetary_disk_mask.convertTo(image, CV_32F, 1. / 255);
    return true;
  }

  cv::Mat blured_image;
  cv::medianBlur(image, blured_image, 5);

  if( _edge_blur_radius > 0 ) {
    cv::GaussianBlur(blured_image, blured_image, cv::Size(),
        _edge_blur_radius, _edge_blur_radius);
  }

  if( _show_blured_image ) {
    image.move(blured_image);
    return true;
  }

  planetary_disk_mask.convertTo(planetary_disk_mask, CV_32F, 1. / 255);
  wadd(image, blured_image, planetary_disk_mask);

  return true;
}

