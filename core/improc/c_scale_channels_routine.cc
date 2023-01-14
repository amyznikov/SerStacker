/*
 * c_scale_channels_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_scale_channels_routine.h"
#include <core/debug.h>
#include <tbb/tbb.h>


c_scale_channels_routine::c_class_factory c_scale_channels_routine::class_factory;

c_scale_channels_routine::c_scale_channels_routine(bool enabled)
  : base(&class_factory, enabled), bias_(0,0,0,0), stretch_(1,1,1,1)
{
}

c_scale_channels_routine::ptr c_scale_channels_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_scale_channels_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, *this, bias);
  LOAD_PROPERTY(settings, *this, stretch);

  return true;
}


bool c_scale_channels_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  //save_settings(settings, "bias", this->bias());
  //save_settings(settings.add_member("bias", c_config_type_traits<cv::Scalar>::type), bias());

  SAVE_PROPERTY(settings, *this, bias);
  SAVE_PROPERTY(settings, *this, stretch);

  return true;

}

template<class T>
static void stretch_channels_(cv::Mat & image, const cv::Mat & mask, const cv::Scalar & bias, const cv::Scalar & stretch )
{

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, image.rows, 512),
      [&image, &mask, bias, stretch](const range & r ) {

     const int cn = image.channels();
     const int cnmax = std::min(cn, 4);

     if ( mask.size() != image.size() ) {  // ignore mask

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          T * imgp = image.ptr<T>(y);

          for ( int x = 0, nx = image.cols; x < nx; ++x ) {
            for ( int c = 0; c < cnmax; ++c ) {

              imgp[x * cn + c] = cv::saturate_cast<T>(
                  (imgp[x * cn + c] * stretch[c] + bias[c]));

            }

          }
        }
      }
      else if ( mask.channels() == 1 ) {  // single channel mask


        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          T * imgp = image.ptr<T>(y);

          const uint8_t * mskp = mask.ptr<uint8_t>(y);

          for ( int x = 0, nx = image.cols; x < nx; ++x ) {
            if ( mskp[x] ) {

              for ( int c = 0; c < cnmax; ++c ) {

                imgp[x * cn + c] = cv::saturate_cast<T>(
                    (imgp[x * cn + c] * stretch[c] + bias[c]));

              }
            }

          }
        }
      }
      else if ( mask.channels() == image.channels() ) {  // multi channel mask

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          T * imgp = image.ptr<T>(y);

          const uint8_t * mskp = mask.ptr<uint8_t>(y);

          for ( int x = 0, nx = image.cols; x < nx; ++x ) {
            for ( int c = 0; c < cnmax; ++c ) {
              if ( mskp[x * cn + c] ) {
                imgp[x * cn + c] = cv::saturate_cast<T>(
                    (imgp[x * cn + c] * stretch[c] + bias[c]));
              }
            }
          }
        }
      }

    });
}

bool c_scale_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat & img = image.getMatRef();
  const cv::Mat & msk = mask.getMat();
  switch (img.depth()) {
    case CV_8U:
      stretch_channels_<uint8_t>(img, msk, bias_, stretch_);
      break;
    case CV_8S:
      stretch_channels_<int8_t>(img, msk, bias_, stretch_);
      break;
    case CV_16U:
      stretch_channels_<uint16_t>(img, msk, bias_, stretch_);
      break;
    case CV_16S:
      stretch_channels_<int16_t>(img, msk, bias_, stretch_);
      break;
    case CV_32S:
      stretch_channels_<int32_t>(img, msk, bias_, stretch_);
      break;
    case CV_32F:
      stretch_channels_<float>(img, msk, bias_, stretch_);
      break;
    case CV_64F:
      stretch_channels_<double>(img, msk, bias_, stretch_);
      break;
    default:
      return false;
  }

  return true;
}

