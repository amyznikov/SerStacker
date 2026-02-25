/*
 * c_scale_channels_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_scale_channels_routine.h"
#include <tbb/tbb.h>

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

void c_scale_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "stretch_r", ctx, &this_class::stretch_r, &this_class::set_stretch_r, "");
  ctlbind(ctls, "bias_r", ctx, &this_class::bias_r, &this_class::set_bias_r, "");
  ctlbind(ctls, "stretch_g", ctx, &this_class::stretch_g, &this_class::set_stretch_g, "");
  ctlbind(ctls, "bias_g", ctx, &this_class::bias_g, &this_class::set_bias_g, "");
  ctlbind(ctls, "stretch_b", ctx, &this_class::stretch_b, &this_class::set_stretch_b, "");
  ctlbind(ctls, "bias_b", ctx, &this_class::bias_b, &this_class::set_bias_b, "");
  ctlbind(ctls, "stretch_a", ctx, &this_class::stretch_a, &this_class::set_stretch_a, "");
  ctlbind(ctls, "bias_a", ctx, &this_class::bias_a, &this_class::set_bias_a, "");
}

bool c_scale_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, bias);
    SERIALIZE_PROPERTY(settings, save, *this, stretch);
    return true;
  }
  return false;
}

bool c_scale_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat & img = image.getMatRef();
  const cv::Mat & msk = mask.getMat();
  switch (img.depth()) {
    case CV_8U:
      stretch_channels_<uint8_t>(img, msk, _bias, _stretch);
      break;
    case CV_8S:
      stretch_channels_<int8_t>(img, msk, _bias, _stretch);
      break;
    case CV_16U:
      stretch_channels_<uint16_t>(img, msk, _bias, _stretch);
      break;
    case CV_16S:
      stretch_channels_<int16_t>(img, msk, _bias, _stretch);
      break;
    case CV_32S:
      stretch_channels_<int32_t>(img, msk, _bias, _stretch);
      break;
    case CV_32F:
      stretch_channels_<float>(img, msk, _bias, _stretch);
      break;
    case CV_64F:
      stretch_channels_<double>(img, msk, _bias, _stretch);
      break;
    default:
      return false;
  }

  return true;
}

