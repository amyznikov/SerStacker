/*
 * mtf-histogram.cc
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#include "mtf-histogram.h"
#include <core/proc/histogram.h>
#include <core/debug.h>

template<class T>
static void build_histogram(cv::InputArray _image, cv::InputArray _mask, c_histogram_builder & builder,
    const c_midtones_transfer_function & mtf)
{
  const cv::Mat image =
      _image.getMat();

  const cv::Mat1b mask =
      _mask.getMat();

  cv::Scalar s;

  const int cn =
      image.channels();

  for( int y = 0; y < image.rows; ++y ) {

    const T * sp =
        image.ptr<const T>(y);

    if( mask.empty() ) {
      for( int x = 0; x < image.cols; ++x ) {
        for( int c = 0; c < cn; ++c ) {
          s[c] = mtf.apply(sp[c + x * cn]);
        }
        builder.add_pixel(s);
      }
    }
    else if( mask.channels() == 1 ) {

      const uint8_t *mp = mask[y];

      for( int x = 0; x < image.cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < cn; ++c ) {
            s[c] = mtf.apply(sp[c + x * cn]);
          }
          builder.add_pixel(s);
        }
      }
    }
  }
}


// @brief build histogram for given multi-channel image.
// Output is single-channel CV_32FC1 matrix of size 'nbins rows' x 'image channels columns'.
// if input mask is not empty then it must be sigle-channel CV_8U matrix of the same size as input image.
bool create_output_histogram(const c_midtones_transfer_function * mtf,
    cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    /*[in, out]*/ double * minval,
    /*[in, out]*/ double * maxval,
    int nbins,
    bool cumulative,
    bool scaled)
{
  if( !mtf ) {
    return create_histogram(image, mask, dst,
        minval, maxval, nbins,
        cumulative,
        scaled);
  }

  if( !mask.empty() && (mask.type() != CV_8UC1 || mask.size() != image.size()) ) {
    CF_ERROR("Invalid mask: %dx%d channels=%d depth=%d. image : %dx%d channels=%d depth=%d\n"
        "Must be single channel matrix of the same size as input image and CV_8UC1 type",
        mask.cols(), mask.rows(), mask.channels(), mask.depth(),
        image.cols(), image.rows(), image.channels(), image.depth() );
    return false;
  }


  mtf->get_output_range(minval, maxval);

  c_histogram_builder builder;
  double imin, imax;

  builder.set_input_range(*minval, *maxval);
  builder.set_bins(nbins);
  builder.set_cumulative(cumulative);
  builder.set_scaled(scaled);
  builder.set_channels(image.channels());

  mtf->get_input_range(&imin, &imax);
  if( imin >= imax ) {

    double adjusted_min, adjusted_max;

    c_midtones_transfer_function::suggest_levels_range(
        image.depth(),
        &adjusted_min,
        &adjusted_max);

    (const_cast<c_midtones_transfer_function*>(mtf))->
        set_input_range(adjusted_min, adjusted_max);
  }

  switch (image.depth()) {
  case CV_8U:
    build_histogram<uint8_t>(image, mask, builder, *mtf);
    break;
  case CV_8S:
    build_histogram<int8_t>(image, mask, builder, *mtf);
    break;
  case CV_16U:
    build_histogram<uint16_t>(image, mask, builder, *mtf);
    break;
  case CV_16S:
    build_histogram<int16_t>(image, mask, builder, *mtf);
    break;
  case CV_32S:
    build_histogram<int32_t>(image, mask, builder, *mtf);
    break;
  case CV_32F:
    build_histogram<float>(image, mask, builder, *mtf);
    break;
  case CV_64F:
    build_histogram<double>(image, mask, builder, *mtf);
    break;
  default:
    break;
  }

  if( imin >= imax ) {
    (const_cast<c_midtones_transfer_function*>(mtf))->
        set_input_range(imin, imax);
  }

  builder.compute(dst);
  return true;
}
