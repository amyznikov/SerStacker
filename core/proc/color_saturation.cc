/*
 * color_saturation.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 *
 * Base on code from
 *  <https://github.com/GNOME/gimp/blob/master/libgimpcolor/gimpcolorspace.c>
 *
 */

#include "color_saturation.h"
#include "minmax.h"
#include <tbb/tbb.h>
#include <core/debug.h>



/**
 * gimp_rgb_to_hsl:
 * @rgb: A color value in the RGB colorspace
 * @hsl: (out caller-allocates): The value converted to HSL
 *
 * Convert an RGB color value to a HSL (Hue, Saturation, Lightness)
 * color value.
 **/
static void gimp_rgb_to_hsl(double b, double g, double r,
    double * h, double * s, double * l)
{
  double max, min, delta;

  max = std::max(b, std::max(g, r));
  min = std::min(b, std::min(g, r));

  *l = (max + min) / 2.0;

  if( max == min ) {
    *s = 0.0;
    *h = 0;
  }
  else {

    if( *l <= 0.5 ) {
      *s = (max - min) / (max + min);
    }
    else {
      *s = (max - min) / (2.0 - max - min);
    }

    delta = max - min;

    if( delta <= FLT_EPSILON ) {
      delta = 1.0;
    }

    if( max == r ) {
      *h = (g - b) / delta;
    }
    else if( max == g ) {
      *h = 2.0 + (b - r) / delta;
    }
    else {
      *h = 4.0 + (r - g) / delta;
    }

    if( (*h /= 6.0) < 0.0 ) {
      *h += 1.0;
    }
  }
}


/**
 * gimp_hsl_to_rgb:
 * @hsl: A color value in the HSL colorspace
 * @rgb: (out caller-allocates): The value converted to a value
 *       in the RGB colorspace
 *
 * Convert a HSL color value to an RGB color value.
 **/
void gimp_hsl_to_rgb(double h, double s, double l,
    double * b, double * g, double * r)
{

  static const auto gimp_hsl_value =
      [](double n1, double n2, double hue) -> double
      {
        double val;

        if( hue > 6.0 ) {
          hue -= 6.0;
        }
        else if( hue < 0.0 ) {
          hue += 6.0;
        }

        if( hue < 1.0 ) {
          val = n1 + (n2 - n1) * hue;
        }
        else if( hue < 3.0 ) {
          val = n2;
        }
        else if( hue < 4.0 ) {
          val = n1 + (n2 - n1) * (4.0 - hue);
        }
        else {
          val = n1;
        }

        return val;
      };

  if( s == 0 ) { /*  achromatic case  */
    *r = *g = *b = l;
  }
  else {
    double m1, m2;

    if( l <= 0.5 ) {
      m2 = l * (1.0 + s);
    }
    else {
      m2 = l + s - l * s;
    }

    m1 = 2.0 * l - m2;

    *r = gimp_hsl_value(m1, m2, h * 6.0 + 2.0);
    *g = gimp_hsl_value(m1, m2, h * 6.0);
    *b = gimp_hsl_value(m1, m2, h * 6.0 - 2.0);
  }
}


bool get_data_range(cv::InputArray _src, cv::InputArray mask,
    double * minval, double * maxval)
{
  switch ( _src.depth() ) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
  case CV_64F :
  default:
    *minval = 0, *maxval = 1;
    getminmax(_src, minval, maxval, mask);
    break;
  }

  return true;
}

template<class T>
static void color_saturation_(cv::InputArray _src, cv::OutputArray _dst,
    double amount, cv::InputArray mask)
{
  const cv::Mat_<cv::Vec<T, 3>> src = _src.getMat();

  cv::Mat_<cv::Vec<T, 3>> dst(src.size());

  double minv, maxv, data_range;

  get_data_range(src, mask, &minv, &maxv);

  data_range = maxv - minv;

  typedef tbb::blocked_range<int> tbb_range;
  tbb::parallel_for(tbb_range(0, src.rows, 64),
      [&src, &dst, data_range, minv, amount](const tbb_range & rows) {

    double b, g, r, h, s, l;

        for( int y = rows.begin(), ny = rows.end(); y < ny; ++y ) {
          for( int x = 0; x < src.cols; ++x ) {

            const cv::Vec<T, 3> &srcp = src[y][x];
            cv::Vec<T, 3> &dstp = dst[y][x];

            gimp_rgb_to_hsl(
                (srcp[0] - minv) / data_range,
                (srcp[1] - minv) / data_range,
                (srcp[2] - minv) / data_range,
                &h, &s, &l);

            gimp_hsl_to_rgb(h,
                std::min(1., std::max(0., s * amount)),
                l,
                &b, &g, &r);

            dstp[0] = cv::saturate_cast<T>(b * data_range + minv);
            dstp[1] = cv::saturate_cast<T>(g * data_range + minv);
            dstp[2] = cv::saturate_cast<T>(r * data_range + minv);
          }
        }
      });

  _dst.move(dst);
}

template<class T>
static void color_saturation_(cv::Mat & _image,
    double amount, cv::InputArray mask)
{
  cv::Mat_<cv::Vec<T, 3>> image = _image;

  double minv, maxv, data_range;

  get_data_range(image, mask, &minv, &maxv);

  data_range = maxv - minv;

  typedef tbb::blocked_range<int> tbb_range;
  tbb::parallel_for(tbb_range(0, image.rows, 64),
      [&image, data_range, minv, amount](const tbb_range & rows) {

        double b, g, r, h, s, l;

        for( int y = rows.begin(), ny = rows.end(); y < ny; ++y ) {
          for( int x = 0; x < image.cols; ++x ) {

            cv::Vec<T, 3> & bgr = image[y][x];

            gimp_rgb_to_hsl(
                (bgr[0] - minv) / data_range,
                (bgr[1] - minv) / data_range,
                (bgr[2] - minv) / data_range,
                &h, &s, &l);

            gimp_hsl_to_rgb(h,
                std::min(1., std::max(0., s * amount)),
                l,
                &b, &g, &r);

            bgr[0] = cv::saturate_cast<T>(b * data_range + minv);
            bgr[1] = cv::saturate_cast<T>(g * data_range + minv);
            bgr[2] = cv::saturate_cast<T>(r * data_range + minv);
          }
        }
      });
}


bool color_saturation_hls(cv::InputArray src, cv::OutputArray dst, double scale,
    cv::InputArray mask)
{
  if( src.channels() != 3 ) { // Not a BGR
    src.copyTo(dst);
    return false;
  }

  if( scale == 1 ) {
    src.copyTo(dst);
    return true;
  }

  switch (src.depth()) {
  case CV_8U:
    color_saturation_<uint8_t>(src, dst, scale, mask);
    break;
  case CV_8S:
    color_saturation_<int8_t>(src, dst, scale, mask);
    break;
  case CV_16U:
    color_saturation_<uint16_t>(src, dst, scale, mask);
    break;
  case CV_16S:
    color_saturation_<int16_t>(src, dst, scale, mask);
    break;
  case CV_32S:
    color_saturation_<int32_t>(src, dst, scale, mask);
    break;
  case CV_32F:
    color_saturation_<float>(src, dst, scale, mask);
    break;
  case CV_64F:
    color_saturation_<double>(src, dst, scale, mask);
    break;
  default:
    CF_ERROR("Unsupported pixel depth %d", src.depth());
    return false;
  }

  return true;
}

bool color_saturation_hls(cv::Mat & image, double scale,
    cv::InputArray mask)
{
  if( image.channels() != 3 ) { // Not a BGR
    return false;
  }

  if( scale == 1 ) {
    return true;
  }


  switch (image.depth()) {
  case CV_8U:
    color_saturation_<uint8_t>(image, scale, mask);
    break;
  case CV_8S:
    color_saturation_<int8_t>(image, scale, mask);
    break;
  case CV_16U:
    color_saturation_<uint16_t>(image, scale, mask);
    break;
  case CV_16S:
    color_saturation_<int16_t>(image, scale, mask);
    break;
  case CV_32S:
    color_saturation_<int32_t>(image, scale, mask);
    break;
  case CV_32F:
    color_saturation_<float>(image, scale, mask);
    break;
  case CV_64F:
    color_saturation_<double>(image, scale, mask);
    break;
  default:
    CF_ERROR("Unsupported pixel depth %d", image.depth());
    return false;
  }

  return true;
}
