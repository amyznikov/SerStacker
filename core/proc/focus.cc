/*
 * focus.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "focus.h"
#include <core/proc/fft.h>
#include <core/proc/weighted_mean.h>
#include <core/proc/morphology.h>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>


static void compute_gradient(const cv::Mat & src, cv::Mat & g, double delta = 0)
{
  INSTRUMENT_REGION("");

  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

//  static thread_local const cv::Matx<float, 1, 3> K(
//      (-0.5f),
//      0.f,
//      (+0.5f));

  constexpr int ddepth = CV_32F;

  cv::Mat gx, gy;

  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
  cv::magnitude(gx, gy, g);
//  cv::add(gx.mul(gx), gy.mul(gy), g);
//  cv::sqrt(g, g);

  //cv::magnitude(gx, gy, g);
  //cv::add(gx.mul(gx), gy.mul(gy), dst);
}


// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
static void compute_laplacian(const cv::Mat & src, cv::Mat & l, double delta)
{
  static float k[5 * 5] = {
      0, 0, -1, 0, 0,
      0, -1, -2, -1, 0,
      -1, -2, 16, -2, -1,
      0, -1, -2, -1, 0,
      0, 0, -1, 0, 0,
  };

  static const cv::Mat1f K =
      cv::Mat1f(5, 5, k) / 16.;

  cv::filter2D(src, l, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  //cv::multiply(l, l, l);
  cv::absdiff(l, 0, l);
  if( delta > 0 ) {
    cv::add(l, delta, l);
  }
}


static void compute_fft(const cv::Mat & src, cv::Mat & dst)
{

  static const auto prepare_channel =
      [](cv::Mat & img) {

        cv::Mat1f image = img;

        for ( int y = 0; y < image.rows; ++y ) {
          for ( int x = 0; x < image.cols; ++x ) {
            if( (x + y) & 0x1 ) {
              image[y][x] = -image[y][x];
            }
          }
        }
      };

  std::vector<cv::Mat> channels;

  const int cn =
      src.channels();

  if ( cn == 1 ) {
    channels.emplace_back(src);
  }
  else {
    cv::split(src, channels);
  }

  //cv::Scalar ww = cv::Scalar::all(0);
  dst.create(src.size(), CV_MAKETYPE(CV_32F, cn));


  for ( int c = 0; c < cn; ++c ) {

    prepare_channel(channels[c]); // instead of fftSwapQuadrants(channels[c]);

    cv::dft(channels[c], channels[c],
        cv::DFT_COMPLEX_OUTPUT);

    cv::Mat2f s =
        channels[c];

    for( int y = 0; y < s.rows; ++y ) {

      float *dstp = dst.ptr<float>(y) + c;

      for( int x = 0; x < s.cols; ++x ) {

        const float re = s[y][x][0];
        const float im = s[y][x][1];
        const float w = sqrt(re * re + im * im);
        //ww[c] += w;
        dstp[x * cn] = w;
      }
    }
  }

  //cv::divide(dst, ww, dst);
}

static bool downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);
  for( int l = 1; l < level; ++l ) {
    cv::pyrDown(dst, dst, cv::Size(), border_mode);
  }
  return true;
}

static bool upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> spyramid;

    spyramid.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      spyramid.emplace_back(nextSize);
    }

    for( int i = spyramid.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, spyramid[i]);
    }
  }

  return true;
}


static int get_bpp(int ddepth)
{
  switch (ddepth) {
    case CV_8U:
    case CV_8S:
      return 8;

    case CV_16U:
    case CV_16S:
      return 16;

    case CV_32S:
      return 32;
  }

  return 0;
}

static double maxval(int ddepth)
{
  switch (CV_MAT_DEPTH(ddepth)) {
    case CV_8U:
      return UINT8_MAX;
    case CV_8S:
      return INT8_MAX;
    case CV_16U:
      return UINT16_MAX;
    case CV_16S:
      return INT16_MAX;
    case CV_32S:
      return INT32_MAX;
  }

  return 1;
}

static void equalizehist(cv::Mat & image)
{
  if( image.depth() == CV_8U ) {

    const int cn = image.channels();
    if( cn == 1 ) {
      cv::equalizeHist(image, image);
    }
    else {
      std::vector<cv::Mat> channels;
      cv::split(image, channels);

      for( int i = 0; i < cn; ++i ) {
        cv::equalizeHist(channels[i], channels[i]);
      }

      cv::merge(channels, image);
    }
  }
}


static inline float square(float x)
{
  return x * x;
}

//static bool isfptype(int depth)
//{
//  return (depth = CV_MAT_DEPTH(depth)) == CV_32F || depth == CV_64F;
//}


cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image,
    cv::OutputArray output_contrast_map, double eps, int dscale, bool avgchannel)
{

  INSTRUMENT_REGION("");

  const cv::Mat src =
      image.getMat();

  cv::Mat s1, s2, e, d, mg, gg, map;

  src.convertTo(s1, CV_32F, 1. / maxval(src.depth()));

  if( avgchannel && s1.channels() > 1 ) {
    reduce_color_channels(s1, s1, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s1, s1, dscale);
  }

  static const cv::Mat1b SE(3, 3, 255);

  cv::morphologyEx(s1, mg, cv::MORPH_GRADIENT, SE);

  downscale(s1, s2, 2);
  upscale(s2, s1.size());
  compute_gradient(s2, gg);

  cv::divide(mg, s2 + cv::Scalar::all(eps), map);

  cv::Scalar rv = weighted_mean(map, gg);

  if( output_contrast_map.needed() ) {
    cv::multiply(map, gg, output_contrast_map);
    cv::divide(output_contrast_map, cv::sum(gg), output_contrast_map);
  }

  return rv;
}


void c_local_contrast_measure::set_dscale(int v)
{
  dscale_ = v;
}

int c_local_contrast_measure::dscale() const
{
  return dscale_;
}

void c_local_contrast_measure::set_eps(double v)
{
  eps_ = v;
}

double c_local_contrast_measure::eps() const
{
  return eps_;
}

void c_local_contrast_measure::set_avgchannel(bool v)
{
  avgchannel_ = v;
}

bool c_local_contrast_measure::avgchannel() const
{
  return avgchannel_;
}

cv::Scalar c_local_contrast_measure::compute(cv::InputArray image)
{
  return compute_contrast_map(image, cv::noArray(),
      eps_, dscale_, avgchannel_);
}



#if 0



cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image,
    cv::OutputArray output_map, double eps, int dscale, bool equalize_hist)
{

  INSTRUMENT_REGION("");

  const cv::Mat1b SE(3, 3, 255);

  const cv::Mat src =
      image.getMat();

  cv::Mat s, g, w;

  if( eps <= 0 ) {
    eps = 1e-6;
  }

  if ( dscale < 1 ) {
    s = src;
  }
  else {
    downscale(src, s, dscale);
  }

  if( equalize_hist ) {
    equalizehist(s);
  }

  compute_gradient(s, g);
  cv::GaussianBlur(g, w, cv::Size(), 1, 1);


//  cv::Scalar rv =
//      cv::mean(g);

  cv::Scalar rv =
      weighted_mean(g, w);

//  cv::Scalar rv =
//      cv::mean(g);

  if ( output_map.needed() ) {
    //output_map.move(g);
    g.copyTo(output_map);
  }

  return rv;
}

cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image,
    cv::OutputArray output_map, double eps, int dscale, double threshold)
{

  INSTRUMENT_REGION("");

  const cv::Mat1b SE(3, 3, 255);

  const cv::Mat src =
      image.getMat();

  cv::Mat s, e, d, r, t;

  if( eps <= 0 ) {
    eps = 1e-6;
  }

  src.convertTo(s, CV_8U, maxval(CV_8U) / maxval(src.depth()));
  if( dscale > 0 ) {
    downscale(s, s, dscale);
  }

  //equalizehist(s);
  //cv::GaussianBlur(s, s, cv::Size(), 1, 1);

  cv::erode(s, e, SE);
  cv::dilate(s, d, SE);
  cv::max(d, 1, d);

  cv::divide(e,  d, r, 1, CV_32F);
  cv::subtract(cv::Scalar::all(1), r, r);



  if( output_map.needed() ) {
    r.copyTo(output_map);
  }

  //cv::compare(r, threshold, t, cv::CMP_GE);

  // return cv::mean(r);
  return weighted_mean(r, r);
}




cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image,
    cv::OutputArray output_contrast_map, double eps, int dscale, double threshold)
{

  INSTRUMENT_REGION("");

  const cv::Mat src =
      image.getMat();

  cv::Mat s, ss, r, map;

  src.convertTo(s, CV_32F, 1. / maxval(src.depth()));

  if( dscale > 0 ) {
    downscale(s, s, dscale);
  }

  // faster replacement for cv::GaussinBlur() for large kernels
  downscale(s, ss, 2);
  upscale(ss, s.size());

#if 0
  //cv::max(ss, cv::Scalar::all(eps), ss);
  cv::divide(s, ss, r, 1, CV_32F);
  cv::absdiff(r, 1, r);
  if( output_contrast_map.needed() ) {
    r.copyTo(output_contrast_map);
  }

  cv::Scalar rv =
      cv::mean(r);
#else

  const cv::Size size =
      s.size();

  const int src_channels =
      s.channels();

  const int cn =
      (std::min)(4, src_channels);

  if( output_contrast_map.needed() ) {
    output_contrast_map.create(size, CV_MAKE_TYPE(CV_32F, cn));
    map = output_contrast_map.getMat();
  }


  // Compute weighted average

  cv::Scalar rv, ww;

  for ( int y = 0; y < size.height; ++y ) {

    const float * sp =
        s.ptr<const float>(y);

    const float * ssp =
        ss.ptr<const float>(y);

    float *mp =
        map.empty() ? nullptr :
            map.ptr<float>(y);

    for ( int x = 0; x < size.width; ++x ) {
      for ( int c = 0; c < cn; ++c ) {

        const float sv = sp[x * src_channels + c];
        const float ssv = ssp[x * src_channels + c];
        const float r = fabs(1.f - sv / (std::max)(ssv, 1e-5f));
        const float w = r;

        rv[c] += r * w;
        ww[c] += w;

        if( mp ) {
          mp[x * cn + c] = r * w;
        }
      }
    }
  }

  for( int c = 0; c < cn; ++c ) {
    rv[c] /= ww[c];
  }

#endif

  return rv;
}


#endif

