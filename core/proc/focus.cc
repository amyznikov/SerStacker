/*
 * focus.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "focus.h"
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

  constexpr int ddepth = CV_32F;

  cv::Mat gx, gy;

  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
  cv::magnitude(gx, gy, g);
  //cv::add(gx.mul(gx), gy.mul(gy), dst);
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
//
//cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image, cv::OutputArray cmap,
//    double eps, cv::InputArray H)
//{
//
//  INSTRUMENT_REGION("");
//
//  cv::Mat s, ss, r;//, g;
//
//  if( eps <= 0 ) {
//    eps = 1e-6;
//  }
//
//  const cv::Mat src =
//      image.getMat();
//
//  if( src.depth() == CV_32F ) {
//    cv::add(src, eps, s);
//  }
//  else {
//    src.convertTo(s, CV_32F, 1. / (1 << get_bpp(src.depth())), eps);
//  }
//
//  if( true ) {
//    INSTRUMENT_REGION("blur");
//    downscale(s, ss, 2);
//    upscale(ss, s.size());
//  }
//  else {
//    INSTRUMENT_REGION("blur");
//    static const cv::Mat1f K = cv::getGaussianKernel(15, 5, CV_32F);
//    cv::sepFilter2D(s, ss, CV_32F, K, K);
//  }
//
//  //compute_gradient(ss, g);
//
//  const bool needcmap =
//      cmap.needed();
//
//  const int src_channels =
//      src.channels();
//
//  const int cn =
//      (std::min)(4, src_channels);
//
//  if( needcmap ) {
//    r.create(s.size(), CV_MAKETYPE(CV_32F, cn));
//  }
//
//  cv::Scalar rr =
//      cv::Scalar::all(0);
//
//  cv::Scalar ww =
//      cv::Scalar::all(0);
//
//  for( int y = 0; y < s.rows; ++y ) {
//
//    const float *sp = s.ptr<const float>(y);
//    const float *ssp = ss.ptr<const float>(y);
//    //const float *gp = g.ptr<const float>(y);
//    float *rp = needcmap ? r.ptr<float>(y) : nullptr;
//
//    for( int x = 0; x < s.cols; ++x ) {
//      for( int c = 0; c < cn; ++c ) {
//
//        const float w =
//            1; // square(gp[x * src_channels + c]);
//
////        const float v =
////            square(sp[x * src_channels + c] / ssp[x * src_channels + c] - 1);
//        const float v =
//            (sp[x * src_channels + c] / ssp[x * src_channels + c]);
//
////        const float w =
////            v ;
//
//        rr[c] += w * v;
//        ww[c] += w;
//
//        if( needcmap ) {
//          rp[x * src_channels + c] = w * v;
//        }
//      }
//    }
//  }
//
//  for( int c = 0; c < cn; ++c ) {
//    rr[c] /= ww[c];
//  }
//
//  if( needcmap ) {
//    cmap.move(r);
//    //cv::divide(r, ww, cmap);
//  }
//
//  return rr;
//}

//static bool isfptype(int depth)
//{
//  return (depth = CV_MAT_DEPTH(depth)) == CV_32F || depth == CV_64F;
//}


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

void c_local_contrast_measure::set_threshold(double v)
{
  threshold_ = v;
}

double c_local_contrast_measure::threshold() const
{
  return threshold_;
}



cv::Scalar c_local_contrast_measure::compute(cv::InputArray image)
{
  return compute_contrast_map(image, cv::noArray(),
      eps_, dscale_, threshold_);
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

#endif

