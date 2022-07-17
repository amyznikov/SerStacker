/*
 * autoclip.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "autoclip.h"
#include <tbb/tbb.h>
#include <core/debug.h>


template<class T>
static inline T clip(double v, double omin, double omax)
{
  if ( v < omin ) {
    v = omin;
  }
  else if ( v > omax ) {
    v = omax;
  }
  return cv::saturate_cast<T>(v);
}

template<class T>
static bool clip_range_(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat1b & mask)
{
  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  const double S = (omax - omin) / (imax - imin);
  const int cn = image.channels();

  if ( mask.empty() ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, S, imin, omin, omax, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = image.ptr<T>(y);
            for ( int x = 0, n = image.cols * cn; x < n; ++x ) {
              dstp[x] = clip<T>(omin + (dstp[x] - imin) * S, omin, omax);
            }
          }
        });
  }
  else if ( mask.channels() == 1 ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, &mask, S, imin, omin, omax, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = image.ptr<T>(y);
            const uint8_t * mskp = mask[y];
            for ( int x = 0; x < image.cols; ++x ) {
              if( mskp[x] ) {
                for ( int c = 0; c < cn; ++c ) {
                  dstp[x * cn + c] = clip<T>(omin + (dstp[x * cn + c] - imin) * S, omin, omax);
                }
              }
            }
          }
        });
  }
  else if ( mask.channels() == cn ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, &mask, S, imin, omin, omax, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            const uint8_t * mskp = mask[y];
            T * dstp = image.ptr<T>(y);
            for ( int x = 0; x < image.cols; ++x ) {
              for ( int c = 0; c < cn; ++c ) {
                if( mskp[x * cn + c] ) {
                  dstp[x * cn + c] = clip<T>(omin + (dstp[x * cn + c] - imin) * S, omin, omax);
                }
              }
            }
          }
        });
  }
  else {
    CF_FATAL("Invalid combination of input image and mask channels");
    return false;
  }

  return true;

}
template<class T>
static bool clip_range_(cv::Mat & image,
    double min, double max,
    const cv::Mat1b & mask)
{
  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  const int cn = image.channels();

  if ( mask.empty() ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, min, max, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = image.ptr<T>(y);
            for ( int x = 0, n = image.cols * cn; x < n; ++x ) {
              if ( dstp[x] < min ) {
                dstp[x] = min;
              }
              else if ( dstp[x] > max ) {
                dstp[x] = max;
              }
            }
          }
        });
  }
  else if ( mask.channels() == 1 ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, &mask, min, max, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = image.ptr<T>(y);
            const uint8_t * mskp = mask[y];
            for ( int x = 0; x < image.cols; ++x ) {
              if( mskp[x] ) {
                for ( int c = 0; c < cn; ++c ) {
                  if ( dstp[x * cn + c] < min ) {
                    dstp[x * cn + c] = min;
                  }
                  else if ( dstp[x * cn + c] > max ) {
                    dstp[x * cn + c] = max;
                  }
                }
              }
            }
          }
        });
  }
  else if ( mask.channels() == cn ) {

    tbb::parallel_for(range(0, image.rows, grain_size),
        [&image, &mask, min, max, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            const uint8_t * mskp = mask[y];
            T * dstp = image.ptr<T>(y);
            for ( int x = 0; x < image.cols; ++x ) {
              for ( int c = 0; c < cn; ++c ) {
                if( mskp[x * cn + c] ) {
                  if ( dstp[x * cn + c] < min ) {
                    dstp[x * cn + c] = min;
                  }
                  else if ( dstp[x * cn + c] > max ) {
                    dstp[x * cn + c] = max;
                  }
                }
              }
            }
          }
        });
  }
  else {
    CF_FATAL("Invalid combination of input image and mask channels");
    return false;
  }

  return true;

}

bool clip_range(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat1b & mask)
{
  switch ( image.depth() ) {
  case CV_8U :
    return clip_range_ < uint8_t > (image, imin, imax, omin, omax, mask);
  case CV_8S :
    return clip_range_ < int8_t > (image, imin, imax, omin, omax, mask);
  case CV_16U :
    return clip_range_ < uint16_t > (image, imin, imax, omin, omax, mask);
  case CV_16S :
    return clip_range_ < int8_t > (image, imin, imax, omin, omax, mask);
  case CV_32S :
    return clip_range_ < int32_t > (image, imin, imax, omin, omax, mask);
  case CV_32F :
    return clip_range_<float>(image, imin, imax, omin, omax, mask);
  case CV_64F :
    return clip_range_<double>(image, imin, imax, omin, omax, mask);
  }

  return false;
}

bool clip_range(cv::Mat & image, double min, double max,
    const cv::Mat1b & mask)
{
  switch ( image.depth() ) {
  case CV_8U :
    return clip_range_ < uint8_t > (image, min, max, mask);
  case CV_8S :
    return clip_range_ < int8_t > (image, min, max, mask);
  case CV_16U :
    return clip_range_ < uint16_t > (image, min, max, mask);
  case CV_16S :
    return clip_range_ < int8_t > (image, min, max, mask);
  case CV_32S :
    return clip_range_ < int32_t > (image, min, max, mask);
  case CV_32F :
    return clip_range_<float>(image, min, max, mask);
  case CV_64F :
    return clip_range_<double>(image, min, max, mask);
  }

  return false;
}

bool clip_range(cv::InputArray src,
    cv::OutputArray dst,
    double imin,
    double imax,
    double omin,
    double omax,
    cv::InputArray mask)
{
  if ( !dst.fixedType() ) {
    if ( src.getMat().data != dst.getMat().data ) {
      src.copyTo(dst);
    }
  }
  else {
    src.getMat().convertTo(dst, dst.depth());
  }

  const cv::Mat1b msk = mask.getMat();

  return clip_range(dst.getMatRef(),
      imin, imax,
      omin, omax,
      msk);
}


/// @brief  convert conventional image histogram H into cumulative
///         by accumulating the bin values along rows
static bool accumulate_histogram(cv::InputArray H, cv::OutputArray cumulative)
{
  cv::Mat1f src;
  cv::Mat1f dst;

  double sums[H.cols()];

  if ( H.channels() != 1 ) {
    CF_FATAL("Invalid arg: Input histogram must be single-channel");
    return false;
  }

  if ( cumulative.fixedType() && cumulative.channels() != 1 ) {
    CF_FATAL("Invalid arg: output cumulative histogram must be single-channel");
    return false;
  }

  if ( H.depth() == src.depth() ) {
    src = H.getMat();
  }
  else {
    H.getMat().convertTo(src, src.depth());
  }


  memset(sums, 0, sizeof(sums));

  dst.create(src.size());

  for ( int y = 0; y < src.rows; ++y ) {
    for ( int x = 0; x < src.cols; ++x ) {
      dst[y][x] = (sums[x] += src[y][x]);
    }
  }

  if ( !cumulative.fixedType() || cumulative.depth() == dst.depth() ) {
    cumulative.move(dst);
  }
  else {
    dst.convertTo(cumulative, cumulative.depth());
  }

  return true;
}


static bool build_histogram(cv::InputArray input_image,
    cv::InputArray mask,
    cv::Mat1f & output_histogram,
    int bins,
    int channel,
    double * input_output_range_min,
    double * input_output_range_max,
    bool uniform,
    bool cumulative,
    bool scaled)
{
  if ( input_image.empty() ) {
    return false;
  }

  const cv::Mat image =
      input_image.getMat();

  const int cn =
      image.channels();

  if ( channel >= cn ) {
    return false;
  }


  if ( bins > 65536 ) {
    bins = 65536;
  }
  else if ( bins < 1 ) {
    switch ( input_image.depth() ) {
    case CV_8U :
      case CV_8S :
      bins = 256;
      break;
    default :
      bins = 65536;
      break;
    }
  }


  double range_min = input_output_range_min ? * input_output_range_min : -1;
  double range_max = input_output_range_max ? * input_output_range_max : -1;

  if ( range_max <= range_min ) {

    switch ( image.depth() ) {
    case CV_8U :
      range_min = 0;
      range_max = UINT8_MAX + 1.;
      break;
    case CV_8S :
      range_min = INT8_MIN;
      range_max = INT8_MAX + 1.;
      break;
    case CV_16U :
      range_min = 0;
      range_max = UINT16_MAX + 1.;
      break;
    case CV_16S :
      range_min = INT16_MIN;
      range_max = INT16_MAX + 1.;
      break;
    case CV_32S :
      range_min = INT32_MIN;
      range_max = INT32_MAX + 1.;
      break;
    case CV_32F :
      case CV_64F : {

      double min, max;
      cv::minMaxLoc(image, &min, &max);

      if ( min >= 0 ) {
        min = 0;
      }
      if ( max <= 1 ) {
        max = 1. + FLT_EPSILON;
      }

      range_min = min;
      range_max = max;

      break;
    }
    default :
      return false;
    }

    if ( input_output_range_min ) {
      * input_output_range_min = range_min;
    }

    if ( input_output_range_max ) {
      *input_output_range_max = range_max;
    }
  }

  if ( range_min >= range_max ) {
    CF_ERROR("Can not estimate input data range: range_min=%g range_max=%g", range_min, range_max);
    return false;
  }


  const float range[] = { (float)range_min, (float)range_max };
  const float * ranges[] = { range };
  const int sizes[] = { bins };

  if ( channel >= 0 ) {

    output_histogram.create(bins, 1);

    try {

      cv::calcHist(&image, 1,
          &channel,
          mask,
          output_histogram,
          1,
          sizes,
          ranges,
          uniform,
          false);
    }
    catch (const std::exception & e) {
      CF_ERROR("cv::calcHist( fails: %s", e.what());
      return false;
    }

  }
  else {

    cv::Mat tmp;

    output_histogram.create(bins, cn);

    for ( int i = 0; i < cn; ++i ) {

      try {

      cv::calcHist(&image, 1,
          &i,
          mask,
          tmp,
          1,
          sizes,
          ranges,
          uniform,
          false);

      }
      catch (const std::exception & e) {
        CF_ERROR("cv::calcHist( fails: %s", e.what());
        return false;
      }

      tmp.copyTo(output_histogram.colRange(i, i + 1));

    }
  }

  if ( scaled ) {
    for ( int i = 0; i < output_histogram.cols; ++i ) {
      double s = cv::sum(output_histogram.col(i))[0];
      cv::multiply(output_histogram.col(i), 1. / s, output_histogram.col(i));
    }
  }

  if ( cumulative ) {
    accumulate_histogram(output_histogram,
        output_histogram);
  }


  if ( 0 ) {
    FILE * fp = fopen("hdbg.txt", "w");
    if ( fp ) {

      fprintf(fp, "bin");
      for ( int c = 0; c < output_histogram.cols; ++c ) {
        fprintf(fp, "\tc%d", c);
      }
      fprintf(fp, "\n");

      for ( int r = 0; r < output_histogram.rows; ++r ) {
        fprintf(fp, "%8d", r);
        for ( int c = 0; c < output_histogram.cols; ++c ) {
          fprintf(fp, "\t%g", output_histogram[r][c]);
        }
        fprintf(fp, "\n");
      }

      fclose(fp);
    }

  }


  return true;
}


static void extract_max_channel(cv::InputArray _src, cv::OutputArray _dst)
{
  const cv::Mat src = _src.getMat();
  cv::Mat dst;

  const int src_rows = src.rows;
  cv::reduce(src.reshape(1, src.total()), dst, 1, cv::REDUCE_MAX);
  dst = dst.reshape(0, src_rows);
  _dst.move(dst);
}


bool compute_clip_levels(const cv::Mat1f & cumulative_normalized_histogram,
    double minv, double maxv,
    double plo, double phi,
    /*out */ double * minval,
    /*out */ double * maxval)
{
  const cv::Mat1f & H =
      cumulative_normalized_histogram;

  double range, hmin, imin, imax;
  int lowidx, highidx;

  plo /= 100;
  phi /= 100;

  const int nbins = H.rows;

  if ( (range = (maxv - minv)) <= 0 ) {
    CF_CRITICAL("build_histogram() returns empty [min..max] range");
    return false;
  }

  hmin = H[lowidx = 0][0];
  while ( lowidx < nbins && H[lowidx][0] == hmin ) {
    ++lowidx;
  }
  while ( lowidx < nbins && H[lowidx][0] < plo ) {
    ++lowidx;
  }
  imin = minv + lowidx * range / nbins;


  highidx = lowidx + 1;
  while ( highidx < nbins && H[highidx][0] <= phi ) {
    ++highidx;
  }
  imax = minv + highidx * range / nbins;

  if ( minval ) {
    *minval = imin;
  }

  if ( maxval ) {
    *maxval = imax;
  }

  return true;
}

bool compute_clip_levels(cv::InputArray image, cv::InputArray mask, double plo, double phi,
    /*out */ double * minval, /*out */ double * maxval)
{
  cv::Mat src, msk;
  cv::Mat1f H;

  //int nbins = 65536;
  double minv = 0, maxv = 0;
  double range, hmin, imin, imax;
  int lowidx, highidx;

  const int cn = image.channels();

  if ( image.channels() == 1 ) {
    src = image.getMat();
  }
  else {
    cv::cvtColor(image, src, cv::COLOR_BGR2GRAY);
  }

  if ( !mask.empty() ) {
    if ( mask.channels() == 1 ) {
        msk = mask.getMat();
    }
    else {
      extract_max_channel(mask, msk);
    }
  }

  if ( !build_histogram(src, msk, H, -1, 0, &minv, &maxv, true, true, true) ) {
    CF_CRITICAL("build_histogram() fails");
    return false;
  }

  return compute_clip_levels(H,
      minv, maxv,
      plo, phi,
      minval,
      maxval);
}

bool autoclip(cv::Mat & image,
    cv::InputArray mask,
    double plo, double phi,
    double omin, double omax,
    double * minval,
    double * maxval)
{
  double imin, imax;

  if ( !compute_clip_levels(image, mask, plo, phi, &imin, &imax) ) {
    CF_FATAL("compute_clip_levels() fails");
    return false;
  }

  if ( minval ) {
    *minval = imin;
  }

  if ( maxval ) {
    *maxval = imax;
  }

  clip_range(image,
      imin, imax,
      omin, omax/*,
      mask.getMat()*/);

  return true;

}

bool autoclip(cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    double plo, double phi,
    double omin, double omax,
    double * minval,
    double * maxval)
{
  double imin, imax;

  //CF_DEBUG("hclips: plo=%g phi=%g image: %dx%d channels=%d depth=%d", plo, phi, image.cols(), image.rows(), image.channels(), image.depth());

  if ( !compute_clip_levels(image, mask, plo, phi, &imin, &imax) ) {
    CF_FATAL("compute_histogram_clips() fails");
    return false;
  }


  //CF_DEBUG("hclips: imin=%g imax=%g", imin, imax);

  if ( minval ) {
    *minval = imin;
  }

  if ( maxval ) {
    *maxval = imax;
  }

  clip_range(image, dst,
      imin, imax,
      omin, omax,
      mask);

  return true;
}


bool histogram_white_balance(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst, double cl, double ch)
{
  if ( src.channels() < 2 ) {
    src.copyTo(dst);
  }
  else {
    const int cn = src.channels();

    cv::Mat channels[cn];
    cv::Mat cmasks[cn];

    double imin[cn], imax[cn];
    int rc = 0;

    cv::split(src.getMat(), channels);

    if ( mask.channels() > 1 ) {
      cv::split(mask.getMat(), cmasks);
    }
    else if ( !mask.empty() && cv::countNonZero(mask) < mask.size().area() ) {
      for ( int i = 0; i < cn; ++i ) {
        cmasks[i] = mask.getMat();
      }
    }

    for ( int i = 0; i < cn; ++i ) {

      imin[i] = -1, imax[i] = -1;

      if ( !compute_clip_levels(channels[i], cmasks[i], cl, ch, &imin[i], &imax[i]) ) {
        CF_ERROR("compute_clip_levels(channel=%d) fails", i);
        return false;
      }

      if ( imax[i] - imin[i] > imax[rc] - imin[rc] ) {
        rc = i;
      }
    }

    for ( int i = 0; i < cn; ++i ) {

      const double alpha = (imax[rc] - imin[rc]) / (imax[i] - imin[i]);
      const double beta = imin[rc] - alpha * imin[i];

      channels[i].convertTo(channels[i],
          channels[i].depth(),
          alpha, beta);
    }

    cv::merge(channels, cn, dst);
  }

  return true;

}
