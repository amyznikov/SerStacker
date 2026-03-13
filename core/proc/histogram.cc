/*
 * histogram.cc
 *
 *  Created on: Mar 8, 2026
 *      Author: amyznikov
 */


#include "histogram.h"
#include <tbb/tbb.h>
#include <cmath>
#include <type_traits>
#include <core/debug.h>

namespace {

struct MinMaxReducer
{
  const cv::Mat & src;
  const cv::Mat & mask;
  const int cn;
  double minVal, maxVal;
  bool found;

  MinMaxReducer(const cv::Mat & s, const cv::Mat & m, int c) :
    src(s), mask(m), cn(c), minVal(DBL_MAX), maxVal(-DBL_MAX), found(false)
  {
  }

  MinMaxReducer(MinMaxReducer & x, tbb::split) :
    src(x.src), mask(x.mask), cn(x.cn), minVal(DBL_MAX), maxVal(-DBL_MAX), found(false)
  {
  }

  template<typename T>
  void process(const tbb::blocked_range2d<int> & r)
  {
    const int _cn = cn;
    const auto & rows = r.rows();
    const auto & cols = r.cols();

    double l_min = minVal;
    double l_max = maxVal;
    bool l_found = found;

    if( mask.empty() ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * pxp = srcp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( std::isfinite(pxp[c]) ) {
              const double v = pxp[c];
              if( v < l_min ) {
                l_min = v;
              }
              if( v > l_max ) {
                l_max = v;
              }
              l_found = true;
            }
          }
        }
      }
    }
    else if( mask.channels() == 1 ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          if( mskp[x] ) {
            const T * pxp = srcp + x * _cn;
            for( int c = 0; c < _cn; ++c ) {
              if( std::isfinite(pxp[c]) ) {
                const double v = pxp[c];
                if( v < l_min ) {
                  l_min = v;
                }
                if( v > l_max ) {
                  l_max = v;
                }
                l_found = true;
              }
            }
          }
        }
      }
    }
    else { // mask.channels() == _cn
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * pxp = srcp + x * _cn;
          const uint8_t * mpxp = mskp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( mpxp[c] && std::isfinite(pxp[c])  ) {
              const double v = pxp[c];
              if( v < l_min ) {
                l_min = v;
              }
              if( v > l_max ) {
                l_max = v;
              }
              l_found = true;
            }
          }
        }
      }
    }

    minVal = l_min;
    maxVal = l_max;
    found = l_found;
  }

  void operator()(const tbb::blocked_range2d<int> & r)
  {
    switch (src.depth()) {
      case CV_8U: process<uint8_t>(r); break;
      case CV_8S: process<int8_t>(r); break;
      case CV_16U:process<uint16_t>(r); break;
      case CV_16S:process<int16_t>(r); break;
      case CV_32S:process<int32_t>(r); break;
      case CV_32F:process<float>(r); break;
      case CV_64F:process<double>(r); break;
    }
  }

  void join(const MinMaxReducer & y)
  {
    if( y.found ) {
      if( y.minVal < minVal ) {
        minVal = y.minVal;
      }
      if( y.maxVal > maxVal ) {
        maxVal = y.maxVal;
      }
      found = true;
    }
  }
};


struct HistReducer
{
  const cv::Mat & src;
  const cv::Mat & mask;
  double minv, maxv, scale;
  uint32_t nbins, cn;

  cv::Mat1d H; // thread local histogram

  HistReducer(const cv::Mat & s, const cv::Mat & m, double mi, double ma, double sc, uint32_t nb, int c) :
    src(s), mask(m), minv(mi), maxv(ma), scale(sc), nbins(nb), cn(c),
    H(cv::Mat1d::zeros(nbins, cn))
  {
  }

  // Copy constructor for TBB (splitting constructor)
  HistReducer(HistReducer & x, tbb::split) :
    src(x.src), mask(x.mask), minv(x.minv), maxv(x.maxv), scale(x.scale), nbins(x.nbins), cn(x.cn),
    H(cv::Mat1d::zeros(nbins, cn))
  {
  }

  // Merge results from different streams
  void join(const HistReducer & y)
  {
    H += y.H;
  }

  template<typename T>
  void process(const tbb::blocked_range<int> & r)
  {
    const double _minv = minv;
    const double _scale = scale;
    const uint32_t _nbins = nbins;
    const int _cn = cn;
    const int _cols = src.cols;

    double * const hp = H.ptr<double>();

    if( mask.empty() ) {
      for( int y = r.begin(); y < r.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        for( int x = 0; x < _cols; ++x ) {
          const T * pxp = srcp + x * _cn; // Pointer to the start of the pixel
          for( int c = 0; c < _cn; ++c ) {
            const uint32_t bin = (uint32_t)((int) ((pxp[c] - _minv) * _scale));
            if( bin < _nbins ) {
              hp[bin * _cn + c] += 1.0;
            }
          }
        }
      }
    }
    else if( mask.channels() == 1 ) {
      for( int y = r.begin(); y < r.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = 0; x < _cols; ++x ) {
          if( mskp[x] ) {
            const T * pxp = srcp + x * _cn; // Pointer to the start of the pixel
            for( int c = 0; c < _cn; ++c ) {
              const uint32_t bin = (uint32_t)((int) ((pxp[c] - _minv) * _scale));
              if( bin < _nbins ) {
                hp[bin * _cn + c] += 1.0;
              }
            }
          }
        }
      }
    }
    else { // mask.channels() == _cn
      for( int y = r.begin(); y < r.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = 0; x < _cols; ++x ) {
          const T * spxp = srcp + x * _cn; // Pointer to the start of the pixel
          const uint8_t * mpxp = mskp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( mpxp[c] ) {
              const uint32_t bin = (uint32_t )((int) ((spxp[c] - _minv) * _scale));
              if( bin >= 0 && bin < _nbins ) {
                hp[bin * _cn + c] += 1.0;
              }
            }
          }
        }
      }
    }
  }

  template<typename T>
  void process(const tbb::blocked_range2d<int> & r)
  {
    const double _minv = minv;
    const double _scale = scale;
    const uint32_t _nbins = nbins;
    const int _cn = cn;
    double * const hp = H.ptr<double>();

    const auto & rows = r.rows();
    const auto & cols = r.cols();

    if( mask.empty() ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * pxp = srcp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            const uint32_t bin = (uint32_t)((int) ((pxp[c] - _minv) * _scale));
            if( bin < _nbins ) {
              hp[bin * _cn + c] += 1.0;
            }
          }
        }
      }
    }
    else if( mask.channels() == 1 ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          if( mskp[x] ) {
            const T * pxp = srcp + x * _cn;
            for( int c = 0; c < _cn; ++c ) {
              const uint32_t bin = (uint32_t )((int) ((pxp[c] - _minv) * _scale));
              if(  bin < _nbins ) {
                hp[bin * _cn + c] += 1.0;
              }
            }
          }
        }
      }
    }
    else { // mask.channels() == _cn
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * spxp = srcp + x * _cn;
          const uint8_t * mpxp = mskp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( mpxp[c] ) {
              const uint32_t bin = (uint32_t )((int) ((spxp[c] - _minv) * _scale));
              if( bin < _nbins ) {
                hp[bin * _cn + c] += 1.0;
              }
            }
          }
        }
      }
    }
  }

  // Main loop over a range of image rows
  void operator()(const tbb::blocked_range2d<int> & r)
  {
    switch (src.depth()) {
      case CV_8U: process<uint8_t>(r); break;
      case CV_8S: process<int8_t>(r); break;
      case CV_16U:process<uint16_t>(r); break;
      case CV_16S:process<int16_t>(r); break;
      case CV_32S:process<int32_t>(r); break;
      case CV_32F:process<float>(r); break;
      case CV_64F:process<double>(r); break;
    }
  }

  // Main loop over a range of image rows
  void operator()(const tbb::blocked_range<int> & r)
  {
    switch (src.depth()) {
      case CV_8U: process<uint8_t>(r); break;
      case CV_8S: process<int8_t>(r); break;
      case CV_16U:process<uint16_t>(r); break;
      case CV_16S:process<int16_t>(r); break;
      case CV_32S:process<int32_t>(r); break;
      case CV_32F:process<float>(r); break;
      case CV_64F:process<double>(r); break;
    }
  }

};

static void findGlobalMinMax(const std::vector<cv::Mat> & srcv, const std::vector<cv::Mat> & maskv,
    double * minv, double * maxv)
{
  double gMin = DBL_MAX;
  double gMax = -DBL_MAX;
  bool anyFound = false;

  for( size_t i = 0; i < srcv.size(); ++i ) {
    if( !srcv[i].empty() ) {

      MinMaxReducer reducer(srcv[i], maskv[i], srcv[i].channels());
      tbb::parallel_reduce(tbb::blocked_range2d<int>(0, srcv[i].rows, 0, srcv[i].cols), reducer,
          tbb::static_partitioner());

      if( reducer.found ) {
        gMin = std::min(gMin, reducer.minVal);
        gMax = std::max(gMax, reducer.maxVal);
        anyFound = true;
      }
    }
  }

  if( !anyFound ) {
    *minv = 0;
    *maxv = 255; // Default if there is no data or everything is masked
  }
  else {
    *minv = gMin;
    *maxv = gMax;
  }
}


static bool suggestHistogramOptions(int depth, const std::vector<cv::Mat> & srcv, const std::vector<cv::Mat> & maskv,
    /*in, out*/ double * minv, /*in, out*/ double * maxv, /*in, out*/ uint32_t * nbins)
{
  const bool useFullTypeRange = (*minv < -0.5 && *maxv < -0.5);
  const bool autoRange = useFullTypeRange || (*maxv <= *minv);

  // nbins is considered "automatic" if it is 0 or inappropriately large
  const bool autoBins = (*nbins == 0 || *nbins > 65535U);

  // Determining the range (minv / maxv)
  if ( useFullTypeRange ) {
    switch (depth) {
      case CV_8U:  *minv = 0; *maxv = UINT8_MAX; break;
      case CV_8S:  *minv = INT8_MIN; *maxv = INT8_MAX; break;
      case CV_16U: *minv = 0; *maxv = UINT16_MAX; break;
      case CV_16S: *minv = INT16_MIN; *maxv = INT16_MAX; break;
      case CV_32S: *minv = (double)INT_MIN; *maxv = (double)INT_MAX; break;
      case CV_32F:
      case CV_64F:
        // There is no "full range" for float, we make calculations based on the data
        findGlobalMinMax(srcv, maskv, minv, maxv);
        break;
    }
  }
  else if ( autoRange ) {
    findGlobalMinMax(srcv, maskv, minv, maxv);
  }

  // Determining the number of bins (nbins)
  if ( autoBins ) {
    const double range = *maxv - *minv;

    if( depth < CV_16U ) {
      /*
       * For 8-bit auto mode:
       * If the range is narrow (e.g., 50-150), make exactly 101 bins (scale=1.0).
       * If the range is wide (0-255), limit to 256.
       */
      *nbins = range < 255.5  ? (uint32_t) (std::round(range) + 1.0) : 256U;
    }
    else if( depth < CV_32S ) { // CV_16U, CV_16S
      // 16 bit mode
      *nbins = range < 65536.5  ? (uint32_t) (std::round(range) + 1.0) : 65536U;
    }
    else {
      /* For the rest (int32, Float) use some reasonable limit */
      *nbins = 10000U;
    }
  }

  // If nbins was passed explicitly (e.g. 1024), we don't touch it.

  // Final Sanity Check
  if (*nbins < 2)  {
    *nbins = 2U;
  }
  else if (*nbins > 16384U) { // Hard limit for Out of Memory protection
    *nbins = 16384U;
  }

  return true;
}

}

/**
 * @param src - one or more input arrays of CV kind STD_ARRAY_MAT, STD_VECTOR_VECTOR, STD_VECTOR_MAT
 * @param masks array of a single- or multi- channel input masks of the same size as src. some of mask may be empty
 * @param minv - range minimum
 * @param maxv - range maximum
 * @param number of bins in output histogram
 * @param H - output cv::Mat1f or cv::Mat1d matrix of the size num bins x num channels,
 *           the number of columns is set as max number of channels found in src
 * @param cumulative
 * @param scaled
 * @param ddepth Depth of output histogram matrix CV_32F or CV_64F
 *
 * Accepted src items may be single or multi channel data of different data type independently one from other,
 *         but all of them must have equal the number color channels
 *
 */
bool createHistogram(cv::InputArrayOfArrays src, cv::InputArrayOfArrays masks,
    double * minv,
    double * maxv,
    uint32_t nbins,
    cv::OutputArray H,
    bool cumulative,
    bool scaled,
    int ddepth)
{
  std::vector<cv::Mat> srcv, maskv;

  if (src.kind() == cv::_InputArray::MAT) {
    // If only a single matrix is ​​passed, put it entirely into the vector
    srcv.push_back(src.getMat());
  }
  else {
    // If a matrix vector is passed, we extract it normally
    src.getMatVector(srcv);
  }

  if( masks.kind() == cv::_InputArray::MAT ) {
    // Even if the mask is empty, it will be ONE element of the vector
    maskv.push_back(masks.getMat());
  }
  else if( !masks.empty() ) {
    // If a matrix vector is passed then extract it normally
    masks.getMatVector(maskv);
  }

  if (maskv.empty()) {
    // if there are no masks at all or there are fewer of them than data
    maskv.assign(srcv.size(), cv::Mat());
  }
  else if (maskv.size() == 1 && srcv.size() > 1) {
    // one mask for all images in the list
    maskv.assign(srcv.size(), maskv[0]);
  }

  if ( srcv.size() != maskv.size() ) {
    CF_ERROR("src.size()=%zu must be same as masks.size()=%zu", srcv.size(), maskv.size());
    return false;
  }

  /* Sanity checking - ensure all src and mask items have acceptable types and sizes */
  const int cn = srcv[0].channels();
  int max_depth = -1;
  for( size_t i = 0; i < srcv.size(); ++i ) {

    // Check the number of data channels
    if( srcv[i].channels() != cn ) {
      CF_ERROR("All src items must have equal number of color channels");
      return false;
    }

    // check the data depth
    if ( srcv[i].depth() > max_depth ) {
      max_depth = srcv[i].depth();
    }

    // Check the mask
    if( !maskv[i].empty() ) {
      const int mt = maskv[i].type();
      if( mt != CV_8UC1 && mt != CV_MAKETYPE(CV_8U, cn) ) {
        CF_ERROR("Invalid input mask type (%d) for mask index %zu", mt, i);
        return false;
      }
      if( maskv[i].size() != srcv[i].size() ) {
        CF_ERROR("Invalid input mask size (%dx%d) for mask index %zu",maskv[i].cols, maskv[i].rows, i);
        return false;
      }
    }
  }

  if ( !suggestHistogramOptions(max_depth, srcv, maskv, minv, maxv, &nbins) ) {
    CF_ERROR("suggest_histogram_options() fails");
    return false;
  }

  /**
   * Build the histogram here
   */

  double range = *maxv - *minv;
  if( range < 1e-12 ) { // Protection against division by 0
    range = 1.0;
  }

  const double scale =
      ((max_depth <= CV_32S) && std::abs((double) nbins - (range + 1.0)) < 0.001) ? 1.0 :
          nbins / (range + range * 1e-7);

  cv::Mat1d Htotal = cv::Mat1d::zeros(nbins, cn);

  for ( size_t i = 0, n = srcv.size(); i < n; ++i ) {
    const cv::Mat & s = srcv[i];
    const cv::Mat & m = maskv[i];
    if ( !s.empty() ) {
      HistReducer reducer(s, m, *minv, *maxv, scale, nbins, cn);
#if 0
      tbb::parallel_reduce(tbb::blocked_range<int>(0, srcv[i].rows), reducer,
          tbb::static_partitioner());
#else
      tbb::parallel_reduce(tbb::blocked_range2d<int>(0, srcv[i].rows, 0, srcv[i].cols),  reducer,
          tbb::static_partitioner());
#endif
      Htotal += reducer.H;
    }
  }

  if( cumulative ) {
    for( int c = 0; c < Htotal.cols; ++c ) {
      double sum = 0;
      for( int r = 0; r < Htotal.rows; ++r ) {
        sum += Htotal(r, c);
        Htotal(r, c) = sum;
      }
    }
  }

  if( scaled ) {
    for( int c = 0; c < Htotal.cols; ++c ) {
      double mv = 0;
      if( cumulative ) { // In a cumulative histogram, the maximum is always in the last cell
        mv = Htotal(Htotal.rows - 1, c);
      }
      else { // search for the maximum in the column
        cv::minMaxLoc(Htotal.col(c), nullptr, &mv);
      }

      if( mv > 1e-12 ) {
        Htotal.col(c) /= mv; // Scale the column to the range [0, 1]
      }
    }
  }

  if( ddepth < 0 ) {
    ddepth = H.fixedType() ? H.depth() : CV_32F;
  }
  if ( (ddepth = CV_MAT_DEPTH(ddepth)) == Htotal.depth() ) {
    H.move(Htotal);
  }
  else {
    Htotal.convertTo(H, ddepth);
  }

  return true;
}

/**
 * Converts a regular histogram to a cumulative one.
 * If Hdst has a fixed type (e.g., cv::Mat1f was passed),
 * the result will be converted to it. Otherwise, the type is inherited from Hsrc.
 *  */
void makeCumulativeHistogram(cv::InputArray Hsrc, cv::OutputArray Hdst, bool normalize)
{
  if( Hsrc.empty() ) {
    Hdst.release();
    return;
  }

  cv::Mat src = Hsrc.getMat();
  const int targetDepth = Hdst.fixedType() ? Hdst.depth() : src.depth();

  cv::Mat1d tmp;
  src.convertTo(tmp, CV_64F);

  const int rows = tmp.rows;
  const int cols = tmp.cols;
  for( int c = 0; c < cols; ++c ) {
    double sum = 0;
    for( int r = 0; r < rows; ++r ) {
      sum += tmp(r, c);
      tmp(r, c) = sum;
    }
  }

  if ( normalize ) {
    normalizeHistogram(tmp, tmp, true);
  }

  tmp.convertTo(Hdst, targetDepth);
}

/**
 * Normalizes each histogram column independently to the range [0, 1].
 * If isCumulative == true, normalization is performed by the last element of the column.
 * Otherwise, normalization is performed by the maximum value in the column.
 *  */
void normalizeHistogram(cv::InputArray Hsrc, cv::OutputArray Hdst, bool isCumulative)
{
  if( Hsrc.empty() ) {
    Hdst.release();
    return;
  }

  cv::Mat src = Hsrc.getMat();

  const int targetDepth = Hdst.fixedType() ? Hdst.depth() : src.depth();

  cv::Mat1d tmp;
  src.convertTo(tmp, CV_64F);

  const int rows = tmp.rows;
  const int cols = tmp.cols;
  for( int c = 0; c < cols; ++c ) {
    double mv = 0;

    if( isCumulative && rows > 0 ) {
      // In a cumulative histogram, the maximum is always in the last cell
      mv = tmp(rows - 1, c);
    }
    else {
      // Find the actual maximum in the column
      cv::Point pos;
      cv::minMaxLoc(tmp.col(c), nullptr, &mv, nullptr, &pos);

      if( isCumulative && pos.y != rows - 1 ) {
        CF_ERROR("BUG DETECTED: pos.y=%d / %d", pos.y, rows - 1);
      }
    }

    // Normalize the column if it is not empty
    if( mv > 1e-12 ) {
      tmp.col(c) /= mv;
    }
  }

  tmp.convertTo(Hdst, targetDepth);
}


/**
* Compute the actual clipping levels for each channel.
*
* @param cumulativeNormalizedHistogram - cumulative normalized histogram (CV_64F, nbins x cn)
* @param vMin - minimum histogram scale (from createHistogram)
* @param vMax - maximum histogram scale (from createHistogram)
* @param qLow - lower quantile (e.g., 0.01 for 1%)
* @param qHigh - upper quantile (e.g., 0.99 for 99%)
* @param realLow - [out] low clipping levels by channel
* @param realHigh - [out] high clipping levels by channel
*  */
bool computeHistogramClipLevels(cv::InputArray cumulativeNormalizedHistogram,
    double vMin, double vMax,
    double qLow, double qHigh,
    cv::Scalar & realLow,
    cv::Scalar & realHigh)
{
  if( cumulativeNormalizedHistogram.empty() ) {
    return false;
  }

  // Need double precision for interpolation
  cv::Mat H = cumulativeNormalizedHistogram.getMat();
  if( H.depth() != CV_64F ) {
    H.convertTo(H, CV_64F);
  }

  const int nbins = H.rows;
  const int cn = (std::min)(4, H.cols); // Scalar supports up to 4 channels
  const double binWidth = (vMax - vMin) / nbins;

  // CF_DEBUG("vMax=%g vMin=%g nbins=%d binWidth=%g", vMax, vMin, nbins, binWidth);

  realLow = cv::Scalar::all(vMin);
  realHigh = cv::Scalar::all(vMax);

  for( int c = 0; c < cn; ++c ) {
    // Search for the lower threshold qLow
    for( int j = 0; j < nbins; ++j ) {
      double val = H.at<double>(j, c);
      if( val >= qLow ) {
        double idx;
        if( j == 0 ) {
          idx = (val > 1e-12) ? (qLow / val) : 0.0;
        }
        else {
          double prev = H.at<double>(j - 1, c);
          double diff = val - prev;
          double fraction = (diff > 1e-12) ? (qLow - prev) / diff : 0.0;
          idx = (double) (j - 1) + fraction;
        }
        realLow[c] = vMin + idx * binWidth;
        break;
      }
    }

    // Search for the upper threshold qHigh
    for( int j = 0; j < nbins; ++j ) {
      double val = H.at<double>(j, c);
      if( val >= qHigh ) {
        double idx;
        if( j == 0 ) {
          idx = (val > 1e-12) ? (qHigh / val) : 0.0;
        }
        else {
          double prev = H.at<double>(j - 1, c);
          double diff = val - prev;
          double fraction = (diff > 1e-12) ? (qHigh - prev) / diff : 0.0;
          idx = (double) (j - 1) + fraction;
        }
        realHigh[c] = vMin + (idx + 0.5) * binWidth;
        break;
      }
    }
  }

  return true;
}

/**
 * Estimate Mode from image histogram
 *
 * @param Hsrc - input image histogram (CV_32F or CV_64F, nbins x cn)
 * @param minv - minimum histogram scale (from createHistogram)
 * @param maxv - maximum histogram scale (from createHistogram)
 */
cv::Scalar computeHistogramMode(cv::InputArray Hist, double minv, double maxv)
{
  // Need double precision for interpolation
  cv::Mat1d H;

  cv::Mat Hsrc = Hist.getMat();
  if( Hsrc.depth() == H.depth() ) {
    H = Hsrc;
  }
  else {
    Hsrc.convertTo(H, H.depth());
  }

  const int nbins = H.rows;
  const int cn = H.cols;
  const double bin_width = (nbins > 1) ? (maxv - minv) / (nbins - 1) : 0;

  cv::Scalar mode = cv::Scalar::all(0);

  for( int c = 0; c < cn; ++c ) {
    // Find the global maximum in the channel
    double max_val = -1.0;
    for( int i = 0; i < nbins; ++i ) {
      if( H(i, c) > max_val ) {
        max_val = H(i, c);
      }
    }

    if ( max_val > 0  ) {
      // Find the boundaries of the FIRST plateau encountered with this maximum
      int first_idx = -1;
      int last_idx = -1;

      for( int i = 0; i < nbins; ++i ) {
        if( H(i, c) == max_val ) {
          if( first_idx == -1 ) {
            first_idx = i; // Found the beginning of the first maximum
          }
          last_idx = i;
        }
        else if( first_idx != -1 ) {
          // If we've already found the beginning of the maximum, and the current value is lower,
          // then the first plateau has ended. Exit the loop.
          break;
        }
      }

      double peak_idx = (first_idx + last_idx) / 2.0;

      // Quadratic interpolation
      // Take values ​​to the left of the plateau's beginning and to the right of its end
      const int left_i = first_idx - 1;
      const int right_i = last_idx + 1;

      if( left_i >= 0 && right_i < nbins ) {
        const double y_peak = max_val;
        const double y_left = H(left_i, c);
        const double y_right = H(right_i, c);
        const double denom = (y_left - 2 * y_peak + y_right);
        if( std::abs(denom) > 1e-9 ) {
          // Offset relative to the plateau center
          const double offset = 0.5 * (y_left - y_right) / denom;
          peak_idx += offset;
        }
      }

      // Convert the index into physical value
      mode[c] = minv + peak_idx * bin_width;
    }
  }

  return mode;
}

/**
 * Estimate Median from cumulative normalized histogram
 *
 * @param cumulativeNormalizedHistogram - input cumulative normalized histogram (CV_32F or CV_64F, nbins x cn)
 * @param minv - minimum histogram scale (from createHistogram)
 * @param maxv - maximum histogram scale (from createHistogram)
 */
cv::Scalar computeHistogramMedian(cv::InputArray cumulativeNormalizedHistogram,
    double minv, double maxv)
{
  // Need double precision for interpolation
  cv::Mat1d H;

  cv::Mat Hsrc = cumulativeNormalizedHistogram.getMat();
  if( Hsrc.depth() == H.depth() ) {
    H = Hsrc;
  }
  else {
    Hsrc.convertTo(H, H.depth());
  }

  cv::Scalar s = cv::Scalar::all(0);;

  for( int c = 0; c < std::min(H.cols, 4); ++c ) {
    if( H.rows < 2 ) {
      s[c] = minv;
      continue;
    }

    // Step of one bin in data units
    const double binWidth = (maxv - minv) / H.rows;

    for( int r = 0; r < H.rows; ++r ) {
      // Value in the current bin already normalized into 0..1
      const double val = H(r, c);
      if( val >= 0.5 ) {
        if( r == 0 ) {
          // Even if the first bin is already >= 0.5, the median is at the very beginning
          const double fraction = 0.5 / val;
          s[c] = minv + fraction * binWidth;
        }
        else {
          // Take the previous value for interpolation
          const double prevVal = H(r - 1, c);
          // How far 0.5 has moved from the previous bin to the current one
          const double fraction = (0.5 - prevVal) / (val - prevVal);
          // Final formula
          s[c] = minv + ((double) (r - 1) + fraction) * binWidth;
        }
        break;
      }
    }
  }

  return s;
}


