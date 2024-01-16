/*
 * threshold.cc
 *
 *  Created on: May 21, 2017
 *      Author: amyznikov
 */


#include <core/proc/threshold.h>
//#include <core/proc/normalize.h>
#include <core/proc/estimate_noise.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<THRESHOLD_TYPE>()
{
  static const c_enum_member members[] = {
      { THRESHOLD_TYPE_VALUE, "VALUE", "Use user-specified value for compare operation"},
      { THRESHOLD_TYPE_OTSU, "OTSU", "Use Otsu algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_TRIANGLE, "TRIANGLE", "Use Triangle algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_MOMENTS, "MOMENTS", "Use MOMENTS algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_ISODATA, "ISODATA", "Use ISODATA algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_HUANG, "HUANG", "Use HUANG algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_YEN, "YEN", "Use YEN algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_MEAN, "MEAN", "Select pixels with values above mean value" },
      { THRESHOLD_TYPE_MINIMUM, "MINIMUM", "Use MINIMUM algorithm to choose the optimal threshold value" },
      { THRESHOLD_TYPE_NOISE, "NOISE", "Estimate noise on the image" },

      { THRESHOLD_TYPE_OTSU }
  };

  return members;
}

double get_threshold_value(cv::InputArray image, cv::InputArray mask, THRESHOLD_TYPE threshold_type, double value)
{
  double threshold_value = value;

  switch (threshold_type) {
    case THRESHOLD_TYPE_OTSU:
      threshold_value = get_otsu_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_TRIANGLE:
      threshold_value = get_triangle_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_MOMENTS:
      threshold_value = get_moments_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_ISODATA:
      threshold_value = get_isodata_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_HUANG:
      threshold_value = get_huang_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_YEN:
      threshold_value = get_yen_threshold(image, mask);
      break;
    case THRESHOLD_TYPE_MEAN:
      threshold_value = cv::mean(image, mask)[0];
      break;
    case THRESHOLD_TYPE_MINIMUM:
      threshold_value = get_minimum_threshold(image, mask);
      break;

    case THRESHOLD_TYPE_NOISE: {

      const int cn =
          image.channels();

      const cv::Scalar s =
          estimate_noise(image, cv::noArray(), mask);

      threshold_value = s[0];
      for( int i = 1; i < cn; ++i ) {
        threshold_value += s[i];
      }
      threshold_value /= cn;

      break;
    }
  }

  return threshold_value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *  Copied from OpenCV sources getThreshVal_Otsu_8u()
 *  */

double get_otsu_threshold_8u(const cv::Mat& _src)
{
  cv::Size size = _src.size();
  int step = (int) _src.step;
  if ( _src.isContinuous() ) {
    size.width *= size.height;
    size.height = 1;
    step = size.width;
  }

  const int N = 256;
  int i, j, h[N] = { 0 };
  for ( i = 0; i < size.height; i++ ) {
    const uchar* src = _src.ptr() + step * i;
    j = 0;
#if CV_ENABLE_UNROLLED
    for ( ; j <= size.width - 4; j += 4 ) {
      int v0 = src[j], v1 = src[j + 1];
      h[v0]++;
      h[v1]++;
      v0 = src[j + 2];
      v1 = src[j + 3];
      h[v0]++;
      h[v1]++;
    }
#endif
    for ( ; j < size.width; j++ )
      h[src[j]]++;
  }

  double mu = 0, scale = 1. / (size.width * size.height);
  for ( i = 0; i < N; i++ ) {
    mu += i * (double) h[i];
  }

  mu *= scale;
  double mu1 = 0, q1 = 0;
  double max_sigma = 0, max_val = 0;

  for ( i = 0; i < N; i++ ) {
    double p_i, q2, mu2, sigma;

    p_i = h[i] * scale;
    mu1 *= q1;
    q1 += p_i;
    q2 = 1. - q1;

    if ( std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON )
      continue;

    mu1 = (mu1 + i * p_i) / q1;
    mu2 = (mu - q1 * mu1) / q2;
    sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
    if ( sigma > max_sigma ) {
      max_sigma = sigma;
      max_val = i;
    }
  }

  return max_val;
}

/* _src must be 1 channel matrix */
template<class T>
static void mkhist_(const cv::Mat & _src, std::vector<int> & h, double vmin, double vmax, cv::InputArray _mask )
{
  const int N = h.size();
  cv::Mat mask = _mask.getMat();
  if ( mask.empty() ) {
    for ( int i = 0; i < _src.rows; ++i ) {
      const T * src = _src.ptr<const T>(i);
      for ( int j = 0; j < _src.cols; j++ ) {
        double v = (double) src[j];
        if ( v < vmin ) {
          v = vmin;
        }
        else if ( v > vmax ) {
          v = vmax;
        }

        h[std::min((int) ((v - vmin) * N / (vmax - vmin)), N - 1)]++;
      }
    }
  }
  else {
    for ( int i = 0; i < _src.rows; ++i ) {
      const T * src = _src.ptr<const T>(i);
      const uint8_t * msk = mask.ptr<const uint8_t>(i);
      for ( int j = 0; j < _src.cols; j++ ) {
        if ( msk[j] ) {
          double v = (double) src[j];
          if ( v < vmin ) {
            v = vmin;
          }
          else if ( v > vmax ) {
            v = vmax;
          }
          h[std::min((int) ((v - vmin) * N / (vmax - vmin)), N - 1)]++;
        }
      }
    }
  }
}

static double get_otsu_threshold_(const std::vector<int> & h, double scale /* , scale = 1. / (size.width * size.height) */)
{
  const int N = h.size();
  double mu = 0, mu1 = 0, q1 = 0;
  double max_sigma = 0, max_val = 0;

  for ( int i = 0; i < N; i++ ) {
    mu += i * (double) h[i];
  }

  mu *= scale;

  for ( int i = 0; i < N; i++ ) {
    double p_i, q2, mu2, sigma;

    p_i = h[i] * scale;
    mu1 *= q1;
    q1 += p_i;
    q2 = 1. - q1;

    if ( std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON ) {
      continue;
    }

    mu1 = (mu1 + i * p_i) / q1;
    mu2 = (mu - q1 * mu1) / q2;
    sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
    if ( sigma > max_sigma ) {
      max_sigma = sigma;
      max_val = i;
    }
  }

  return max_val;
}


template<class T>
double get_otsu_threshold_(const cv::Mat& _src, int N, double vmin, double vmax, cv::InputArray mask)
{
  std::vector<int> h(N);
  double t, scale;

  mkhist_<T>(_src, h, vmin, vmax, mask);
  scale = mask.empty() ? 1. / (_src.rows * _src.cols) : 1. / countNonZero(mask);
  t = get_otsu_threshold_(h, scale);
  return vmin + t * (vmax - vmin) / N;
}


double get_otsu_threshold(cv::InputArray src, cv::InputArray _mask)
{
  const cv::Mat _src = src.getMat();

  if ( !_src.isContinuous() ) {
    CF_FATAL("Unsupported not continuous matrix encountered");
  }
  else if ( _src.channels() != 1 ) {
    CF_FATAL("Unsupported multichannel matrix encountered");
  }
  else {
    switch ( _src.type() ) {
      case CV_8UC1 :
      return get_otsu_threshold_8u(_src);
      case CV_16UC1 :
      return get_otsu_threshold_<uint16_t>(_src, 65536, 0, 65535, _mask);
      case CV_16SC1 :
      return get_otsu_threshold_<int16_t>(_src, 65536, INT16_MIN, INT16_MAX, _mask);
      case CV_32SC1 :
      return get_otsu_threshold_<int32_t>(_src, 65536, INT32_MIN, INT32_MAX, _mask);
      case CV_32FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_otsu_threshold_<float>(_src, 65536, min, max, _mask);
      }
      case CV_64FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_otsu_threshold_<double>(_src, 65536, min, max, _mask);
      }
      default :
      CF_FATAL("Unsupported matrix type '%d' encountered", _src.type());
      break;
    }
  }
  return NAN;
}


void otsu_threshold(const cv::Mat & _src, cv::Mat & _dst, double ts, cv::InputArray _mask )
{
  cv::Mat src;
  double t;

  if ( _src.type() == CV_8UC1 ) {
    src =  _src;
  }
  else {
    //normalizeMeanStdev(_src, src, 5, 0, 255, _mask);
    cv::normalize(_src, src, 0, 255, cv::NORM_MINMAX);
    src.convertTo(src, CV_8UC1);
  }

  t = get_otsu_threshold_8u(src);
  if ( ts > 0 && ts != 1 ) {
    t *= std::max(ts, 0.75);
  }

  cv::threshold(src, _dst, t, 255, cv::THRESH_BINARY);

  if ( !_mask.empty() ) {
    _dst.setTo(0, ~_mask.getMat());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

double get_isodata_threshold(cv::InputArray _src, cv::InputArray _srcmask)
{
  double min, max, t, tp;

  cv::Mat src = _src.getMat();
  cv::Mat srcmask = _srcmask.getMat();

  minMaxLoc(src, &min, &max, NULL, NULL, _srcmask);

  t = (min + max) / 2;

  do {
    cv::Mat mask;

    if ( srcmask.empty() ) {
      mask = src <= t;
    }
    else {
      bitwise_and(srcmask, src <= t, mask);
    }

    double bkg = mean(src, mask).val[0];
    double frg = mean(src, ~mask).val[0];

    tp = t, t = (bkg + frg) / 2;

  } while ( fabs(t - tp) > FLT_EPSILON );

  return t;
}

void isodata_threshold(const cv::Mat & src, cv::Mat & dst, double maxval, cv::InputArray _mask, int ttype)
{
  cv::threshold(src, dst, get_isodata_threshold(src, _mask), maxval, ttype);
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Implements Huang's fuzzy thresholding method
// Uses Shannon's entropy function (one can also use Yager's entropy function)
// Huang L.-K. and Wang M.-J.J. (1995) "Image Thresholding by Minimizing
// the Measures of Fuzziness" Pattern Recognition, 28(1): 41-51
// M. Emre Celebi  06.15.2007
// Ported from ImageJ https://imagej.nih.gov/ij/developer/source/ij/process/AutoThresholder.java.html
static int Huang(float hist[256])
{
  int threshold = -1;
  int ih, it;
  int first_bin;
  int last_bin;
  double sum_pix;
  double num_pix;
  double term;
  double ent;  // entropy
  double min_ent; // min entropy
  double mu_x;

  double mu_0[256];
  double mu_1[256];

  /* Determine the first non-zero bin */
  first_bin = 0;
  for ( ih = 0; ih < 256; ih++ ) {
    if ( hist[ih] != 0 ) {
      first_bin = ih;
      break;
    }
  }

  /* Determine the last non-zero bin */
  last_bin = 255;
  for ( ih = 255; ih >= first_bin; ih-- ) {
    if ( hist[ih] != 0 ) {
      last_bin = ih;
      break;
    }
  }
  term = 1.0 / (double) (last_bin - first_bin);
  sum_pix = num_pix = 0;
  for ( ih = first_bin; ih < 256; ih++ ) {
    sum_pix += (double) ih * hist[ih];
    num_pix += hist[ih];
    /* NUM_PIX cannot be zero ! */
    mu_0[ih] = sum_pix / num_pix;
  }

  sum_pix = num_pix = 0;
  for ( ih = last_bin; ih > 0; ih-- ) {
    sum_pix += (double) ih * hist[ih];
    num_pix += hist[ih];
    /* NUM_PIX cannot be zero ! */
    mu_1[ih - 1] = sum_pix / (double) num_pix;
  }

  /* Determine the cv::threshold that minimizes the fuzzy entropy */
  threshold = -1;
  min_ent = DBL_MAX; // Double.MAX_VALUE;
  for ( it = 0; it < 256; it++ ) {
    ent = 0.0;
    for ( ih = 0; ih <= it; ih++ ) {
      /* Equation (4) in Ref. 1 */
      mu_x = 1.0 / (1.0 + term * fabs(ih - mu_0[it]));
      if ( !((mu_x < 1e-06) || (mu_x > 0.999999)) ) {
        /* Equation (6) & (8) in Ref. 1 */
        ent += hist[ih] * (-mu_x * log(mu_x) - (1.0 - mu_x) * log(1.0 - mu_x));
      }
    }

    for ( ih = it + 1; ih < 256; ih++ ) {
      /* Equation (4) in Ref. 1 */
      mu_x = 1.0 / (1.0 + term * fabs(ih - mu_1[it]));
      if ( !((mu_x < 1e-06) || (mu_x > 0.999999)) ) {
        /* Equation (6) & (8) in Ref. 1 */
        ent += hist[ih] * (-mu_x * log(mu_x) - (1.0 - mu_x) * log(1.0 - mu_x));
      }
    }
    /* No need to divide by NUM_ROWS * NUM_COLS * LOG(2) ! */
    if ( ent < min_ent ) {
      min_ent = ent;
      threshold = it;
    }
  }

  return threshold;
}


/* fixme: this computes index in 256-bin histogram instead of real threshold */
double get_huang_threshold(cv::InputArray _src, cv::InputArray _mask)
{
  const int numbins = 256;
  int histSize[] = { numbins };
  int channels[] = { 0 };
  float hrange[] = { (float) 0, (float) 255 };
  const float * ranges[] = { hrange };

  cv::Mat src, gshist;


  src = _src.getMat();

  try {
    calcHist(&src, 1, channels, _mask, gshist, 1, histSize, ranges, true, false);
  }
  catch( const std::exception &e ) {
    CF_ERROR("cv::calcHist( fails: %s", e.what());
    return 127;
  }

  if ( gshist.empty() ) {
    CF_FATAL("calcHist fails");
    return 127;
  }

  if ( gshist.cols == 1 ) {
    transpose(gshist, gshist);
  }

  if ( gshist.cols != 256 ) {
    CF_FATAL("calcHist returns unexpected hist size: %dx%d type=%d", gshist.rows, gshist.cols, gshist.type());
    return 127;
  }

  if ( gshist.type() != CV_32FC1 ) {
    gshist.convertTo(gshist, CV_32FC1);
  }

  return Huang(gshist.ptr<float>(0));
}

void huang_threshold(const cv::Mat & _src, cv::Mat & _dst, cv::InputArray _mask)
{
  cv::Mat src;
  if ( _src.type() == CV_8UC1 ) {
    src = _src;
  }
  else {
    cv::normalize(_src, src, 0, 255, cv::NORM_MINMAX, CV_8UC1, _mask);
  }

  cv::threshold(src, _dst, get_huang_threshold(src, _mask), 255, cv::THRESH_BINARY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void mean_threshold(cv::InputArray & _src, cv::Mat & _dst, cv::InputArray _mask)
{
  // C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
  // CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
  // The threshold is the mean of the gray scale data

  cv::threshold(_src, _dst, mean(_src, _mask).val[0], 255, cv::THRESH_BINARY);
  if (_dst.type() != CV_8UC1 ) {
    _dst.convertTo(_dst, CV_8UC1);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////



//
// https://github.com/imagej/imagej-ops/blob/master/src/main/java/net/imagej/ops/threshold/triangle/ComputeTriangleThreshold.java
// Zack, G. W., Rogers, W. E. and Latt, S. A., 1977,
// Automatic Measurement of Sister Chromatid Exchange Frequency,
// Journal of Histochemistry and Cytochemistry 25 (7), pp. 741-753
//
// modified from Johannes Schindelin plugin
static int get_triangle_threshold_histogram_bin(std::vector<int> & histogram)
{
  int min = 0, max = 0, min2 = 0;
  int dmax = 0;

  const int hlen = histogram.size();

  // find min and max
  for ( int i = 0; i < hlen; i++ ) {
    if ( histogram[i] > 0 ) {
      min = i;
      break;
    }
  }

  if ( min > 0 ) {
    --min; // line to the (p==0) point, not to histogram[min]
  }

  // The Triangle algorithm cannot tell whether the data is skewed to one
  // side or another. This causes a problem as there are 2 possible thresholds
  // between the max and the 2 extremes of the histogram. Here I propose
  // to find out to which side of the max point the data is furthest, and use
  // that as the other extreme.
  for ( int i = hlen - 1; i > 0; i-- ) {
    if ( histogram[i] > 0 ) {
      min2 = i;
      break;
    }
  }

  // line to the (p==0) point, not to histogram[min]
  if ( min2 < hlen - 1 ) {
    ++min2;
  }

  for ( int i = 0; i < hlen; i++ ) {
    if ( histogram[i] > dmax ) {
      max = i;
      dmax = histogram[i];
    }
  }

  // find which is the furthest side
  // IJ.log(""+min+" "+max+" "+min2);
  bool inverted = false;

  if ( (max - min) < (min2 - max) ) {
    // reverse the histogram
    // IJ.log("Reversing histogram.");
    inverted = true;
    int left = 0; // index of leftmost element
    int right = hlen - 1; // index of rightmost element
    while ( left < right ) {
      // exchange the left and right elements
      int temp = histogram[left];
      histogram[left] = histogram[right];
      histogram[right] = temp;
      // move the bounds toward the center
      left++;
      right--;
    }
    min = hlen - 1 - min2;
    max = hlen - 1 - max;
  }

  if ( min == max ) {
    // IJ.log("Triangle:  min == max.");
    return min;
  }

  // describe line by nx * x + ny * y - d = 0
  double nx, ny, d;
  // nx is just the max frequency as the other point has freq=0
  // lowest value bmin = (p=0)% in the image
  nx = histogram[max]; // -min; // histogram[min];
  ny = min - max;
  d = hypot(nx, ny);
  nx /= d;
  ny /= d;
  d = nx * min + ny * histogram[min];

  // find split point
  int split = min;
  double splitDistance = 0;
  for ( int i = min + 1; i <= max; i++ ) {
    double newDistance = nx * i + ny * histogram[i] - d;
    if ( newDistance > splitDistance ) {
      split = i;
      splitDistance = newDistance;
    }
  }
  --split;


  if ( inverted ) {
    // The histogram might be used for something else, so let's reverse
    // it
    // back
    int left = 0;
    int right = hlen - 1;
    while ( left < right ) {
      int temp = histogram[left];
      histogram[left] = histogram[right];
      histogram[right] = temp;
      left++;
      right--;
    }
    return (hlen - 1 - split);
  }
  return split;
}

template<class T>
static double get_triangle_threshold_(const cv::Mat& _src, int N, double vmin, double vmax, cv::InputArray mask)
{
  std::vector<int> h(N);
  double t;

  mkhist_<T>(_src, h, vmin, vmax, mask);
  t = get_triangle_threshold_histogram_bin(h);
  return vmin + t * (vmax - vmin) / N;
}

double get_triangle_threshold(cv::InputArray src, cv::InputArray _mask)
{
  const cv::Mat _src = src.getMat();

  if ( !_src.isContinuous() ) {
    CF_FATAL("Unsupperted not continuous matrix encountered");
  }
  else if ( _src.channels() != 1 ) {
    CF_FATAL("Unsupperted multichannel matrix encountered");
  }
  else {
    switch ( _src.type() ) {
      case CV_8SC1 :
      return get_triangle_threshold_<int8_t>(_src, 256, INT8_MIN, INT8_MAX, _mask);
      case CV_8UC1 :
      return get_triangle_threshold_<uint8_t>(_src, 256, 0, UINT8_MAX, _mask);
      case CV_16SC1 :
      return get_triangle_threshold_<int16_t>(_src, 65536, INT16_MIN, INT16_MAX, _mask);
      case CV_16UC1 :
      return get_triangle_threshold_<uint16_t>(_src, 65536, 0, UINT16_MAX, _mask);
      case CV_32SC1 :
      return get_triangle_threshold_<int32_t>(_src, 65536, INT32_MIN, INT32_MAX, _mask);
      case CV_32FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_triangle_threshold_<float>(_src, 65536, min, max, _mask);
      }
      case CV_64FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_triangle_threshold_<double>(_src, 65536, min, max, _mask);
      }
      default :
      CF_FATAL("Unsupported matrix type '%d' encountered", _src.type());
      break;
    }
  }
  return NAN;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://imagej.nih.gov/ij/developer/source/ij/process/AutoThresholder.java.html
//  W. Tsai, "Moment-preserving thresholding: a new approach," Computer Vision,
// Graphics, and Image Processing, vol. 29, pp. 377-393, 1985.
// Ported to ImageJ plugin by G.Landini from the the open source project FOURIER 0.8
// by  M. Emre Celebi , Department of Computer Science,  Louisiana State University in Shreveport
// Shreveport, LA 71115, USA
//  http://sourceforge.net/projects/fourier-ipal
//  http://www.lsus.edu/faculty/~ecelebi/fourier.htm

static int get_moments_threshold_histogram_bin(const std::vector<int> & histogram)
{
  double total = 0;
  double m0 = 1.0, m1 = 0.0, m2 = 0.0, m3 = 0.0, sum = 0.0, p0 = 0.0;
  double cd, c0, c1, z0, z1; /* auxiliary variables */
  int threshold = -1;

  const int hlen = histogram.size();
  std::vector<double> histo(hlen);

  for ( int i = 0; i < hlen; i++ ) {
    total += histogram[i];
  }

  for ( int i = 0; i < hlen; i++ ) {
    histo[i] = (double) (histogram[i] / total);    //normalised histogram
  }

  /* Calculate the first, second, and third order moments */
  for ( int i = 0; i < hlen; i++ ) {
    double di = i;
    m1 += di * histo[i];
    m2 += di * di * histo[i];
    m3 += di * di * di * histo[i];
  }

  /*
   First 4 moments of the gray-level image should match the first 4 moments
   of the target binary image. This leads to 4 equalities whose solutions
   are given in the Appendix of Ref. 1
   */
  cd = m0 * m2 - m1 * m1;
  c0 = (-m2 * m2 + m1 * m3) / cd;
  c1 = (m0 * -m3 + m2 * m1) / cd;
  z0 = 0.5 * (-c1 - sqrt(c1 * c1 - 4.0 * c0));
  z1 = 0.5 * (-c1 + sqrt(c1 * c1 - 4.0 * c0));
  p0 = (z1 - m1) / (z1 - z0); /* Fraction of the object pixels in the target binary image */

  // The threshold is the gray-level closest
  // to the p0-tile of the normalized histogram
  sum = 0;
  for ( int i = 0; i < hlen; i++ ) {
    sum += histo[i];
    if ( sum > p0 ) {
      threshold = i;
      break;
    }
  }

  return threshold;
}

template<class T>
static double get_moments_threshold_(const cv::Mat& _src, int N, double vmin, double vmax, cv::InputArray mask)
{
  std::vector<int> h(N);
  double t;

  mkhist_<T>(_src, h, vmin, vmax, mask);
  t = get_moments_threshold_histogram_bin(h);
  return vmin + t * (vmax - vmin) / N;
}

double get_moments_threshold(cv::InputArray src, cv::InputArray _mask)
{
  const cv::Mat _src = src.getMat();

  if ( !_src.isContinuous() ) {
    CF_FATAL("Unsupperted not continuous matrix encountered");
  }
  else if ( _src.channels() != 1 ) {
    CF_FATAL("Unsupperted multichannel matrix encountered");
  }
  else {
    switch ( _src.type() ) {
      case CV_8SC1 :
      return get_moments_threshold_<int8_t>(_src, 256, INT8_MIN, INT8_MAX, _mask);
      case CV_8UC1 :
      return get_moments_threshold_<uint8_t>(_src, 256, 0, UINT8_MAX, _mask);
      case CV_16SC1 :
      return get_moments_threshold_<int16_t>(_src, 65536, INT16_MIN, INT16_MAX, _mask);
      case CV_16UC1 :
      return get_moments_threshold_<uint16_t>(_src, 65536, 0, UINT16_MAX, _mask);
      case CV_32SC1 :
      return get_moments_threshold_<int32_t>(_src, 65536, INT32_MIN, INT32_MAX, _mask);
      case CV_32FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_moments_threshold_<float>(_src, 65536, min, max, _mask);
      }
      case CV_64FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_moments_threshold_<double>(_src, 65536, min, max, _mask);
      }
      default :
      CF_FATAL("Unsupported matrix type '%d' encountered", _src.type());
      break;
    }
  }
  return NAN;
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////


// https://imagej.nih.gov/ij/developer/source/ij/process/AutoThresholder.java.html
// J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
// Annals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
// ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab code (GPL)
// Original Matlab code Copyright (C) 2004 Antti Niemisto
// See http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
// and the original Matlab code.
//
// Assumes a bimodal histogram. The histogram needs is smoothed (using a
// running average of size 3, iteratively) until there are only two local maxima.
// Threshold t is such that yt-1 > yt <= yt+1.
// Images with histograms having extremely unequal peaks or a broad and
// flat valleys are unsuitable for this method.
static bool bimodalTest(const std::vector<float> & y)
{
  bool b = false;
  int modes = 0;

  for ( int k = 1, length = y.size(); k < length - 1; k++ ) {
    if ( y[k - 1] < y[k] && y[k + 1] < y[k] ) {
      modes++;
      if ( modes > 2 ) {
        return false;
      }
    }
  }
  if ( modes == 2 ) {
    b = true;
  }
  return b;
}

int get_minimum_threshold_histogram_bin(const std::vector<float> & H)
{
  int iter = 0;
  int threshold = -1;

  const int hlen = H.size();
  std::vector<float> iHisto = H;
  std::vector<float> tHisto(hlen);

  while ( !bimodalTest(iHisto) ) {

    //smooth with a 3 point running mean filter
    for ( int i = 1; i < hlen - 1; i++ ) {
      tHisto[i] = (iHisto[i - 1] + iHisto[i] + iHisto[i + 1]) / 3;
    }

    tHisto[0] = (iHisto[0] + iHisto[1]) / 3;    //0 outside
    tHisto[hlen - 1] = (iHisto[hlen - 2] + iHisto[hlen - 1]) / 3;    //0 outside

    iHisto = tHisto;

    iter++;
    if ( iter > 10000 ) {
      threshold = -1;
      CF_FATAL("Minimum: threshold not found after 10000 iterations.");
      return threshold;
    }
  }

  // The threshold is the minimum between the two peaks.
  for ( int i = 1; i < hlen - 1; i++ ) {
    if ( iHisto[i - 1] > iHisto[i] && iHisto[i + 1] >= iHisto[i] ) {
      threshold = i;
      break;
    }
  }

  return threshold;
}


int get_minimum_threshold_histogram_bin(std::vector<int> & data)
{
  const int hlen = data.size();
  std::vector<float> H(hlen);
  for ( int i = 0; i < hlen; ++i ) {
    H[i] = data[i];
  }
  return get_minimum_threshold_histogram_bin(H);
}


template<class T>
static double get_minimum_threshold_(const cv::Mat& _src, int N, double vmin, double vmax, cv::InputArray mask)
{
  std::vector<int> h(N);
  double t;

  mkhist_<T>(_src, h, vmin, vmax, mask);
  t = get_minimum_threshold_histogram_bin(h);
  return vmin + t * (vmax - vmin) / N;
}

double get_minimum_threshold(cv::InputArray src, cv::InputArray _mask)
{
  const cv::Mat _src = src.getMat();

  if ( !_src.isContinuous() ) {
    CF_FATAL("Unsupperted not continuous matrix encountered");
  }
  else if ( _src.channels() != 1 ) {
    CF_FATAL("Unsupperted multichannel matrix encountered");
  }
  else {
    switch ( _src.type() ) {
      case CV_8SC1 :
      return get_minimum_threshold_<int8_t>(_src, 256, INT8_MIN, INT8_MAX, _mask);
      case CV_8UC1 :
      return get_minimum_threshold_<uint8_t>(_src, 256, 0, UINT8_MAX, _mask);
      case CV_16SC1 :
      return get_minimum_threshold_<int16_t>(_src, 65536, INT16_MIN, INT16_MAX, _mask);
      case CV_16UC1 :
      return get_minimum_threshold_<uint16_t>(_src, 65536, 0, UINT16_MAX, _mask);
      case CV_32SC1 :
      return get_minimum_threshold_<int32_t>(_src, 65536, INT32_MIN, INT32_MAX, _mask);
      case CV_32FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_minimum_threshold_<float>(_src, 65536, min, max, _mask);
      }
      case CV_64FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_minimum_threshold_<double>(_src, 65536, min, max, _mask);
      }
      default :
      CF_FATAL("Unsupported matrix type '%d' encountered", _src.type());
      break;
    }
  }
  return NAN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// https://github.com/imagej/imagej-ops/blob/master/src/main/java/net/imagej/ops/threshold/yen/ComputeYenThreshold.java

static double get_yen_threshold_histogram_bin(const std::vector<int> & histogram)
{
  // Implements Yen thresholding method
  // 1) Yen J.C., Chang F.J., and Chang S. (1995) "A New Criterion
  // for Automatic Multilevel Thresholding" IEEE Trans. on Image
  // Processing, 4(3): 370-378
  // 2) Sezgin M. and Sankur B. (2004) "Survey over Image Thresholding
  // Techniques and Quantitative Performance Evaluation" Journal of
  // Electronic Imaging, 13(1): 146-165
  // http://citeseer.ist.psu.edu/sezgin04survey.html
  //
  // M. Emre Celebi
  // 06.15.2007
  // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8
  // routines
  int threshold;
  int ih, it;
  double crit;
  double max_crit;

  const int hsize = histogram.size();

  double norm_histo[hsize];  // normalized histogram
  double P1[hsize]; // cumulative normalized histogram
  double P1_sq[hsize];
  double P2_sq[hsize];

  int total = 0;
  for ( ih = 0; ih < hsize; ++ih ) {
    total += histogram[ih];
  }

  for ( ih = 0; ih < hsize; ++ih ) {
    norm_histo[ih] = (double) histogram[ih] / total;
  }

  P1[0] = norm_histo[0];
  for ( ih = 1; ih < hsize; ++ih ) {
    P1[ih] = P1[ih - 1] + norm_histo[ih];
  }

  P1_sq[0] = norm_histo[0] * norm_histo[0];
  for ( ih = 1; ih < hsize; ++ih ) {
    P1_sq[ih] = P1_sq[ih - 1] + norm_histo[ih] * norm_histo[ih];
  }

  P2_sq[hsize - 1] = 0.0;
  for ( ih = hsize - 2; ih >= 0; --ih ) {
    P2_sq[ih] = P2_sq[ih + 1] + norm_histo[ih + 1] * norm_histo[ih + 1];
  }

  /* Find the threshold that maximizes the criterion */
  threshold = -1;
  max_crit = -INFINITY;
  for ( it = 0; it < hsize; it++ ) {
    crit = -1.0 * ((P1_sq[it] * P2_sq[it]) > 0.0 ? log(P1_sq[it] * P2_sq[it]) : 0.0)
        + 2 * ((P1[it] * (1.0 - P1[it])) > 0.0 ? log(P1[it] * (1.0 - P1[it])) :
            0.0);
    if ( crit > max_crit ) {
      max_crit = crit;
      threshold = it;
    }
  }
  return threshold;
}


template<class T>
static double get_yen_threshold_(const cv::Mat& _src, int N, double vmin, double vmax, cv::InputArray mask)
{
  std::vector<int> h(N);
  double t;

  mkhist_<T>(_src, h, vmin, vmax, mask);
  t = get_yen_threshold_histogram_bin(h);
  return vmin + t * (vmax - vmin) / N;
}

double get_yen_threshold(cv::InputArray src, cv::InputArray _mask)
{
  const cv::Mat _src = src.getMat();

  if ( _src.channels() != 1 ) {
    CF_FATAL("Unsupperted multichannel matrix encountered");
  }
  else {
    switch ( _src.type() ) {
      case CV_8SC1 :
      return get_yen_threshold_<int8_t>(_src, 256, INT8_MIN, INT8_MAX, _mask);
      case CV_8UC1 :
      return get_yen_threshold_<uint8_t>(_src, 256, 0, UINT8_MAX, _mask);
      case CV_16SC1 :
      return get_yen_threshold_<int16_t>(_src, 65536, INT16_MIN, INT16_MAX, _mask);
      case CV_16UC1 :
      return get_yen_threshold_<uint16_t>(_src, 65536, 0, UINT16_MAX, _mask);
      case CV_32SC1 :
      return get_yen_threshold_<int32_t>(_src, 65536, INT32_MIN, INT32_MAX, _mask);
      case CV_32FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_yen_threshold_<float>(_src, 65536, min, max, _mask);
      }
      case CV_64FC1 : {
        double min = 0, max = 0;
        minMaxLoc(_src, &min, &max, 0, 0, _mask);
        return get_yen_threshold_<double>(_src, 65536, min, max, _mask);
      }
      default :
      CF_FATAL("Unsupported matrix type '%d' encountered", _src.type());
      break;
    }
  }
  return NAN;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//// https://imagej.nih.gov/ij/developer/source/ij/process/AutoThresholder.java.html
//static int get_intermodes_threshold_histogram_bin(const int data[], int len)
//{
//  // J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
//  // Annals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
//  // ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab code (GPL)
//  // Original Matlab code Copyright (C) 2004 Antti Niemisto
//  // See http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
//  // and the original Matlab code.
//  //
//  // Assumes a bimodal histogram. The histogram needs is smoothed (using a
//  // running average of size 3, iteratively) until there are only two local maxima.
//  // j and k
//  // Threshold t is (j+k)/2.
//  // Images with histograms having extremely unequal peaks or a broad and
//  // flat valleys are unsuitable for this method.
//
//  int minbin = -1, maxbin = -1;
//  for ( int i = 0; i < data.length; i++ )
//    if ( data[i] > 0 )
//      maxbin = i;
//  for ( int i = data.length - 1; i >= 0; i-- )
//    if ( data[i] > 0 )
//      minbin = i;
//  int length = (maxbin - minbin) + 1;
//  double [] hist = new double[length];
//  for ( int i = minbin; i <= maxbin; i++ )
//    hist[i - minbin] = data[i];
//
//  int iter = 0;
//  int threshold = -1;
//  while ( !bimodalTest(hist) ) {
//    //smooth with a 3 point running mean filter
//    double previous = 0, current = 0, next = hist[0];
//    for ( int i = 0; i < length - 1; i++ ) {
//      previous = current;
//      current = next;
//      next = hist[i + 1];
//      hist[i] = (previous + current + next) / 3;
//    }
//    hist[length - 1] = (current + next) / 3;
//    iter++;
//    if ( iter > 10000 ) {
//      threshold = -1;
//      IJ.log("Intermodes Threshold not found after 10000 iterations.");
//      return threshold;
//    }
//  }
//
//  // The threshold is the mean between the two peaks.
//  int tt = 0;
//  for ( int i = 1; i < length - 1; i++ ) {
//    if ( hist[i - 1] < hist[i] && hist[i + 1] < hist[i] ) {
//      tt += i;
//      //IJ.log("mode:" +i);
//    }
//  }
//  threshold = (int) Math.floor(tt / 2.0);
//  return threshold + minbin;
//}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
