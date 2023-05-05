/*
 * c_sweepscan_stereo_matcher.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "c_sweepscan_stereo_matcher.h"

#include <core/proc/pyrscale.h>
#include <core/proc/threshold.h>
#include <core/proc/reduce_channels.h>
#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/proc/array2d.h>

//#include <opencv2/stereo.hpp>

#include <deque>
#include <core/debug.h>

namespace cv {
  typedef Mat_<uint32_t> Mat1u;
} // namespace cv

namespace {

double maxval_for_pixel_depth(int ddepth)
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
  return FLT_MAX;
}

void pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale)
{
  cv::Mat m;
  cv::Scalar mean, stdev;
  double f = 0;

  pyramid_downscale(src, m, pscale, cv::BORDER_REPLICATE);
  pyramid_upscale(m, src.size());
  m.convertTo(m, CV_32F);
  cv::subtract(src, m, m, cv::noArray(), CV_32F);

  cv::meanStdDev(m, mean, stdev);
  for( int i = 0, cn = src.channels(); i < cn; ++i ) {
    f += stdev[i];
  }

  m.convertTo(dst, CV_8U, 24. * src.channels() / f, 128);
}

template<class T>
class c_error_history_track
{
public:
  typedef cv::Mat_<T> MatType;

  c_error_history_track(const cv::Size & max_image_size)
  {
    for( int i = 0; i < 3; ++i ) {
      e[i].create(max_image_size);
      p.emplace_back(&e[i]);
    }
  }

  MatType& push_next()
  {
    if( !p.empty() ) {
      q.emplace_back(p.back());
      p.pop_back();
    }
    else {
      MatType *m = q.front();
      q.pop_front();
      q.push_back(m);
    }

    return *q.back();
  }

  MatType& back()
  {
    return *q.back();
  }

  const MatType& back() const
  {
    return *q.back();
  }

  int size() const
  {
    return q.size();
  }

  const std::deque<MatType*>& deque() const
  {
    return q;
  }

protected:
  std::deque<MatType*> q, p;
  MatType e[3];
};

static FILE* create_debug_points_fp(const std::string & debug_directory, int scale, int npoints)
{
  FILE *fp = nullptr;

  if( !debug_directory.empty() ) {

    std::string dbgfilename =
        ssprintf("%s/dbgp.%03d.txt", debug_directory.c_str(),
            scale);

    if( !(fp = fopen(dbgfilename.c_str(), "w")) ) {
      CF_ERROR("fopen('%s') fails: %s", dbgfilename.c_str(),
          strerror(errno));
    }
    else {
      for( int i = 0; i < npoints; ++i ) {
        fprintf(fp, "disp%d\te%d%s", i, i, i == npoints - 1 ? "\n" : "\t");
      }
    }
  }

  return fp;
}

template<class MT>
static void dump_debug_points(FILE * fp, int disparity, int scale, const cv::Mat_<MT> & E,
    const std::vector<cv::Point> & points)
{
  if( fp ) {

    for( int i = 0, n = points.size(); i < n; ++i ) {

      const cv::Point &p =
          points[i];

      const int xx = p.x >> scale;
      const int yy = p.y >> scale;

      const float e =
          xx >= 0 && xx < E.cols && yy >= 0 && yy < E.rows ?
              E[yy][xx] :
              -1.f;

      fprintf(fp, "%d\t%g%s",
          disparity, e,
          i == n - 1 ?
              "\n" :
              "\t");
    }
  }
}

//
//static void compute_gradient(const cv::Mat & src, cv::Mat & gx, cv::Mat & gy)
//{
//  static thread_local const cv::Matx<float, 1, 5> K(
//      (+1.f / 12),
//      (-8.f / 12),
//      0.f,
//      (+8.f / 12),
//      (-1.f / 12));
//
//  cv::filter2D(src, gx, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  gx.convertTo(gx, CV_8U, 0.5, 128);
//
//  cv::filter2D(src, gy, CV_32F, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  gy.convertTo(gy, CV_8U, 0.5, 128);
//}
//
//static void compute_descriptors(const cv::Mat3b & image, cv::Mat1u & desc)
//{
//  cv::Mat s, gx, gy, c;
//
//  cv::GaussianBlur(image, s, cv::Size(11, 11), 2, 1.5, cv::BORDER_REPLICATE);
//  compute_gradient(s, gx, gy);
//  reduce_color_channels(gx, gx, cv::REDUCE_MAX);
//  reduce_color_channels(gy, gy, cv::REDUCE_MAX);
//  cv::cvtColor(s, c, cv::COLOR_BGR2HLS_FULL);
//  cv::cvtColor(s, s, cv::COLOR_BGR2GRAY);
//
//  desc.create(s.size());
//
//  cv::Mat1b gs, ggx, ggy;
//  cv::Mat3b gc;
//
//  for( int y = 0; y < s.rows; ++y ) {
//    for( int x = 0; x < s.cols; ++x ) {
//
//      const uint32_t v =
//          ((uint32_t) gs[y][x] << 24) |
//              ((uint32_t) ggx[y][x] << 16) |
//              ((uint32_t) ggy[y][x] << 8) |
//              ((uint32_t) gc[y][x][0] << 0);
//
//      desc[y][x] = v;
//    }
//  }
//}


} // namespace


template<>
const c_enum_member* members_of<c_sweepscan_stereo_matcher::OutputType>()
{
  static constexpr c_enum_member members[] = {
      { c_sweepscan_stereo_matcher::OutputTextureMap, "TextureMap", "TextureMap" },
      { c_sweepscan_stereo_matcher::OutputTextureMask, "TextureMask", "TextureMask" },
      { c_sweepscan_stereo_matcher::OutputDisparityMap, "DisparityMap", "DisparityMap" },
      { c_sweepscan_stereo_matcher::OutputErrorMap, "ErrorMap", "DisparityMap" },
      { c_sweepscan_stereo_matcher::OutputDisparityMap }
  };

  return members;
}

c_sweepscan_stereo_matcher::c_sweepscan_stereo_matcher()
{
}

void c_sweepscan_stereo_matcher::set_output_type(OutputType v)
{
  output_type_ = v;
}

c_sweepscan_stereo_matcher::OutputType c_sweepscan_stereo_matcher::output_type() const
{
  return output_type_;
}

void c_sweepscan_stereo_matcher::set_max_disparity(int v)
{
  max_disparity_ = v;
}

int c_sweepscan_stereo_matcher::max_disparity() const
{
  return max_disparity_;
}

void c_sweepscan_stereo_matcher::set_ssflags(int v)
{
  ss_flags_ = v;
}

int c_sweepscan_stereo_matcher::ssflags() const
{
  return ss_flags_;
}

void c_sweepscan_stereo_matcher::set_enable_reverse_checks(bool v)
{
  enable_reverse_checks_ = v;
}

bool c_sweepscan_stereo_matcher::enable_reverse_checks() const
{
  return enable_reverse_checks_;
}

void c_sweepscan_stereo_matcher::set_ss_sigma(double v)
{
  ss_sigma_ = v;
}

double c_sweepscan_stereo_matcher::ss_sigma() const
{
  return ss_sigma_;
}

void c_sweepscan_stereo_matcher::set_ss_radius(int v)
{
  ss_radius_ = v;
}

int c_sweepscan_stereo_matcher::ss_radius() const
{
  return ss_radius_;
}

void c_sweepscan_stereo_matcher::set_max_scale(int v)
{
  ss_maxlvl_ = v;
}

int c_sweepscan_stereo_matcher::max_scale() const
{
  return ss_maxlvl_;
}

void c_sweepscan_stereo_matcher::set_kernel_sigma(double v)
{
  kernel_sigma_ = v;
}

double c_sweepscan_stereo_matcher::kernel_sigma() const
{
  return kernel_sigma_;
}

void c_sweepscan_stereo_matcher::set_kernel_radius(int v)
{
  kernel_radius_ = v;
}

int c_sweepscan_stereo_matcher::kernel_radius() const
{
  return kernel_radius_;
}

void c_sweepscan_stereo_matcher::set_normalization_scale(int v)
{
  pscale_ = v;
}

int c_sweepscan_stereo_matcher::normalization_scale() const
{
  return pscale_;
}

void c_sweepscan_stereo_matcher::set_debug_directory(const std::string & v)
{
  debug_directory_ = v;
}

const std::string& c_sweepscan_stereo_matcher::debug_directory() const
{
  return debug_directory_;
}

const std::vector<cv::Point>& c_sweepscan_stereo_matcher::debug_points() const
{
  return debug_points_;
}

void c_sweepscan_stereo_matcher::set_debug_points(const std::vector<cv::Point> & v)
{
  debug_points_ = v;
}

static inline uint8_t absv(uint8_t v)
{
  constexpr uint8_t u128 = (uint8_t) 128;
  return v >= u128 ? v - u128 : 128 - v;
}

bool c_sweepscan_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat & outputImage, cv::Mat1b * outputMask)
{
  INSTRUMENT_REGION("");

  ////////////

  if( currentImage.size() != referenceImage.size() ) {
    CF_ERROR("current (%dx%d) and reference (%dx%d) image sizes not match",
        currentImage.cols(), currentImage.rows(),
        referenceImage.cols(), referenceImage.rows());
    return false;
  }

  if( currentImage.type() != referenceImage.type() ) {
    CF_ERROR("current (%d) and reference (%d) image types not match",
        currentImage.type(), referenceImage.type());
    return false;
  }

  if( !debug_directory_.empty() && !create_path(debug_directory_) ) {
    CF_ERROR("create_path(debug_direcory_='%s') fails: %s",
        debug_directory_.c_str(), strerror(errno));
    return false;
  }


  const cv::Mat images[2] = {
      currentImage.getMat(),
      referenceImage.getMat()
  };

  std::vector<c_ssarray> descs[2];
  cv::Mat1w disps, errs;
  cv::Mat1b texture_map;
  cv::Mat1b texture_mask;

  for( int i = 0; i < 2; ++i ) {
    ssa_pyramid(images[i], descs[i], ss_maxlvl_, ss_flags_);
  }


  {
    INSTRUMENT_REGION("texture_map");
    const c_ssarray &rdesc = descs[1][0];

    texture_map.create(rdesc.size());

    for( int y = 0; y < texture_map.rows; ++y ) {

      const ssdesc *ssp = rdesc[y];

      for( int x = 0; x < texture_map.cols; ++x ) {

        const ssdesc &ss = ssp[x];

        uint8_t maxv = absv(ss.g[0]);
        for( int i = 1; i < 4; ++i ) {
          maxv = std::max(maxv, absv(ss.g[i]));
        }

        texture_map[y][x] = maxv;
      }
    }

    cv::compare(texture_map, 3, texture_mask, cv::CMP_GT);

    if ( output_type_ == OutputTextureMap ) {
      outputImage = texture_map;
      if( outputMask ) {
        *outputMask = texture_mask;
      }
      return true;
    }

    if ( output_type_ == OutputTextureMask ) {
      texture_mask.copyTo(outputImage);
      if( outputMask ) {
        *outputMask = texture_mask;
      }
      return true;
    }

  }

  {
    INSTRUMENT_REGION("ssa_match");
    ssa_match(descs[0], descs[1], max_disparity_, disps, errs, texture_mask, enable_reverse_checks_);
  }

  switch (output_type_) {
    case OutputTextureMap:
      outputImage.release();
      if( outputMask ) {
        outputMask->release();
      }
      break;

    case OutputTextureMask:
      outputImage.release();
      if( outputMask ) {
        outputMask->release();
      }
      break;

    case OutputDisparityMap:
      outputImage = disps;
      break;

    case OutputErrorMap:
      outputImage = errs;
      break;
  }

  return true;
}

//
//bool c_sweepscan_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
//    cv::InputArray referenceImage, cv::InputArray referenceMask,
//    cv::Mat & outputImage, cv::Mat1b * outputMask)
//{
//  INSTRUMENT_REGION("");
//
//  ////////////
//
//  if( currentImage.size() != referenceImage.size() ) {
//    CF_ERROR("current (%dx%d) and reference (%dx%d) image sizes not match",
//        currentImage.cols(), currentImage.rows(),
//        referenceImage.cols(), referenceImage.rows());
//    return false;
//  }
//
//  if( currentImage.type() != referenceImage.type() ) {
//    CF_ERROR("current (%d) and reference (%d) image types not match",
//        currentImage.type(), referenceImage.type());
//    return false;
//  }
//
//  if( !debug_directory_.empty() && !create_path(debug_directory_) ) {
//    CF_ERROR("create_path(debug_direcory_='%s') fails: %s",
//        debug_directory_.c_str(), strerror(errno));
//    return false;
//  }
//
//
//  const cv::Mat images[2] = {
//      currentImage.getMat(),
//      referenceImage.getMat()
//  };
//
//  cv::Mat texture_map;
//  cv::Mat1b texture_mask;
//
//  if( lpg_.k() >= 0 ) {
//
//    INSTRUMENT_REGION("LPG");
//
//    lpg_.create_map(images[1], texture_map);
//    if( texture_map.channels() > 1 ) {
//      reduce_color_channels(texture_map, texture_map, cv::REDUCE_MAX);
//    }
//
//    cv::compare(texture_map, get_triangle_threshold(texture_map),
//        texture_mask,
//        cv::CMP_GT);
//  }
//
//  if( output_type_ == OutputTextureMap ) {
//
//    if( !texture_map.empty() ) {
//      outputImage = texture_map;
//    }
//    else {
//      outputImage.create(images[1].size(), CV_32F);
//      outputImage.setTo(0);
//    }
//
//    if( outputMask ) {
//      if( !texture_mask.empty() ) {
//        *outputMask = texture_mask;
//      }
//      else {
//        outputMask->release();
//      }
//    }
//
//    return true;
//  }
//
//  if( output_type_ == OutputTextureMask ) {
//
//    if( !texture_mask.empty() ) {
//      outputImage = texture_mask;
//    }
//    else {
//      outputImage.create(images[1].size(), CV_8UC1);
//      outputImage.setTo(255);
//    }
//
//    if( outputMask ) {
//      if( !texture_mask.empty() ) {
//        *outputMask = texture_mask;
//      }
//      else {
//        outputMask->release();
//      }
//    }
//
//    return true;
//  }
//
//
//  std::vector<c_ssarray> descs[2];
//  cv::Mat1w disps, errs;
//
//  {
//    INSTRUMENT_REGION("ssa_pyramid");
//    for( int i = 0; i < 2; ++i ) {
//      ssa_pyramid(images[i], descs[i], ss_maxlvl_, ss_flags_);
//    }
//  }
//
//  {
//    INSTRUMENT_REGION("ssa_match");
//    ssa_match(descs[0], descs[1], max_disparity_, disps, errs, texture_mask);
//  }
//
//  switch (output_type_) {
//    case OutputTextureMap:
//      outputImage.release();
//      if( outputMask ) {
//        outputMask->release();
//      }
//      break;
//
//    case OutputTextureMask:
//      outputImage.release();
//      if( outputMask ) {
//        outputMask->release();
//      }
//      break;
//
//    case OutputDisparityMap:
//      outputImage = disps;
//      break;
//
//    case OutputErrorMap:
//      outputImage = errs;
//      break;
//  }
//
//  return true;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////

cv::Ptr<cSweepScanStereoMatcher> cSweepScanStereoMatcher::create()
{
  return Ptr(new this_class());
}

void cSweepScanStereoMatcher::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  cv::Mat1w outputMatches;
  cv::Mat1b outputMask;

  if( base::match(left, cv::noArray(), right, cv::noArray(), outputMatches, &outputMask) ) {
    outputMatches.convertTo(disparity, CV_32F);
  }
  else {
    CF_ERROR("c_scale_sweep_stereo_matcher ::match() fails");
    disparity.release();
  }
}

int cSweepScanStereoMatcher::getMinDisparity() const
{
  return 0;
}

void cSweepScanStereoMatcher::setMinDisparity(int /*minDisparity*/)
{
  // ignore
}

int cSweepScanStereoMatcher::getNumDisparities() const
{
  return base::max_disparity() + 1;
}

void cSweepScanStereoMatcher::setNumDisparities(int numDisparities)
{
  return base::set_max_disparity(std::max(1, numDisparities - 1));
}

int cSweepScanStereoMatcher::getBlockSize() const
{
  return 1;
}

void cSweepScanStereoMatcher::setBlockSize(int /*blockSize*/)
{
  // ignore
}

int cSweepScanStereoMatcher::getSpeckleWindowSize() const
{
  return 0;
}

void cSweepScanStereoMatcher::setSpeckleWindowSize(int /*speckleWindowSize*/)
{
  // ignore
}

int cSweepScanStereoMatcher::getSpeckleRange() const
{
  return 0;
}

void cSweepScanStereoMatcher::setSpeckleRange(int /*speckleRange*/)
{
  // ignore
}

int cSweepScanStereoMatcher::getDisp12MaxDiff() const
{
  return 0;
}

void cSweepScanStereoMatcher::setDisp12MaxDiff(int /*disp12MaxDiff*/)
{
  // ignore
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#if 0

template<class MT>
bool c_sweepscan_stereo_matcher::match_impl(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat & outputImage, cv::Mat1b * outputMask)
{
  INSTRUMENT_REGION("");

  typedef cv::Mat_<MT> MatType;

  const bool enable_debug_point =
      !debug_points_.empty();

  FILE *debug_points_fp = nullptr;

  double ksigma;
  int ksize;

  cv::Mat images[2];
  cv::Mat texture_map, texture_mask;

  images[0] = currentImage.getMat();
  images[1] = referenceImage.getMat();

  if( lpg_.k() >= 0 ) {

    INSTRUMENT_REGION("LPG");

    lpg_.create_map(images[1], texture_map);
    if( texture_map.channels() > 1 ) {
      reduce_color_channels(texture_map, texture_map, cv::REDUCE_MAX);
    }

    cv::compare(texture_map, get_triangle_threshold(texture_map),
        texture_mask,
        cv::CMP_GT);
  }

  if( output_type_ == OutputTextureMap ) {

    if( !texture_map.empty() ) {
      outputImage = texture_map;
    }
    else {
      outputImage.create(images[1].size(), CV_32F);
      outputImage.setTo(0);
    }

    if( outputMask ) {
      if( !texture_mask.empty() ) {
        *outputMask = texture_mask;
      }
      else {
        outputMask->release();
      }
    }

    return true;
  }

  if( output_type_ == OutputTextureMask ) {

    if( !texture_mask.empty() ) {
      outputImage = texture_mask;
    }
    else {
      outputImage.create(images[1].size(), CV_8UC1);
      outputImage.setTo(255);
    }

    if( outputMask ) {
      if( !texture_mask.empty() ) {
        *outputMask = texture_mask;
      }
      else {
        outputMask->release();
      }
    }

    return true;
  }

  if( kernel_radius_ > 0 && kernel_sigma_ > 0 ) {
    ksize = 2 * kernel_radius_ + 1;
    ksigma = kernel_sigma_;
  }
  else if( kernel_radius_ > 0 ) {
    ksize = 2 * kernel_radius_ + 1;
    ksigma = std::max(0.75, kernel_radius_ / 4.);
  }
  else {
    ksize = 7;
    ksigma = 1;
  }

  const cv::Mat1f G =
      cv::getGaussianKernel(ksize, ksigma, CV_32F);

  const cv::Mat &queryImage =
      images[0];

  const cv::Mat &trainImage =
      images[1];

  const cv::Size max_image_size =
      trainImage.size();

  std::deque<cv::Mat> Eq;
  cv::Mat Emin, M1, M2, M3;
  cv::Mat1s D(max_image_size, (int16_t)(-1));

  if ( true ) {

    INSTRUMENT_REGION("SCAN");

    for( int disparity = 0; disparity < max_disparity_; ++disparity ) {

      const cv::Rect qrc(disparity, 0,
          max_image_size.width - disparity,
          max_image_size.height);

      const cv::Rect rrc(0, 0,
          max_image_size.width - disparity,
          max_image_size.height);

      const cv::Mat Q =
          queryImage(qrc);

      const cv::Mat R =
          trainImage(rrc);

      // compute error image
      cv::Mat E;

      {
        INSTRUMENT_REGION("ABSDIFF");
        cv::absdiff(Q, R, E);
      }

      {
        INSTRUMENT_REGION("sepFilter2D");
        cv::sepFilter2D(E, E, E.depth(), G, G, cv::Point(-1, -1), 0,
            cv::BORDER_REPLICATE);
      }

      if( E.channels() > 1 ) {
        INSTRUMENT_REGION("cvtColor");
        cv::cvtColor(E, E, cv::COLOR_BGR2GRAY);
      }

      Eq.emplace_back(E);

      const int N =
          Eq.size();

      if( N == 1 ) {
        E.copyTo(Emin);
      }
      else if( N == 2 ) {

        const cv::Mat &En = Eq.back();
        const cv::Mat Ec = Eq.front()(rrc);

        En.copyTo(Emin(rrc), En < Emin(rrc));
        D(rrc).setTo(disparity - 1, En > Ec);
      }
      else {

        INSTRUMENT_REGION("COMP");

        const cv::Mat &En = Eq[N - 1];
        const cv::Mat Ec = Eq[N - 2](rrc);
        const cv::Mat Ep = Eq[N - 3](rrc);
        cv::Mat Em = Emin(rrc);

        M1 = En < Em;
        En.copyTo(Em, M1);

        D(rrc).setTo(-1, M1);
        D(rrc).setTo(disparity - 1, (En > Ec) & (Ep > Ec) & (Ec == Em) );
      }

      if ( Eq.size() > 2 ) {
        Eq.pop_front();
      }

    }
  }

  if( output_type_ == OutputErrorMap ) {
    Emin.copyTo(outputImage);
  }
  else if( output_type_ == OutputDisparityMap ) {
    D.setTo(-1, ~texture_mask);
    D.copyTo(outputImage);

    if ( outputMask ) {
      *outputMask = D >= 0;
    }
  }

  return true;
}


bool c_sweepscan_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat & outputMatches, cv::Mat1b * outputMask)
{
  ////////////

  if( currentImage.size() != referenceImage.size() ) {
    CF_ERROR("current (%dx%d) and reference (%dx%d) image sizes not match",
        currentImage.cols(), currentImage.rows(),
        referenceImage.cols(), referenceImage.rows());
    return false;
  }

  if( currentImage.type() != referenceImage.type() ) {
    CF_ERROR("current (%d) and reference (%d) image types not match",
        currentImage.type(), referenceImage.type());
    return false;
  }

  if( !debug_directory_.empty() && !create_path(debug_directory_) ) {
    CF_ERROR("create_path(debug_direcory_='%s') fails: %s",
        debug_directory_.c_str(), strerror(errno));
    return false;
  }

  switch (currentImage.depth()) {
    case CV_8U:
      return match_impl<uint8_t>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_8S:
      return match_impl<int8_t>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_16U:
      return match_impl<uint16_t>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_16S:
      return match_impl<int16_t>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_32S:
      return match_impl<int32_t>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_32F:
      return match_impl<float>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    case CV_64F:
      return match_impl<double>(currentImage, currentMask, referenceImage, referenceMask, outputMatches, outputMask);
    default:
      CF_ERROR("ERROR: Not supported image depth %d", currentImage.depth());
      break;
  }

  return false;
}

#endif

