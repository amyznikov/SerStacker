/*
 * c_scale_sweep_stereo_matcher.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "c_scale_sweep_stereo_matcher.h"

#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/proc/array2d.h>
#include <deque>
#include <core/debug.h>

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

}

c_scale_sweep_stereo_matcher::c_scale_sweep_stereo_matcher()
{
}

void c_scale_sweep_stereo_matcher::set_max_disparity(int v)
{
  max_disparity_ = v;
}

int c_scale_sweep_stereo_matcher::max_disparity() const
{
  return max_disparity_;
}

void c_scale_sweep_stereo_matcher::set_max_scale(int v)
{
  max_scale_ = v;
}

int c_scale_sweep_stereo_matcher::max_scale() const
{
  return max_scale_;
}

void c_scale_sweep_stereo_matcher::set_kernel_sigma(double v)
{
  kernel_sigma_ = v;
}

double c_scale_sweep_stereo_matcher::kernel_sigma() const
{
  return kernel_sigma_;
}

void c_scale_sweep_stereo_matcher::set_kernel_radius(int v)
{
  kernel_radius_ = v;
}

int c_scale_sweep_stereo_matcher::kernel_radius() const
{
  return kernel_radius_;
}

void c_scale_sweep_stereo_matcher::set_debug_directory(const std::string & v)
{
  debug_directory_ = v;
}

const std::string& c_scale_sweep_stereo_matcher::debug_directory() const
{
  return debug_directory_;
}

const std::vector<cv::Point>& c_scale_sweep_stereo_matcher::debug_points() const
{
  return debug_points_;
}

void c_scale_sweep_stereo_matcher::set_debug_points(const std::vector<cv::Point> & v)
{
  debug_points_ = v;
}

template<class MT>
bool c_scale_sweep_stereo_matcher::match_impl(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat1w & outputMatches, cv::Mat1b * outputMask)
{
  INSTRUMENT_REGION("");

  typedef cv::Mat_<MT> MatType;

  const bool enable_debug_point =
      !debug_points_.empty();

  FILE *debug_points_fp = nullptr;

  std::vector<cv::Mat> currentImagePyramid;
  std::vector<cv::Mat> referenceImagePyramid;
  std::vector<cv::Mat1b> currentMaskPyramid;
  std::vector<cv::Mat1b> referenceMaskPyramid;
  std::vector<cv::Mat1w> disparityPyramid;

  double ksigma;
  int ksize;

  cv::buildPyramid(currentImage, currentImagePyramid, max_scale_);
  cv::buildPyramid(currentMask.empty() ? cv::Mat1b(currentImage.size(), 255) :
      currentMask, currentMaskPyramid, max_scale_);
  for( uint i = 1, n = currentMaskPyramid.size(); i < n; ++i ) {
    cv::compare(currentMaskPyramid[i], 255, currentMaskPyramid[i], cv::CMP_GE);
  }

  cv::buildPyramid(referenceImage, referenceImagePyramid, max_scale_);
  cv::buildPyramid(referenceMask.empty() ? cv::Mat1b(referenceImage.size(), 255) :
      referenceMask, referenceMaskPyramid, max_scale_);
  for( uint i = 1, n = referenceMaskPyramid.size(); i < n; ++i ) {
    cv::compare(referenceMaskPyramid[i], 255, referenceMaskPyramid[i], cv::CMP_GE);
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

  for( int scale = max_scale_; scale >= 0; --scale ) {

    const cv::Mat &queryImage =
        currentImagePyramid[scale];

    const cv::Mat &trainImage =
        referenceImagePyramid[scale];

    const cv::Mat1b &queryMask =
        currentMaskPyramid[scale];

    cv::Mat1b &trainMask =
        referenceMaskPyramid[scale];

    const cv::Size max_image_size =
        trainImage.size();

    c_error_history_track<MT> track(max_image_size);

    disparityPyramid.emplace_back(
        cv::Mat1w(max_image_size,
            (uint16_t) (UINT16_MAX)));

    cv::Mat1w &disp =
        disparityPyramid.back();

    const int max_disparity =
        max_disparity_ >> scale;

    cv::Mat E, Ef;
    MatType Emin;
    cv::Mat1b M;

    if( enable_debug_point && !debug_directory_.empty() ) {
      if( !create_debug_points_fp(debug_directory_, scale, debug_points_.size()) ) {
        CF_ERROR("create_debug_points_fp() fails");
        return false;
      }
    }

    for( int disparity = 0; disparity < max_disparity; ++disparity ) {

      const cv::Rect qrc(disparity, 0, max_image_size.width - disparity, max_image_size.height);
      const cv::Rect rrc(0, 0, max_image_size.width - disparity, max_image_size.height);

      const cv::Mat Q = queryImage(qrc);
      const cv::Mat1b QM = queryMask(qrc);

      const cv::Mat T = trainImage(rrc);
      const cv::Mat1b TM = trainMask(rrc);

      cv::bitwise_and(QM, TM, M);

      // compute error image
      cv::absdiff(Q, T, E);

      MatType &EE =
          track.push_next();

      if( E.channels() == 1 ) {
        cv::sepFilter2D(E, EE(rrc), E.depth(), G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      }
      else {
        cv::sepFilter2D(E, Ef, E.depth(), G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        cv::cvtColor(Ef, EE(rrc), cv::COLOR_BGR2GRAY);
      }

      dump_debug_points(debug_points_fp,
          disparity,
          scale,
          track.back(),
          debug_points_);

      const std::deque<MatType*> &q =
          track.deque();

      const MatType &c =
          (*q.back());

      const int qsize =
          q.size();

      cv::Mat1w &disp =
          disparityPyramid.back();

      if( qsize == 1 ) {
        c.copyTo(Emin);
      }
      else if( qsize == 2 ) {

        const MatType &p =
            *q[qsize - 2];

        const cv::Mat1w *pdisp =
            disparityPyramid.size() > 1 ?
                &disparityPyramid[disparityPyramid.size() - 2] :
                nullptr;

        for( int y = 0; y < rrc.height; ++y ) {
          for( int x = 0; x < rrc.width; ++x ) {
            if( M[y][x] ) {

              if( c[y][x] < Emin[y][x] ) {
                Emin[y][x] = c[y][x];
              }
              else if( c[y][x] > Emin[y][x] ) {

                if( !pdisp ) {
                  disp[y][x] = disparity - 1;
                }
                else {
                  const int xx = x >> 1;
                  const int yy = y >> 1;
                  const int d = (disparity - 1) >> 1;

                  if( std::abs((int) (*pdisp)[yy][xx] - d) <= 1 ) {
                    disp[y][x] = disparity - 1;
                  }
                }
              }
            }
          }
        }
      }

      else if( qsize == 3 ) {

        const MatType &p =
            *q[qsize - 2];

        const MatType &l =
            *q[qsize - 3];

        const cv::Mat1w *pdisp =
            disparityPyramid.size() > 1 ?
                &disparityPyramid[disparityPyramid.size() - 2] :
                nullptr;

        for( int y = 0; y < rrc.height; ++y ) {
          for( int x = 0; x < rrc.width; ++x ) {
            if( M[y][x] ) {

              if( c[y][x] < Emin[y][x] ) {
                Emin[y][x] = c[y][x];
                disp[y][x] = UINT16_MAX;
              }
              else if( c[y][x] > Emin[y][x] && p[y][x] == Emin[y][x] && l[y][x] > Emin[y][x] ) {

                if( !pdisp ) {
                  disp[y][x] = disparity - 1;
                }
                else {
                  const int xx = x >> 1;
                  const int yy = y >> 1;
                  const int d = (disparity - 1) >> 1;

                  if( std::abs((int) (*pdisp)[yy][xx] - d) <= 1 ) {
                    disp[y][x] = disparity - 1;
                  }
                }
              }
              else if( c[y][x] == Emin[y][x] ) {
                disp[y][x] = UINT16_MAX;
              }
            }
          }
        }
      }

      if( !debug_directory_.empty() ) {

        if( true ) {

          cv::Mat EE(max_image_size, track.back().type(), cv::Scalar::all(0));

          track.back()(rrc).copyTo(EE(rrc));
          save_image(EE, ssprintf("%s/scale%02d/E/E.%04d.tiff",
              debug_directory_.c_str(),
              scale,
              disparity));
        }

        if( true ) {

          cv::Mat B(max_image_size, T.type(), cv::Scalar::all(0));

          cv::addWeighted(Q, 0.5, T, 0.5, 0, B(rrc));

          save_image(B, ssprintf("%s/scale%02d/B/B.%04d.tiff",
              debug_directory_.c_str(),
              scale,
              disparity));

        }

      }
    }

    if( debug_points_fp ) {
      fclose(debug_points_fp);
    }

    cv::Mat1b dispMask = disp < UINT16_MAX;
    disp.setTo(0, ~dispMask);

    if( !debug_directory_.empty() ) {
      save_image(currentImagePyramid[scale], currentMaskPyramid[scale],
          ssprintf("%s/scale%02d/Q.tiff",
              debug_directory_.c_str(),
              scale));
      save_image(referenceImagePyramid[scale], referenceMaskPyramid[scale],
          ssprintf("%s/scale%02d/T.tiff",
              debug_directory_.c_str(),
              scale));

      save_image(disp, dispMask,
          ssprintf("%s/scale%02d/disps.tiff",
              debug_directory_.c_str(),
              scale));

      save_image(Emin, dispMask,
          ssprintf("%s/scale%02d/errs.tiff",
              debug_directory_.c_str(),
              scale));
    }

    if( scale == 0 ) {

      outputMatches =
          disparityPyramid.back();

      if( outputMask ) {
        *outputMask = dispMask;
      }
    }
  }

  return true;
}

bool c_scale_sweep_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat1w & outputMatches, cv::Mat1b * outputMask)
{

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

///////////////////////////////////////////////////////////////////////////////////////////////////

cv::Ptr<cScaleSweepStereoMatcher> cScaleSweepStereoMatcher::create()
{
  return Ptr(new this_class());
}

void cScaleSweepStereoMatcher::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  cv::Mat1w outputMatches;
  cv::Mat1b outputMask;

  base::match(left, cv::noArray(), right, cv::noArray(),
      outputMatches, &outputMask);
}

int cScaleSweepStereoMatcher::getMinDisparity() const
{
  return 0;
}

void cScaleSweepStereoMatcher::setMinDisparity(int /*minDisparity*/)
{
  // ignore
}

int cScaleSweepStereoMatcher::getNumDisparities() const
{
  return base::max_disparity() + 1;
}

void cScaleSweepStereoMatcher::setNumDisparities(int numDisparities)
{
  return base::set_max_disparity(std::max(1, numDisparities - 1));
}

int cScaleSweepStereoMatcher::getBlockSize() const
{
  return 1;
}

void cScaleSweepStereoMatcher::setBlockSize(int /*blockSize*/)
{
  // ignore
}

int cScaleSweepStereoMatcher::getSpeckleWindowSize() const
{
  return 0;
}

void cScaleSweepStereoMatcher::setSpeckleWindowSize(int /*speckleWindowSize*/)
{
  // ignore
}

int cScaleSweepStereoMatcher::getSpeckleRange() const
{
  return 0;
}

void cScaleSweepStereoMatcher::setSpeckleRange(int /*speckleRange*/)
{
  // ignore
}

int cScaleSweepStereoMatcher::getDisp12MaxDiff() const
{
  return 0;
}

void cScaleSweepStereoMatcher::setDisp12MaxDiff(int /*disp12MaxDiff*/)
{
  // ignore
}

///////////////////////////////////////////////////////////////////////////////////////////////////
