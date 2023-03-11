/*
 * c_regular_stereo_matcher.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "c_regular_stereo_matcher.h"
#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
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
}

c_regular_stereo_matcher::c_regular_stereo_matcher()
{
}


void c_regular_stereo_matcher::set_max_disparity(int v)
{
  max_disparity_ = v;
}

int c_regular_stereo_matcher::max_disparity() const
{
  return max_disparity_;
}

void c_regular_stereo_matcher::set_max_scale(int v)
{
  max_scale_ = v;
}

int c_regular_stereo_matcher::max_scale() const
{
  return max_scale_;
}

void c_regular_stereo_matcher::set_debug_direcory(const std::string & v)
{
  debug_direcory_ = v;
}

const std::string & c_regular_stereo_matcher::debug_direcory() const
{
  return debug_direcory_;
}

bool c_regular_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat2f & outputMatches)
{
  INSTRUMENT_REGION("");

  if( currentImage.size() != referenceImage.size() ) {
    CF_ERROR("current (%dx%d) and reference (%dxd) image sizes not match",
        currentImage.cols(), currentImage.rows(),
        referenceImage.cols(), referenceImage.rows());
    return false;
  }

  if( !debug_direcory_.empty() && !create_path(debug_direcory_) ) {
    CF_ERROR("create_path(debug_direcory_='%s') fails: %s",
        debug_direcory_.c_str(), strerror(errno));
    return false;
  }

  const cv::Size size =
      currentImage.size();

  const cv::Mat queryImage =
      currentImage.getMat();

  const cv::Mat1b queryMask =
      currentMask.empty() ? cv::Mat1b(queryImage.size(), 255) :
          currentMask.getMat();

  const cv::Mat trainImage =
      referenceImage.getMat();

  const cv::Mat1b trainMask =
      referenceMask.empty() ? cv::Mat1b(trainImage.size(), 255) :
          referenceMask.getMat();

  const cv::Mat1f G =
      cv::getGaussianKernel(7, 1, CV_32F);

  cv::Mat D, Dmin;
  cv::Mat1b DM;
  cv::Mat1b M;
  cv::Mat1b Disp;

  for( int disparity = 0; disparity < max_disparity_; ++disparity ) {

    const cv::Rect qrc(disparity, 0, size.width - disparity, size.height);
    const cv::Rect rrc(0, 0, size.width - disparity, size.height);

    const cv::Mat Q = queryImage(qrc);
    const cv::Mat1b QM = queryMask(qrc);

    const cv::Mat R = trainImage(rrc);
    const cv::Mat1b RM = trainMask(rrc);

    cv::absdiff(Q, R, D);
    cv::bitwise_and(QM, RM, DM);
    cv::sepFilter2D(D, D, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    if( D.channels() != 1 ) {
      cv::cvtColor(D, D, cv::COLOR_BGR2GRAY);
    }

    if ( Dmin.empty() ) {
      Dmin.create(size, D.depth());
      Dmin.setTo(maxval_for_pixel_depth(D.depth()));
      Disp.create(size);
      Disp.setTo(0);
    }

    cv::compare(D, Dmin(rrc), M, cv::CMP_LT);
    D.copyTo(Dmin(rrc), M);
    Disp(rrc).setTo(disparity, M);

    if( !debug_direcory_.empty() ) {
      save_image(D, ssprintf("%s/D.%04d.tiff",
          debug_direcory_.c_str(), disparity));
    }
  }

  if( !debug_direcory_.empty() ) {
    save_image(Disp, ssprintf("%s/Disparity.tiff",
        debug_direcory_.c_str()));
  }

  return true;
}
