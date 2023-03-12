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

  std::vector<cv::Mat> currentImagePyramid;
  std::vector<cv::Mat> referenceImagePyramid;
  std::vector<cv::Mat1b> currentMaskPyramid;
  std::vector<cv::Mat1b> referenceMaskPyramid;

  std::vector<cv::Mat1b> dispPyramid;

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

  static const cv::Mat1f G =
      cv::getGaussianKernel(7, 1, CV_32F);

  for( uint scale = 0, nscales = currentImagePyramid.size(); scale < nscales; ++scale ) {

    const cv::Mat &queryImage =
        currentImagePyramid[scale];

    const cv::Mat &trainImage =
        referenceImagePyramid[scale];

    const cv::Mat1b &queryMask =
        currentMaskPyramid[scale];

    const cv::Mat1b &trainMask =
        referenceMaskPyramid[scale];

    const cv::Size size =
        queryImage.size();

    const int max_disparity =
        max_disparity_ / (1 << scale);

    cv::Mat E, Emin;
    cv::Mat1b M, EM;

    dispPyramid.emplace_back(size, (uint8_t) (max_disparity_));


    for( int disparity = 0; disparity < max_disparity; ++disparity ) {

      const cv::Rect qrc(disparity, 0, size.width - disparity, size.height);
      const cv::Rect rrc(0, 0, size.width - disparity, size.height);

      const cv::Mat Q = queryImage(qrc);
      const cv::Mat1b QM = queryMask(qrc);

      const cv::Mat T = trainImage(rrc);
      const cv::Mat1b TM = trainMask(rrc);

      // compute error image
      cv::absdiff(Q, T, E);
      cv::sepFilter2D(E, E, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      if( E.channels() != 1 ) {
        cv::cvtColor(E, E, cv::COLOR_BGR2GRAY);
      }

      // create minimum error image if empty
      if( disparity == 0 ) {
        Emin.create(size, E.depth());
        Emin.setTo(maxval_for_pixel_depth(E.depth()));
      }


      // compare current and minimum error images
      cv::compare(E, Emin(rrc), M, cv::CMP_LT);
      cv::bitwise_and(QM, M, M);
      cv::bitwise_and(TM, M, M);
      E.copyTo(Emin(rrc), M);
      dispPyramid[scale](rrc).setTo(disparity, M);

      if( !debug_direcory_.empty() ) {
        save_image(E, ssprintf("%s/scale%02d/D.%04d.tiff",
            debug_direcory_.c_str(),
            scale,
            disparity));
      }
    }

    if( !debug_direcory_.empty() ) {
      save_image(dispPyramid[scale], ssprintf("%s/scale%02d/Disparity.tiff",
          debug_direcory_.c_str(),
          scale));
      save_image(currentImagePyramid[scale], currentMaskPyramid[scale],
          ssprintf("%s/scale%02d/Q.tiff",
              debug_direcory_.c_str(),
              scale));
      save_image(referenceImagePyramid[scale], referenceMaskPyramid[scale],
          ssprintf("%s/scale%02d/T.tiff",
              debug_direcory_.c_str(),
              scale));
    }
  }


  return true;
}
