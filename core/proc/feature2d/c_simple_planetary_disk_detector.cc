/*
 * c_simple_planetary_disk_detector.cc
 *
 *  Created on: Jun 5, 2022
 *      Author: amyznikov
 */

#include "c_simple_planetary_disk_detector.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/debug.h>

c_simple_planetary_disk_detector::c_simple_planetary_disk_detector(double gsigma, int se_radius) :
  _gsigma(gsigma),
  _se_radius(_se_radius)
{
}

cv::Ptr<c_simple_planetary_disk_detector> c_simple_planetary_disk_detector::create(double gsigma, int se_radius)
{
  return cv::Ptr<this_class>(new c_simple_planetary_disk_detector(gsigma, se_radius));
}

void c_simple_planetary_disk_detector::detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints,
    cv::InputArray _mask)
{
  cv::Point2f centrold;

  CF_DEBUG("simple_planetary_disk_detector: src: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d ",
      _src.cols(), _src.rows(), _src.channels(), _src.depth(),
      _mask.cols(), _mask.rows(), _mask.channels(), _mask.depth());

  bool fOk =
      simple_planetary_disk_detector(
          _src,
          _mask,
          _gsigma,
          _se_radius,
          &centrold,
          &_component_rect,
          nullptr,
          nullptr);

  if( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails. Can not detect planetary disk on given image");
  }
  else {
    //float angle=-1, float response=0, int octave=0, int class_id=-1
    keypoints.emplace_back(cv::KeyPoint(centrold, std::max(_component_rect.width, _component_rect.height)));
    // CF_DEBUG("keypoints.size=%zu", keypoints.size());
  }
}

void c_simple_planetary_disk_detector::compute(cv::InputArray /*image*/,
    CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
    cv::OutputArray descriptors)
{
  cv::Mat1f descs(keypoints.size(), 4);

  for ( int i = 0; i < keypoints.size(); ++i ) {
    const auto & kp = keypoints[i];
    float * __restrict dstp = descs[i];
    dstp[0] = kp.pt.x;
    dstp[1] = kp.pt.y;
    dstp[2] = kp.size;
    dstp[3] = kp.octave;
  }
  descriptors.move(descs);
}

void c_simple_planetary_disk_detector::detectAndCompute(cv::InputArray image, cv::InputArray mask,
    CV_OUT std::vector<cv::KeyPoint> & keypoints,
    cv::OutputArray descriptors,
    bool useProvidedKeypoints /*= false*/)
{
  if ( !useProvidedKeypoints ) {
    keypoints.clear();
    detect(image, keypoints, mask);
  }
  compute(image, keypoints, descriptors);
}
