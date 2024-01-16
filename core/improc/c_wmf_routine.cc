/*
 * c_wmf_routine.cc
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#include "c_wmf_routine.h"
#include <core/proc/normalize.h>

template<>
const c_enum_member * members_of<cv::ximgproc::WMFWeightType>()
{
  using namespace cv::ximgproc;

  static const c_enum_member members[] = {
      {WMF_OFF, "OFF", "unweighted"},
      {WMF_EXP, "EXP", "exp(-|I1-I2|^2/(2*sigma^2))"},
      {WMF_IV1, "IV1", "(|I1-I2|+sigma)^-1"},
      {WMF_IV2, "IV2", "(|I1-I2|^2+sigma^2)^-1"},
      {WMF_COS, "COS", "dot(I1,I2)/(|I1|*|I2|)"},
      {WMF_JAC, "JAC", "(min(r1,r2)+min(g1,g2)+min(b1,b2))/(max(r1,r2)+max(g1,g2)+max(b1,b2))"},
      {WMF_OFF}, // must be last
  };

  return members;
}

bool c_wmf_routine::process(cv::InputOutputArray _image, cv::InputOutputArray _mask)
{
  cv::Mat joint;
  cv::Mat mask;

  if ( !ignore_mask_ ) {
    mask = _mask.getMat();
  }

  if( weightType_ == cv::ximgproc::WMF_OFF ) {
    joint = cv::Mat1b::ones(_image.size());
  }
  else {
    switch (_image.depth()) {
    case CV_8U:
      case CV_8S:
      joint = _image.getMatRef();
      break;
    default:
      normalize_meanStdDev(_image.getMat(), joint, 6, 0, 255, _mask, cv::BORDER_CONSTANT);
      joint.convertTo(joint, CV_8U);
      break;
    }
  }

  cv::ximgproc::weightedMedianFilter(joint,
      _image,
      _image,
      radius_,
      sigma_,
      weightType_,
      mask);

  return true;
}

