/*
 * c_local_peak_routine.cc
 *
 *  Created on: Mar 16, 2026
 *      Author: amyznikov
 */

#include "c_local_peak_routine.h"


template<>
const c_enum_member* members_of<c_local_peak_routine::Direction>()
{
  static const c_enum_member members[] = {
      { c_local_peak_routine::DirectionVert, "Vert", ""},
      { c_local_peak_routine::DirectionHorz, "Horz"},
//      { c_local_peak_routine::DirectionBoth, "Both"},
      { c_local_peak_routine::DirectionVert},
  };

  return members;
}

template<>
const c_enum_member* members_of<c_local_peak_routine::Output>()
{
  static const c_enum_member members[] = {
      { c_local_peak_routine::Output1, "Output1", ""},
      { c_local_peak_routine::Output2, "Output2"},
      { c_local_peak_routine::OutputProd, "Prod"},
      { c_local_peak_routine::OutputProd},
  };

  return members;
}

static cv::Mat1f createPeakSearchKernel(int peak_radius)
{
  const int ksize = 6 * peak_radius + 1;
  const int center = ksize / 2;

  cv::Mat1f K = cv::Mat1f::zeros(1, ksize);

  for( int i = 0; i < ksize; ++i ) {
    if( i == center ) {
      continue;
    }
    // Linear attenuation towards the edges:
    const float dist = (float) std::abs(i - center);
    const float weight = 1.0f - (dist / (ksize / 2.0f));
    K(0, i)  = i < center ? -weight : +weight;
  }

  // Norm the L1 to 2.0 to preserve the amplitude
  const float pos_sum = cv::sum(cv::max(K, 0))[0];
  if( pos_sum > 0 ) {
    K /= pos_sum;
  }

  return K;
}


static void processPeaks(cv::InputArray src, cv::OutputArray finalResponse, int peak_radius, int direction, int output, bool applyMax)
{
  cv::Mat res1, res2;

  cv::Mat1f K = createPeakSearchKernel(peak_radius);
  const int ksize = K.cols;

  if ( direction == c_local_peak_routine::DirectionHorz ) {
    cv::filter2D(src, res1, CV_32F, K, cv::Point(ksize / 2 + peak_radius, 0), 0, cv::BORDER_REPLICATE);
    cv::filter2D(src, res2, CV_32F, -K, cv::Point(ksize / 2 - peak_radius, 0), 0, cv::BORDER_REPLICATE);
  }
  else if ( direction == c_local_peak_routine::DirectionVert ) {
    K = K.t();
    cv::filter2D(src, res1, CV_32F, K, cv::Point(0, ksize / 2 + peak_radius), 0, cv::BORDER_REPLICATE);
    cv::filter2D(src, res2, CV_32F, -K, cv::Point(0, ksize / 2 - peak_radius), 0, cv::BORDER_REPLICATE);
  }
  else { // c_local_peak_routine::DirectionBoth
  }

  // Remove negative values ​​(we are only interested in growth on the left and decline on the right)
  if ( applyMax ) {
    cv::max(res1, cv::Scalar::all(0), res1);
    cv::max(res2, cv::Scalar::all(0), res2);
  }

  // Calculate the full response.
  // Multiplication (multiply) works like a "soft AND" and greatly enhances the peak contrast.
  // If you want to preserve the original amplitude scale, you can use cv::min(res1, res2).
  if ( output == c_local_peak_routine::Output1 ) {
    finalResponse.move(res1);
  }
  else if ( output == c_local_peak_routine::Output2 ) {
    finalResponse.move(res2);
  }
  else { // OutputProduct
    cv::multiply(res1, res2, finalResponse);
    cv::sqrt(finalResponse, finalResponse);
  }
}

void c_local_peak_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "peak_radius", ctx(&this_class::_peak_radius), "peak_radius");
  ctlbind(ctls, "direction", ctx(&this_class::_direction), "direction");
  ctlbind(ctls, "output", ctx(&this_class::_output), "output");
  ctlbind(ctls, "applyMax", ctx(&this_class::_applyMax), "applyMax");
}

bool c_local_peak_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _peak_radius);
    SERIALIZE_OPTION(settings, save, *this, _direction);
    SERIALIZE_OPTION(settings, save, *this, _output);
    SERIALIZE_OPTION(settings, save, *this, _applyMax);
    return true;
  }
  return false;
}

bool c_local_peak_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  processPeaks(image.getMat(), image, _peak_radius, _direction, _output, _applyMax);
  return true;
}

