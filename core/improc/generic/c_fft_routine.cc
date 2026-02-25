/*
 * c_fft_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_fft_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_fft_routine::DisplayType>()
{

  static const c_enum_member members[] = {
      {c_fft_routine::DisplayPower, "Power", "Display power spectrum"},
      {c_fft_routine::DisplayPhase, "Phase", "Display phase spectrum"},
      {c_fft_routine::DisplayReal, "Real", "Display Real part of spectrum"},
      {c_fft_routine::DisplayImag, "Imag", "Display Imaginary part of spectrum"},
      {c_fft_routine::DisplayPower},
  };

  return members;
}

void c_fft_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "output_display", ctx(&this_class::_output_display), "Output image display");
   ctlbind(ctls, "dft_scale", ctx(&this_class::_dft_scale), "Set cv::DFT_SCALE flag");
}

bool c_fft_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _output_display);
    SERIALIZE_OPTION(settings, save, *this, _dft_scale);
    return true;
  }
  return false;
}

bool c_fft_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int cn =
      image.channels();

  cv::Mat channels[cn];

  if( cn == 1 ) {
    image.getMat().convertTo(channels[0], CV_32F);
  }
  else {
    cv::split(image.getMat(), channels);
    for( int c = 0; c < cn; ++c ) {
      channels[c].convertTo(channels[c], CV_32F);
    }
  }

  int flags = cv::DFT_COMPLEX_OUTPUT;
  if( _dft_scale ) {
    flags |= cv::DFT_SCALE;
  }

  for( int c = 0; c < cn; ++c ) {
    cv::dft(channels[c], channels[c], flags);
  }

  switch (_output_display) {
    case DisplayPower:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumPower(channels[c], channels[c]);
      }
      break;

    case DisplayPhase:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumPhase(channels[c], channels[c]);
      }
      break;

    case DisplayReal:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 0);
      }
      break;

    case DisplayImag:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 1);
      }
      break;
  }

  for( int c = 0; c < cn; ++c ) {
    fftSwapQuadrants(channels[c]);
  }

  if ( cn == 1 ) {
    channels[0].copyTo(image);
  }
  else {
    cv::merge(channels, cn, image);
  }

  if ( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}

