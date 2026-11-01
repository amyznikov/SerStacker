/*
 * c_dft_routine.cc
 *
 *  Created on: Nov 1, 2026
 *      Author: amyznikov
 */

#include "c_dft_routine.h"

void c_dft_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "DFT_SCALE", ctx(&this_class::_dftScale), "Append cv:DFT_SCALE flag");
  ctlbind(ctls, "DFT_COMPLEX_OUTPUT", ctx(&this_class::_dftComplexOutput), "Append cv:DFT_COMPLEX_OUTPUT flag");
  ctlbind(ctls, "Polar", ctx(&this_class::_polar), "");
  ctlbind(ctls, "centerDC", ctx(&this_class::_centerDC), "");


//   ctlbind(ctls, "ppsDecomposition", ctx(&this_class::_ppsDecomposition), "Use pps decomposition");
//   ctlbind(ctls, "dftComplex", ctx(&this_class::_dftComplex), "Set cv::DFT_COMPLEX_OUTPUT flag");
//   ctlbind(ctls, "dftScale", ctx(&this_class::_dftScale), "Set cv::DFT_SCALE flag");
//   ctlbind(ctls, "centerDC", ctx(&this_class::_centerDC), "");

}

bool c_dft_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _dftScale);
    SERIALIZE_OPTION(settings, save, *this, _dftComplexOutput);
    SERIALIZE_OPTION(settings, save, *this, _centerDC);
    SERIALIZE_OPTION(settings, save, *this, _polar);
//    SERIALIZE_OPTION(settings, save, *this, _centerDC);
    return true;
  }
  return false;
}

bool c_dft_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.type() != CV_32FC1 ) {
    CF_ERROR("Single-channel input image required");
    return false;
  }

  int flags = 0;
  if ( _dftScale ) {
    flags |= cv::DFT_SCALE;
  }

  if ( _dftComplexOutput ) {
    flags |= cv::DFT_COMPLEX_OUTPUT;
  }

  cv::dft(image, image, flags);

  if ( _polar ) {
    if ( image.channels() == 1 ) {
      fftSpectrumToPolarCCS(image, image, _centerDC);
    }
    else {
      fftSpectrumToPolar(image);
      if ( _centerDC ) {
        fftSwapQuadrants(image);
      }
    }
  }
  else if ( _centerDC && image.channels() == 2 ) {
    fftSwapQuadrants(image);
  }

  return true;
}
