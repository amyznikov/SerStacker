/*
 * c_fft_routine.h
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_routine_h__
#define __c_fft_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/fft.h>

class c_fft_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_routine,
      "fft", "Display fft spectrum from cv::dft()");

  enum DisplayType {
    DisplayPower,
    DisplayPhase,
    DisplayReal,
    DisplayImag,
  };

  void set_display(DisplayType v)
  {
    display_type_ = v;
  }

  DisplayType display() const
  {
    return display_type_;
  }

  void set_dft_scale(bool v)
  {
    dft_scale_ = v;
  }

  bool dft_scale() const
  {
    return dft_scale_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display, "Specify output type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, dft_scale, "Set cv::DFT_SCALE flag");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, display);
      SERIALIZE_PROPERTY(settings, save, *this, dft_scale);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
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
    if( dft_scale_ ) {
      flags |= cv::DFT_SCALE;
    }

    for( int c = 0; c < cn; ++c ) {
      cv::dft(channels[c], channels[c], flags);
    }

    switch (display_type_) {
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

protected:
  DisplayType display_type_ = DisplayPower;
  bool dft_scale_ = false;
  //cv::BorderTypes borderType_ = cv::BORDER_REFLECT101;

};

#endif /* __c_fft_routine_h__ */
