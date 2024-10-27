/*
 * c_asi_frame_check_routine.cc
 *
 *  Created on: Oct 22, 2024
 *      Author: amyznikov
 */

#include "c_asi_frame_check_routine.h"
#include <core/ssprintf.h>
#include <core/debug.h>



template<>
const c_enum_member* members_of<c_asi_frame_check_routine::BAYER_PATTERN>()
{
  static const c_enum_member members[] = {
      { c_asi_frame_check_routine::BAYER_RGGB, "RGGB", "" },
      { c_asi_frame_check_routine::BAYER_GRBG, "GRBG", "" },
      { c_asi_frame_check_routine::BAYER_GBRG, "GBRG", "" },
      { c_asi_frame_check_routine::BAYER_BGGR, "BGGR", "" },
      { c_asi_frame_check_routine::BAYER_CYYM, "CYYM", "" },
      { c_asi_frame_check_routine::BAYER_YCMY, "YCMY", "" },
      { c_asi_frame_check_routine::BAYER_YMCY, "YMCY", "" },
      { c_asi_frame_check_routine::BAYER_MYYC, "MYYC", "" },
      { c_asi_frame_check_routine::BAYER_RGGB }
  };

  return members;
}

template<>
const c_enum_member* members_of<c_asi_frame_check_routine::DISPLAY_CHANNEL>()
{
  static const c_enum_member members[] = {
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_0, "0", "" },
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_1, "1", "" },
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_2, "2", "" },
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_3, "3", "" },
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_0 },
  };

  return members;
}



void c_asi_frame_check_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, bayer_pattern, "");
  BIND_PCTRL(ctls, display_channel, "");
  BIND_PCTRL(ctls, differentiate, "");

}

bool c_asi_frame_check_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, bayer_pattern);
    SERIALIZE_PROPERTY(settings, save, *this, display_channel);
    SERIALIZE_PROPERTY(settings, save, *this, differentiate);

    return true;
  }
  return false;
}

bool c_asi_frame_check_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat tmp;

  if ( !extract_bayer_planes(image.getMat(), tmp, (COLORID)_bayer_pattern) ) {
    CF_ERROR("extract_bayer_planes() fails");
    return false;
  }

  cv::extractChannel(tmp, image,
      _display_channel);


  if ( _differentiate ) {

    const cv::Mat src =
        image.getMat();

    const cv::Mat src1 =
        src(cv::Rect(0, 0, src.cols, src.rows-1));

    const cv::Mat src2 =
        src(cv::Rect(0, 1, src.cols, src.rows-1));

    cv::absdiff(src1, src2,
        image);
  }


  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
