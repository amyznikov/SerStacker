/*
 * c_asi_frame_check_routine.cc
 *
 *  Created on: Oct 22, 2024
 *      Author: amyznikov
 */

#include "c_asi_frame_check_routine.h"
#include <core/io/debayer.h>
//#include <core/ssprintf.h>
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
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_MAX, "MAX", "" },
      { c_asi_frame_check_routine::DISPLAY_CHANNEL_0 },
  };

  return members;
}


//static bool


void c_asi_frame_check_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, bayer_pattern, "");
  BIND_PCTRL(ctls, display_channel, "");
  BIND_PCTRL(ctls, differentiate, "");
  BIND_PCTRL(ctls, reduce, "");
  BIND_PCTRL(ctls, medianhat, "");
  BIND_PCTRL(ctls, threshold, "");
}

bool c_asi_frame_check_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, bayer_pattern);
    SERIALIZE_PROPERTY(settings, save, *this, display_channel);
    SERIALIZE_PROPERTY(settings, save, *this, differentiate);
    SERIALIZE_PROPERTY(settings, save, *this, reduce);
    SERIALIZE_PROPERTY(settings, save, *this, medianhat);
    SERIALIZE_PROPERTY(settings, save, *this, threshold);

    return true;
  }
  return false;
}

bool c_asi_frame_check_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const bool is_bad_asi_frame =
      is_corrupted_asi_bayer_frame(image.getMat(),
          (COLORID) _bayer_pattern,
          _threshold);

  if ( is_bad_asi_frame ) {
    CF_DEBUG("is_bad_asi_frame=%d",
        is_bad_asi_frame);
  }

  cv::Mat tmp;

  if ( !extract_bayer_planes(image.getMat(), tmp, (COLORID)_bayer_pattern) ) {
    CF_ERROR("extract_bayer_planes() fails");
    return false;
  }

  if ( _display_channel < DISPLAY_CHANNEL_MAX ) {
    cv::extractChannel(tmp, tmp,
        _display_channel);
  }

  if ( _differentiate ) {

    const cv::Mat src =
        tmp;

    const cv::Mat src1 =
        src(cv::Rect(0, 0, src.cols, src.rows-1));

    const cv::Mat src2 =
        src(cv::Rect(0, 1, src.cols, src.rows-1));

    cv::absdiff(src1, src2,
        tmp);
  }

  const cv::Size size =
      tmp.size();

  if ( _reduce ) {

    cv::reduce(tmp, tmp, 1,
        cv::REDUCE_AVG,
        CV_32F);
  }

  if( _medianhat ) {
    cv::Mat mb;
    cv::medianBlur(tmp, mb, 5);
    cv::absdiff(tmp, mb, tmp);
  }


  if( _display_channel == DISPLAY_CHANNEL_MAX ) {
    cv::reduce(tmp.reshape(1, tmp.total()), tmp, 1, cv::REDUCE_MAX);
    tmp = tmp.reshape(0, size.height);
  }



  if ( _threshold > 0 ) {

    cv::compare(tmp, _threshold, tmp,
        cv::CMP_GE);
  }


  if( tmp.cols == size.width ) {
    image.move(tmp);
  }
  else {
    cv::repeat(tmp, 1,
        size.width,
        image);
  }


  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
