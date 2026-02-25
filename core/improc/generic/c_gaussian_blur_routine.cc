/*
 * c_gaussian_filter_routine.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "c_gaussian_blur_routine.h"


template<>
const c_enum_member* members_of<c_gaussian_blur_routine::StereoMode>()
{
  static const c_enum_member members[] = {
      { c_gaussian_blur_routine::StereoNone, "None", "Not at stereo (single frame)" },
      { c_gaussian_blur_routine::StereoHLayout, "HLayout", "Horizontal stereo frame" },
      { c_gaussian_blur_routine::StereoVLayout, "VLayout", "Vertical stereo frame" },
      { c_gaussian_blur_routine::StereoNone },
  };

  return members;
}

void c_gaussian_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "sigmax", ctx(&this_class::_sigmax), "");
   ctlbind(ctls, "sigmay", ctx(&this_class::_sigmay), "");
   ctlbind(ctls, "ksizex", ctx(&this_class::_ksizex), "");
   ctlbind(ctls, "ksizey", ctx(&this_class::_ksizey), "");
   ctlbind(ctls, "stereo_mode", ctx(&this_class::_stereo_mode), "");
   ctlbind(ctls, "border_type", ctx(&this_class::_border_type), "");
   // ctlbind(ctls, "border_value", ctx(&this_class::_border_value), "");
}

bool c_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _sigmax);
    SERIALIZE_OPTION(settings, save, *this, _sigmay);
    SERIALIZE_OPTION(settings, save, *this, _ksizex);
    SERIALIZE_OPTION(settings, save, *this, _ksizey);
    SERIALIZE_OPTION(settings, save, *this, _border_type);
    //  SERIALIZE_OPTION(settings, save, *this, _border_value);
    SERIALIZE_OPTION(settings, save, *this, _stereo_mode);
    return true;
  }
  return false;
}

bool c_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  switch (_stereo_mode) {
    case StereoNone:
      if( _ignore_mask || mask.empty() || cv::countNonZero(mask) == mask.size().area() ) {
        c_gaussian_filter(_sigmax, _sigmay, cv::Size(_ksizex, _ksizey)).
            apply(image.getMat(), cv::noArray(), image, _border_type);
      }
      else {
        cv::Mat tmp;
        image.getMat().copyTo(tmp);
        tmp.setTo(0, ~mask.getMat());
        c_gaussian_filter(_sigmax, _sigmay, cv::Size(_ksizex, _ksizey)).
            apply(tmp, mask, image, _border_type);
      }
      break;

    case StereoHLayout: {

      const cv::Mat src_image =
          image.getMat();

      const cv::Mat src_mask =
          mask.getMat();

      const cv::Size size =
          src_image.size();

      cv::Mat frames[2];
      cv::Mat masks[2];

      src_image(cv::Rect(0, 0, size.width / 2, size.height)).copyTo(frames[0]);
      src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)).copyTo(frames[1]);

      if ( !_ignore_mask && !src_mask.empty() ) {
        src_mask(cv::Rect(0, 0, size.width / 2, size.height)).copyTo(masks[0]);
        src_mask(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)).copyTo(masks[1]);
      }

      for( int i = 0; i < 2; ++i ) {
        if( !masks[i].empty() ) {
          frames[i].setTo(0, ~masks[i]);
        }
        c_gaussian_filter(_sigmax, _sigmay, cv::Size(_ksizex, _ksizey)).
            apply(frames[i], masks[i], frames[i], _border_type);
      }

      frames[0].copyTo(src_image(cv::Rect(0, 0, size.width / 2, size.height)));
      frames[1].copyTo(src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)));

      break;
    }

    case StereoVLayout: {

      const cv::Mat src_image =
          image.getMat();

      const cv::Mat src_mask =
          mask.getMat();

      const cv::Size size =
          src_image.size();

      cv::Mat frames[2];
      cv::Mat masks[2];

      src_image(cv::Rect(0, 0, size.width, size.height / 2)).copyTo(frames[0]);
      src_image(cv::Rect(0, size.height / 2, size.width, size.height - size.width / 2)).copyTo(frames[1]);

      if( !_ignore_mask && !src_mask.empty() ) {
        src_mask(cv::Rect(0, 0, size.width, size.height / 2)).copyTo(masks[0]);
        src_mask(cv::Rect(0, size.height / 2, size.width, size.height - size.width / 2)).copyTo(masks[1]);
      }

      for( int i = 0; i < 2; ++i ) {
        if( !masks[i].empty() ) {
          frames[i].setTo(0, ~masks[i]);
        }
        c_gaussian_filter(_sigmax, _sigmay, cv::Size(_ksizex, _ksizey)).
            apply(frames[i], masks[i], frames[i], _border_type);
      }

      frames[0].copyTo(src_image(cv::Rect(0, 0, size.width / 2, size.height)));
      frames[1].copyTo(src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)));

      break;
    }

    default:
      break;
  }

  return true;
}

