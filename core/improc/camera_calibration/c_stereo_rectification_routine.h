/*
 * c_stereo_rectification_routine.h
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 *
 *  Apply stereo rectification to horizontally laid out stereo frame.
 *
 *  This routine will apply stereo rectification to horizontally laid out input stereo frame.
 *
 *  The rectification remap is constructed from data read from user-provided calibration YML files
 *  created by OpenCV or stereo calibration pipeline.
 *  For actual YML file format see c_stereo_calibration::save_current_camera_parameters()
 *
 */

#pragma once
#ifndef __c_stereo_rectification_routine_h__
#define __c_stereo_rectification_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/stereo/ssdesc.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>
//#include <opencv2/xfeatures2d.hpp>



class c_stereo_rectification_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_stereo_rectification_routine,
      "stereo_rectification",
      "Apply stereo rectification to horizontal layout stereo frame.<br>");

  enum DisplayMode {
    DisplayHLayout,
    DisplayVLayout,
    DisplayBlend,
    DisplayAbsdiff,
    DisplaySSA,
    DisplayBlendSSA,
    DisplayDiffSSA,
  };

  enum SwapFramesMode {
    SwapFramesNone,
    SwapFramesBeforeRectification,
    SwapFramesAfterRectification,
  };

  void set_enable_rectification(bool v)
  {
    enable_rectification_ = v;
  }

  bool enable_rectification() const
  {
    return enable_rectification_;
  }

  void set_intrinsics_filename(const std::string & v)
  {
    stereo_intrinsics_filename_  = v;
    have_stereo_calibration_ = false;
  }

  const std::string & intrinsics_filename() const
  {
    return stereo_intrinsics_filename_;
  }

  void set_extrinsics_filename(const std::string & v)
  {
    stereo_extrinsics_filename_  = v;
    have_stereo_calibration_ = false;
  }

  const std::string & extrinsics_filename() const
  {
    return stereo_extrinsics_filename_;
  }

  void set_swap_frames(SwapFramesMode v)
  {
    swap_frames_ = v;
  }

  SwapFramesMode swap_frames() const
  {
    return swap_frames_;
  }

  void set_display_mode(DisplayMode v)
  {
    display_mode_ = v;
  }

  DisplayMode display_mode() const
  {
    return display_mode_;
  }

  void set_overlay_offset(int v)
  {
    overlay_offset_ = v;
  }

  int overlay_offset() const
  {
    return overlay_offset_;
  }

  void set_ss_sigma(double v)
  {
    ss_sigma_ = v;
  }

  double ss_sigma() const
  {
    return ss_sigma_;
  }

  void set_ss_radius(int v)
  {
    ss_radius_ = v;
  }

  int ss_radius() const
  {
    return ss_radius_;
  }

  void set_ss_maxlvl(int v)
  {
    ss_maxlvl_ = v;
  }

  int ss_maxlvl() const
  {
    return ss_maxlvl_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, enable_rectification, "Enable image rectification");
    BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, intrinsics_filename, "intrinsics_filename", "Stereo intrinsics YML file");
    BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, extrinsics_filename, "extrinsics_filename",  "Stereo extrinsics YML file");
    BIND_PCTRL(ctls, swap_frames, "Swap Left and Right frames");
    BIND_PCTRL(ctls, display_mode, "Overlay two stereo frames into one frame");
    BIND_PCTRL(ctls, ss_sigma, "ss_sigma");
    BIND_PCTRL(ctls, ss_radius, "ss_radius");
    BIND_PCTRL(ctls, ss_maxlvl, "ss_maxlvl");
    BIND_SPINBOX_CTRL(ctls, overlay_offset, 0, 511, 1, "overlay_offset", "Shift left image before overlay");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, enable_rectification);
      SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, extrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, swap_frames);
      SERIALIZE_PROPERTY(settings, save, *this, display_mode);
      SERIALIZE_PROPERTY(settings, save, *this, overlay_offset);
      SERIALIZE_PROPERTY(settings, save, *this, ss_sigma);
      SERIALIZE_PROPERTY(settings, save, *this, ss_radius);
      SERIALIZE_PROPERTY(settings, save, *this, ss_maxlvl);

      return true;
    }

    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask);



protected:
  std::string stereo_intrinsics_filename_;
  std::string stereo_extrinsics_filename_;
  DisplayMode display_mode_ = DisplayHLayout;
  SwapFramesMode swap_frames_ = SwapFramesNone;
  int overlay_offset_ = 0;
  double ss_sigma_ = 2;
  int ss_radius_ = 0;
  int ss_maxlvl_ = 4;

  bool enable_rectification_ = true;
  c_stereo_camera_intrinsics intrinsics_;
  c_stereo_camera_extrinsics extrinsics_;

  bool have_stereo_calibration_ = false;
  cv::Mat2f rmaps[2];

  //cv::Ptr<cv::xfeatures2d::DAISY> daisy_;
};

#endif /* __c_stereo_rectification_routine_h__ */
