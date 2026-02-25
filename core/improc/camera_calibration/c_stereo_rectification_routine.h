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
      "Apply cv::stereoRectify() to horizontally laid-out stereo frame");

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
    _enable_rectification = v;
  }

  bool enable_rectification() const
  {
    return _enable_rectification;
  }

  void set_intrinsics_filename(const std::string & v)
  {
    _stereo_intrinsics_filename  = v;
    _have_stereo_calibration = false;
  }

  const std::string & intrinsics_filename() const
  {
    return _stereo_intrinsics_filename;
  }

  void set_extrinsics_filename(const std::string & v)
  {
    _stereo_extrinsics_filename  = v;
    _have_stereo_calibration = false;
  }

  const std::string & extrinsics_filename() const
  {
    return _stereo_extrinsics_filename;
  }

  void set_swap_frames(SwapFramesMode v)
  {
    _swap_frames = v;
  }

  SwapFramesMode swap_frames() const
  {
    return _swap_frames;
  }

  void set_display_mode(DisplayMode v)
  {
    _display_mode = v;
  }

  DisplayMode display_mode() const
  {
    return _display_mode;
  }

  void set_overlay_offset(int v)
  {
    _overlay_offset = v;
  }

  int overlay_offset() const
  {
    return _overlay_offset;
  }

  void set_ss_sigma(double v)
  {
    _ss_sigma = v;
  }

  double ss_sigma() const
  {
    return _ss_sigma;
  }

  void set_ss_radius(int v)
  {
    _ss_radius = v;
  }

  int ss_radius() const
  {
    return _ss_radius;
  }

  void set_ss_maxlvl(int v)
  {
    _ss_maxlvl = v;
  }

  int ss_maxlvl() const
  {
    return _ss_maxlvl;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _stereo_intrinsics_filename;
  std::string _stereo_extrinsics_filename;
  DisplayMode _display_mode = DisplayHLayout;
  SwapFramesMode _swap_frames = SwapFramesNone;
  int _overlay_offset = 0;
  double _ss_sigma = 2;
  int _ss_radius = 0;
  int _ss_maxlvl = 4;

  bool _enable_rectification = true;
  c_stereo_camera_intrinsics _intrinsics;
  c_stereo_camera_extrinsics _extrinsics;

  bool _have_stereo_calibration = false;
  cv::Mat2f rmaps[2];

  //cv::Ptr<cv::xfeatures2d::DAISY> daisy_;
};

#endif /* __c_stereo_rectification_routine_h__ */
