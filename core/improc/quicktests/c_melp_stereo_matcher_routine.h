/*
 * c_melp_stereo_matcher_routine.h
 *
 *  Created on: Jun 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_melp_stereo_matcher_routine_h__
#define __c_melp_stereo_matcher_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/stereo/c_melp_stereo_matcher.h>

class c_melp_stereo_matcher_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_melp_stereo_matcher_routine,
      "melp_stereo", "c_melp_stereo_matcher_routine");

  enum DisplayType {
    DisplayHlayout,
    DisplayVlayout,
    DisplayBlend,
    DisplayAbsdiff,
    DisplaySAD,
    DisplayDisparity,
    DisplayMM0,
    DisplayMM1,
    DisplayTextureMap,
    DisplayTextureMask,
  };

  void set_displayType(DisplayType v)
  {
    _displayType = v;
  }

  DisplayType displayType() const
  {
    return _displayType;
  }

  void set_displaypos(const std::vector<int> & v)
  {
    _displaypos = v;
  }

  const std::vector<int> & displaypos() const
  {
    return _displaypos;
  }

  void set_overlay_offset(int v)
  {
    _overlay_offset = v;
  }

  int overlay_offset() const
  {
    return _overlay_offset;
  }

  void set_minimum_image_size(int v)
  {
    _m.set_minimum_image_size(v);
  }

  int minimum_image_size() const
  {
    return _m.minimum_image_size();
  }

  void set_texture_threshold(double v)
  {
    _m.set_texture_threshold(v);
  }

  double texture_threshold() const
  {
    return _m.texture_threshold();
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_melp_stereo_matcher _m;
  std::vector<int> _displaypos;
  DisplayType _displayType = DisplayDisparity;
  int _overlay_offset = 0;
};

#endif /* __c_melp_stereo_matcher_routine_h__ */
