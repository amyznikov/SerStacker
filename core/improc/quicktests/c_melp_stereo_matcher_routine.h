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
    displayType_ = v;
  }

  DisplayType displayType() const
  {
    return displayType_;
  }

  void set_displaypos(const std::vector<int> & v)
  {
    displaypos_ = v;
  }

  const std::vector<int> & displaypos() const
  {
    return displaypos_;
  }

  void set_overlay_offset(int v)
  {
    overlay_offset_ = v;
  }

  int overlay_offset() const
  {
    return overlay_offset_;
  }

  void set_minimum_image_size(int v)
  {
    m.set_minimum_image_size(v);
  }

  int minimum_image_size() const
  {
    return m.minimum_image_size();
  }

  void set_texture_threshold(double v)
  {
    m.set_texture_threshold(v);
  }

  double texture_threshold() const
  {
    return m.texture_threshold();
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  c_melp_stereo_matcher m;
  std::vector<int> displaypos_;
  DisplayType displayType_ = DisplayDisparity;
  int overlay_offset_ = 0;
};

#endif /* __c_melp_stereo_matcher_routine_h__ */
