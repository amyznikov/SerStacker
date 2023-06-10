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

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  c_melp_stereo_matcher m;
};

#endif /* __c_melp_stereo_matcher_routine_h__ */
