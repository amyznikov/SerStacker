/*
 * QLiveRegularStereoPipeline.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveRegularStereoPipeline_h__
#define __QLiveRegularStereoPipeline_h__

#include "QLivePipeline.h"
#include <core/pipeline/rstereo/c_regular_stereo.h>

namespace serimager {

class QLiveRegularStereoPipeline :
    public QLivePipeline
{
public:
  typedef QLiveRegularStereoPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveRegularStereoPipeline(const QString & name, QObject * parent = nullptr);

  c_regular_stereo & rstereo();
  const c_regular_stereo & rstereo() const;

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:
  c_regular_stereo rstereo_;

};

} /* namespace serimager */

#endif /* __QLiveRegularStereoPipeline_h__ */