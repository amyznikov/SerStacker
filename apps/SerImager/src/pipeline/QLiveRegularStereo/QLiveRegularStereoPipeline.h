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
#if 0
#include <core/pipeline/rstereo/c_regular_stereo.h>

namespace serimager {

class QLiveRegularStereoPipeline :
    public QLivePipeline,
    public c_regular_stereo
{
public:
  typedef QLiveRegularStereoPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveRegularStereoPipeline(const QString & name, QObject * parent = nullptr);

  const QString & getClassName() const override
  {
    return className();
  }

  static const QString & className()
  {
    static const QString className_ = "RegularStereo";
    return className_;
  }

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:
  COLORID displayColorid_ = COLORID_UNKNOWN;
};

} /* namespace serimager */

#endif
#endif /* __QLiveRegularStereoPipeline_h__ */
