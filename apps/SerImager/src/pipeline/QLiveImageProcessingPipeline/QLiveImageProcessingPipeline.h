/*
 * QLiveImageProcessingPipeline.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveImageProcessingPipeline_h__
#define __QLiveImageProcessingPipeline_h__

#include "QLivePipeline.h"
#include <core/pipeline/generic/c_generic_image_processor.h>

namespace serimager {

class QLiveImageProcessingPipeline :
    public QLivePipeline,
    public c_generic_image_processor
{
public:
  typedef QLiveImageProcessingPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveImageProcessingPipeline(const QString & name, QObject * parent = nullptr);

  const QString & getClassName() const override
  {
    return className();
  }

  static const QString & className()
  {
    static const QString className_ = "Generic";
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

} // namespace serimager

#endif /* __QLiveImageProcessingPipeline_h__ */
