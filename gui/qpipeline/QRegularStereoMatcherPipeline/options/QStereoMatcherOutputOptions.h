/*
 * QStereoMatcherOutputOptions.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherOutputOptions_h__
#define __QStereoMatcherOutputOptions_h__

#include <gui/qpipeline/QPipelineOutputOptions.h>
#include <core/pipeline/c_stereo_matcher_pipeline/c_stereo_matcher_pipeline.h>

class QStereoMatcherOutputOptions :
    public QPipelineOutputOptions<c_stereo_matcher_output_options>
{
public:
  typedef QStereoMatcherOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_stereo_matcher_output_options> Base;

  QStereoMatcherOutputOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

  QCheckBox * save_depthmaps_ctl = nullptr;
  QLineEditBox * depthmaps_filename_ctl = nullptr;

  QCheckBox * save_cloud3d_image_ctl = nullptr;
  QLineEditBox * cloud3d_image_filename_ctl = nullptr;

  QCheckBox * save_cloud3d_ply_ctl = nullptr;
  QLineEditBox * cloud3d_ply_filename_ctl = nullptr;

};

#endif /* __QStereoMatcherOutputOptions_h__ */
