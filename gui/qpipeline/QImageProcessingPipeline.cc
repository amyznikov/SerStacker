/*
 * QImageProcessingPipeline.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessingPipeline.h"
#include "QOutputFrameWriterOptions.h"
#include <core/pipeline/c_generic_image_processor_pipeline/c_generic_image_processor_pipeline.h>
#include <core/pipeline/c_dkgen_pipeline/c_dkgen_pipeline.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>
#include <core/pipeline/c_camera_calibration_pipeline/c_camera_calibration_pipeline.h>
#include <core/pipeline/c_stereo_calibration_pipeline/c_stereo_calibration_pipeline.h>
#include <core/pipeline/c_stereo_matcher_pipeline/c_stereo_matcher_pipeline.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>
#include <core/pipeline/c_running_average_pipeline/c_running_average_pipeline.h>
#include <core/pipeline/c_virtual_stereo_pipeline/c_virtual_stereo_pipeline.h>
#include <core/pipeline/c_roi_tracker_pipeline/c_roi_tracker_pipeline.h>
#include <core/pipeline/c_epipolar_alignment_pipeline/c_epipolar_alignment_pipeline.h>
#include <core/pipeline/c_cte_pipeline/c_cte_pipeline.h>
#include <core/pipeline/c_jdr_pipeline/c_jdr_pipeline.h>


void registerPipelineClasses()
{
#define REGISTER_PIPELINE_CLASS(C) \
    c_image_processing_pipeline::register_class(C::class_name(), C::tooltip(), \
        [](const std::string & name, const c_input_sequence::sptr & input_sequence) { \
          using Q = QImageProcessingPipelineTemplate<C>; \
          return c_image_processing_pipeline::sptr(new Q(QString::fromStdString(name), input_sequence)); \
        });

  REGISTER_PIPELINE_CLASS(c_generic_image_processor_pipeline);
  REGISTER_PIPELINE_CLASS(c_dkgen_pipeline);
  REGISTER_PIPELINE_CLASS(c_image_stacking_pipeline);
  REGISTER_PIPELINE_CLASS(c_camera_calibration_pipeline);
  REGISTER_PIPELINE_CLASS(c_stereo_calibration_pipeline);
  REGISTER_PIPELINE_CLASS(c_stereo_matcher_pipeline);
  REGISTER_PIPELINE_CLASS(c_live_stacking_pipeline);
  REGISTER_PIPELINE_CLASS(c_running_average_pipeline);
  REGISTER_PIPELINE_CLASS(c_virtual_stereo_pipeline);
  REGISTER_PIPELINE_CLASS(c_roi_tracker_pipeline);
  REGISTER_PIPELINE_CLASS(c_epipolar_alignment_pipeline);
  REGISTER_PIPELINE_CLASS(c_cte_pipeline);
  REGISTER_PIPELINE_CLASS(c_jdr_pipeline);

#undef REGISTER_PIPELINE_CLASS
}


///////////////////////////////////////////////////////////////////////////////
