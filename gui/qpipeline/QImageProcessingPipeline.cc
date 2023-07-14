/*
 * QImageProcessingPipeline.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessingPipeline.h"
#include "QImageStackingPipeline/QImageStackingPipeline.h"
#include "QStereoCalibrationPipeline/QStereoCalibrationPipeline.h"
#include "QCameraCalibrationPipeline/QCameraCalibrationPipeline.h"
#include "QRegularStereoMatcherPipeline/QRegularStereoMatcherPipeline.h"
#include "QGenericImageProcessingPipeline/QGenericImageProcessingPipeline.h"
// #include "QVirtualStereoMatcherPipeline/QVirtualStereoMatcherPipeline.h"

#include <core/debug.h>

void registerPipelineClasses()
{
  c_image_processing_pipeline::register_class(
      QImageStackingPipeline::class_name(),
      QImageStackingPipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new QImageStackingPipeline(name.c_str(), input_sequence));
      });

  c_image_processing_pipeline::register_class(
      QCameraCalibrationPipeline::class_name(),
      QCameraCalibrationPipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new QCameraCalibrationPipeline(name.c_str(), input_sequence));
      });

  c_image_processing_pipeline::register_class(
      QStereoCalibrationPipeline::class_name(),
      QStereoCalibrationPipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new QStereoCalibrationPipeline(name.c_str(), input_sequence));
      });

  c_image_processing_pipeline::register_class(
      QRegularStereoMatcherPipeline::class_name(),
      QRegularStereoMatcherPipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new QRegularStereoMatcherPipeline(name.c_str(), input_sequence));
      });

  c_image_processing_pipeline::register_class(
      QGenericImageProcessingPipeline::class_name(),
      QGenericImageProcessingPipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new QGenericImageProcessingPipeline(name.c_str(), input_sequence));
      });


}

