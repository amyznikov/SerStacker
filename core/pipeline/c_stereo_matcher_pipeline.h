/*
 * c_stereo_matcher_pipeline.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_matcher_pipeline_h__
#define __c_stereo_matcher_pipeline_h__

#include "c_image_processing_pipeline.h"
#include "stereo/c_stereo_input.h"
#include <core/pipeline/rstereo/c_regular_stereo.h>

struct c_stereo_matcher_processing_options {

  double camera_focus = 7.215377e+02; // from KITTI
  double stereo_baseline = 0.54;// [m], from KITTI
};


class c_stereo_matcher_pipeline :
    public c_image_processing_pipeline,
    public c_regular_stereo
{
public:
  typedef c_stereo_matcher_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_stereo_matcher_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_stereo_matcher_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "stereo_matcher";

    return classname_;
  }

  c_stereo_input_options & input_options();
  const c_stereo_input_options & input_options() const;

  c_stereo_matcher_processing_options & processing_options();
  const c_stereo_matcher_processing_options & processing_options() const ;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  bool serialize(c_config_setting settings, bool save) override;


protected:
  bool canceled() const override;
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool open_input_source();
  void close_input_source();
  bool seek_input_source(int pos);

protected:
  c_stereo_input_source input_;
  c_stereo_input_options input_options_;
  c_stereo_matcher_processing_options processing_options_;
  mutable std::mutex display_lock_;
};

#endif /* __c_stereo_matcher_pipeline_h__ */
