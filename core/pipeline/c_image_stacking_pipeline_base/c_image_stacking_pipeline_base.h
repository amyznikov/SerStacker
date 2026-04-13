/*
 * c_image_stacking_pipeline_base.h
 *
 *  Created on: Mar 20, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_stacking_pipeline_base_h__
#define __c_image_stacking_pipeline_base_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/c_master_frame_selection.h>

struct c_image_stacking_pipeline_base_input_options :
    public c_image_processing_pipeline_input_options
{
  std::string darkbayer_filename;
  std::string flatbayer_filename;
  std::string missing_pixel_mask_filename;

  bool missing_pixels_marked_black = true;
  bool inpaint_missing_pixels = true;
  bool filter_bad_pixels = false;
  bool detect_bad_asi_frames = false;
  bool enable_bground_normalization  = false;

  double bad_asi_frame_median_hat_threshold = 300;
  double bad_pixels_variation_threshold = 9;
  c_histogram_normalization_options background_normalization_options;
  c_image_processor::sptr input_image_processor;
};

class c_image_stacking_pipeline_base :
    public c_image_processing_pipeline
{
public:
  typedef c_image_stacking_pipeline_base this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_image_stacking_pipeline_base(const std::string & name, const c_input_sequence::sptr & input_sequence = nullptr);

protected:
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;

protected:
  bool read_input_frame(const c_input_sequence::sptr & input_sequence,
      const c_image_stacking_pipeline_base_input_options & input_options,
      cv::Mat & output_image, cv::Mat & output_mask,
      bool is_external_master_frame,
      bool save_raw_bayer);

  int select_master_frame(const c_input_sequence::sptr & input_sequence,
      const c_image_stacking_pipeline_base_input_options & input_opts,
      const c_master_frame_selection_options & selection_opts);

protected:
  cv::Mat _darkbayer;
  cv::Mat _flatbayer;
  cv::Mat _raw_bayer_image;
  COLORID _raw_bayer_colorid = COLORID_UNKNOWN;

  cv::Mat _current_master_frame_candidate;
  cv::Mat _current_master_frame_candidate_mask;
};

bool serialize_base_image_stacking_input_options(c_config_setting section, bool save,
    c_image_stacking_pipeline_base_input_options & opts);

template<class RootObjectType>
void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_image_stacking_pipeline_base_input_options> & ctx)
{
  using S = c_image_stacking_pipeline_base_input_options;

  ctlbind(ctls, "start frame index", ctx(&S::start_frame_index), "");
  ctlbind(ctls, "max input frames", ctx(&S::max_input_frames), "");
  ctlbind(ctls, "debayer method", ctx(&S::debayer_method), "");
  ctlbind(ctls, "enable color maxtrix", ctx(&S::enable_color_maxtrix), "");
  ctlbind(ctls, "input_image_processor", ctx(&S::input_image_processor), "");
  ctlbind_browse_for_file(ctls, "Dark frame", ctx(&S::darkbayer_filename), "");
  ctlbind_browse_for_file(ctls, "Flat frame", ctx(&S::flatbayer_filename), "");

  ctlbind_expandable_group(ctls, "missing pixels...", "");
   ctlbind_browse_for_file(ctls, "missing pixel mask", ctx(&S::missing_pixel_mask_filename), "");
   ctlbind(ctls, "inpaint missing pixels", ctx(&S::inpaint_missing_pixels), "");
   ctlbind(ctls, "missing pixels are black", ctx(&S::missing_pixels_marked_black ), "");
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "bad_pixels...", "");
   ctlbind(ctls, "filter_bad_pixels", ctx(&S::filter_bad_pixels), "");
   ctlbind_group(ctls, ctx(&S::filter_bad_pixels));
     ctlbind(ctls, "bad_pixels_variance_threshold", ctx(&S::bad_pixels_variation_threshold), "");
   ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "bad asi frames...", "");
    ctlbind(ctls, "detect bad asi frames", ctx(&S::detect_bad_asi_frames), "");
    ctlbind_group(ctls, ctx(&S::detect_bad_asi_frames));
      ctlbind(ctls, "bad_asi_frame_median_hat_threshold", ctx(&S::bad_asi_frame_median_hat_threshold), "");
    ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "bground normalization...", "");
    ctlbind(ctls, "enable bground normalization ", ctx(&S::enable_bground_normalization ), "");
    ctlbind_group(ctls, ctx(&S::enable_bground_normalization));
     ctlbind(ctls, ctx(&S::background_normalization_options));
    ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);
}

#endif /* __c_image_stacking_pipeline_base_h__ */
