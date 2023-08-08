/*
 * c_stacking_pipeline.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __c_stacking_pipeline_h__
#define __c_stacking_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/roi_selection/c_roi_rectangle_selection.h>
#include <core/roi_selection/c_planetary_disk_selection.h>
#include <core/average/c_frame_accumulation.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/proc/white_balance/histogram_normalization.h>
#include <core/proc/c_anscombe_transform.h>
#include <core/proc/focus.h>
#include <core/io/c_output_frame_writer.h>
#include <core/improc/c_image_processor.h>
#include <core/feature2d/feature2d.h>
#include <core/settings.h>
#include <atomic>


enum roi_selection_method {
  roi_selection_none = 0,
  roi_selection_planetary_disk = 1,
  roi_selection_rectange_crop = 2
};

enum frame_accumulation_method {
  frame_accumulation_none = -1,
  frame_accumulation_average = 0,
  frame_accumulation_weighted_average,
  frame_accumulation_focus_stack,
  frame_accumulation_fft,
  frame_accumulation_bayer_average,
};

enum frame_upscale_stage {
  frame_upscale_stage_unknown = -1,
  frame_upscale_after_align = 1,
  frame_upscale_before_align = 2,
};

enum frame_upscale_option {
  frame_upscale_none = 0,
  frame_upscale_pyrUp = 1,
  frame_upscale_x15 = 2,
  frame_upscale_x30 = 3,
};


enum STACKING_STAGE {
  stacking_stage_idle = 0,
  stacking_stage_initialize,
  stacking_stage_select_master_frame_index,
  stacking_stage_generate_reference_frame,
  stacking_stage_in_progress,
  stacking_stage_finishing
};

struct c_image_stacking_input_options :
    c_image_processing_pipeline_input_options
{
  enum anscombe_method anscombe = anscombe_none;
};

struct c_roi_selection_options
{
  enum roi_selection_method method = roi_selection_none;
  cv::Size planetary_disk_crop_size;
  cv::Rect rectangle_roi_selection;
  double planetary_disk_gbsigma = 1;
  double planetary_disk_stdev_factor = 0.25;
};


struct c_frame_upscale_options
{
  enum frame_upscale_option upscale_option = frame_upscale_none;
  enum frame_upscale_stage upscale_stage = frame_upscale_after_align;

  bool need_upscale_before_align() const
  {
    return  upscale_option != frame_upscale_none &&
        upscale_stage == frame_upscale_before_align;
  }

  bool need_upscale_after_align() const
  {
    return upscale_option != frame_upscale_none &&
        upscale_stage == frame_upscale_after_align;
  }

  double image_scale() const
  {
    switch (upscale_option) {
    case frame_upscale_x15:
      return 1.5;
    case frame_upscale_pyrUp:
      return 2;
    case frame_upscale_x30:
      return 3;
    default:
      break;
    }
    return 1.0;
  }
};

struct c_frame_accumulation_options
{
  enum frame_accumulation_method accumulation_method  =
      frame_accumulation_average;

  c_lpg_options lpg;
  c_laplacian_pyramid_focus_stacking::options fs;

  c_frame_accumulation_options()
  {
    lpg.dscale = (-1); // disable this feature by default
  }
};

struct c_image_processing_options
{
  c_image_processor::sptr input_image_processor;
  c_image_processor::sptr ecc_image_processor;
  c_image_processor::sptr aligned_image_processor;
  c_image_processor::sptr incremental_frame_processor;
  c_image_processor::sptr accumulated_image_processor;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_image_stacking_output_options  :
    c_image_processing_pipeline_output_options
{
  std::string output_preprocessed_frames_filename;
  std::string output_aligned_frames_filename;
  std::string output_ecc_frames_filename;
  std::string output_postprocessed_frames_filename;
  std::string output_incremental_frames_filename;
  std::string output_accumulation_masks_filename;

  bool save_preprocessed_frames = false;
  bool save_preprocessed_frame_mapping = false;
  bool save_aligned_frames = false;
  bool save_aligned_frame_mapping = false;
  bool save_ecc_frames = false;
  bool save_ecc_frame_mapping = false;
  bool save_processed_aligned_frames = false;
  bool save_processed_aligned_frame_mapping = false;
  bool save_accumulation_masks = false;
  bool save_accumulation_masks_frame_mapping = false;
  bool save_incremental_frames = false;
  bool save_incremental_frame_mapping = false;
  bool dump_reference_data_for_debug = false;
  bool write_image_mask_as_alpha_channel = true;

  bool debug_frame_registration = false;
  std::vector<int> debug_frame_registration_frame_indexes;
};



class c_image_stacking_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_image_stacking_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_image_stacking_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence = nullptr);
  ~c_image_stacking_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "image_stacking";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_image_stacking_pipeline.</strong><br>"
        "Generic pipeline for image stacking<br>";
    return tooltip_;
  }

  const c_anscombe_transform & anscombe() const;

  c_image_stacking_input_options& input_options();
  const c_image_stacking_input_options& input_options() const;

  c_roi_selection_options& roi_selection_options();
  const c_roi_selection_options& roi_selection_options() const;
  c_roi_selection::ptr create_roi_selection() const;

  c_frame_upscale_options& upscale_options();
  const c_frame_upscale_options& upscale_options() const;

  c_sparse_feature_extractor_options& sparse_feature_extractor_options();
  const c_sparse_feature_extractor_options& sparse_feature_extractor_options() const;

  c_sparse_feature_detector_options& sparse_feature_detector_options();
  const c_sparse_feature_detector_options& sparse_feature_detector_options() const;

  c_sparse_feature_descriptor_options& sparse_feature_descriptor_options();
  const c_sparse_feature_descriptor_options& sparse_feature_descriptor_options() const;

  c_master_frame_options& master_frame_options();
  const c_master_frame_options& master_frame_options() const;

  c_image_registration_options& registration_options();
  const c_image_registration_options& registration_options() const;

  c_frame_registration::sptr create_frame_registration(const c_image_registration_options & options) const;
  c_frame_registration::sptr create_frame_registration() const;

  c_frame_accumulation_options& accumulation_options();
  const c_frame_accumulation_options& accumulation_options() const;
  c_frame_accumulation::ptr create_frame_accumulation() const;

  c_image_stacking_output_options& output_options();
  const c_image_stacking_output_options& output_options() const;

  c_image_processing_options& image_processing_options();
  const c_image_processing_options& image_processing_options() const;

  std::string output_file_name() const;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  bool serialize(c_config_setting setting, bool save) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

  bool copyParameters(const c_image_processing_pipeline::sptr & dst) const override;
  bool has_master_frame() const  override;
  void set_master_source(const std::string & master_source_path) override;
  std::string master_source() const override;
  void set_master_frame_index(int v) override;
  int master_frame_index() const override;

protected:
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool run_image_stacking();

  bool create_reference_frame(const c_input_sequence::sptr & input_sequence,
      int master_frame_pos, int max_frames_to_stack,
      cv::Mat & output_reference_frame, cv::Mat & output_reference_mask);

  bool process_input_sequence(const c_input_sequence::sptr & input_sequence,
      int startpos, int endpos);

  int select_master_frame(const c_input_sequence::sptr & input_sequence);

  bool read_input_frame(const c_input_sequence::sptr & input_sequence,
      cv::Mat & output_image, cv::Mat & output_mask,
      bool is_external_master_frame) const;

  static bool select_image_roi(const c_roi_selection::ptr & roi_selection,
      const cv::Mat & src, const cv::Mat & srcmask,
      cv::Mat & dst, cv::Mat & dstmask);

  static bool write_image(const std::string & output_file_name,
      const c_image_stacking_output_options & output_options,
      const cv::Mat & output_image,
      const cv::Mat & output_mask);

  bool save_preprocessed_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex) const;

  bool save_ecc_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex) const;

  bool save_aligned_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex ) const;

  bool save_postprocessed_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex) const;

  bool save_incremental_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex) const;

  bool save_accumulation_mask(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_output_frame_writer & output_writer, int seqindex) const;

  static void remove_bad_pixels(cv::Mat & image,
      const c_image_stacking_input_options & input_optons,
      bool isbayer);

  static void upscale_image(enum frame_upscale_option scale,
      cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst, cv::OutputArray dstmask);

  static void upscale_remap(enum frame_upscale_option scale,
      cv::InputArray srcmap,
      cv::OutputArray dstmap);

  static void upscale_optflow(enum frame_upscale_option scale,
      cv::InputArray srcmap,
      cv::OutputArray dstmap);

  bool weights_required() const;
  void compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst) const;
  static void compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel);
  static double compute_image_noise(const cv::Mat & image, const cv::Mat & mask, color_channel_type channel);

  bool upscale_required(frame_upscale_stage current_stage) const;

protected:

  c_image_stacking_input_options input_options_;
  c_roi_selection_options roi_selection_options_;
  c_frame_upscale_options upscale_options_;
  c_image_registration_options image_registration_options_;
  c_frame_accumulation_options accumulation_options_;
  c_image_stacking_output_options output_options_;
  c_image_processing_options image_processing_options_;

  int master_frame_pos_ = -1;
  bool master_frame_generation_ = false;
  bool external_master_frame_ = false;

  std::string output_file_name_;

  double ecc_normalization_noise_ = 0;
  double reference_sharpness_ = 0;

  cv::Mat selected_master_frame_;
  cv::Mat selected_master_frame_mask_;

  c_anscombe_transform anscombe_;
  c_roi_selection::ptr roi_selection_;
  c_frame_registration::sptr frame_registration_;
  c_frame_weigthed_average::ptr flow_accumulation_;
  mutable std::mutex registration_lock_;

  c_frame_accumulation::ptr frame_accumulation_;
  c_sharpness_norm_measure::ptr sharpness_norm_accumulation_;
  mutable std::mutex accumulator_lock_;

  mutable std::string output_file_name_postfix_;
};

#endif /* __c_stacking_pipeline_h__ */
