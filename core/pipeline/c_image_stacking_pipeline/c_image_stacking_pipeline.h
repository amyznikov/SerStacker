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
#include <core/improc/c_image_processor.h>
#include <core/feature2d/feature2d.h>
#include <core/settings.h>
#include <atomic>


enum roi_selection_method
{
  roi_selection_none = 0,
  roi_selection_planetary_disk = 1,
  roi_selection_rectange_crop = 2
};

enum frame_accumulation_method
{
  frame_accumulation_none = -1,
  frame_accumulation_average = 0,
  frame_accumulation_weighted_average,
  frame_accumulation_focus_stack,
  frame_accumulation_fft,
  frame_accumulation_bayer_average,
};

enum frame_upscale_stage
{
  frame_upscale_stage_unknown = -1,
  frame_upscale_after_align = 1,
  frame_upscale_before_align = 2,
};

enum frame_upscale_option
{
  frame_upscale_none = 0,
  frame_upscale_pyrUp = 1,
  frame_upscale_x15 = 2,
  frame_upscale_x30 = 3,
};


enum STACKING_STAGE
{
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
  int se_close_size = 2;
};


struct c_frame_upscale_options
{
  enum frame_upscale_option upscale_option =
      frame_upscale_none;

  enum frame_upscale_stage upscale_stage =
      frame_upscale_after_align;

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
  double max_weights_ratio = 0;

  c_frame_accumulation_options()
  {
    lpg.dscale = 2;
  }
};

struct c_image_processing_options
{
  c_image_processor::sptr ecc_image_processor;
  c_image_processor::sptr aligned_image_processor;
  c_image_processor::sptr incremental_frame_processor;
  c_image_processor::sptr accumulated_image_processor;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_eccflow_output_frame_writer_options :
    c_output_frame_writer_options
{
  double hsv_max_motion = 0;
};

struct c_image_stacking_output_options  :
    c_image_processing_pipeline_output_options
{
  std::string output_file_name;

  bool dump_reference_data_for_debug = false;
  bool write_image_mask_as_alpha_channel = true;

  bool debug_frame_registration = false;
  std::vector<int> debug_frame_registration_frame_indexes;

  bool save_preprocessed_frames = false;
  bool save_aligned_frames = false;
  bool save_ecc_frames = false;
  bool save_accumulation_masks = false;
  bool save_incremental_frames = false;
  bool save_eccflow_frames = false;
  bool save_sparse_match_blend_frames = false;
  bool save_sparse_matches_video = false;

  c_output_frame_writer_options output_preprocessed_video_options;
  c_output_frame_writer_options output_aligned_video_options;
  c_output_frame_writer_options output_ecc_video_options;
  c_output_frame_writer_options output_acc_masks_video_options;
  c_output_frame_writer_options output_incremental_video_options;
  c_output_frame_writer_options output_sparse_match_blend_options;
  c_output_frame_writer_options output_sparse_matches_video_options;
  c_eccflow_output_frame_writer_options output_eccflow_options;

};

struct c_image_stacking_master_options
{
  struct c_master_frame_selection_options master_selection;
  struct c_image_registration_options registration;
  struct c_frame_accumulation_options accumulation;

  color_channel_type master_channel = color_channel_dont_change;
  int max_frames_to_generate_master_frame = 3000;

  bool apply_input_image_processor = true;
  bool generate_master_frame = true;
  bool stop_after_master_frame_generation = false;
  bool save_master_frame = true;


  double unsharp_sigma = 0;
  double unsharp_alpha = 0.8;

};

struct c_image_stacking_options
{
  c_image_registration_options registration;
  c_frame_accumulation_options accumulation;

  double unsharp_sigma = 0;
  double unsharp_alpha = 0;
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

  c_frame_registration::sptr create_frame_registration(const c_image_registration_options & options) const;

  c_frame_accumulation::ptr create_frame_accumulation(const c_frame_accumulation_options & opts) const;

  c_image_stacking_output_options& output_options();
  const c_image_stacking_output_options& output_options() const;

  c_image_processing_options& image_processing_options();
  const c_image_processing_options& image_processing_options() const;

  std::string output_file_name() const;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  bool serialize(c_config_setting setting, bool save) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

  bool copyParameters(const base::sptr & dst) const override;
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

  // bool run_planetary_disk_derotation();

  bool create_reference_frame(cv::Mat & reference_frame, cv::Mat & reference_mask, double * reference_timestamp);

  bool create_reference_frame(const c_input_sequence::sptr & input_sequence, bool is_external_file,
      int master_frame_pos, int max_frames_to_stack,
      cv::Mat & output_reference_frame, cv::Mat & output_reference_mask,
      double * output_reference_timestamp);

  bool setup_frame_registration(const c_frame_registration::sptr & frame_registration,
      cv::Mat & reference_frame, cv::Mat & reference_mask);

  bool process_input_sequence(const c_input_sequence::sptr & input_sequence,
      int startpos, int endpos);

  int select_master_frame(const c_input_sequence::sptr & input_sequence);

  bool read_input_frame(const c_input_sequence::sptr & input_sequence,
      cv::Mat & output_image, cv::Mat & output_mask,
      bool is_external_master_frame,
      bool save_raw_bayer_image) const;

  static bool select_image_roi(const c_roi_selection::ptr & roi_selection,
      const cv::Mat & src, const cv::Mat & srcmask,
      cv::Mat & dst, cv::Mat & dstmask);

  static bool write_image(const std::string & output_file_name,
      const c_image_stacking_output_options & output_options,
      const cv::Mat & output_image,
      const cv::Mat & output_mask);

  bool save_preprocessed_video(const cv::Mat & current_frame, const cv::Mat & curren_mask, int seqindex);
  bool save_sparse_matches_blend_video(int seqindex);
  bool save_ecc_video(int seqindex);
  bool save_eccflow_video(int seqindex);
  bool save_aligned_video(const cv::Mat & current_frame, const cv::Mat & curren_mask, int seqindex);
  bool save_incremental_video(const cv::Mat & current_frame, const cv::Mat & curren_mask, int seqindex);
  bool save_accumulation_masks_video(const cv::Mat & current_frame, const cv::Mat & curren_mask, int seqindex);
  bool save_sparse_matches_video(int seqindex);

  std::string generate_output_file_name() const;

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

  bool weights_required(const c_frame_accumulation_options & opts) const;
  void compute_weights(const c_frame_accumulation_options & opts, const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst) const;
  static void compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel);
  static double compute_image_noise(const cv::Mat & image, const cv::Mat & mask, color_channel_type channel);

  bool upscale_required(frame_upscale_stage current_stage, bool generating_master_frame) const;

protected:
  c_image_stacking_input_options _input_options;
  c_roi_selection_options _roi_selection_options;
  c_frame_upscale_options _upscale_options;
  c_image_stacking_master_options _master_options;
  c_image_stacking_options _stacking_options;
  c_image_stacking_output_options _output_options;
  c_image_processing_options _image_processing_options;


  std::string _output_file_name;

  cv::Mat selected_master_frame_;
  cv::Mat selected_master_frame_mask_;

  c_anscombe_transform anscombe_;
  c_roi_selection::ptr roi_selection_;
  c_frame_registration::sptr frame_registration_;
  c_frame_weigthed_average::ptr flow_accumulation_;
  c_frame_accumulation::ptr frame_accumulation_;

  mutable std::string output_file_name_postfix_;

  c_output_frame_writer preprocessed_video_writer_;
  c_output_frame_writer sparse_match_blend_writer_;
  c_output_frame_writer ecc_writer_;
  c_output_frame_writer eccflow_writer_;
  c_output_frame_writer aligned_video_writer_;
  c_output_frame_writer incremental_video_writer_;
  c_output_frame_writer accumulation_masks_writer_;
  c_output_frame_writer sparse_matches_video_writer_;

  bool _generating_master_frame = false;
};

#endif /* __c_stacking_pipeline_h__ */
