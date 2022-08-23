/*
 * c_stacking_pipeline.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __c_stacking_pipeline_h__
#define __c_stacking_pipeline_h__

#include <core/io/c_input_sequence.h>
#include <core/roi_selection/c_roi_rectangle_selection.h>
#include <core/roi_selection/c_planetary_disk_selection.h>
#include <core/average/c_frame_accumulation.h>
#include <core/registration/c_frame_registration.h>
#include <core/proc/c_anscombe_transform.h>
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
  frame_accumulation_weighted_average = 0,
  frame_accumulation_fft = 1,
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

struct c_input_options
{
  std::string missing_pixel_mask_filename;
  bool missing_pixels_marked_black = true;
  bool inpaint_missing_pixels = true;

  bool filter_bad_pixels = true;
  bool drop_bad_asi_frames = true;
  bool enable_color_maxtrix = false;

  enum anscombe_method anscombe = anscombe_none;
  double hot_pixels_variation_threshold = 6;

  int start_frame_index = 0;
  int max_input_frames = 0;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_roi_selection_options
{
  enum roi_selection_method method = roi_selection_none;
  cv::Size planetary_disk_crop_size;
  cv::Rect rectangle_roi_selection;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_master_frame_options
{
  std::string master_source_path;
  int master_frame_index = 0; // relative, in master source
  bool apply_input_frame_processor = true;
  bool generate_master_frame = true;
  int max_input_frames_to_generate_master_frame = 1000;
  int eccflow_scale = 5;
  double master_sharpen_factor = 0.5;
  double accumulated_sharpen_factor = 1;
  bool save_master_frame = true;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_frame_upscale_options
{
  enum frame_upscale_option upscale_option = frame_upscale_none;
  enum frame_upscale_stage upscale_stage = frame_upscale_after_align;

  bool need_upscale_before_align() const {
    return  upscale_option != frame_upscale_none && upscale_stage == frame_upscale_before_align;
  }

  bool need_upscale_after_align() const {
    return  upscale_option != frame_upscale_none && upscale_stage == frame_upscale_after_align;
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


  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);

};

struct c_frame_accumulation_options
{
  enum frame_accumulation_method accumulation_method  =
      frame_accumulation_weighted_average;

  int lksize = 0;
  int scale_size = 3;
  double minv = 1;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};


struct c_frame_registration_options
{
  c_master_frame_options master_frame_options;
  c_image_registration_options image_registration_options;
  bool accumulate_and_compensate_turbulent_flow = true;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};


struct c_image_processing_options
{
  c_image_processor::ptr input_image_processor;
  c_image_processor::ptr ecc_image_processor;
  c_image_processor::ptr aligned_image_processor;
  c_image_processor::ptr accumulated_image_processor;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};

struct c_image_stacking_output_options {

  std::string output_directory;

  std::string output_preprocessed_frames_filename;
  std::string output_aligned_frames_filename;
  std::string output_postprocessed_frames_filename;
  std::string output_accumulation_masks_filename;

  bool save_preprocessed_frames = false;
  bool save_aligned_frames = false;
  bool save_postprocessed_frames = false;
  bool save_accumulation_masks = false;
  bool dump_reference_data_for_debug = false;
  bool write_image_mask_as_alpha_channel = true;

  bool debug_frame_registration = false;
  std::vector<int> debug_frame_registration_frame_indexes;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);
};


class c_image_stacking_options
{
public:
  typedef c_image_stacking_options this_class;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create(const std::string & name = "");

  void set_name(const std::string & name);
  const std::string & name() const;
  const char * cname() const;

  void set_input_sequence(const c_input_sequence::ptr & sequence);
  const c_input_sequence::ptr & input_sequence() const;

  const std::vector<c_input_source::ptr> & input_sources() const;
  c_input_source::ptr add_input_source(const std::string & pathfilename);
  bool add_input_sources(const std::vector<std::string> & pathfilenames);
  void remove_input_source(const c_input_source::ptr & source);

  c_input_options & input_options();
  const c_input_options & input_options() const;

  c_roi_selection_options & roi_selection_options();
  const c_roi_selection_options & roi_selection_options() const;
  c_roi_selection::ptr create_roi_selection() const;

  c_frame_upscale_options & upscale_options();
  const c_frame_upscale_options & upscale_options() const;

  c_sparse_feature_extractor_options & sparse_feature_extractor_options();
  const c_sparse_feature_extractor_options & sparse_feature_extractor_options() const;

  c_sparse_feature_detector_options & sparse_feature_detector_options();
  const c_sparse_feature_detector_options & sparse_feature_detector_options() const;

  c_sparse_feature_descriptor_options & sparse_feature_descriptor_options() ;
  const c_sparse_feature_descriptor_options & sparse_feature_descriptor_options() const;

  c_master_frame_options & master_frame_options();
  const c_master_frame_options & master_frame_options() const;

  c_frame_registration_options & frame_registration_options();
  const c_frame_registration_options & frame_registration_options() const;
  c_frame_registration::sptr create_frame_registration(const c_image_registration_options & options) const;
  c_frame_registration::sptr create_frame_registration() const;

  c_frame_accumulation_options & accumulation_options();
  const c_frame_accumulation_options & accumulation_options() const;
  c_frame_accumulation::ptr create_frame_accumulation() const;

  c_image_stacking_output_options & output_options();
  const c_image_stacking_output_options & output_options() const;

  c_image_processing_options & image_processing_options();
  const c_image_processing_options & image_processing_options() const;

  std::string get_displaypatch() const;

  static ptr load(const std::string & cfgfilename);
  bool save(const std::string & cfgfilename) const;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);


protected:
  c_image_stacking_options(const std::string & name = "");

  std::string name_;
  c_input_sequence::ptr input_sequence_;
  c_input_options input_options_;
  c_roi_selection_options roi_selection_options_;
  c_frame_upscale_options upscale_options_;
  c_frame_registration_options frame_registration_options_;
  c_frame_accumulation_options accumulation_options_;
  c_image_stacking_output_options output_options_;
  c_image_processing_options image_processing_options_;

};


class c_image_stacks_collection
{
public:
  typedef c_image_stacks_collection this_class;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create();

  size_t size() const;

  const std::vector<c_image_stacking_options::ptr> & items() const;
  c_image_stacking_options::ptr item(size_t index) const;
  c_image_stacking_options::ptr item(const std::string & name) const;

  void add(const c_image_stacking_options::ptr & stack);
  bool remove(const c_image_stacking_options::ptr & stack);
  void set(int pos, const c_image_stacking_options::ptr & stack);

  ssize_t indexof(const c_image_stacking_options::ptr & stack) const;
  ssize_t indexof(const std::string & name) const;

  bool save(const std::string & cfgfilename = "") const;
  bool load(const std::string & cfgfilename = "");

  static const std::string & default_config_filename();
  static void set_default_config_filename(const std::string & v);


protected:
  c_image_stacks_collection();

  std::vector<c_image_stacking_options::ptr> stacks_;
  mutable std::string filename_;
  static std::string default_config_filename_;
};


class c_image_stacking_pipeline
{
public:
  typedef c_image_stacking_pipeline this_class;
  typedef std::shared_ptr<this_class> ptr;

public:
  c_image_stacking_pipeline()
  {
  }

  virtual ~c_image_stacking_pipeline()
  {
    set_canceled(true);
  }


  bool run(const c_image_stacking_options::ptr & stacking_options);

  void set_canceled(bool canceled);
  bool canceled() const;

  const c_image_stacking_options::ptr & options() const;

  const c_anscombe_transform & anscombe() const
  {
    return anscombe_;
  }

  int total_frames() const;
  int processed_frames() const;
  int accumulated_frames() const;
  std::string status_message() const ;

  bool compute_accumulated_image(cv::OutputArray dst,
      cv::OutputArray dstmask=cv::noArray()) const ;

protected:

  void set_status_msg(const std::string & msg);

  bool initialize(const c_image_stacking_options::ptr & options);
  bool actual_run();
  void cleanup();

  bool create_reference_frame(const c_input_sequence::ptr & input_sequence,
      int master_frame_index, int max_frames_to_stack,
      cv::Mat & output_reference_frame, cv::Mat & output_reference_mask);

  bool process_input_sequence(const c_input_sequence::ptr & input_sequence,
      int startpos, int endpos);

  bool read_input_frame(const c_input_sequence::ptr & input_sequence,
      const c_input_options & input_options,
      cv::Mat & output_reference_image, cv::Mat & output_reference_mask) const;

  static bool select_image_roi(const c_roi_selection::ptr & roi_selection,
      const cv::Mat & src, const cv::Mat & srcmask,
      cv::Mat & dst, cv::Mat & dstmask);

  static bool write_image(const std::string & output_file_name,
      const c_image_stacking_output_options & output_options,
      const cv::Mat & output_image,
      const cv::Mat & output_mask);

  class c_video_writer;

  void save_preprocessed_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_video_writer & output_writer) const;

  void save_aligned_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_video_writer & output_writer) const;

  void save_postprocessed_frame(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_video_writer & output_writer) const;

  void save_accumulation_mask(const cv::Mat & current_frame, const cv::Mat & curren_mask,
      c_video_writer & output_writer) const;


  static void remove_bad_pixels(cv::Mat & image,
      const c_input_options & input_optons);

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

  virtual void emit_status_changed() {}
  virtual void emit_accumulator_changed() {}


protected:

  using lock_guard = std::lock_guard<std::mutex>;

  c_image_stacking_options::ptr options_;
  c_input_sequence::ptr input_sequence_;

  volatile bool canceled_ = false;
  bool master_frame_generation_ = false;
  int master_frame_index_ = -1;
  bool external_master_frame_ = false;

  std::string output_directory_;

  double ecc_normalization_noise_ = 0;
  double reference_sharpness_ = 0;

  int total_frames_ = 0;
  int processed_frames_ = 0;

  cv::Mat missing_pixel_mask_;
  cv::Mat1f reference_weights_;

  std::string statusmsg_;
  mutable std::mutex status_lock_;

  c_anscombe_transform anscombe_;
  c_roi_selection::ptr roi_selection_;
  c_frame_registration::sptr frame_registration_;
  c_frame_weigthed_average::ptr flow_accumulation_;
  mutable std::mutex registration_lock_;

  c_frame_accumulation::ptr frame_accumulation_;
  c_sharpness_norm_measure::ptr sharpness_norm_accumulation_;
  mutable std::mutex accumulator_lock_;

  std::vector<uint> badframes_; // global indexes
  void gather_badframe_indexes();
};

#endif /* __c_stacking_pipeline_h__ */
