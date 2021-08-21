/*
 * c_stacking_pipeline.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __c_stacking_pipeline_h__
#define __c_stacking_pipeline_h__

#include <core/io/c_input_sequence.h>
#include <core/roi_selection/c_planetary_disk_selection.h>
#include <core/registration/c_feature_based_registration.h>
#include <core/registration/c_planetary_disk_registration.h>
#include <core/registration/c_star_field_registration.h>
#include <core/average/c_frame_accumulation.h>
#include <core/proc/c_anscombe_transform.h>
#include <core/improc/c_image_processor.h>
#include <atomic>


enum roi_selection_method {
  roi_selection_none = 0,
  roi_selection_planetary_disk = 1
};

enum frame_registration_method {
  frame_registration_none = -1,
  frame_registration_method_surf = 0,
  frame_registration_method_planetary_disk = 1,
  frame_registration_method_star_field = 2,
};

enum frame_accumulation_method {
  frame_accumulation_none = -1,
  frame_accumulation_average_masked = 0,
  frame_accumulation_average_weighted = 1,
  frame_accumulation_fft = 2,
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


const extern struct roi_selection_method_desc {
  const char * name;
  enum roi_selection_method value;
} roi_selection_methods[];

std::string toStdString(enum roi_selection_method v);
enum roi_selection_method fromStdString(const std::string  & s,
    enum roi_selection_method defval );

const extern struct frame_registration_method_desc {
  const char * name;
  enum frame_registration_method value;
} frame_registration_methods[];

std::string toStdString(enum frame_registration_method v);
enum frame_registration_method fromStdString(const std::string  & s,
    enum frame_registration_method defval );


const extern struct frame_accumulation_method_desc {
  const char * name;
  enum frame_accumulation_method value;
} frame_accumulation_methods[];

std::string toStdString(enum frame_accumulation_method v);
enum frame_accumulation_method fromStdString(const std::string  & s,
    enum frame_accumulation_method defval );

const extern struct frame_upscale_stage_desc {
  const char * name;
  enum frame_upscale_stage value;
} frame_upscale_stages[];

std::string toStdString(enum frame_upscale_stage v);
enum frame_upscale_stage fromStdString(const std::string  & s,
    enum frame_upscale_stage defval );

const extern struct frame_upscale_option_desc {
  const char * name;
  enum frame_upscale_option value;
} frame_upscale_options[];

std::string toStdString(enum frame_upscale_option v);
enum frame_upscale_option fromStdString(const std::string  & s,
    enum frame_upscale_option defval );


struct c_input_options {
  c_image_processor::ptr input_frame_processor;

  bool remove_bad_pixels = true;
  bool enable_color_maxtrix = false;

  enum anscombe_method anscombe = anscombe_none;
  double bad_pixels_variation_threshold = 7;
};

struct c_roi_selection_options {
  enum roi_selection_method method = roi_selection_none;
  cv::Size crop_size;
};

struct c_master_frame_options {
  std::string master_source_path;
  int master_frame_index = 0; // relative, in master source
  bool apply_input_frame_processor = true;
  bool generate_master_frame = true;
  int max_input_frames_to_generate_master_frame = 200;
  int eccflow_scale = 0;
  bool compensate_master_flow = true;
};

struct c_frame_upscale_options {
  enum frame_upscale_option upscale_option = frame_upscale_none;
  enum frame_upscale_stage upscale_stage = frame_upscale_after_align;

  bool need_upscale_before_align() const {
    return  upscale_option != frame_upscale_none && upscale_stage == frame_upscale_before_align;
  }

  bool need_upscale_after_align() const {
    return  upscale_option != frame_upscale_none && upscale_stage == frame_upscale_after_align;
  }

};

struct c_frame_accumulation_options {
  enum frame_accumulation_method accumulation_method  = frame_accumulation_average_masked;
};


struct c_frame_registration_options {
  enum frame_registration_method registration_method = frame_registration_method_surf;
  bool incremental_mode = false; // STUPID TEST, DON'T USE
  c_frame_registration_base_options base_options;
  c_feature_based_registration_options feature_options;
  c_planetary_disk_registration_options planetary_disk_options;
  c_star_field_registration_options star_field_options;
};


struct c_image_stacking_output_options {

  std::string output_directory;

  std::string output_aligned_video_filename;

  c_image_processor::ptr frame_processor;
  std::string processed_frame_filename;

  c_image_processor::ptr accumuated_image_processor;

  bool write_aligned_video = false;
  bool save_processed_frames = false;
  bool dump_reference_data_for_debug = false;
  bool write_image_mask_as_alpha_channel = true;
};


class c_image_stacking_options
{
public:
  typedef c_image_stacking_options this_class;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create(const std::string & name = "");

  void set_name(const std::string & name);
  const std::string & name() const;

  void set_input_sequence(const c_input_sequence::ptr & sequence);
  const c_input_sequence::ptr & input_sequence() const;

  const std::vector<c_input_source::ptr> & input_sources() const;
  c_input_source::ptr add_input_source(const std::string & pathfilename);
  bool add_input_sources(const std::vector<std::string> & pathfilenames);
  bool remove_input_source(const c_input_source::ptr & source);

  c_input_options & input_options();
  const c_input_options & input_options() const;

  c_roi_selection_options & roi_selection_options();
  const c_roi_selection_options & roi_selection_options() const;
  c_feature_based_roi_selection::ptr create_roi_selection() const;

  c_frame_upscale_options & upscale_options();
  const c_frame_upscale_options & upscale_options() const;

  c_master_frame_options & master_frame_options();
  const c_master_frame_options & master_frame_options() const;

  c_frame_registration_options & frame_registration_options();
  const c_frame_registration_options & frame_registration_options() const;
  c_frame_registration::ptr create_frame_registration() const;

  c_frame_accumulation_options & accumulation_options();
  const c_frame_accumulation_options & accumulation_options() const;
  c_frame_accumulation::ptr create_frame_accumulation() const;

  c_image_stacking_output_options & output_options();
  const c_image_stacking_output_options & output_options() const;

  std::string get_displaypatch() const;

protected:
  c_image_stacking_options(const std::string & name = "");

  std::string name_;
  c_input_sequence::ptr input_sequence_;
  c_input_options input_options_;
  c_roi_selection_options roi_selection_options_;
  c_frame_upscale_options upscale_options_;
  c_master_frame_options master_frame_options_;
  c_frame_registration_options frame_registration_options_;
  c_frame_accumulation_options accumulation_options_;
  c_image_stacking_output_options output_options_;
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

  void add(const c_image_stacking_options::ptr & pipeline);
  bool remove(const c_image_stacking_options::ptr & pipeline);

  ssize_t indexof(const c_image_stacking_options::ptr & pipeline) const;
  ssize_t indexof(const std::string & name) const;


protected:
  c_image_stacks_collection();

  std::vector<c_image_stacking_options::ptr> stacks_;
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

  const c_image_stacking_options::ptr & stacking_options() const;

  const std::string & master_file_name() const {
    return master_file_name_;
  }

  const cv::Mat & reference_image() const {
    return reference_frame_;
  }

  const cv::Mat & current_image() const {
    return current_frame_;
  }

  const cv::Mat current_weights() const {
    return current_weights_;
  }

  const c_anscombe_transform & anscombe() const {
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

  bool run_regular_mode(const c_input_sequence::ptr & input_sequence);

  // STUPID TEST, DON'T USE
  bool run_incremental_mode(const c_input_sequence::ptr & input_sequence,
      cv::Mat * output_accumulated_frame,
      cv::Mat1f * output_accumulated_counts,
      cv::Mat1b * output_accumulated_mask);

  bool create_reguar_mode_reference_frame();
  bool generate_reference_frame(const c_input_sequence::ptr & input_sequence);



  bool read_input_frame(const c_input_sequence::ptr & input_sequence,
      const c_input_options & input_options,
      cv::Mat & output_reference_image, cv::Mat & output_reference_mask) const;

  static bool select_image_roi(const c_feature_based_roi_selection::ptr & roi_selection,
      const cv::Mat & src, const cv::Mat & srcmask,
      cv::Mat & dst, cv::Mat & dstmask);


  static bool write_image(const std::string & output_file_name,
      const c_image_stacking_output_options & output_options,
      const cv::Mat & output_image,
      const cv::Mat & output_mask);

  static bool save_processed_frame(const cv::Mat & curren_frame, const cv::Mat & current_mask,
      const c_image_stacking_output_options & output_options,
      const std::string & output_directory,
      const std::string & sequence_name,
      const c_input_sequence::ptr & input_sequence);

  static void remove_bad_pixels(cv::Mat & image,
      const c_input_options & input_optons);


  static void upscale_image(enum frame_upscale_option scale,
      cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst, cv::OutputArray dstmask);

  static void upscale_remap(enum frame_upscale_option scale,
      cv::InputArray srcmap,
      cv::OutputArray dstmap);

  static void compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst);
  static void compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel);
  static double compute_image_noise(const cv::Mat & image, const cv::Mat & mask, color_channel_type channel);


  virtual void emit_status_changed() {}
  virtual void emit_accumulator_changed() {}


protected:

  using lock_guard = std::lock_guard<std::mutex>;

  c_image_stacking_options::ptr options_;

  volatile bool canceled_ = false;

  std::string output_directory_;
  std::string master_file_name_;
  int master_source_index_ = -1;
  int master_frame_index_ = -1;
  cv::Mat reference_frame_;
  cv::Mat reference_mask_;
  cv::Mat reference_weights_;
  cv::Mat current_frame_;
  cv::Mat1b current_mask_;
  cv::Mat current_weights_;
  //cv::Mat current_master_flow_;
  double ecc_normalization_noise_ = 0;

  int total_frames_ = 0;
  int processed_frames_ = 0;

  std::string statusmsg_;
  mutable std::mutex status_lock_;

  c_anscombe_transform anscombe_;
  c_feature_based_roi_selection::ptr roi_selection_;
  c_frame_registration::ptr frame_registration_;
  c_frame_accumulation_with_mask::ptr masterflow_accumulation_;
  mutable std::mutex registration_lock_;

  c_frame_accumulation::ptr frame_accumulation_;
  mutable std::mutex accumulator_lock_;


};

#endif /* __c_stacking_pipeline_h__ */
