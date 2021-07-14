/*
 * c_stacking_pipeline.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __c_stacking_pipeline_h__
#define __c_stacking_pipeline_h__

#include <core/io/c_input_sequence.h>
#include <core/registration/c_feature_based_registration.h>
#include <core/registration/c_planetary_disk_registration.h>
#include <core/registration/c_star_field_registration.h>
#include <core/average/c_frame_accumulation.h>
#include <condition_variable>
#include <atomic>


enum frame_registration_method {
  frame_registration_method_unknown = -1,
  frame_registration_method_surf = 0,
  frame_registration_method_planetary_disk = 1,
  frame_registration_method_star_field = 2,
  frame_registration_method_skip = 3,
};

enum frame_accumulation_method {
  frame_accumulation_method_unknown = -1,
  frame_accumulation_average_masked = 0,
  frame_accumulation_average_weighted = 1,
};

enum frame_upscale_option {
  frame_upscale_disabled = 0,
  frame_upscale_before_align = 1,
  frame_upscale_after_align = 2,
};



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


const extern struct frame_upscale_option_desc {
  const char * name;
  enum frame_upscale_option value;
} frame_upscale_options[];

std::string toStdString(enum frame_upscale_option v);
enum frame_upscale_option fromStdString(const std::string  & s,
    enum frame_upscale_option defval );


struct c_image_stacking_input_options {
  bool enable_remove_bad_pixels = true;
  bool enable_color_maxtrix = false;
};

struct c_stacking_master_frame_options {
  std::string master_path;
  int master_frame_index = 0; // relative, in master source
  bool use_ffts_from_master_path = false;
};

struct c_frame_accumulation_options {
  enum frame_accumulation_method accumulation_method  = frame_accumulation_average_masked;
  enum frame_upscale_option upscale_option = frame_upscale_disabled;
};


struct c_frame_registration_options {
  enum frame_registration_method registration_method = frame_registration_method_surf;
  c_frame_registration_base_options base_options;
  c_feature_based_registration_options feature_options;
  c_planetary_disk_registration_options planetary_disk_options;
  c_star_field_registration_options star_field_options;
};


struct c_image_stacking_output_options {

  std::string output_directory;

  std::string output_aligned_video_filename;
  bool write_aligned_video = false;
  bool write_registartion_reference_frames = false;
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

  c_image_stacking_input_options & input_options();
  const c_image_stacking_input_options & input_options() const;

  c_stacking_master_frame_options & master_frame_options();
  const c_stacking_master_frame_options & master_frame_options() const;

  c_frame_registration_options & frame_registration_options();
  const c_frame_registration_options & frame_registration_options() const;
  c_frame_registration::ptr create_frame_registration();

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
  c_image_stacking_input_options input_options_;
  c_stacking_master_frame_options master_frame_options_;
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

  class auto_lock {
    c_image_stacking_pipeline * t_ =  nullptr;
  public:
    auto_lock(c_image_stacking_pipeline * t) : t_(t) {
      t->lock();
    }
    ~auto_lock() {
      t_->unlock();
    }
  };


public:
  c_image_stacking_pipeline()
    : cancel_requested_(false)
  {
  }

  virtual ~c_image_stacking_pipeline()
  {
    cancel(true);
  }

  bool run(const c_image_stacking_options::ptr & stacking_options);
  void cancel(bool lock);

  void lock();
  void unlock();

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

  int num_frames_total() const {
    return num_frames_total_;
  }

  int num_frames_processed() const {
    return num_frames_processed_;
  }

  int num_frames_accumulated() const {
    return num_frames_accumulated_;
  }

  bool compute_accumulated_image(cv::OutputArray dst,
      cv::OutputArray dstmask=cv::noArray()) const ;

protected:

  bool select_and_load_reference_frame(const c_image_stacking_options::ptr & options);

  static bool read_input_frame(const c_input_sequence::ptr & input_sequence,
      const c_image_stacking_input_options & input_options,
      cv::Mat & output_reference_image, cv::Mat & output_reference_mask);

  static bool write_image(const std::string output_file_name,
      const c_image_stacking_output_options & output_options,
      const cv::Mat & output_image,
      const cv::Mat & output_mask);


  static void remove_bad_pixels(cv::Mat & image);
  static void upscale(cv::InputArray src, cv::InputArray srcmask,  cv::OutputArray dst, cv::OutputArray dstmask);
  static void compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst);


  virtual void on_frame_processed() {}
  virtual void on_frame_accumulated() {}


protected:
  std::mutex mtx;
  //std::condition_variable condvar;
  c_image_stacking_options::ptr stacking_options_;
  /*volatile */std::atomic_bool cancel_requested_;

  std::string master_file_name_;
  int master_source_index_ = -1;
  int master_frame_index_ = -1;
  //int reference_channel_ = -1;
  cv::Mat reference_frame_;
  cv::Mat reference_mask_;
  cv::Mat current_frame_;
  cv::Mat current_weights_;
  cv::Mat current_mask_;
  double ecc_normalization_noise_ = 0;

  c_frame_accumulation::ptr frame_accumulation_;

  int num_frames_total_ = 0;
  int num_frames_processed_ = 0;
  int num_frames_accumulated_ = 0;
};

#endif /* __c_stacking_pipeline_h__ */
