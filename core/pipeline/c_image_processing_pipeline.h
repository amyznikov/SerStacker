/*
 * c_image_processing_pipeline.h
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_processing_pipeline_h__
#define __c_image_processing_pipeline_h__

#include <opencv2/opencv.hpp>
#include <core/io/c_input_sequence.h>
#include <core/proc/white_balance/histogram_normalization.h>
#include <core/settings/opencv_settings.h>
#include <atomic>
#include "c_image_processing_pipeline_ctrl.h"


struct c_image_processing_pipeline_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;

  DEBAYER_ALGORITHM debayer_method = DEBAYER_NN2;

  std::string darkbayer_filename;
  std::string flatbayer_filename;
  std::string missing_pixel_mask_filename;

  bool missing_pixels_marked_black = true;
  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
  bool filter_bad_pixels = false;
  bool detect_bad_asi_frames = false;
  bool enable_bground_normalization  = false;

  double bad_asi_frame_median_hat_threshold = 300;
  double bad_pixels_variation_threshold = 15;
  c_histogram_normalization_options background_normalization_options;

  c_image_processor::sptr input_image_processor;
};

bool serialize_base_input_options(c_config_setting section, bool save, c_image_processing_pipeline_input_options & opts);

#define POPULATE_PIPELINE_INPUT_OPTIONS(ctrls) \
  PIPELINE_CTLC(ctrls, _input_options.start_frame_index, "start frame index", "", _this->_input_sequence != nullptr); \
  PIPELINE_CTL(ctrls, _input_options.max_input_frames, "max input frames", "");\
  PIPELINE_CTL(ctrls, _input_options.debayer_method, "debayer method", "");\
  PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, _input_options.darkbayer_filename, "Dark frame", "");\
  PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, _input_options.flatbayer_filename, "Flat frame", "");\
  PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, _input_options.missing_pixel_mask_filename, "missing pixel mask", "");\
  PIPELINE_CTL(ctrls, _input_options.missing_pixels_marked_black, "missing pixels are black", "");\
  PIPELINE_CTL(ctrls, _input_options.inpaint_missing_pixels, "inpaint missing pixels", "");\
  PIPELINE_CTL(ctrls, _input_options.enable_color_maxtrix, "enable color maxtrix", "");\
  PIPELINE_CTL(ctrls, _input_options.detect_bad_asi_frames, "detect bad asi frames", "");\
  PIPELINE_CTL(ctrls, _input_options.bad_asi_frame_median_hat_threshold, "bad_asi_frame_median_hat_threshold", "");\
  PIPELINE_CTL(ctrls, _input_options.filter_bad_pixels, "filter bad pixels", "");\
  PIPELINE_CTL(ctrls, _input_options.bad_pixels_variation_threshold, "bad pixels variation", "");\
  PIPELINE_CTL(ctrls, _input_options.enable_bground_normalization, "bground normalization", "");\
  PIPELINE_CTLC(ctrls, _input_options.background_normalization_options.norm_type, "norm type", "norm type", _this->_input_options.enable_bground_normalization);\
  PIPELINE_CTLC(ctrls, _input_options.background_normalization_options.stretch, "stretch", "stretch", _this->_input_options.enable_bground_normalization);\
  PIPELINE_CTLC(ctrls, _input_options.background_normalization_options.offset, "offset", "offset", _this->_input_options.enable_bground_normalization);\
  PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _input_options.input_image_processor, "input_image_processor", "");



struct c_image_processing_pipeline_output_options
{
  std::string output_directory;
  int default_display_type = -1;
};

class c_image_processing_pipeline
{
public:
  typedef c_image_processing_pipeline this_class;
  typedef std::shared_ptr<this_class> sptr;
  using lock_guard = std::lock_guard<std::mutex>;

public: // pipeline methods
  c_image_processing_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence = nullptr);
  virtual ~c_image_processing_pipeline();
  virtual bool initialize();

  void set_name(const std::string & name);
  const std::string & name() const;
  const char * cname() const;

  virtual const std::string & get_class_name() const = 0;
  virtual bool copyParameters(const sptr & dst) const ;


  virtual bool run(const c_input_sequence::sptr & input_sequence = nullptr);
  virtual void cancel(bool v = true);
  virtual bool canceled() const;
  virtual bool serialize(c_config_setting settings, bool save);

  std::mutex & mutex();

  const c_input_sequence::sptr & input_sequence() const;
  const char * csequence_name() const;

  virtual bool has_master_frame() const ;
  virtual void set_master_source(const std::string & master_source_path);
  virtual std::string master_source() const;
  virtual void set_master_frame_index(int v);
  virtual int master_frame_index() const;

  virtual std::string generate_output_filename(const std::string & ufilename,
      const std::string & postfix,
      const std::string & suffix) const;

  int total_frames() const;
  int processed_frames() const;
  int accumulated_frames() const;
  int pipeline_stage() const;
  std::string status_message() const ;

  bool is_running() const;

  void set_display_type(int v);
  int display_type() const;

  virtual const c_enum_member * get_display_types() const;
  virtual bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);




protected:
  virtual bool initialize_pipeline();
  virtual void cleanup_pipeline();
  virtual bool run_pipeline();

protected:
  virtual bool open_input_sequence();
  virtual void close_input_sequence();
  virtual bool seek_input_sequence(int pos);

protected:
  virtual void on_frame_processed();
  virtual void on_state_changed();
  virtual void on_status_update();

protected:
  virtual bool start_pipeline(int start_frame_index, int max_input_frames);
  virtual void set_running(bool v);
  void set_pipeline_stage(int stage);
  void set_status_msg(const std::string & msg);
  virtual std::string create_output_path(const std::string & output_directory) const;
  virtual void gather_badframe_indexes();
  virtual bool is_bad_frame_index(int global_pos) const;

  virtual bool open_output_writer(c_output_frame_writer & writer,
      const c_output_frame_writer_options & opts,
      const std::string & postfix,
      const std::string & suffix) const;

  virtual bool add_output_writer(c_output_frame_writer & writer,
      const c_output_frame_writer_options & opts,
      const std::string & postfix,
      const std::string & suffix);

  virtual bool add_output_writer(c_output_text_writer & writer,
      const std::string & filename);

protected:
  bool read_input_frame(const c_input_sequence::sptr & input_sequence,
      const c_image_processing_pipeline_input_options & input_options,
      cv::Mat & output_image, cv::Mat & output_mask,
      bool is_external_master_frame,
      bool save_raw_bayer) const;



public: // factory methods

  typedef std::function<c_image_processing_pipeline::sptr(const std::string & name,
      const c_input_sequence::sptr & input_sequence)> instance_creator;

  struct factory_item
  {
    std::string class_name;
    std::string tooltip;
    c_image_processing_pipeline::instance_creator create_instance;

    factory_item(const std::string & _class_name, const std::string & _tooltip,
        const c_image_processing_pipeline::instance_creator & _create_instance);
  };

  static bool register_class(const std::string & class_name, const std::string & tooltip,
      const c_image_processing_pipeline::instance_creator & create_instance);
  static const std::vector<factory_item>& registered_classes();
  static const factory_item* find_class(const std::string & class_name);
  static const factory_item* find_class(const sptr & pipeline);

  static c_image_processing_pipeline::sptr create_instance(const std::string & class_name,
      const std::string & name, const c_input_sequence::sptr & input_sequence = nullptr);

protected:
  static std::vector<factory_item> _registered_classes;

protected:
  std::string _name;
  c_input_sequence::sptr _input_sequence;
  std::vector<uint32_t> _badframes; // global indexes
  std::string _output_path;
  std::vector<c_output_frame_writer*> _opened_writers;
  std::vector<c_output_text_writer*> _opened_text_writers;

  cv::Mat _darkbayer;
  cv::Mat _flatbayer;
  cv::Mat _missing_pixel_mask;
  mutable cv::Mat _raw_bayer_image;
  mutable COLORID _raw_bayer_colorid = COLORID_UNKNOWN;

  int _display_type = 0;

  int _total_frames = 0;
  int _processed_frames = 0;
  int _accumulated_frames = 0;

  mutable std::mutex _lock;
  mutable std::mutex _status_lock; // FIXME: get rid of obsolete status_lock_
  std::string _statusmsg;

  int _pipeline_stage = 0;

  volatile std::atomic_bool _is_running = false;
  volatile std::atomic_bool _canceled = false;
};


class c_image_sequence :
    public c_input_sequence
{
public:
  typedef c_image_sequence this_class;
  typedef c_input_sequence base;
  typedef std::shared_ptr<this_class> sptr;

  c_image_sequence(const std::string & name = "" );
  virtual ~c_image_sequence() = default;

  std::string get_display_path() const;

  void set_current_pipeline(const std::string & name);
  void set_current_pipeline(const c_image_processing_pipeline::sptr& pipeline);
  const c_image_processing_pipeline::sptr& current_pipeline() const;

  const std::vector<c_image_processing_pipeline::sptr> & pipelines() const;
  void add_pipeline(const c_image_processing_pipeline::sptr & pipeline);
  void remove_pipeline(const c_image_processing_pipeline::sptr & pipeline);
  void remove_pipeline(const std::string & name);
  c_image_processing_pipeline::sptr find_pipeline(const std::string & name) const;
  bool pipeline_exists(const std::string & name) const;

  // bool serialize(c_config_setting setting, bool save) override;
  //static sptr load(const std::string & filename);

protected:
  std::vector<c_image_processing_pipeline::sptr> pipelines_;
  c_image_processing_pipeline::sptr current_pipeline_;
};


#endif /* __c_image_processing_pipeline_h__ */
