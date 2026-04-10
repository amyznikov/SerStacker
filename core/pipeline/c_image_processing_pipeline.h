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
#include <core/proc/histogram-tools.h>
#include <core/settings/opencv_settings.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/improc/c_image_processor.h>
#include "c_output_frame_writer.h"
#include <atomic>
//#include "c_image_processing_pipeline_ctrl.h"

struct c_image_processing_pipeline_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;
  DEBAYER_ALGORITHM debayer_method = DEBAYER_NN2;
  bool enable_color_maxtrix = true;

  bool inpaint_missing_pixels = true;
  bool missing_pixels_marked_black = true;
  std::string missing_pixel_mask_filename;
};

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
  std::mutex & mutex();

  void set_name(const std::string & name);
  const std::string & name() const;
  const char * cname() const;
  virtual const std::string & get_class_name() const = 0;

  const c_input_sequence::sptr & input_sequence() const;
  const char * csequence_name() const;

  virtual bool has_master_frame() const ;
  virtual void set_master_source(const std::string & master_source_path);
  virtual std::string master_source() const;
  virtual void set_master_frame_index(int v);
  virtual int master_frame_index() const;

  virtual const c_enum_member * get_display_types() const;
  void set_display_type(int v);
  int display_type() const;
  bool get_display(cv::OutputArray displayImage, cv::OutputArray displayMask);

  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool copy_parameters(const sptr & dst) const ;

  int total_frames() const;
  int processed_frames() const;
  int accumulated_frames() const;
  std::string status_message() const ;

  virtual bool run(const c_input_sequence::sptr & input_sequence = nullptr);
  virtual void cancel(bool v = true);
  virtual bool canceled() const;
  bool is_running() const;

  virtual std::string generate_output_filename(const std::string & ufilename,
      const std::string & postfix,
      const std::string & suffix) const;


public:
  struct c_scope_guard
  {
    std::function<void()> _fn;

    template<class F>
    c_scope_guard(F && f) : _fn(f)
    {
    }
    ~c_scope_guard()
    {
      if( _fn ) {
        _fn();
      }
    }
  };

  template<typename Fn>
  inline auto synchronized(Fn && fn)
  {
    std::scoped_lock lock(mutex());
    if constexpr ( std::is_void_v<std::invoke_result_t<Fn>> ) {
      std::forward<Fn>(fn)();
    }
    else {
      return std::forward<Fn>(fn)();
    }
  }

protected:
  virtual bool initialize_pipeline();
  virtual void cleanup_pipeline();
  virtual bool run_pipeline();
  virtual bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

protected:
  virtual bool open_input_sequence();
  virtual void close_input_sequence();
  virtual bool seek_input_sequence(int pos);
  virtual void close_all_writers();

protected:
  virtual void on_frame_processed();
  virtual void on_state_changed();
  virtual void on_status_update();

protected:
  virtual bool start_pipeline(int start_frame_index, int max_input_frames);
  virtual void set_running(bool v);
  void set_status_msg(const std::string & msg);
  virtual std::string create_output_path(const std::string & output_directory) const;
  virtual bool is_bad_frame_index(uint32_t global_pos) const;


protected:
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
  std::string _output_path;
  std::vector<c_output_frame_writer*> _opened_writers;
  std::vector<c_output_text_writer*> _opened_text_writers;

  cv::Mat _missing_pixel_mask;

  int _display_type = 0;

  int _total_frames = 0;
  int _processed_frames = 0;
  int _accumulated_frames = 0;

  mutable std::mutex _lock;
  mutable std::mutex _status_lock; // FIXME: get rid of obsolete status_lock_
  std::string _statusmsg;

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
  std::vector<c_image_processing_pipeline::sptr> _pipelines;
  c_image_processing_pipeline::sptr _current_pipeline;
};


bool serialize_base_input_options(c_config_setting section, bool save,
    c_image_processing_pipeline_input_options & opts);

template<class RootObjectType>
void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_image_processing_pipeline_input_options> & ctx)
{
  using S = c_image_processing_pipeline_input_options;

  ctlbind(ctls, "start frame index", ctx(&S::start_frame_index), "");
  ctlbind(ctls, "max input frames", ctx(&S::max_input_frames), "");
  ctlbind(ctls, "debayer method", ctx(&S::debayer_method), "");
  ctlbind(ctls, "enable color maxtrix", ctx(&S::enable_color_maxtrix), "");

  ctlbind_expandable_group(ctls, "missing pixels...", "");
   ctlbind_browse_for_file(ctls, "missing pixel mask", ctx(&S::missing_pixel_mask_filename), "");
   ctlbind(ctls, "inpaint missing pixels", ctx(&S::inpaint_missing_pixels), "");
   ctlbind(ctls, "missing pixels are black", ctx(&S::missing_pixels_marked_black ), "");
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_image_processing_pipeline_output_options> & ctx)
{
  using S = c_image_processing_pipeline_output_options;
  ctlbind_browse_for_directory(ctls, "output_directory", ctx(&S::output_directory), "");
  ctlbind(ctls, "default_display_type", ctx(&S::default_display_type), "");
}

#endif /* __c_image_processing_pipeline_h__ */
