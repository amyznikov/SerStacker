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
#include <core/notification.h>

class c_image_processing_pipeline
{
public:
  typedef c_image_processing_pipeline this_class;
  typedef std::shared_ptr<this_class> sptr;
  using lock_guard = std::lock_guard<std::mutex>;

  typedef std::function<c_image_processing_pipeline::sptr(const std::string & name,
      const c_input_sequence::sptr & input_sequence)> instance_creator;

public: // factory methods

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
      const std::string & name, const c_input_sequence::sptr & input_sequence);

public: // output writer utility class

  class c_video_writer
  {
  public:
    ~c_video_writer();
    bool is_open() const;
    bool open(const std::string & filename, const cv::Size & frameSize, bool color, bool write_frame_mapping = false);
    bool write(cv::InputArray currenFrame, cv::InputArray currentMask, bool with_alpha_mask, int seqindex);
    void close();

  protected:
    cv::VideoWriter aviVideo;
    c_ser_writer serVideo;
    cv::Mat tmp;

    enum
    {
      output_type_unknown,
      output_type_video,
      output_type_ser,
      output_type_images,
    } output_type = output_type_unknown;

    std::string output_file_name;
    FILE *frame_mapping_fp = nullptr;
    int current_frame_index = 0;
    int current_input_sequence_index = 0;
  };


public: // pipeline methods

  c_image_processing_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence);
  virtual ~c_image_processing_pipeline();
  virtual const std::string & get_class_name() const = 0;


  void set_name(const std::string & name);
  const std::string & name() const;
  const char * cname() const;

  void set_sequence_name(const std::string & v);
  const std::string& sequence_name() const;
  const char * csequence_name() const;

  // void set_input_sequence(const c_input_sequence::sptr& input_sequence);
  const c_input_sequence::sptr& input_sequence() const;

  void set_output_directory(const std::string & output_directory);
  const std::string & output_directory() const;

  void set_master_source(const std::string & master_source_path);
  const std::string & master_source() const;

  void set_master_frame_index(int v);
  int master_frame_index() const;

  virtual std::string generate_output_file_name(const std::string & ufilename,
      const std::string & postfix,
      const std::string & suffix) const;

  int total_frames() const;
  int processed_frames() const;
  int accumulated_frames() const;
  int pipeline_stage() const;
  std::string status_message() const ;

  bool canceled() const;
  virtual void cancel(bool v = true);
  virtual bool run();
  virtual bool serialize(c_config_setting setting, bool save);

  c_notification<void(int oldstage, int newstage)> on_pipeline_stage_changed;
  c_notification<void(const std::string & statusmsg)> on_status_msg_changed;
  c_notification<void()> on_status_changed;

protected:
  void set_pipeline_stage(int stage);
  void set_status_msg(const std::string & msg) const;
  virtual void update_output_path();
  virtual void gather_badframe_indexes();
  virtual bool is_bad_frame_index(int global_pos) const;

  virtual bool initialize_pipeline();
  virtual void cleanup_pipeline();
  virtual bool run_pipeline();

protected:
  static std::vector<factory_item> registered_classes_;

protected:
  std::string name_;
  std::string sequence_name_;
  c_input_sequence::sptr input_sequence_;
  std::vector<uint> badframes_; // global indexes
  std::string output_directory_;
  std::string output_path_;

  std::string master_source_;
  int master_frame_index_ = 0; // relative, in master source

  int total_frames_ = 0;
  int processed_frames_ = 0;
  int accumulated_frames_ = 0;

  mutable std::string statusmsg_;
  mutable std::mutex status_lock_;

  int pipeline_stage_ = 0;

  volatile bool canceled_ = false;
};



class c_image_sequence
{
public:
  typedef c_image_sequence this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_image_sequence(const std::string & name = "" );
  virtual ~c_image_sequence() = default;

  void set_name(const std::string & name);
  const std::string & name() const;
  const char* cname() const;

  std::string displaypatch() const;

  //void set_input_sequence(const c_input_sequence::ptr & input_sequence);
  const c_input_sequence::sptr& input_sequence() const;

  void set_current_pipeline(const std::string & name);
  void set_current_pipeline(const c_image_processing_pipeline::sptr& pipeline);
  const c_image_processing_pipeline::sptr& current_pipeline() const;

  const std::vector<c_image_processing_pipeline::sptr> & pipelines() const;
  void add_pipeline(const c_image_processing_pipeline::sptr & pipeline);
  void remove_pipeline(const c_image_processing_pipeline::sptr & pipeline);
  void remove_pipeline(const std::string & name);
  c_image_processing_pipeline::sptr find_pipeline(const std::string & name) const;
  bool pipeline_exists(const std::string & name) const;


  virtual bool serialize(c_config_setting setting, bool save);

  static sptr load(const std::string & filename);

protected:

protected:
  std::string name_;
  c_input_sequence::sptr input_sequence_ = c_input_sequence::create();
  std::vector<c_image_processing_pipeline::sptr> pipelines_;
  c_image_processing_pipeline::sptr current_pipeline_;

};



class c_image_sequence_collection
{
public:
  typedef c_image_sequence_collection this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_image_sequence_collection();

  size_t size() const;

  const std::vector<c_image_sequence::sptr> & items() const;
  c_image_sequence::sptr item(size_t index) const;
  c_image_sequence::sptr item(const std::string & name) const;

  void add(const c_image_sequence::sptr & sequence);
  bool remove(const c_image_sequence::sptr & sequence);
  void set(int pos, const c_image_sequence::sptr & sequence);

  ssize_t indexof(const c_image_sequence::sptr & sequence) const;
  ssize_t indexof(const std::string & name) const;

  bool save(const std::string & cfgfilename = "") const;
  bool load(const std::string & cfgfilename = "");

  static const std::string & default_config_filename();
  static void set_default_config_filename(const std::string & v);


protected:

  std::vector<c_image_sequence::sptr> items_;

  mutable std::string config_filename_;
  static std::string default_config_filename_;
};


#endif /* __c_image_processing_pipeline_h__ */
