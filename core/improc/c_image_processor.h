/*
 * c_image_processor.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __c_image_processor_h__
#define __c_image_processor_h__

#include <opencv2/opencv.hpp>
#include <core/proc/normalize.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/smap.h>
#include <core/settings/opencv_settings.h>

///////////////////////////////////////////////////////////////////////////////

class c_image_processor_routine
{
public:
  typedef c_image_processor_routine this_class;
  typedef std::shared_ptr<this_class> ptr;
  typedef std::function<this_class::ptr()> factory;

  typedef std::function<void(this_class *,
      cv::InputOutputArray image,
      cv::InputOutputArray mask)>
  notify_callback;

  struct class_factory
  {
    std::string class_name;
    std::string display_name;
    std::string tooltip;
    factory create_instance;

    class_factory(const std::string & _class_name,
        const std::string & _display_name,
        const std::string & _tooltip,
        const factory & _create_instance);
  };

  struct class_list_guard_lock
  {
    class_list_guard_lock() {
      mtx().lock();
    }

    ~class_list_guard_lock() {
      mtx().unlock();
    }

    static std::mutex & mtx() {
      static std::mutex mtx_;
      return mtx_;
    }
  };

  static void register_class_factory(const class_factory * class_factory);
  static void register_all();

  static const std::vector<const class_factory*> & class_list();

  const class_factory * classfactory() const;

  virtual ~c_image_processor_routine() = default;


  static ptr create(const std::string & class_name);
  static ptr create(c_config_setting settings);
  virtual bool deserialize(c_config_setting settings);
  virtual bool serialize(c_config_setting settings) const;

  const std::string & class_name() const
  {
    return class_factory_->class_name;
  }

  const std::string & display_name() const
  {
    return class_factory_->display_name;
  }

  const std::string & tooltip() const
  {
    return class_factory_->tooltip;
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  void set_preprocess_notify_callback(const notify_callback & preprocess_notify)
  {
    preprocess_notify_ = preprocess_notify;
  }

  const notify_callback & preprocess_notify_callback() const
  {
    return preprocess_notify_;
  }

  void set_postprocess_notify_callback(const notify_callback & postprocess_notify)
  {
    postprocess_notify_ = postprocess_notify;
  }

  const notify_callback & postprocess_notify_callback() const
  {
    return postprocess_notify_;
  }

  void emit_preprocess_notify(cv::InputOutputArray image, cv::InputOutputArray mask) {
    if ( preprocess_notify_ ) {
      preprocess_notify_(this, image, mask);
    }
  }

  void emit_postprocess_notify(cv::InputOutputArray image, cv::InputOutputArray mask) {
    if ( postprocess_notify_ ) {
      postprocess_notify_(this, image, mask);
    }
  }

  virtual bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) = 0;


protected:
  c_image_processor_routine(const class_factory * _class_factory, bool enabled = true)
    : class_factory_(_class_factory), enabled_(enabled)
  {
  }

protected:
  const class_factory * const class_factory_;
  notify_callback preprocess_notify_;
  notify_callback postprocess_notify_;
  bool enabled_;
};

class c_image_processor :
    public std::vector<c_image_processor_routine::ptr>
{
  std::string name_;
  mutable std::string filename_;
  bool enable_debug_messages_ = false;

public:
  typedef c_image_processor this_class;
  typedef std::vector<c_image_processor_routine::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  c_image_processor(const std::string & objname, const std::string & filename = "");

  static ptr create(const std::string & objname);
  static ptr load(const std::string & filename);
  static ptr deserialize(c_config_setting settings);

  bool save(const std::string & path_or_filename = "",
      const std::string & objname = "" ,
      bool incude_disabled_functions = true) const;

  bool serialize(c_config_setting settings,
      const std::string & objname = "",
      bool incude_disabled_functions = true ) const;

  iterator find(const c_image_processor_routine::ptr & p);
  const_iterator find(const c_image_processor_routine::ptr & p) const;

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) const;

  void set_name(const std::string & v)
  {
    name_ = v;
  }

  const std::string & name() const
  {
    return name_;
  }

  const char * cname() const
  {
    return name_.c_str();
  }

  void set_filename(const std::string & v)
  {
    filename_ = v;
  }

  const std::string & filename() const
  {
    return filename_;
  }

  const char * cfilename() const
  {
    return filename_.c_str();
  }

  void set_enable_debug_messages(bool v)
  {
    enable_debug_messages_ = v;
  }

  bool enable_debug_messages() const
  {
    return enable_debug_messages_;
  }

};


class c_image_processor_collection :
    public std::vector<c_image_processor::ptr>
{
public:
  typedef c_image_processor_collection this_class;
  typedef std::vector<c_image_processor::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create();
  static ptr create(c_config_setting settings);

  bool load(const std::string & input_directrory);
  bool deserialize(c_config_setting settings);

  bool save(const std::string & output_directrory = "") const;
  bool serialize(c_config_setting settings) const;

  iterator find(const std::string & name);
  const_iterator find(const std::string & name) const;

  c_image_processor::ptr get(const std::string & name) const;

  iterator find(const c_image_processor::ptr &);
  const_iterator find(const c_image_processor::ptr &) const;


  static const std::string & default_processor_collection_path();
  static void set_default_processor_collection_path(const std::string & );

  static c_image_processor_collection::ptr default_instance();

protected:
  static std::string default_processor_collection_path_;
  static c_image_processor_collection::ptr default_instance_;
};


#endif /* __c_image_processor_h__ */

