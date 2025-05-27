/*
 * c_image_processor.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __c_image_processor_h__
#define __c_image_processor_h__

#include <opencv2/opencv.hpp>
#include <core/settings/opencv_settings.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/io/load_image.h>
#include <map>
///////////////////////////////////////////////////////////////////////////////

struct c_image_processor_artifact
{
  cv::Mat image;
  cv::Mat mask;
};

typedef std::map<std::string, c_image_processor_artifact, std::less<std::string>>
  c_image_processor_artifacts;

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
      mutex().lock();
    }

    ~class_list_guard_lock() {
      mutex().unlock();
    }

    static std::mutex & mutex() {
      static std::mutex mtx_;
      return mtx_;
    }
  };


  enum DATA_CHANNEL {
    IMAGE,
    MASK
  };


  static void register_class_factory(const class_factory * class_factory);
  static void register_all();

  static const std::vector<const class_factory*> & class_list();

  const class_factory * classfactory() const;

  virtual ~c_image_processor_routine() = default;

  static ptr create(const std::string & class_name);
  static ptr create(c_config_setting settings);

  virtual bool serialize(c_config_setting settings, bool save);

  virtual void parameter_changed();

  const std::string & class_name() const
  {
    return _class_factory->class_name;
  }

  const std::string & display_name() const
  {
    return _display_name;
  }

  void set_display_name(const std::string & v)
  {
    _display_name = v;
  }

  const std::string & tooltip() const
  {
    return _class_factory->tooltip;
  }

  std::mutex & mutex()
  {
    return _mtx;
  }

  void set_enabled(bool v)
  {
    _enabled = v;
  }

  bool enabled() const
  {
    return _enabled;
  }

  void set_input_channel(DATA_CHANNEL v )
  {
    _input_channel = v;
  }

  DATA_CHANNEL input_channel() const
  {
    return _input_channel;
  }

  void set_output_channel(DATA_CHANNEL v )
  {
    _output_channel = v;
  }

  DATA_CHANNEL output_channel() const
  {
    return _output_channel;
  }

  void set_ignore_mask(bool v)
  {
    _ignore_mask = v;
  }

  bool ignore_mask() const
  {
    return _ignore_mask;
  }

  void set_preprocess_notify_callback(const notify_callback & preprocess_notify)
  {
    _preprocess_notify = preprocess_notify;
  }

  const notify_callback & preprocess_notify_callback() const
  {
    return _preprocess_notify;
  }

  void set_postprocess_notify_callback(const notify_callback & postprocess_notify)
  {
    _postprocess_notify = postprocess_notify;
  }

  const notify_callback & postprocess_notify_callback() const
  {
    return _postprocess_notify;
  }

  void emit_preprocess_notify(cv::InputOutputArray image, cv::InputOutputArray mask) {
    if ( _preprocess_notify ) {
      _preprocess_notify(this, image, mask);
    }
  }

  void emit_postprocess_notify(cv::InputOutputArray image, cv::InputOutputArray mask) {
    if ( _postprocess_notify ) {
      _postprocess_notify(this, image, mask);
    }
  }

  virtual bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) = 0;


  virtual void get_parameters(std::vector<c_ctrl_bind> * ctls)
  {
    BIND_PCTRL(ctls, ignore_mask, "ignore_mask");
  }

  /////////////////////////////////////////////////////////////////////////////
  static c_image_processor_artifacts & artifacts()
  {
    static c_image_processor_artifacts artifacts_;
    return artifacts_;
  }

  static void clear_artifacts()
  {
    artifacts().clear();
  }

  static void add_artifact(const std::string & name, cv::InputArray image, cv::InputArray mask = cv::noArray())
  {
    auto ii =
        artifacts().find(name);

    if( ii == artifacts().end() ) {
      ii = artifacts().emplace(name, std::move(c_image_processor_artifact())).first;
    }

    image.getMat().copyTo(ii->second.image);
    mask.getMat().copyTo(ii->second.mask);
  }

  static bool get_artifact(const std::string & name, cv::OutputArray image, cv::OutputArray mask)
  {
    const auto ii =
        artifacts().find(name);

    if ( ii != artifacts().end() ) {
      ii->second.image.copyTo(image);
      ii->second.mask.copyTo(mask);
      return true;
    }

    if ( load_image(name, image, mask) ) {
      return true;
    }

    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  static c_image_processor_artifacts & globals()
  {
    static c_image_processor_artifacts artifacts_;
    return artifacts_;
  }

  static void clear_globals()
  {
    globals().clear();
  }

  static void add_global(const std::string & name, cv::InputArray image, cv::InputArray mask = cv::noArray())
  {
    auto ii =
        globals().find(name);

    if( ii == artifacts().end() ) {
      ii = globals().emplace(name, std::move(c_image_processor_artifact())).first;
    }

    image.getMat().copyTo(ii->second.image);
    mask.getMat().copyTo(ii->second.mask);
  }

  static bool get_global(const std::string & name, cv::OutputArray image, cv::OutputArray mask)
  {
    const auto ii =
        globals().find(name);

    if ( ii != globals().end() ) {
      ii->second.image.copyTo(image);
      ii->second.mask.copyTo(mask);
      return true;
    }

    return false;
  }
  /////////////////////////////////////////////////////////////////////////////

protected:
  c_image_processor_routine(const class_factory * cfactory, bool enabled = true) :
      _class_factory(cfactory),
      _enabled(enabled),
      _display_name(_class_factory->display_name)
  {
  }

  virtual bool initialize()
  {
    return true;
  }

protected:
  const class_factory * const _class_factory;
  notify_callback _preprocess_notify;
  notify_callback _postprocess_notify;
  std::string _display_name;
  std::mutex _mtx;
  DATA_CHANNEL _input_channel = IMAGE;
  DATA_CHANNEL _output_channel = IMAGE;
  bool _ignore_mask = false;
  bool _enabled;
};

#define DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(class_name, display_name, tooltip ) \
    typedef class_name this_class; \
    typedef c_image_processor_routine base; \
    typedef std::shared_ptr<this_class> ptr; \
    struct c_class_factory : public base::class_factory { \
      c_class_factory() : \
        base::class_factory(#class_name, display_name, "<strong>" #class_name "</strong>: " tooltip , factory([]() {return ptr(new this_class());})) {} \
    }; \
    \
    static const c_class_factory* class_factory_instance() { \
      static c_class_factory class_factory_instance_; \
      return &class_factory_instance_; \
    } \
    class_name(bool enabled = true) : \
      base(class_factory_instance(), enabled) { \
    } \
    static ptr create(bool enabled = true) { \
        return ptr(new this_class(enabled)); \
    } \


class c_image_processor :
    public std::vector<c_image_processor_routine::ptr>
{
public:
  typedef c_image_processor this_class;
  typedef std::vector<c_image_processor_routine::ptr> base;
  typedef std::shared_ptr<this_class> sptr;

  struct edit_lock:
      public std::unique_lock<std::mutex>
  {
    typedef edit_lock this_class;
    typedef std::unique_lock<std::mutex> base;

    edit_lock(const sptr & ) : base(emutex())
    {
    }
    edit_lock(const c_image_processor * ) : base(emutex())
    {
    }
  };


  c_image_processor(const std::string & objname, const std::string & filename = "");

  static sptr create(const std::string & objname);
  static sptr load(const std::string & filename);
  static sptr deserialize(c_config_setting settings);

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

  static std::mutex & emutex()
  {
    static std::mutex emutex_;
    return emutex_;
  }

protected:
  std::string name_;
  mutable std::string filename_;
  bool enable_debug_messages_ = false;
};


class c_image_processor_collection :
    public std::vector<c_image_processor::sptr>
{
public:
  typedef c_image_processor_collection this_class;
  typedef std::vector<c_image_processor::sptr> base;
  typedef std::shared_ptr<this_class> sptr;

  static sptr create();
  static sptr create(c_config_setting settings);

  bool load(const std::string & input_directrory);
  bool deserialize(c_config_setting settings);

  bool save(const std::string & output_directrory = "") const;
  bool serialize(c_config_setting settings) const;

  iterator find(const std::string & name);
  const_iterator find(const std::string & name) const;

  c_image_processor::sptr get(const std::string & name) const;

  iterator find(const c_image_processor::sptr &);
  const_iterator find(const c_image_processor::sptr &) const;


  static const std::string & default_processor_collection_path();
  static void set_default_processor_collection_path(const std::string & );

  static c_image_processor_collection::sptr default_instance();

protected:
  static std::string default_processor_collection_path_;
  static c_image_processor_collection::sptr default_instance_;
};


inline bool save_settings(c_config_setting settings, const std::string & name,
    const c_image_processor::sptr & processor)
{
  if( processor ) {
    save_settings(settings, name, processor->name());
  }
  return true;
}

inline bool load_settings(c_config_setting settings, const std::string & name,
    c_image_processor::sptr * processor)
{
  std::string s;
  if( load_settings(settings, name, &s) && !s.empty() ) {
    *processor = c_image_processor_collection::default_instance()->get(s);
  }
  return true;
}

#define SERIALIZE_IMAGE_PROCESSOR(settings, save, obj, proc) \
  if( (save) ) { \
    if ( (obj).proc ) { \
      save_settings(settings, #proc, ((obj).proc)->name()); \
    }\
  } \
  else { \
    std::string s; \
    if( load_settings(settings, #proc, &s) && !s.empty() ) { \
      (obj).proc = c_image_processor_collection::default_instance()->get(s); \
    } \
  }

#define LOAD_IMAGE_PROCESSOR(settings, obj, proc) \
    load_settings(settings, #proc, &(obj).proc)

#define SAVE_IMAGE_PROCESSOR(settings, obj, proc) \
    save_settings(settings, #proc, (obj).proc)

#endif /* __c_image_processor_h__ */

