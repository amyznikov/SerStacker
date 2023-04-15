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

///////////////////////////////////////////////////////////////////////////////

class c_image_processor_routine;

enum c_image_processor_ctl_type {
  c_image_processor_ctl_numeric_box = 0,
  c_image_processor_ctl_check_box,
  c_image_processor_ctl_enum_combobox,
  c_image_processor_ctl_flags_chkbox,
  c_image_processor_ctl_browse_for_existing_file,
  c_image_processor_ctl_begin_group,
  c_image_processor_ctl_end_group,
  c_image_processor_ctl_spinbox,
};

struct c_image_processor_routine_ctrl {
  std::string name;
  std::string tooltip;
  c_image_processor_ctl_type ctl_type;
  double min = 0, max = 100, step = 1;
  const c_enum_member * (*get_enum_members)() = nullptr;
  std::function<std::string(void)> get_value;
  std::function<void (const std::string &)> set_value;
};

#define ADD_IMAGE_PROCESSOR_CTRL2(ctls, param, cname, desc) \
  if ( true ) { \
    c_image_processor_ctl_type ctype; \
    if( std::is_enum<decltype(param())>::value ) { \
      ctype = c_image_processor_ctl_enum_combobox; \
    } \
    else if( std::is_same<decltype(param()), bool>::value ) { \
      ctype = c_image_processor_ctl_check_box; \
    } \
    else { \
      ctype = c_image_processor_ctl_numeric_box; \
    } \
    c_image_processor_routine_ctrl tmp; \
      tmp.name = #cname; \
      tmp.tooltip = desc; \
      tmp.ctl_type = ctype; \
      tmp.get_enum_members = get_members_of<decltype(param())>(); \
      tmp.get_value = [this](void) { \
        return toString(param()); \
      }; \
      tmp.set_value = [this](const std::string & s) { \
        std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
        if( fromString(s, &v) ) { \
          std::lock_guard<std::mutex> lock(this->mutex()); \
          set_##param(v); \
        } \
      }; \
    (ctls)->emplace_back(tmp); \
  }

#define ADD_IMAGE_PROCESSOR_FLAGS_CTRL(ctls, param, cname, enumtype, desc) \
  if ( true ) { \
    c_image_processor_routine_ctrl tmp = { \
        .name = #cname, \
        .tooltip = desc, \
        .ctl_type = c_image_processor_ctl_flags_chkbox, \
        .get_enum_members = get_members_of<enumtype>(), \
    }; \
    tmp.get_value = [this](void) { \
        return toString(param()); \
      }; \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(this->mutex()); \
        set_##param(v); \
      } \
    }; \
    (ctls)->emplace_back(tmp); \
  }

#define ADD_IMAGE_PROCESSOR_SPINBOX_CTRL(ctls, param, minvalue, maxvalue, stepvalue, desc) \
  if ( true ) { \
    c_image_processor_routine_ctrl tmp = { \
        .name = #param, \
        .tooltip = desc, \
        .ctl_type = c_image_processor_ctl_spinbox, \
        .min = minvalue, \
        .max = maxvalue, \
        .step = stepvalue, \
    }; \
    tmp.get_value = [this](void) { \
        return toString(param()); \
      }; \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(this->mutex()); \
        set_##param(v); \
      } \
    }; \
    (ctls)->emplace_back(tmp); \
  }

#define ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, param, desc) \
  if ( true ) { \
    c_image_processor_routine_ctrl tmp = { \
      .name = #param, \
      .tooltip = desc, \
      .ctl_type = c_image_processor_ctl_browse_for_existing_file, \
    }; \
    tmp.get_value = [this](void) { \
        return param(); \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::lock_guard<std::mutex> lock(this->mutex()); \
      set_##param(s); \
    }; \
   (ctls)->emplace_back(tmp); \
  }


#define ADD_IMAGE_PROCESSOR_CTRL(ctls, param, desc) \
    ADD_IMAGE_PROCESSOR_CTRL2(ctls, param, param, desc)

#define ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, cname, desc) \
  if ( true ) { \
    c_image_processor_routine_ctrl tmp = { \
        .name = cname, \
        .tooltip = desc, \
        .ctl_type = c_image_processor_ctl_begin_group, \
    }; \
    (ctls)->emplace_back(tmp); \
  }

#define END_IMAGE_PROCESSOR_CTRL_GROUP(ctls) \
  if ( true ) { \
    c_image_processor_routine_ctrl tmp = { \
        .name = "", \
        .tooltip = "", \
        .ctl_type = c_image_processor_ctl_end_group, \
    }; \
    (ctls)->emplace_back(tmp); \
  }


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

  static void register_class_factory(const class_factory * class_factory);
  static void register_all();

  static const std::vector<const class_factory*> & class_list();

  const class_factory * classfactory() const;

  virtual ~c_image_processor_routine() = default;

  static ptr create(const std::string & class_name);
  static ptr create(c_config_setting settings);

  virtual bool serialize(c_config_setting settings, bool save);


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

  std::mutex & mutex()
  {
    return mtx_;
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


  virtual void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * params)
  {
  }

protected:
  c_image_processor_routine(const class_factory * _class_factory, bool enabled = true)
    : class_factory_(_class_factory), enabled_(enabled)
  {
  }

protected:


protected:
  const class_factory * const class_factory_;
  notify_callback preprocess_notify_;
  notify_callback postprocess_notify_;
  std::mutex mtx_;
  bool enabled_;
};

#define DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(class_name, display_name, tooltip ) \
    typedef class_name this_class; \
    typedef c_image_processor_routine base; \
    typedef std::shared_ptr<this_class> ptr; \
    struct c_class_factory : public base::class_factory { \
      c_class_factory() : \
        base::class_factory(#class_name, display_name, tooltip , factory([]() {return ptr(new this_class());})) {} \
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
  typedef std::shared_ptr<this_class> ptr;

  static ptr create();
  static ptr create(c_config_setting settings);

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

  static c_image_processor_collection::ptr default_instance();

protected:
  static std::string default_processor_collection_path_;
  static c_image_processor_collection::ptr default_instance_;
};



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



#endif /* __c_image_processor_h__ */

