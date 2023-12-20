/*
 * c_data_frame_processor.h
 *
 *  Created on: Dec 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dataframe_processor_h__
#define __c_dataframe_processor_h__

#include <opencv2/opencv.hpp>
#include <core/settings/opencv_settings.h>
// #include <core/feature2d/feature2d.h>
#include <mutex>
#include <core/io/c_data_frame.h>

enum c_data_processor_ctl_type {
  c_data_processor_ctl_numeric_box = 0,
  c_data_processor_ctl_check_box,
  c_data_processor_ctl_enum_combobox,
  c_data_processor_ctl_flags_chkbox,
  c_data_processor_ctl_browse_for_existing_file,
  c_data_processor_ctl_browse_for_directory,
  c_data_processor_ctl_begin_group,
  c_data_processor_ctl_end_group,
  c_data_processor_ctl_spinbox,
  c_data_processor_ctl_double_slider,
  c_data_processor_ctl_math_expression,
};

struct c_data_processor_routine_ctrl
{
  std::string name;
  std::string tooltip;
  c_data_processor_ctl_type ctl_type;

  struct {
    double min = 0, max = 100, step = 1;
  } range;

  struct {
    int rows = 0, cols = 0;
  } matx;

  const c_enum_member * (*get_enum_members)() = nullptr;

  std::function<bool (std::string *)> get_value;
  std::function<bool(const std::string&)> set_value;
  std::function<bool ()> is_enabled;
};



#define ADD_DATA_PROCESSOR_CTRL(ctls, param, cname, desc) \
  if ( true ) { \
    c_data_processor_ctl_type ctype; \
    if( std::is_enum<decltype(param())>::value ) { \
      ctype = c_data_processor_ctl_enum_combobox; \
    } \
    else if( std::is_same<decltype(param()), bool>::value ) { \
      ctype = c_data_processor_ctl_check_box; \
    } \
    else { \
      ctype = c_data_processor_ctl_numeric_box; \
    } \
    c_data_processor_routine_ctrl tmp; \
      tmp.name = cname; \
      tmp.tooltip = desc; \
      tmp.ctl_type = ctype; \
      tmp.get_enum_members = get_members_of<decltype(param())>(); \
      tmp.get_value = [this](std::string * v) -> bool { \
        *v = toString(param()); \
        return true; \
      }; \
      tmp.set_value = [this](const std::string & s) { \
        std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
        if( fromString(s, &v) ) { \
          std::lock_guard<std::mutex> lock(this->mutex()); \
          set_##param(v); \
          return true; \
        } \
        return false; \
      }; \
    (ctls)->emplace_back(tmp); \
  }

#define ADD_DATA_PROCESSOR_FLAGS_CTRL(ctls, param, cname, enumtype, desc) \
  if ( true ) { \
    c_data_processor_routine_ctrl tmp; \
    tmp.name = cname, \
    tmp.tooltip = desc; \
    tmp.ctl_type = c_data_processor_ctl_flags_chkbox, \
    tmp.get_enum_members = get_members_of<enumtype>(), \
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

#define ADD_DATA_PROCESSOR_SPINBOX_CTRL(ctls, param, minvalue, maxvalue, stepvalue, desc) \
  if ( true ) { \
    c_data_processor_routine_ctrl tmp = { \
        .name = #param, \
        .tooltip = desc, \
        .ctl_type = c_data_processor_ctl_spinbox, \
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

#define ADD_DATA_PROCESSOR_DOUBLE_SLIDER_CTRL(ctls, param, minvalue, maxvalue, stepvalue, desc) \
  if ( true ) { \
    c_data_processor_routine_ctrl tmp = { \
        .name = #param, \
        .tooltip = desc, \
        .ctl_type = c_data_processor_ctl_double_slider, \
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

#define ADD_DATA_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, param, desc) \
  if ( true ) { \
    c_data_processor_routine_ctrl tmp = { \
      .name = #param, \
      .tooltip = desc, \
      .ctl_type = c_data_processor_ctl_browse_for_existing_file, \
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

#define ADD_DATA_PROCESSOR_CTRL_BROWSE_FOR_DIRECTORY(ctls, param, desc) \
  if ( true ) { \
    c_data_processor_routine_ctrl tmp = { \
      .name = #param, \
      .tooltip = desc, \
      .ctl_type = c_data_processor_ctl_browse_for_directory, \
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



#define DECLARE_DATA_PROCESSOR_CLASS_FACTORY(class_name, display_name, tooltip ) \
    typedef class_name this_class; \
    typedef std::shared_ptr<this_class> sptr; \
    typedef c_data_frame_processor_routine base; \
    struct c_class_factory : public base::class_factory { \
      c_class_factory() : \
        base::class_factory(#class_name, \
            display_name, \
            tooltip , \
            factory([]() {\
                return sptr(new this_class());\
         })) {} \
    }; \
    \
    static const c_class_factory* class_factory_instance() { \
      static c_class_factory class_factory_instance_; \
      return &class_factory_instance_; \
    } \
    class_name(bool enabled = true) : \
      base(class_factory_instance(), enabled) { \
    } \
    static sptr create(bool enabled = true) { \
        return sptr(new this_class(enabled)); \
    } \

class c_data_frame_processor_routine
{
public:
  typedef c_data_frame_processor_routine this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::function<this_class::sptr()> factory;

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

  virtual ~c_data_frame_processor_routine() = default;

  const class_factory * classfactory() const;

  static const std::vector<const class_factory*> & class_list();

  static void register_class_factory(const class_factory * class_factory);

  static void register_all();

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

  bool enabled() const
  {
    return enabled_;
  }

  void set_enabled(bool v)
  {
    if( enabled_ != v ) {
      enabled_ = v;
      onstatechanged();
    }
  }

  void set_ignore_mask(bool v)
  {
    ignore_mask_ = v;
  }

  bool ignore_mask() const
  {
    return ignore_mask_;
  }

  static sptr create(const std::string & class_name);
  static sptr create(c_config_setting settings);

  virtual void get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls);
  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool process(c_data_frame::sptr & dataframe) = 0;

protected:
  virtual void onstatechanged()
  {
  }

protected:
  c_data_frame_processor_routine(const class_factory * _class_factory, bool enabled = true) :
    class_factory_(_class_factory), enabled_(enabled)
  {
  }

protected:
  std::mutex mtx_;
  const class_factory * const class_factory_;
  bool enabled_;
  bool ignore_mask_ = false;
};


class c_data_frame_processor
{
public:
  typedef c_data_frame_processor this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::vector<c_data_frame_processor_routine::sptr>::iterator iterator;
  typedef std::vector<c_data_frame_processor_routine::sptr>::const_iterator const_iterator;

  struct edit_lock :
      public std::unique_lock<std::mutex>
  {
    typedef edit_lock this_class;
    typedef std::unique_lock<std::mutex> base;

    edit_lock(const sptr & ) :
      base(c_data_frame_processor::mutex())
    {
    }

    edit_lock(const c_data_frame_processor * ) :
      base(c_data_frame_processor::mutex())
    {
    }
  };


  c_data_frame_processor(const std::string & objname, const std::string & filename = "");
  virtual ~c_data_frame_processor() = default;

  static sptr create(const std::string & objname);
  static sptr load(const std::string & filename);
  static sptr deserialize(c_config_setting settings);

  bool save(const std::string & path_or_filename = "",
      const std::string & objname = "" ,
      bool incude_disabled_functions = true) const;

  bool serialize(c_config_setting settings,
      const std::string & objname = "",
      bool incude_disabled_functions = true ) const;

  virtual bool process(c_data_frame::sptr & dataframe);

  static std::mutex & mutex()
  {
    static std::mutex mutex_;
    return mutex_;
  }

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

  const std::vector<c_data_frame_processor_routine::sptr> & routines() const
  {
    return routines_;
  }

  iterator find(const c_data_frame_processor_routine::sptr & routine)
  {
    return std::find(routines_.begin(), routines_.end(), routine);
  }

  const_iterator find(const c_data_frame_processor_routine::sptr & routine) const
  {
    return std::find(routines_.begin(), routines_.end(), routine);
  }

  iterator begin()
  {
    return routines_.begin();
  }

  const_iterator begin() const
  {
    return routines_.begin();
  }

  iterator end()
  {
    return routines_.end();
  }

  const_iterator end() const
  {
    return routines_.end();
  }

  void erase(iterator pos)
  {
    routines_.erase(pos);
  }

  void insert(iterator pos, const c_data_frame_processor_routine::sptr & routine)
  {
    routines_.insert(pos, routine);
  }

  void reserve(size_t size)
  {
    routines_.reserve(size);
  }

  template<typename... _Args>
  void emplace_back(_Args&&... __args)
  {
    routines_.emplace_back(std::forward<_Args>(__args)...);
  }


protected:
  std::string name_;
  mutable std::string filename_;
  std::vector<c_data_frame_processor_routine::sptr> routines_;
};


class c_data_frame_processor_collection :
    public std::vector<c_data_frame_processor::sptr>
{
public:
  typedef c_data_frame_processor_collection this_class;
  typedef std::vector<c_data_frame_processor::sptr> base;
  typedef std::shared_ptr<this_class> sptr;

  static sptr create();
  static sptr create(c_config_setting settings);

  bool load(const std::string & input_directrory);
  bool deserialize(c_config_setting settings);

  bool save(const std::string & output_directrory = "") const;
  bool serialize(c_config_setting settings) const;

  iterator find(const std::string & name);
  const_iterator find(const std::string & name) const;

  c_data_frame_processor::sptr get(const std::string & name) const;

  iterator find(const c_data_frame_processor::sptr &);
  const_iterator find(const c_data_frame_processor::sptr &) const;


  static const std::string & default_processor_collection_path();
  static void set_default_processor_collection_path(const std::string & );

  static c_data_frame_processor_collection::sptr default_instance();

protected:
  static std::string default_processor_collection_path_;
  static c_data_frame_processor_collection::sptr default_instance_;
};



#endif /* __c_dataframe_processor_h__ */
