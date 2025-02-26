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
#include <core/io/c_data_frame.h>
#include <core/ctrlbind/ctrlbind.h>
#include <mutex>

#define DECLARE_DATA_PROCESSOR_CLASS_FACTORY(class_name, display_name, tooltip ) \
    typedef class_name this_class; \
    typedef std::shared_ptr<this_class> sptr; \
    typedef c_data_frame_processor_routine base; \
    struct c_class_factory : public base::class_factory { \
      c_class_factory() : \
        base::class_factory(#class_name, \
            display_name, \
            "<strong>" #class_name "</strong>: " tooltip , \
            factory([]() {\
                return sptr(new this_class());\
         })) {} \
    }; \
    \
    static const c_class_factory* class_factory_instance() { \
      static c_class_factory class_factory_instance_; \
      return &class_factory_instance_; \
    } \
    class_name(bool enabled = false) : \
      base(class_factory_instance(), enabled) { \
    } \
    static sptr create(bool enabled = false) { \
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
    return _class_factory->class_name;
  }

  const std::string & tooltip() const
  {
    return _class_factory->tooltip;
  }

  void set_display_name(const std::string & v)
  {
    _display_name = v;
  }

  const std::string & display_name() const
  {
    return _display_name;
  }

  std::mutex & mutex()
  {
    return _mtx;
  }

  bool enabled() const
  {
    return _enabled;
  }

  void set_enabled(bool v)
  {
    if( _enabled != v ) {
      _enabled = v;
      onstatechanged();
    }
  }

  void set_ignore_mask(bool v)
  {
    _ignore_mask = v;
  }

  bool ignore_mask() const
  {
    return _ignore_mask;
  }

  static sptr create(const std::string & class_name);
  static sptr create(c_config_setting settings);

  virtual void get_parameters(std::vector<c_ctrl_bind> * ctls);
  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool process(c_data_frame::sptr & dataframe) = 0;

protected:
  virtual void onstatechanged()
  {
  }

protected:
  c_data_frame_processor_routine(const class_factory * cfactory, bool enabled = false) :
      _class_factory(cfactory),
      _display_name(cfactory->display_name),
      _enabled(enabled)
  {
  }

protected:
  std::mutex _mtx;
  const class_factory * const _class_factory;
  std::string _display_name;
  bool _enabled;
  bool _ignore_mask = false;
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
