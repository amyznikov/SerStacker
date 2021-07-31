/*
 * c_image_processor.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __c_image_processor_h__
#define __c_image_processor_h__

#include <opencv2/opencv.hpp>
#include <core/settings.h>
#include <core/proc/normalize.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/smap.h>

///////////////////////////////////////////////////////////////////////////////

class c_image_processor_routine
{
public:
  typedef c_image_processor_routine this_class;
  typedef std::shared_ptr<this_class> ptr;
  typedef std::function<this_class::ptr()> factory;

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

  static const std::vector<const class_factory*> & class_list();

  const class_factory * classfactory() const;

  virtual ~c_image_processor_routine() = default;


  static ptr create(const std::string & class_name);
  static ptr create(c_config_setting settings);
  virtual bool load(c_config_setting settings);
  virtual bool save(c_config_setting settings) const;

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


  virtual bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) = 0;


protected:
  c_image_processor_routine(const class_factory * _class_factory, bool enabled = true)
    : class_factory_(_class_factory), enabled_(enabled)
  {
  }

protected:
  const class_factory * const class_factory_;
  bool enabled_;
};

class c_image_processor :
    public std::vector<c_image_processor_routine::ptr>
{
  std::string name_;
  bool enabled_ = true;

public:
  typedef c_image_processor this_class;
  typedef std::vector<c_image_processor_routine::ptr> base;
  typedef std::shared_ptr<this_class> ptr;


  c_image_processor(const std::string & objname);

  static ptr create(const std::string & objname);
  static ptr load(const std::string & filename);
  static ptr load(c_config_setting settings);
  bool save(const std::string & filename) const;
  bool save(c_config_setting settings) const;


  void set_name(const std::string & v)
  {
    name_ = v;
  }

  const std::string & name() const
  {
    return name_;
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) const
  {
    if ( enabled_ ) {
      for ( const c_image_processor_routine::ptr & processor : *this ) {
        if ( processor && processor->enabled() ) {
          if ( !processor->process(image, mask) ) {
            return false;
          }
        }
      }
    }
    return true;
  }

};


inline bool save_settings(c_config_setting settings, const c_image_processor::ptr & obj)
{
  return obj->save(settings);
}

inline bool save_settings(c_config_setting settings, const c_image_processor & obj)
{
  return obj.save(settings);
}


class c_image_processor_collection :
    public std::vector<c_image_processor::ptr>
{
public:
  typedef c_image_processor_collection this_class;
  typedef std::vector<c_image_processor::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create()
  {
    return ptr(new this_class());
  }

  static ptr create(c_config_setting settings)
  {
    ptr obj(new this_class());
    if ( obj->load(settings) ) {
      return obj;
    }
    return nullptr;
  }

  bool load(c_config_setting settings)
  {
    clear();

    if ( !settings ) {
      return false;
    }

    c_config_setting processors_list_item =
        settings["processors"];

    if ( !processors_list_item || !processors_list_item.isList() ) {
      return false;
    }

    const int num_processors = processors_list_item.length();

    reserve(num_processors);

    for ( int i = 0; i < num_processors; ++i ) {

      c_config_setting processor_item =
          processors_list_item.get_element(i);

      if ( processor_item && processor_item.isGroup() ) {
        c_image_processor::ptr processor = c_image_processor::load(processor_item);
        if ( processor ) {
          emplace_back(processor);
        }
      }
    }

    return true;
  }

  bool save(c_config_setting settings) const
  {
    if ( !settings ) {
      return false;
    }

    c_config_setting processors_list_item =
        settings.add_list("processors");

    for ( const c_image_processor::ptr & processor : *this ) {
      if ( processor && !processor->save(processors_list_item.add_element(CONFIG_TYPE_GROUP)) ) {
        return false;
      }
    }

    return true;
  }

};


inline bool save_settings(c_config_setting settings, const c_image_processor_collection::ptr & obj)
{
  return obj->save(settings);
}

inline bool save_settings(c_config_setting settings, const c_image_processor_collection & obj)
{
  return obj.save(settings);
}



#endif /* __c_image_processor_h__ */
