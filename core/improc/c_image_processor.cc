/*
 * c_image_processor.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include <functional>
#include "c_image_processor.h"

static std::vector<const c_image_processor_routine::class_factory*> c_image_processor_routine_class_list_;

c_image_processor_routine::class_factory::class_factory(const std::string & _class_name,
    const std::string & _display_name,
    const std::string & _tooltip,
    const std::function<c_image_processor_routine::ptr()> & _create_instance) :
        class_name(_class_name), display_name(_display_name), tooltip(_tooltip), create_instance(_create_instance)
{
  class_list_guard_lock guard_lock;
  c_image_processor_routine_class_list_.emplace_back(this);
}


const std::vector<const c_image_processor_routine::class_factory*> & c_image_processor_routine::class_list()
{
  return c_image_processor_routine_class_list_;
}


const c_image_processor_routine::class_factory * c_image_processor_routine::classfactory() const
{
  return class_factory_;
}


c_image_processor::c_image_processor(const std::string & objname) :
    name_(objname)
{
}

c_image_processor::ptr c_image_processor::create(const std::string & objname)
{
  return ptr(new this_class(objname));
}

c_image_processor::ptr c_image_processor::load(const std::string & filename)
{
  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return nullptr;
  }


  std::string object_class;
  if ( !load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("load_settings(object_class) fails", filename.c_str());
    return nullptr;
  }

  if ( object_class != "c_image_processor" ) {
    CF_FATAL("Incorect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return nullptr;
  }

  c_config_setting root = cfg.root().get("c_image_processor");
  if ( !root || !root.isGroup() ) {
    CF_FATAL("group 'c_image_processor' is not found in file '%s''",
        filename.c_str());
    return nullptr;
  }

  return load(root);
}

c_image_processor::ptr c_image_processor::load(c_config_setting settings)
{
  std::string objname;

  if ( !settings.get("name", &objname) || objname.empty() ) {
    CF_ERROR("c_image_processor: settings.get('name') fails");
    return nullptr;
  }

  ptr obj = create(objname);
  if ( !obj ) {
    CF_ERROR("c_image_processor::create(objname=%s) fails", objname.c_str());
    return nullptr;
  }

  settings.get("enabled", &obj->enabled_);

  c_config_setting chain =
      settings["chain"];

  if ( chain && !chain.isList() ) {
    CF_ERROR("c_image_processor::load(objname=%s): 'chain' item must be libconfig list", objname.c_str());
    return nullptr;
  }

  if ( chain ) {

    const int chain_length = chain.length();

    obj->reserve(chain_length);

    for ( int i = 0; i < chain_length; ++i ) {

      c_config_setting group =
          chain.get_element(i);

      if ( !group || !group.isGroup() ) {
        CF_ERROR("c_image_processor:  chain.get_element(objname=%s, index=%d, type=GROUP) fails", objname.c_str(), i);
        return nullptr;
      }

      c_image_processor_routine::ptr routine =
          c_image_processor_routine::create(group);

      if ( !routine ) {
        CF_ERROR("c_image_processor_routine::create(objname=%s, chain index=%d) fails", objname.c_str(), i);
        return nullptr;
      }

      obj->emplace_back(routine);
    }
  }

  return obj;
}


bool c_image_processor::save(const std::string & filename) const
{
  c_config cfg(filename);

  time_t t = time(0);

  if ( !save_settings(cfg.root(), "object_class", std::string("c_image_processor")) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save(cfg.root().add_group("c_image_processor"))) {
    CF_FATAL("save(c_image_processor) fails");
    return false;
  }

  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  return true;
}

bool c_image_processor::save(c_config_setting settings) const
{
  if ( !settings ) {
    CF_ERROR("c_image_processor::save() gets invalid argument: c_config_setting = null");
    return false;
  }

  settings.set("name", name_);
  settings.set("enabled", enabled_);

  c_config_setting chain =
      settings.add_list("chain");

  for ( const c_image_processor_routine::ptr & routine : *this ) {
    if ( routine && !routine->save(chain.add_element(CONFIG_TYPE_GROUP)) ) {
      return false;
    }
  }

  return true;
}


c_image_processor_routine::ptr c_image_processor_routine::create(c_config_setting settings)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine::create(c_config_setting): settings is null");
    return nullptr;
  }

  std::string objname;
  if ( !settings.get("routine", &objname) || objname.empty() ) {
    CF_ERROR("No 'routine' string found");
    return nullptr;
  }

  c_image_processor_routine::ptr routine =
      c_image_processor_routine::create(objname);

  if( !routine ) {
    CF_ERROR("c_image_processor_routine::create(routine=%s) fails", objname.c_str());
    return nullptr;
  }

  if ( !routine->load(settings) ) {
    CF_ERROR("routine->load(routine=%s, c_config_setting) fails", objname.c_str());
    return nullptr;
  }

  return routine;
}

c_image_processor_routine::ptr c_image_processor_routine::create(const std::string & processor_name)
{
  if ( processor_name.empty() ) {
    return nullptr;
  }

  class_list_guard_lock lock;

  const char * cname = processor_name.c_str();
  for ( const class_factory * f : c_image_processor_routine_class_list_ ) {
    if ( strcasecmp(cname, f->class_name.c_str()) == 0 ) {
      return f->create_instance();
    }
  }

  return nullptr;
}


bool c_image_processor_routine::load(c_config_setting settings)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine::load(c_config_setting) : settings is null");
    return false;
  }

  settings.get("enabled", &enabled_);

  return true;
}

bool c_image_processor_routine::save(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  settings.set("routine", class_name());
  settings.set("display_name", display_name());
  settings.set("tooltip", tooltip());
  settings.set("enabled", enabled());

  return true;
}

