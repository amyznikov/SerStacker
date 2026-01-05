/*
 * settings.h
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 *
 */

#ifndef __libconfig_settings__h__
#define __libconfig_settings__h__

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <string>
#include <vector>
#include <type_traits>
#include <libconfig.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<class T> struct c_config_type_traits { static constexpr int type = CONFIG_TYPE_GROUP;};
template<class T> struct c_config_type_traits<std::vector<T>> { static constexpr int type = CONFIG_TYPE_LIST;};
template<> struct c_config_type_traits<bool> {static constexpr int type = CONFIG_TYPE_BOOL;};
template<> struct c_config_type_traits<uint8_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<int8_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<int16_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<uint16_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<int32_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<uint32_t> {static constexpr int type = CONFIG_TYPE_INT;};
template<> struct c_config_type_traits<int64_t> {static constexpr int type = CONFIG_TYPE_INT64;};
template<> struct c_config_type_traits<uint64_t> {static constexpr int type = CONFIG_TYPE_INT64;};
template<> struct c_config_type_traits<float> {static constexpr int type = CONFIG_TYPE_FLOAT;};
template<> struct c_config_type_traits<double> { static constexpr int type = CONFIG_TYPE_FLOAT;};
template<> struct c_config_type_traits<const char *> { static constexpr int type = CONFIG_TYPE_STRING;};
template<> struct c_config_type_traits<char *> { static constexpr int type = CONFIG_TYPE_STRING;};
template<> struct c_config_type_traits<std::string> { static constexpr int type = CONFIG_TYPE_STRING;};

template<class T> struct is_config_atomic_type {static constexpr bool value = false;};
template<> struct is_config_atomic_type<bool> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<uint8_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<int8_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<int16_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<uint16_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<int32_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<uint32_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<int64_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<uint64_t> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<float> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<double> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<const char *> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<char *> {static constexpr bool value = true;};
template<> struct is_config_atomic_type<std::string> {static constexpr bool value = true;};



class c_config_setting {
public:

  c_config_setting();

  int type() const;
  int length() const;
  const char * name() const;

  bool isNull() const;
  bool isRoot() const;
  bool isGroup() const;
  bool isString() const;
  bool isArray() const;
  bool isList() const;
  bool isAggregate() const;
  bool isScalar() const;
  bool isNumber() const;
  bool isInteger() const;

  operator bool () const;

  enum Format {
    FormatDefault = CONFIG_FORMAT_DEFAULT,
    FormatHex = CONFIG_FORMAT_HEX
  };

  Format format() const;
  void set_format(Format format);

  uint32_t source_line() const;
  const char * source_file() const;



  bool has_parent() const;

  c_config_setting parent() const;


  c_config_setting add_member(const std::string & name, int type) {
    return add_member(setting_, name.c_str(), type);
  }

  c_config_setting add_element(int type) {
    return add_element(setting_, type);
  }

  bool add_element(const std::string & value) {
    return c_config_setting(add_element(setting_, CONFIG_TYPE_STRING)).set(value);
  }

  c_config_setting add_group(const std::string & name = "")
  {
    return (isList() || isArray()) ? add_element(CONFIG_TYPE_GROUP) :
        add_member(setting_, name.c_str(), CONFIG_TYPE_GROUP);
  }

  c_config_setting add_list(const std::string & name = "") {
    return (isList() || isArray()) ? add_element(CONFIG_TYPE_LIST) :
        add_member(setting_, name.c_str(), CONFIG_TYPE_LIST);
  }

  c_config_setting add_array(const std::string & name = "") {
    return (isList() || isArray()) ? add_element(CONFIG_TYPE_ARRAY) :
        add_member(setting_, name.c_str(), CONFIG_TYPE_ARRAY);
  }



  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
   add(const T & value) { // add element to a list or array
    return add_element(c_config_type_traits<T>::type).set(value);
  }

  c_config_setting get(const std::string & name) const {
    return get_member(setting_, name.c_str());
  }

  c_config_setting get_list(const std::string & name) const {
    c_config_setting cfg = get_member(setting_, name.c_str());
    return cfg.isList() ? cfg : c_config_setting();
  }

  c_config_setting operator [](const std::string & name) const {
    return get_member(setting_, name.c_str());
  }

  c_config_setting operator [](const char * name) const {
    return get_member(setting_, name);
  }

  c_config_setting get_element(uint32_t index) const {
    return get_element(setting_ , index);
  }

  c_config_setting operator [](int index) const {
    return get_element(setting_, index);
  }

  bool remove_element(uint32_t index) {
    return remove_element(setting_, index);
  }

  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline get(T * value) const {
    return get_value(setting_, value);
  }
  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline get(const std::string & name, T * value) const {
    return get_value(get_member(setting_, name.c_str()), value);
  }
  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline get(uint32_t index, T * value) const {
    return get_value(get_element(setting_, index), value);
  }
  template<class T> typename std::enable_if<std::is_enum<T>::value, bool>::type
  inline get(const std::string & name, T * v) {
    std::string s;
    return get(name, &s) && (s.empty() || fromString(s, v));
  }


  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline set(const T & value) {
    return set_value(setting_, value);
  }
  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline set(const std::string & name, const T & value) {
    return set_value(add_member(setting_, name.c_str(), c_config_type_traits<T>::type), value);
  }
  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
  inline set(uint32_t index, const T & value) {
    return set_value(get_element(setting_, index), value);
  }
  template<class T> typename std::enable_if<std::is_enum<T>::value, bool>::type
  inline set(const std::string & name, T v) {
    return set(name, toString(v));
  }



protected:
  friend class c_config;
  c_config_setting(config_setting_t *setting);

  static config_setting_t * get_member(const config_setting_t *setting, const char * name);
  static config_setting_t * add_member(config_setting_t *setting, const char * name, int type);

  static config_setting_t * get_element(const config_setting_t *setting, uint32_t index);
  static config_setting_t * add_element(config_setting_t *setting, int type);
  static bool remove_element(config_setting_t *setting, uint32_t index);

  static bool get_value(const config_setting_t *setting, bool * value);
  static bool get_value(const config_setting_t *setting, int8_t * value);
  static bool get_value(const config_setting_t *setting, uint8_t * value);
  static bool get_value(const config_setting_t *setting, int16_t * value);
  static bool get_value(const config_setting_t *setting, uint16_t * value);
  static bool get_value(const config_setting_t *setting, int32_t * value);
  static bool get_value(const config_setting_t *setting, uint32_t * value);
  static bool get_value(const config_setting_t *setting, int64_t * value);
  static bool get_value(const config_setting_t *setting, uint64_t * value);
  static bool get_value(const config_setting_t *setting, double * value);
  static bool get_value(const config_setting_t *setting, float * value);
  static bool get_value(const config_setting_t *setting, const char ** value);
  static bool get_value(const config_setting_t *setting, std::string * value);

  static bool set_value(config_setting_t *setting, bool value);
  static bool set_value(config_setting_t *setting, int32_t value);
  static bool set_value(config_setting_t *setting, uint32_t value);
  static bool set_value(config_setting_t *setting, int64_t value);
  static bool set_value(config_setting_t *setting, uint64_t value);
  static bool set_value(config_setting_t *setting, double value);
  static bool set_value(config_setting_t *setting, float  value);
  static bool set_value(config_setting_t *setting, const char * value);
  static bool set_value(config_setting_t *setting, const std::string & value);

protected:
  config_setting_t * setting_ = nullptr;
};



class c_config {
public:

  // @brief options copy-pasted from libconfig++.h
  enum Option {
    OptionNone = 0,
    OptionAutoConvert = 0x01,
    OptionSemicolonSeparators = 0x02,
    OptionColonAssignmentForGroups = 0x04,
    OptionColonAssignmentForNonGroups = 0x08,
    OptionOpenBraceOnSeparateLine = 0x10,
    OptionAllowScientificNotation = 0x20,
    OptionFsync = 0x40
  };


  // @brief default constructor
  c_config();

  // @brief constructor setting default file name
  c_config(const std::string & filename);

  // @brief destructor
  virtual ~c_config();

  // @brief get default config file name
  const std::string & filename() const;

  // @brief set default config file name
  void set_filename(const std::string & filename);

  void set_options(int options);
  int options() const;

  void set_option(Option option, bool flag);
  bool option(Option option) const;

  void set_auto_convert(bool flag);
  bool auto_convert() const;

  void set_default_format(c_config_setting::Format format);
  c_config_setting::Format default_format() const;

  void set_tab_width(uint16_t width);
  uint16_t tab_width() const;

  void set_float_precision(uint16_t digits);
  uint16_t float_precision() const;

  void set_include_dir(const char *includeDir);
  const char * include_dir() const;

  const virtual char ** evaluate_include_path(const char *path, const char **error);

  // @brief no-exception write current config into specified file
  bool write(const std::string & filename = std::string());

  // @brief write config into in-memory string
  std::string write_string() const;

  // @brief no-exception read config from specified file
  bool read(const std::string & filename = std::string());

  // @brief read config from in-memory string
  bool read_string(const std::string & s);

  // @brief read config from in-memory string
  bool read_string(const char * s);

  void clear();

  c_config_setting root() const;

protected:
  config_t config_;
  c_config_setting::Format defaultFormat_ = c_config_setting::FormatDefault;
  std::string defaultFilename_;

private:
  void construct();
};


template<class T>
typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
inline save_settings(c_config_setting cfg, const T & v) {
  return cfg.set(v);
}
template<class T>
typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
inline load_settings(c_config_setting cfg, T * v) {
  return cfg.get(v);
}






template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline save_settings(c_config_setting cfg, T v) {
  return cfg.set(toString(v));
}
template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline load_settings(c_config_setting cfg, T * v) {
  std::string s;
  return cfg.get(&s) && (s.empty() || fromString(s, v));
}






template<class T>
typename std::enable_if<!std::is_enum<T>::value, bool>::type
inline save_settings(c_config_setting cfg, const std::string & name, const T & v) {
  return save_settings(cfg.add_member(name, c_config_type_traits<T>::type), v);
}
template<class T>
typename std::enable_if<!std::is_enum<T>::value, bool>::type
inline load_settings(c_config_setting cfg, const std::string & name, T * v) {
  return load_settings(cfg.get(name), v);
}



template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline save_settings(c_config_setting cfg, const std::string & name, const T & v) {
  return cfg.set(name, toString(v));
}
template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline load_settings(c_config_setting cfg, const std::string & name, T * v) {
  return cfg.get(name, v);
}






template<class T>
inline bool save_settings(c_config_setting list, const std::vector<T> & values)
{
  if ( list.isList() || list.isArray() ) {
    while ( list.length() > 0 ) {
      list.remove_element(0);
    }
    for ( size_t i = 0, n = values.size(); i < n; ++i ) {
      if ( !save_settings(list.add_element(c_config_type_traits<T>::type), values[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}

template<class T>
inline bool load_settings(c_config_setting list, std::vector<T> * values)
{
  if ( list.isList() || list.isArray() ) {
    const int n = list.length();
    values->resize(n);
    for ( int i = 0; i < n; ++i ) {
      if ( !load_settings(list.get_element(i), &(*values)[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}









template<class T>
inline bool save_c_array(c_config_setting list, const T values[], size_t count)
{
  if ( list.isList() || list.isArray() ) {
    while ( list.length() ) {
      list.remove_element(0);
    }
    for ( size_t i = 0, n = count; i < n; ++i ) {
      if ( !save_settings(list.add_element(c_config_type_traits<T>::type), values[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}

//template<class T>
//inline bool load_settings(c_config_setting list, T values[], size_t max_count)
//{
//  if ( list.isList() || list.isArray() ) {
//    const int n = list.length();
//    values->resize(n);
//    for ( int i = 0; i < n; ++i ) {
//      if ( !load_settings(list.get_element(i), &(*values)[i]) ) {
//        return false;
//      }
//    }
//    return true;
//  }
//  return false;
//}



template<class T>
inline bool save_c_array(c_config_setting cfg, const std::string &name, const T values[], size_t count)
{
  return save_c_array(cfg.add_list(name), values, count);
}

template<class Obj, class getFn, class setFn>
inline bool load_settings(c_config_setting settings, const std::string & propname,
    Obj * obj, getFn get, setFn set)
{
  auto v = (obj->*get)();
  if ( ::load_settings(settings, propname, &v) ) {
    ((obj)->*set)(v);
    return true;
  }
  return false;
}



struct c_libconfig_flag_desc {
  const char * flag;
  uint32_t value;
};

bool libconfig_parse_flags(c_config_setting settings,
    const c_libconfig_flag_desc fdescs[/*ndescs*/],
    uint32_t ndescs, int * flags);

#define BEGIN_LOAD_OPTIONS(cfg) \
  if ( !(cfg) ) { \
    CF_ERROR("libconfig settings is null in %s", __PRETTY_FUNCTION__); \
    return false; \
  } \
  for ( int tmpi = 0, tmpn = (cfg).length(); tmpi < tmpn; ++tmpi ) { \
    c_config_setting current_option = (cfg)[tmpi]; \
    const char * current_option_name = current_option.name(); \
    if ( !current_option_name || !*current_option_name ) { \
      continue; \
    }

#define LOAD_OPTIONS(cfg, opts, param) \
  if ( strcmp(current_option_name, #param) == 0 ) { \
    if ( !load_settings(current_option, &(opts).param) ) { \
      CF_ERROR("load_settings('%s.%s') fails", (cfg).name(), current_option_name); \
      return false; \
    } \
    continue; \
  }

#define LOAD_FLAGS(cfg, opts, param, fdesc) \
  if ( strcmp(current_option_name, #param) == 0 ) { \
    int flags = 0; \
    const uint32_t nflags = sizeof(fdesc) / sizeof(fdesc[0]); \
    if ( !::libconfig_parse_flags(current_option, fdesc, nflags, &flags) ) { \
      CF_ERROR("libconfig_parse_flags(%s.%s) fails", (cfg).name(), current_option_name); \
      return false; \
    } \
    (opts).param = flags; \
    continue; \
  } \


#define END_LOAD_OPTIONS(cfg) \
  CF_ERROR("WARNING: INVALID OR NOT SUPPORTED OPTIONS '%s.%s' in %s", \
      (cfg).name(), current_option_name, __PRETTY_FUNCTION__); \
  } \


//#define LOAD_PROPERTY(cfg, obj, prop) \
//  ::load_settings(cfg, #prop, (obj), \
//      &std::remove_reference<decltype(*obj)>::type::prop, \
//      &std::remove_reference<decltype(*obj)>::type::set_##prop)

#define LOAD_PROPERTY(cfg, obj, prop) \
  if ( true ) { \
    auto v = (obj).prop(); \
    if ( ::load_settings(cfg, #prop,  &v) ) { \
      (obj).set_##prop(v); \
    }\
  }

#define SAVE_PROPERTY(cfg, obj, prop) \
  ::save_settings(cfg, #prop, (obj).prop())

#define SAVE_OPTION(cfg, obj, prop) \
  ::save_settings(cfg, #prop, (obj).prop)

#define LOAD_OPTION(cfg, obj, prop) \
  ::load_settings(cfg, #prop, &(obj).prop)

#define SERIALIZE_PROPERTY(cfg, save, obj, prop) \
  if ((save)) { \
    ::save_settings(cfg, #prop, (obj).prop()); \
  }\
  else { \
    auto v = (obj).prop(); \
    if ( ::load_settings(cfg, #prop,  &v) ) { \
      (obj).set_##prop(v); \
    }\
  }

#define SERIALIZE_OPTION(cfg, save, obj, prop) \
  if ( cfg ) { \
    if ((save)) { \
      ::save_settings((cfg), #prop, (obj).prop); \
    }\
    else { \
      ::load_settings((cfg), #prop, &(obj).prop); \
    } \
  }

#define SERIALIZE_OBJECT(cfg, save, obj) \
  if ( cfg ) { \
    if ((save)) { \
      ::save_settings((cfg), (obj)); \
    }\
    else { \
      ::load_settings((cfg), &(obj)); \
    } \
  }

#define SERIALIZE_GROUP(settings, save, name) \
      ((save) ? settings.add_group(name) : settings[name].isGroup()? settings[name] : c_config_setting())

#define GET_SETTINGS_SECTION(settings, name) \
    (settings[name].isGroup()? settings[name] : c_config_setting())

#endif /* __libconfig_settings__h__ */
