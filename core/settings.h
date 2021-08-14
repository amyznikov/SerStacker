/*
 * settings.h
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 *
 */

#ifndef __qwcc_core_settings_h__
#define __qwcc_core_settings_h__

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <string>
#include <vector>
#include <type_traits>
#include <libconfig.h>
#include <core/debug.h>


template<class T> struct c_config_type_traits { static constexpr int type = CONFIG_TYPE_GROUP;};
template<class T> struct c_config_type_traits<std::vector<T>> { static constexpr int type = CONFIG_TYPE_LIST;};
template<> struct c_config_type_traits<bool> {static constexpr int type = CONFIG_TYPE_BOOL;};
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
  bool isArray() const;
  bool isList() const;
  bool isAggregate() const;
  bool isScalar() const;
  bool isNumber() const;

  operator bool () const;

  enum Format {
    FormatDefault = CONFIG_FORMAT_DEFAULT,
    FormatHex = CONFIG_FORMAT_HEX
  };

  Format format() const;
  void set_format(Format format);

  uint source_line() const;
  const char * source_file() const;



  bool has_parent() const;

  c_config_setting parent() const;


  c_config_setting add_member(const std::string & name, int type) {
    return add_member(setting_, name.c_str(), type);
  }

  c_config_setting add_group(const std::string & name = "") {
    return add_member(setting_, name.c_str(), CONFIG_TYPE_GROUP);
  }

  c_config_setting add_list(const std::string & name = "") {
    return add_member(setting_, name.c_str(), CONFIG_TYPE_LIST);
  }

  c_config_setting add_array(const std::string & name = "") {
    return add_member(setting_, name.c_str(), CONFIG_TYPE_ARRAY);
  }

  c_config_setting add_element(int type) {
    return add_element(setting_, type);
  }


  template<class T> typename std::enable_if<is_config_atomic_type<T>::value, bool>::type
   add(const T & value) { // add element to a list or array
    return add_element(c_config_type_traits<T>::type).set(value);
  }

  c_config_setting get(const std::string & name) const {
    return get_member(setting_, name.c_str());
  }

  c_config_setting operator [](const std::string & name) const {
    return get_member(setting_, name.c_str());
  }

  c_config_setting operator [](const char * name) const {
    return get_member(setting_, name);
  }

  c_config_setting get_element(uint index) const {
    return get_element(setting_ , index);
  }

  c_config_setting operator [](int index) const {
    return get_element(setting_, index);
  }

  bool remove_element(uint index) {
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
  inline get(uint index, T * value) const {
    return get_value(get_element(setting_, index), value);
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
  inline set(uint index, const T & value) {
    return set_value(get_element(setting_, index), value);
  }



protected:
  friend class c_config;
  c_config_setting(config_setting_t *setting);

  static config_setting_t * get_member(const config_setting_t *setting, const char * name);
  static config_setting_t * add_member(config_setting_t *setting, const char * name, int type);

  static config_setting_t * get_element(const config_setting_t *setting, uint index);
  static config_setting_t * add_element(config_setting_t *setting, int type);
  static bool remove_element(config_setting_t *setting, uint index);

  static bool get_value(const config_setting_t *setting, bool * value);
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

  void set_tab_width(ushort width);
  ushort tab_width() const;

  void set_float_precision(ushort digits);
  ushort float_precision() const;

  void set_include_dir(const char *includeDir);
  const char * include_dir() const;

  const virtual char ** evaluate_include_path(const char *path, const char **error);

  // @brief no-exception write current config into specified file
  bool write(const std::string & filename = std::string());

  // @brief no-exception read config from specified file
  bool read(const std::string & filename = std::string());

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
  return cfg.set(toStdString(v));
}
template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline load_settings(c_config_setting cfg, T * v) {
  std::string s;
  return cfg.get(&s) ? *v = fromStdString(s, *v), true : false;
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
  return cfg.set(name, toStdString(v));
}
template<class T>
typename std::enable_if<std::is_enum<T>::value, bool>::type
inline load_settings(c_config_setting cfg, const std::string & name, T * v) {
  std::string s;
  return cfg.get(name, &s) ? *v = fromStdString(s, *v), true : false;
}






template<class T>
inline bool save_settings(c_config_setting list, const std::vector<T> & values)
{
  if ( list.isList() || list.isArray() ) {
    while ( list.length() ) {
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






#ifdef CV_VERSION
// This block will compile only when <opencv.hpp> is included BEFORE this include,
// thus include opencv-related headers BEFORE this header if you need these declarations.

template<class T> struct c_config_type_traits<cv::Point_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Point3_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Size_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Rect_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T, int cn> struct c_config_type_traits<cv::Vec<T, cn>> { static constexpr int type = CONFIG_TYPE_ARRAY; };
template<class T, int m, int n> struct c_config_type_traits<cv::Matx<T, m, n>> { static constexpr int type = CONFIG_TYPE_ARRAY; };




template<class T>
inline bool save_settings(c_config_setting item, const cv::Size_<T> & value) {
  return item.isGroup() && item.set("width", value.width) && item.set("height", value.height);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Size_<T> * value) {
  return item.isGroup() && item.get("width", &value->width) && item.get("height", &value->height);
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Point_<T> & value) {
  return item.isGroup() && item.set("x", value.x) && item.set("y", value.y);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Point_<T> * value) {
  return item.isGroup() && item.get("x", &value->x) && item.get("y", &value->y);
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Point3_<T> & value) {
  return item.isGroup() && item.set("x", value.x) && item.set("y", value.y) && item.set("z", value.z);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Point3_<T> * value) {
  return item.isGroup() && item.get("x", &value->x) && item.get("y", &value->y) && item.get("z", &value->z);
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Rect_<T> & value) {
  return item.isGroup() && item.set("x", value.x) && item.set("y", value.y) &&
      item.set("width", value.width) && item.set("height", value.height);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Rect_<T> * value) {
  return item.isGroup() && item.get("x", &value->x) && item.get("y", &value->y) &&
      item.get("width", &value->width) && item.get("height", &value->height);
}




template<class T, int cn>
inline bool save_settings(c_config_setting array, const cv::Vec<T, cn> & value)
{
  if ( array.isArray() ) {
    while (array.length() ) {
      array.remove_element(0);
    }
    for ( int i = 0; i < cn; ++i ) {
      if ( !array.add(value[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}
template<class T, int cn>
inline bool load_settings(c_config_setting array, cv::Vec<T, cn> * value)
{
  if ( (array.isArray() || array.isList()) && array.length() == cn ) {
    for ( int i = 0; i < cn; ++i ) {
      if ( !array.get(i, &value->val[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}

template<class T>
inline bool save_settings(c_config_setting group, const cv::Scalar_<T> & v)
{
  group.set("v0", v.val[0]);
  group.set("v1", v.val[1]);
  group.set("v2", v.val[2]);
  group.set("v3", v.val[3]);
  return true;
}

template<class T>
inline bool load_settings(c_config_setting group, cv::Scalar_<T> * v)
{
  if ( group.isGroup() ) {
    group.get("v0", &v->val[0]);
    group.get("v1", &v->val[1]);
    group.get("v2", &v->val[2]);
    group.get("v3", &v->val[3]);
    return true;
  }
  return false;
}

template<class T, int m, int n>
inline bool save_settings(c_config_setting array, const cv::Matx<T, m, n> & value)
{
  if ( array.isArray() ) {
    while (array.length() ) {
      array.remove_element(0);
    }
    for ( int i = 0; i < m; ++i ) {
      for ( int j = 0; j < n; ++j ) {
        if ( !array.add(value(i,j)) ) {
          return false;
        }
      }
    }
    return true;
  }
  return false;
}
template<class T, int m, int n>
inline bool load_settings(c_config_setting array, cv::Matx<T, m, n> * value)
{
  if ( (array.isArray() || array.isList()) && array.length() == m * n ) {
    for ( int i = 0; i < m; ++i ) {
      for ( int j = 0; j < n; ++j ) {
        if ( !array.get(i * n + j, &(*value)(i,j)) ) {
          return false;
        }
      }
    }
    return true;
  }
  return false;
}



#endif // CV_VERSION


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



template<class T>
inline bool save_settings(const std::string & config_file, const T & obj, const std::string & group = "")
{
  c_config cfg(config_file);

  time_t t = time(0);
  if ( !save_settings(cfg.root(), "creation_date", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  c_config_setting root = group.empty() ? cfg.root() : cfg.root().add_group(group);
  if ( !save_settings(root, obj) ) {
    CF_FATAL("save_settings('%s') fails", config_file.c_str());
    return false;
  }

  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  return true;
}

template<class T>
inline bool load_settings(const std::string & config_file, T * obj, const std::string & group = "")
{
  c_config cfg(config_file);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", cfg.filename().c_str());
    return false;
  }

  c_config_setting root = group.empty() ? cfg.root() : cfg.root().add_group(group);
  if ( !load_settings(root, obj) ) {
    CF_FATAL("load_settings('%s') fails", config_file.c_str());
    return false;
  }

  return true;
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


#define SAVE_PROPERTY(cfg, obj, prop) \
    ::save_settings(cfg, #prop, (obj).prop())

#define LOAD_PROPERTY(cfg, obj, prop) \
    ::load_settings(cfg, #prop, (obj), \
        &std::remove_reference<decltype(*obj)>::type::prop, \
        &std::remove_reference<decltype(*obj)>::type::set_##prop)

#define SAVE_SETTINGS(cfg, obj, prop) \
    ::save_settings(cfg, #prop, (obj).prop())

#define LOAD_SETTINGS(cfg, obj, prop) \
    ::load_settings(cfg, #prop, &(obj)->prop())

#endif /* __qwcc_core_settings_h__ */
