/*
 * settings.cc
 *
 *  Created on: September 5, 2019
 *      Author: amyznikov
 */
#include <core/settings.h>
#include <core/readdir.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////


c_config::c_config()
{
  construct();
}

c_config::c_config(const std::string & filename) :
    defaultFilename_(filename)
{
  construct();
}

c_config::~c_config()
{
  config_destroy(&config_);
}

void c_config::construct()
{
  config_init(&config_);
  set_float_precision(16);
  set_auto_convert(true);
  set_option(OptionOpenBraceOnSeparateLine, false);
  set_option(OptionAllowScientificNotation, true);
  set_option(OptionColonAssignmentForGroups, false);
}


// @brief get default config file name
const std::string & c_config::filename() const
{
  return defaultFilename_;
}
// @brief set default config file name
void c_config::set_filename(const std::string & filename)
{
  defaultFilename_ = filename;
}


void c_config::set_options(int options)
{
  config_set_options(&config_, options);
}

int c_config::options() const
{
  return config_get_options(&config_);
}

void c_config::set_option(Option option, bool flag)
{
  config_set_option(&config_, (int)option, flag ? CONFIG_TRUE : CONFIG_FALSE);
}

bool c_config::option(Option opt) const
{
  return config_get_option(&config_, (int)opt);
}

void c_config::set_auto_convert(bool v)
{
  set_option(OptionAutoConvert, v);
}

bool c_config::auto_convert() const
{
  return option(OptionAutoConvert);
}

void c_config::set_default_format(c_config_setting::Format format)
{
  if ( (defaultFormat_ = format) != c_config_setting::FormatHex ) {
    defaultFormat_ = c_config_setting::FormatDefault;
  }
  config_set_default_format(&config_, static_cast<short>(defaultFormat_));
}

c_config_setting::Format c_config::default_format() const
{
  return defaultFormat_;
}

void c_config::set_tab_width(uint16_t width)
{
  config_set_tab_width(&config_, width);
}

uint16_t c_config::tab_width() const
{
  return(config_get_tab_width(&config_));
}

void c_config::set_float_precision(uint16_t digits)
{
  return (config_set_float_precision(&config_,digits));
}

uint16_t c_config::float_precision() const
{
  return (config_get_float_precision(&config_));
}

void c_config::set_include_dir(const char *includeDir)
{
  config_set_include_dir(&config_, includeDir);
}

const char * c_config:: include_dir() const
{
  return(config_get_include_dir(&config_));
}

const char ** c_config::evaluate_include_path(const char *path, const char **error)
{
  return(config_default_include_func(&config_, include_dir(), path, error));
}


bool c_config::write(const std::string & filename)
{
  errno = 0;

  const std::string fname =
      expand_path(filename.empty() ?
          defaultFilename_ :
          filename);

  if ( fname.empty() ) {
    CF_FATAL("ERROR: c_config::write(): No config file name specified, "
        "can not write config file");
    errno = EINVAL;
    return false;
  }


  if ( !file_exists(fname) ) {

    const std::string path =
        get_parent_directory(fname);

    if ( !path.empty() && !create_path(path) ) {
      CF_FATAL("ERROR: c_config::write(): create_path('%s') fails: %s",
          fname.c_str(),
          strerror(errno));
      return false;
    }
  }

  if ( !config_write_file(&config_, fname.c_str()) ) {
    CF_FATAL("ERROR: config_write_file('%s') fails.\n"
        "ERRNO=%d (%s), Hope it helps)",
        fname.c_str(),
        errno,
        strerror(errno));

    return false;
  }

  return true;
}

// @brief write config into in-memory string
std::string c_config::write_string() const
{
  FILE *fp = nullptr;
  char *bufloc = nullptr;
  size_t sizeloc = 0;

  std::string s;

  if( !(fp = open_memstream(&bufloc, &sizeloc)) ) {
    CF_ERROR("ERROR: c_config: open_memstream() fails: %s",
        strerror(errno));
  }
  else {
    config_write(&config_, fp);
    fclose(fp);
    s = bufloc;
    free(bufloc);
  }

  return s;
}

bool c_config::read(const std::string & filename)
{
  errno = 0;

  const std::string fname =
      expand_path(filename.empty() ? defaultFilename_ :
          filename);

  if ( fname.empty() ) {
    CF_FATAL("ERROR: c_config::read(): No config file name was specified.");
    errno = EINVAL;
    return false;
  }

  if ( !file_readable(fname) ) {
    CF_FATAL("ERROR: c_config::read(): Requested config file ('%s') not exist or is not readable.\n"
        "errno = %d (%s)",
        fname.c_str(),
        errno,
        strerror(errno));
    return false;
  }

  if ( !config_read_file(&config_, fname.c_str()) ) {

    switch ( config_error_type(&config_) )
    {
      case CONFIG_ERR_PARSE :
        CF_FATAL("ERROR: config_read_file('%s') fails:\n"
            "Parse error on line %d of '%s':\n(%s).\n",
            fname.c_str(),
            config_error_line(&config_),
            config_error_file(&config_),
            config_error_text(&config_)
            );
        break;

      case CONFIG_ERR_FILE_IO :
        CF_FATAL("ERROR: config_read_file('%s') fails: CONFIG_ERR_FILE_IO.\n"
            "ERRNO=%d (%s), Hope it helps)",
            fname.c_str(),
            errno,
            strerror(errno));
        break;

      case CONFIG_ERR_NONE :
        default :
        CF_FATAL("ERROR: config_read_file('%s') fails: Unknown error.\n"
            "ERRNO=%d (%s), Hope it helps)",
            fname.c_str(),
            errno,
            strerror(errno));
        break;

    }

    return false;
  }

  return true;
}

// @brief read config from in-memory string
bool c_config::read_string(const std::string & s)
{
  return read_string(s.c_str());
}

// @brief read config from in-memory string
bool c_config::read_string(const char * s)
{
  if( !config_read_string(&config_, s) ) {

    switch (config_error_type(&config_))
    {
    case CONFIG_ERR_PARSE:
      CF_FATAL("ERROR: config_read_string() fails: CONFIG_ERR_PARSE\n"
          "Parse error on line %d :\n(%s).\n",
          config_error_line(&config_),
          config_error_text(&config_)
          );
      break;

    case CONFIG_ERR_FILE_IO:
      CF_FATAL("ERROR: config_read_string() fails: CONFIG_ERR_FILE_IO.\n"
          "ERRNO=%d (%s), Hope it helps)",
          errno,
          strerror(errno));
      break;

    case CONFIG_ERR_NONE:
      default:
      CF_FATAL("ERROR: config_read_string() fails: Unknown error.\n"
          "ERRNO=%d (%s), Hope it helps)",
          errno,
          strerror(errno));
      break;
    }

    return false;
  }

  return true;
}

void c_config::clear()
{
  config_clear(&config_);
}

c_config_setting c_config::root() const
{
  return c_config_setting(config_root_setting(&config_));
}




///////////////////////////////////////////////////////////////////////////////


c_config_setting::c_config_setting()
  : setting_(nullptr)
{
}

c_config_setting::c_config_setting(config_setting_t * setting)
  : setting_(setting)
{
}

c_config_setting::operator bool() const
{
  return setting_ != nullptr;
}

int c_config_setting::type() const
{
  return setting_ ? config_setting_type(setting_) : CONFIG_TYPE_NONE;
}

int c_config_setting::length() const
{
  return setting_ ? config_setting_length(setting_) : 0;
}

const char * c_config_setting::name() const
{
  return setting_ ? setting_->name : "";
}


bool c_config_setting::isNull() const
{
  return !setting_;
}

bool c_config_setting::isRoot() const
{
  return setting_ && config_setting_is_root(setting_);
}

bool c_config_setting::isGroup() const
{
  return setting_ && config_setting_type(setting_) == CONFIG_TYPE_GROUP;
}

bool c_config_setting::isString() const
{
  return setting_ && config_setting_type(setting_) == CONFIG_TYPE_STRING;
}


bool c_config_setting::isArray() const
{
  return setting_ && config_setting_type(setting_) == CONFIG_TYPE_ARRAY;
}

bool c_config_setting::isList() const
{
  return setting_ && config_setting_type(setting_) == CONFIG_TYPE_LIST;
}

bool c_config_setting::isAggregate() const
{
  const int type = setting_ ? config_setting_type(setting_) : CONFIG_TYPE_NONE;
  return type == CONFIG_TYPE_GROUP || type == CONFIG_TYPE_LIST || type == CONFIG_TYPE_ARRAY;
}

bool c_config_setting::isScalar() const
{
  const int type = setting_ ? config_setting_type(setting_) : CONFIG_TYPE_NONE;
  return type == CONFIG_TYPE_INT || type == CONFIG_TYPE_INT64 || type == CONFIG_TYPE_FLOAT || type == CONFIG_TYPE_STRING
      || type == CONFIG_TYPE_BOOL;
}

bool c_config_setting::isNumber() const
{
  const int type = setting_ ? config_setting_type(setting_) : CONFIG_TYPE_NONE;
  return type == CONFIG_TYPE_INT || type == CONFIG_TYPE_INT64 || type == CONFIG_TYPE_FLOAT;
}

bool c_config_setting::isInteger() const
{
  const int type = setting_ ? config_setting_type(setting_) : CONFIG_TYPE_NONE;
  return type == CONFIG_TYPE_INT || type == CONFIG_TYPE_INT64;
}




c_config_setting::Format c_config_setting::format() const
{
  return setting_ ? static_cast<c_config_setting::Format>(config_setting_get_format(setting_)) : FormatDefault;
}

void c_config_setting::set_format(c_config_setting::Format format)
{
  if ( setting_ ) {
    config_setting_set_format(setting_, format == FormatHex ? FormatHex : FormatDefault);
  }
}

uint32_t c_config_setting::source_line() const
{
  return setting_ ? (config_setting_source_line(setting_)) : 0;
}

const char * c_config_setting::source_file() const
{
  return setting_ ? (config_setting_source_file(setting_)) : nullptr;
}

bool c_config_setting::has_parent() const
{
  return setting_ ? config_setting_parent(setting_) != nullptr : false;
}

c_config_setting c_config_setting::parent() const
{
  return setting_ ? config_setting_parent(setting_) : nullptr;
}


config_setting_t * c_config_setting::get_member(const config_setting_t *setting, const char * name)
{
  return setting ? config_setting_get_member(setting, name) : nullptr;
//  config_setting_t * mp  = config_setting_get_member(setting, name);
//  if ( !mp ) {
//    CF_ERROR("config_setting_get_member(setting=%p, name='%s') fails", (void*)setting, name);
//  }
//
//  return mp;
}

config_setting_t * c_config_setting::add_member(config_setting_t *setting, const char * name, int type)
{
  if ( !setting ) {
    return nullptr;
  }

  config_setting_t * item = get_member(setting, name);
  if( item ) {
    if( item->type == type ) {
      return item; // already exists
    }
    if( !config_setting_remove(setting, name) ) {
      return nullptr;
    }
  }

  if ( !(item  = config_setting_add(setting, name, type)) ) {
    CF_ERROR("config_setting_add(setting=%p, name='%s' type=%d) fails", (void*)setting, name, type);
  }

  return item;
}


config_setting_t * c_config_setting::get_element(const config_setting_t *setting, uint32_t index)
{
  return setting ? config_setting_get_elem(setting, index) : nullptr;
}


config_setting_t * c_config_setting::add_element(config_setting_t *setting, int type)
{
  return setting ? config_setting_add(setting, "", type) : nullptr;
}


bool c_config_setting::remove_element(config_setting_t *setting, uint32_t index)
{
  return setting ? config_setting_remove_elem(setting, index) : false;
}


bool c_config_setting::get_value(const config_setting_t * setting, bool * value)
{
  if ( setting && setting->type == CONFIG_TYPE_BOOL ) {
    *value = setting->value.ival;
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t *setting, int8_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    const int v = config_setting_get_int(setting);
    if ( v >= INT8_MIN && v <= INT8_MAX ) {
      *value = v;
      return true;
    }
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t *setting, uint8_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    const int v = config_setting_get_int(setting);
    if ( v >= 0 && v <= UINT8_MAX ) {
      *value = v;
      return true;
    }
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t * setting, int32_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = config_setting_get_int(setting);
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t *setting, uint32_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = static_cast<uint32_t>(config_setting_get_int(setting));
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t *setting, int64_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = config_setting_get_int64(setting);
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t * setting, uint64_t * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = static_cast<uint64_t>(config_setting_get_int64(setting));
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t * setting, double * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = config_setting_get_float(setting);
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t * setting, float * value)
{
  if ( setting && config_setting_is_number(setting) ) {
    *value = static_cast<float>(config_setting_get_float(setting));
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t * setting, const char ** value)
{
  if ( setting && setting->type == CONFIG_TYPE_STRING ) {
    *value = setting->value.sval;
    return true;
  }
  return false;
}

bool c_config_setting::get_value(const config_setting_t *setting, std::string * value)
{
  if ( setting && setting->type == CONFIG_TYPE_STRING ) {
    *value = setting->value.sval ? (const char *)setting->value.sval : "";
    return true;
  }
  return false;
}


bool c_config_setting::set_value(config_setting_t *setting, bool value)
{
  return setting && config_setting_set_bool(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, int32_t value)
{
  return setting && config_setting_set_int(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, uint32_t value)
{
  return setting && config_setting_set_int(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, int64_t value)
{
  return setting && config_setting_set_int64(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, uint64_t value)
{
  return setting && config_setting_set_int64(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, double value)
{
  return setting && config_setting_set_float(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, float  value)
{
  return setting && config_setting_set_float(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, const char * value)
{
  return setting && config_setting_set_string(setting, value);
}

bool c_config_setting::set_value(config_setting_t *setting, const std::string & value)
{
  return setting && config_setting_set_string(setting, value.c_str());
}


bool libconfig_parse_flags(c_config_setting settings, const c_libconfig_flag_desc fdescs[/*ndescs*/], uint32_t ndescs, int * flags)
{
  if ( !settings ) {
    CF_ERROR("libconfig settings is null in %s",
        __PRETTY_FUNCTION__);
    return false;
  }

  std::string sflags;

  if ( ::load_settings(settings, &sflags) ) {

    const std::vector<std::string> tokens =
        strsplit(sflags, " \t\n|+;");

    uint32_t i;

    for ( const std::string & sflag : tokens ) {
      const char * cflag = sflag.c_str();
      for ( i = 0; i < ndescs; ++i ) {
        if ( strcasecmp(cflag, fdescs[i].flag) == 0 ) {
          *flags |= fdescs[i].value;
          break;
        }
      }
      if ( i == ndescs ) {
        CF_ERROR("Invalid or not supported flag: %s", cflag);
        return false;
      }
    }
  }

  return true;
}

