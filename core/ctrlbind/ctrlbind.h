/*
 * ctrlbind.h
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __ctrlbind_h__
#define __ctrlbind_h__

#include <opencv2/opencv.hpp>
#include <core/settings/opencv_settings.h>
#include <type_traits>
#include <mutex>

struct c_sparse_feature_detector_options;

enum ctrl_bind_type
{
  ctrl_bind_begin_group,
  ctrl_bind_end_group,
  ctrl_bind_numeric_box,
  ctrl_bind_enum_combobox,
  ctrl_bind_check_box,
  ctrl_bind_flags_chkbox,
  ctrl_bind_spinbox,
  ctrl_bind_double_slider,
  ctrl_bind_browse_for_existing_file,
  ctrl_bind_browse_for_directory,
  ctrl_bind_math_expression,
  ctrl_bind_string_combobox,
  ctrl_bind_sparse_feature_detector,
  ctrl_bind_data_annotation_selector,
  ctrl_bind_data_command_button,
};

typedef std::function<void(const std::string&)> ctrlbind_copy_to_clipboard_callback;
void set_ctrlbind_copy_to_clipboard_callback(const ctrlbind_copy_to_clipboard_callback & fn);
const ctrlbind_copy_to_clipboard_callback & get_ctrlbind_copy_to_clipboard_callback();

struct c_ctrl_bind
{
  std::string ctl_name;
  std::string ctl_tooltip;
  ctrl_bind_type ctl_type;

  struct {
    double min = 0;
    double max = 100;
    double step = 1;
  } range;

  std::function<std::string ()> helpstring;
  std::function<bool (std::string *)> get_value;
  std::function<bool(const std::string&)> set_value;
  std::function<bool(std::vector<std::string>*, bool*)> get_strings;
  std::function<c_sparse_feature_detector_options *()> sparse_feature_detector;
  std::function<bool (int cmap, int * label)> get_data_annotation;
  std::function<bool(int cmap, int label)> set_data_annotation;
  std::function<void()> on_button_click;

  const c_enum_member * (*get_enum_members)() = nullptr;
  std::function<bool ()> is_enabled;
};


#define BIND_CTRL_BEGIN_GROUP(ctls, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_begin_group; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_CTRL_END_GROUP(ctls) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_type = ctrl_bind_end_group; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_CTRL(ctls, param, cname, cdesc) \
  if ( true ) { \
    ctrl_bind_type ctype; \
    if( std::is_enum<decltype(param())>::value ) { \
      ctype = ctrl_bind_enum_combobox; \
    } \
    else if( std::is_same<decltype(param()), bool>::value ) { \
      ctype = ctrl_bind_check_box; \
    } \
    else { \
      ctype = ctrl_bind_numeric_box; \
    } \
    \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctype; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(mutex()); \
        set_##param(v); \
        return true; \
      } \
      return false; \
    }; \
    \
    if ( ctype == ctrl_bind_enum_combobox ) { \
      tmp.get_enum_members = get_members_of<decltype(param())>(); \
    } \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_CTRLC(ctls, param, cname, cond, cdesc) \
  if ( true ) { \
    ctrl_bind_type ctype; \
    if( std::is_enum<decltype(param())>::value ) { \
      ctype = ctrl_bind_enum_combobox; \
    } \
    else if( std::is_same<decltype(param()), bool>::value ) { \
      ctype = ctrl_bind_check_box; \
    } \
    else { \
      ctype = ctrl_bind_numeric_box; \
    } \
    \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctype; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(mutex()); \
        set_##param(v); \
        return true; \
      } \
      return false; \
    }; \
    \
    if ( ctype == ctrl_bind_enum_combobox ) { \
      tmp.get_enum_members = get_members_of<decltype(param())>(); \
    } \
    \
    tmp.is_enabled = [this]() { \
      return (cond); \
    };\
    \
    (ctls)->emplace_back(tmp); \
  }

/*

*/

#define BIND_PCTRL(ctls, param, cdesc) \
    BIND_CTRL(ctls, param, #param, cdesc )

#define BIND_FLAGS_CTRL(ctls, param, enumtype, cname, cdesc) \
  if ( true ) { \
    \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_flags_chkbox; \
    \
    tmp.get_enum_members = get_members_of<enumtype>(); \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(mutex()); \
        set_##param(v); \
        return true; \
      } \
      return false; \
    }; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, param, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_browse_for_existing_file; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::lock_guard<std::mutex> lock(mutex()); \
      set_##param(s); \
      return true; \
    }; \
    \
   (ctls)->emplace_back(tmp); \
  }

#define BIND_BROWSE_FOR_DIRECTORY_CTRL(ctls, param, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_browse_for_directory; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::lock_guard<std::mutex> lock(mutex()); \
      set_##param(s); \
      return true; \
    }; \
    \
   (ctls)->emplace_back(tmp); \
  }

#define BIND_MATH_EXPRESSION_CTRL(ctls, param, chelp, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_math_expression; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::lock_guard<std::mutex> lock(mutex()); \
      set_##param(s); \
      return true; \
    }; \
    tmp.helpstring = [this]() -> std::string { \
      return chelp(); \
    }; \
    \
   (ctls)->emplace_back(tmp); \
  }

#define BIND_STRINGLIST_CTRL(ctls, getfn, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_string_combobox; \
    \
    tmp.get_strings = [this](std::vector<std::string> * strings, bool * readonly) -> bool { \
      return getfn(strings, readonly); \
    }; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_DATA_ANNOTATION_CTRL(ctls, param, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_data_annotation_selector; \
    tmp.get_data_annotation = [this](int cmap, int * lb) -> bool { \
      return get_##param(cmap, lb); \
    }; \
    tmp.set_data_annotation = [this](int cmap, int lb) -> bool { \
      return set_##param(cmap, lb); \
    }; \
    (ctls)->emplace_back(tmp); \
  }


#define BIND_SPINBOX_CTRL(ctls, param, minvalue, maxvalue, stepvalue, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_spinbox; \
    tmp.range.min = minvalue; \
    tmp.range.max = maxvalue; \
    tmp.range.step = stepvalue; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(mutex()); \
        set_##param(v); \
        return true; \
      } \
      return false; \
    }; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_DOUBLE_SLIDER_CTRL(ctls, param, minvalue, maxvalue, stepvalue, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_double_slider; \
    tmp.range.min = minvalue; \
    tmp.range.max = maxvalue; \
    tmp.range.step = stepvalue; \
    \
    tmp.get_value = [this](std::string * v) -> bool { \
      *v = toString(param()); \
      return true; \
    }; \
    tmp.set_value = [this](const std::string & s) { \
      std::remove_const<std::remove_reference<decltype(param())>::type>::type v; \
      if( fromString(s, &v) ) { \
        std::lock_guard<std::mutex> lock(mutex()); \
        set_##param(v); \
        return true; \
      } \
      return false; \
    }; \
    \
    (ctls)->emplace_back(tmp); \
  }

#define BIND_SPARSE_FEATURE_DETECTOR_CTRL(ctls, param, cname, cdesc) \
  if ( true ) { \
    c_ctrl_bind tmp; \
    tmp.ctl_name = cname; \
    tmp.ctl_tooltip = cdesc; \
    tmp.ctl_type = ctrl_bind_sparse_feature_detector; \
    tmp.sparse_feature_detector = [this]() -> c_sparse_feature_detector_options * { \
        return param(); \
    }; \
    \
   (ctls)->emplace_back(tmp); \
  }

#define BIND_COMMAND_BUTTON(ctls, func, cname, cdesc) \
    if ( true ) { \
      c_ctrl_bind tmp; \
      tmp.ctl_name = cname; \
      tmp.ctl_tooltip = cdesc; \
      tmp.ctl_type = ctrl_bind_data_command_button; \
      tmp.on_button_click = [this]() { \
          func(); \
      }; \
      \
     (ctls)->emplace_back(tmp); \
    }


#endif /* __ctrlbind_h__ */
