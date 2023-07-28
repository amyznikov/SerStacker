/*
 * c_image_processing_pipeline_ctrl.h
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_processing_pipeline_ctrl_h__
#define __c_image_processing_pipeline_ctrl_h__

#include <core/settings/opencv_settings.h>
#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/ssprintf.h>

class c_image_processing_pipeline;

enum c_image_processing_pipeline_ctrl_type {
  c_image_processor_pipeline_ctl_begin_group,
  c_image_processor_pipeline_ctl_end_group,
  c_image_processor_pipeline_ctl_numeric_box,
  c_image_processor_pipeline_ctl_enum_combobox,
  c_image_processor_pipeline_ctl_spinbox,
  c_image_processor_pipeline_ctl_check_box,
  c_image_processor_pipeline_ctl_flags_chkbox,
  c_image_processor_pipeline_ctl_browse_for_existing_file,
  c_image_processor_pipeline_ctl_browse_for_directory,
  c_image_processor_pipeline_ctl_image_processor_selection_combo,
  c_image_processor_pipeline_ctl_input_source_selection_combo,
  c_image_processor_pipeline_ctl_image_registration_options,
};


struct c_image_processing_pipeline_ctrl
{
  std::string name;
  std::string tooltip;
  c_image_processing_pipeline_ctrl_type type;
  const c_enum_member* (*get_enum_members)() = nullptr;
  struct {
    double min = 0, max = 100, step = 1;
  } range;
  std::function<bool (const c_image_processing_pipeline*, std::string *)> get_value;
  std::function<bool(c_image_processing_pipeline * p, const std::string&)> set_value;
  std::function<const c_image_processor::sptr & (const c_image_processing_pipeline*)> get_processor;
  std::function< bool (c_image_processing_pipeline*, const c_image_processor::sptr & )> set_processor;
  std::function<c_image_registration_options* (c_image_processing_pipeline*)> get_image_registration_options;
  std::function<bool (const c_image_processing_pipeline*)> is_enabled;
};


#define PIPELINE_CTL_GROUP(ctrls, _name, _tooltip ) \
  if ( true ) { \
    c_image_processing_pipeline_ctrl ctl; \
    ctl.type = c_image_processor_pipeline_ctl_begin_group; \
    ctl.name = _name; \
    ctl.tooltip = _tooltip; \
    ctrls.emplace_back(ctl);\
  }

#define PIPELINE_CTL_END_GROUP(ctrls) \
  if ( true ) { \
    c_image_processing_pipeline_ctrl ctl; \
    ctl.type = c_image_processor_pipeline_ctl_end_group; \
    ctrls.emplace_back(ctl);\
  }

#define PIPELINE_CTL(ctrls, c, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<decltype(this_class::c)> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<decltype(this_class::c)>(); \
      } \
      else if( std::is_same_v<decltype(this_class::c), bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->c); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? fromString(v, &_this->c) : false; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTLC(ctrls, c, _name, _tooltip, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<decltype(this_class::c)> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<decltype(this_class::c)>(); \
      } \
      else if( std::is_same_v<decltype(this_class::c), bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->c); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? fromString(v, &_this->c) : false; \
          }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }


#define PIPELINE_CTL_BITFLAGS(ctrls, c, enum_type, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_flags_chkbox; \
      ctl.get_enum_members = get_members_of<enum_type>(); \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? _this->c = flagsFromString<enum_type>(v), true : false; \
          }; \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = flagsToString<enum_type>(_this->c); \
              return true; \
            } \
            return false; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, c, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_browse_for_existing_file; \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = _this->c; \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? _this->c = v, true : false; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, c, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_browse_for_directory; \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = _this->c; \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? _this->c = v, true : false; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, c, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_image_processor_selection_combo; \
      ctl.get_processor = \
          [](const c_image_processing_pipeline * p) -> const c_image_processor::sptr & { \
            static const c_image_processor::sptr null_processor; \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return  (_this ? _this->c :  null_processor); \
          }; \
      ctl.set_processor = \
          [](c_image_processing_pipeline * p, const c_image_processor::sptr & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? _this->c = v, true : false; \
          }; \
      ctrls.emplace_back(ctl); \
    }


#define PIPELINE_CTL_INPUT_SOURCE_SELECTION(ctrls, c, _name, _tooltip, _cond ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_input_source_selection_combo; \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = _this->c; \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? _this->c = v, true : false; \
          }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_IMAGE_REGISTRATION_OPTIONS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_image_registration_options; \
      ctl.get_image_registration_options = \
          [](c_image_processing_pipeline * p) -> c_image_registration_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctrls.emplace_back(ctl); \
    }





#endif /* __c_image_processing_pipeline_ctrl_h__ */
