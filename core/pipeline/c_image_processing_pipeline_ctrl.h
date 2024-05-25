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
#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>
#include <core/ssprintf.h>
#include "c_output_frame_writer.h"

class c_image_processing_pipeline;
//struct c_feature_registration_options;
//struct c_master_frame_selection_options;


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
  c_image_processor_pipeline_ctl_cv_matx,
  c_image_processor_pipeline_ctl_camera_intrinsicts,
  c_image_processor_pipeline_ctl_feature2d_detector_options,
  c_image_processor_pipeline_ctl_feature2d_descriptor_options,
  c_image_processor_pipeline_ctl_feature2d_matcher_options,
  c_image_processor_pipeline_ctl_stereo_matcher_options,
  c_image_processor_pipeline_ctl_output_writer,
  c_image_processor_pipeline_ctl_master_frame_selection,
  c_image_processor_pipeline_ctl_feature_registration_options,
  c_image_processor_pipeline_ctl_ecc_registration_options,
  c_image_processor_pipeline_ctl_eccflow_registration_options,
  c_image_processor_pipeline_ctl_jovian_derotation_options,
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
  struct {
    int rows = 0, cols = 0;
  } matx;
  std::function<bool (const c_image_processing_pipeline*, std::string *)> get_value;
  std::function<bool(c_image_processing_pipeline * p, const std::string&)> set_value;
  std::function<const c_image_processor::sptr & (const c_image_processing_pipeline*)> get_image_processor;
  std::function< bool (c_image_processing_pipeline*, const c_image_processor::sptr & )> set_image_processor;
  std::function<c_image_registration_options* (c_image_processing_pipeline*)> get_image_registration_options;
  std::function<c_sparse_feature_detector_options* (c_image_processing_pipeline*)> get_feature2d_detector_options;
  std::function<c_sparse_feature_descriptor_options* (c_image_processing_pipeline*)> get_feature2d_descriptor_options;
  std::function<c_feature2d_matcher_options* (c_image_processing_pipeline*)> get_feature2d_matcher_options;
  std::function<c_camera_intrinsics *(c_image_processing_pipeline *)> get_camera_intrinsicts;
  std::function<c_regular_stereo_matcher *(c_image_processing_pipeline *)> get_stereo_matcher;
  std::function<c_output_frame_writer_options *(c_image_processing_pipeline *)> get_output_writer_options;
  std::function<c_master_frame_selection_options *(c_image_processing_pipeline *)> get_master_frame_selection_options;
  std::function<c_feature_registration_options *(c_image_processing_pipeline *)> get_feature_registration_options;
  std::function<c_ecc_registration_options *(c_image_processing_pipeline *)> get_ecc_registration_options;
  std::function<c_eccflow_registration_options *(c_image_processing_pipeline *)> get_eccflow_registration_options;
  std::function<c_jovian_derotation_options *(c_image_processing_pipeline *)> get_jovian_derotation_options;

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

#define PIPELINE_CTL_GROUPC(ctrls, _name, _tooltip, _cond ) \
  if ( true ) { \
    c_image_processing_pipeline_ctrl ctl; \
    ctl.type = c_image_processor_pipeline_ctl_begin_group; \
    ctl.name = _name; \
    ctl.tooltip = _tooltip; \
    ctl.is_enabled = \
        [](const c_image_processing_pipeline * p) -> bool { \
          const this_class * _this = dynamic_cast<const this_class * >(p); \
          return (_this) && (_cond); \
        }; \
    ctrls.emplace_back(ctl);\
  }

#define PIPELINE_CTL_END_GROUP(ctrls) \
  if ( true ) { \
    c_image_processing_pipeline_ctrl ctl; \
    ctl.type = c_image_processor_pipeline_ctl_end_group; \
    ctrls.emplace_back(ctl);\
  }

#define PIPELINE_CTL(ctrls, prop, _name, _tooltip ) \
    if ( true ) { \
      using proptype = decltype(this_class::prop); \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype, bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->prop); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? fromString(v, &_this->prop) : false; \
          }; \
      ctrls.emplace_back(ctl); \
    }


#define PIPELINE_CTLC(ctrls, prop, _name, _tooltip, _cond) \
    if ( true ) { \
      using proptype = decltype(this_class::prop); \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype, bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->prop); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? fromString(v, &_this->prop) : false; \
          }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTLP(ctrls, prop, _name, _tooltip ) \
    if ( true ) { \
      using proptype = decltype(((this_class*)(nullptr))->prop()); \
      \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype, bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->prop()); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            if ( _this ) { \
              proptype propval; \
              if ( fromString(v, &propval) ) { \
                _this->set_##prop(propval); \
                return true; \
              } \
            } \
            return false; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTLP2(ctrls, obj, prop, _name, _tooltip ) \
    if ( true ) { \
      using objtype = decltype(this_class::obj); \
      using proptype = decltype( ((this_class*)(nullptr))->obj.prop()); \
      \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype, bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->obj.prop()); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            if ( _this ) { \
              proptype value; \
              if ( fromString(v, &value) ) { \
                _this->obj.set_##prop(value); \
                return true; \
              } \
            } \
            return false; \
          }; \
      ctrls.emplace_back(ctl); \
    }




#define PIPELINE_CTLPC(ctrls, prop, _name, _tooltip, _cond) \
    if ( true ) { \
      using proptype = decltype(((this_class*)(nullptr))->prop()); \
      \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype,bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->prop()); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            if ( _this ) { \
              proptype propval; \
              if ( fromString(v, &propval) ) { \
                _this->set_##prop(propval); \
                return true; \
              } \
            } \
            return false; \
          }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTLPC2(ctrls, obj, prop, _name, _tooltip, _cond ) \
    if ( true ) { \
      using objtype = decltype(this_class::obj); \
      using proptype = decltype( ((this_class*)(nullptr))->obj.prop()); \
      \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      if( std::is_enum_v<proptype> ) { \
        ctl.type = c_image_processor_pipeline_ctl_enum_combobox; \
        ctl.get_enum_members = get_members_of<proptype>(); \
      } \
      else if( std::is_same_v<proptype, bool> ) { \
        ctl.type = c_image_processor_pipeline_ctl_check_box; \
      } \
      else { \
        ctl.type = c_image_processor_pipeline_ctl_numeric_box; \
      } \
      \
      ctl.get_value = \
          [](const c_image_processing_pipeline * p, std::string * v) { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            if ( _this ) { \
              *v = toString(_this->obj.prop()); \
              return true; \
            } \
            return false; \
          }; \
      ctl.set_value = \
          [](c_image_processing_pipeline * p, const std::string & v) { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            if ( _this ) { \
              proptype value; \
              if ( fromString(v, &value) ) { \
                _this->obj.set_##prop(value); \
                return true; \
              } \
            } \
            return false; \
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
      ctl.get_image_processor = \
          [](const c_image_processing_pipeline * p) -> const c_image_processor::sptr & { \
            static const c_image_processor::sptr null_processor; \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return  (_this ? _this->c :  null_processor); \
          }; \
      ctl.set_image_processor = \
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

#define PIPELINE_CTL_CV_MATX(ctrls, c, _name, _tooltip ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_cv_matx; \
      ctl.matx.rows = decltype(this_class::c)::rows; \
      ctl.matx.cols = decltype(this_class::c)::cols; \
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

#define PIPELINE_CTL_CV_MATXC(ctrls, c, _name, _tooltip, _cond ) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.name = _name; \
      ctl.tooltip = _tooltip; \
      ctl.type = c_image_processor_pipeline_ctl_cv_matx; \
      ctl.matx.rows = decltype(this_class::c)::rows; \
      ctl.matx.cols = decltype(this_class::c)::cols; \
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


#define PIPELINE_CTL_CAMERA_INTRINSICTS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_camera_intrinsicts; \
      ctl.get_camera_intrinsicts = \
          [](c_image_processing_pipeline * p) -> c_camera_intrinsics * { \
            this_class * _this = dynamic_cast<this_class * >(p); \
            return _this ? &(_this->c) : nullptr; \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_FEATURE2D_DETECTOR_OPTIONS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_feature2d_detector_options; \
      ctl.get_feature2d_detector_options = \
          [](c_image_processing_pipeline * p) -> c_sparse_feature_detector_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_FEATURE2D_DESCRIPTOR_OPTIONS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_feature2d_descriptor_options; \
      ctl.get_feature2d_descriptor_options = \
          [](c_image_processing_pipeline * p) -> c_sparse_feature_descriptor_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_FEATURE2D_MATCHER_OPTIONS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_feature2d_matcher_options; \
      ctl.get_feature2d_matcher_options = \
          [](c_image_processing_pipeline * p) -> c_feature2d_matcher_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctrls.emplace_back(ctl); \
    }


#define PIPELINE_CTL_STEREO_MATCHER_OPTIONS(ctrls, c) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_stereo_matcher_options; \
      ctl.get_stereo_matcher = \
          [](c_image_processing_pipeline * p) ->  c_regular_stereo_matcher * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, c, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_output_writer; \
      ctl.get_output_writer_options = \
          [](c_image_processing_pipeline * p) -> c_output_frame_writer_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }


#define PIPELINE_CTL_MASTER_FRAME_SELECTION(ctrls, c, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_master_frame_selection; \
      ctl.get_master_frame_selection_options = \
          [](c_image_processing_pipeline * p) -> c_master_frame_selection_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }

#define PIPELINE_CTL_FEATURE_REGISTRATION_OPTIONS(ctrls, c, _cond) \
  if ( true ) { \
    c_image_processing_pipeline_ctrl ctl; \
    ctl.type = c_image_processor_pipeline_ctl_feature_registration_options; \
    ctl.get_feature_registration_options = \
        [](c_image_processing_pipeline * p) -> c_feature_registration_options * { \
        this_class * _this = dynamic_cast<this_class * >(p); \
        return _this ? &(_this->c) : (nullptr); \
    }; \
    ctl.is_enabled = \
        [](const c_image_processing_pipeline * p) -> bool { \
          const this_class * _this = dynamic_cast<const this_class * >(p); \
          return (_this) && (_cond); \
        }; \
    ctrls.emplace_back(ctl); \
  }


//c_image_processor_pipeline_ctl_ecc_registration_options,
#define PIPELINE_CTL_ECC_REGISTRATION_OPTIONS(ctrls, c, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_ecc_registration_options; \
      ctl.get_ecc_registration_options = \
          [](c_image_processing_pipeline * p) -> c_ecc_registration_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }


//c_image_processor_pipeline_ctl_eccflow_registration_options,
#define PIPELINE_CTL_ECCFLOW_REGISTRATION_OPTIONS(ctrls, c, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_eccflow_registration_options; \
      ctl.get_eccflow_registration_options = \
          [](c_image_processing_pipeline * p) -> c_eccflow_registration_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }

// c_jovian_derotation_options
#define PIPELINE_CTL_JOVIAN_DEROTATION_OPTIONS(ctrls, c, _cond) \
    if ( true ) { \
      c_image_processing_pipeline_ctrl ctl; \
      ctl.type = c_image_processor_pipeline_ctl_jovian_derotation_options; \
      ctl.get_jovian_derotation_options = \
          [](c_image_processing_pipeline * p) -> c_jovian_derotation_options * { \
          this_class * _this = dynamic_cast<this_class * >(p); \
          return _this ? &(_this->c) : (nullptr); \
      }; \
      ctl.is_enabled = \
          [](const c_image_processing_pipeline * p) -> bool { \
            const this_class * _this = dynamic_cast<const this_class * >(p); \
            return (_this) && (_cond); \
          }; \
      ctrls.emplace_back(ctl); \
    }





#endif /* __c_image_processing_pipeline_ctrl_h__ */
