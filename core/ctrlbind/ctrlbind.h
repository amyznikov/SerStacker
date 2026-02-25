/*
 * ctrlbind.h
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __ctrlbind_h__
#define __ctrlbind_h__

#include <vector>
#include <string>
#include <functional>
#include <type_traits>
#include <core/settings.h>
#include <core/ssprintf.h>

// temporary forward declarations
struct c_camera_intrinsics;
struct c_sparse_feature_detector_options;
struct c_sparse_feature_descriptor_options;
struct c_feature2d_matcher_options;
struct c_feature_registration_options;
struct c_ecc_registration_options;
struct c_eccflow_registration_options;
struct c_master_frame_selection_options;
struct c_stereo_input_source_options;
//class c_input_sequence;

template <typename T>
using is_normal_integer = std::disjunction<
    std::is_same<T, int8_t>,   std::is_same<T, uint8_t>,
    std::is_same<T, int16_t>,  std::is_same<T, uint16_t>,
    std::is_same<T, int32_t>,  std::is_same<T, uint32_t>,
    std::is_same<T, int64_t>,  std::is_same<T, uint64_t>,
    std::is_same<T, long>,     std::is_same<T, unsigned long>,
    std::is_same<T, long long>,std::is_same<T, unsigned long long>
>;
template <typename T>
constexpr bool is_normal_integer_v =
    is_normal_integer<typename std::decay_t<T>>::value;

template<class RootObjectType>
struct c_ctlbind
{
  enum class CtlType
  {
    None,
    BeginGroup,
    BeginExpandableGroup,
    EndGroup,
    Textbox,
    NumericBox,
    Checkbox,
    FlagsCheckbox,
    EnumCombobox,
    SpinBox,
    DoubleSpinBox,
    DoublesliderSpinBox,
    BrowseForDirectory,
    BrowseForExistingFile,
    MathExpression,
    CommandButton,
    MenuButton,

    ImageProcessorCombobox,
    CameraIntrinsicts,
    SparseFeatureDetector,
    SparseFeatureDescriptor,
    SparseFeatureMatcher,
    FeatureRegistrationOptions,
    ECCRegistrationOptions,
    ECCFlowRegistrationOptions,
    //InputSourceSelection,
    MasterFrameSelection,
    StereoInputSourceSelection,
    DataAnnotationSelector,
  };

  struct MenuItem {
    std::string name;
    std::function<bool(RootObjectType * obj)> onclick;
  };

  std::string cname;
  std::string cdesc;
  CtlType ctype = CtlType::None;

  struct {
    double min = 0;
    double max = 100;
    double step = 1;
  } range;

  const c_enum_member * (*get_enum_members)() = nullptr;
  std::function<bool(const RootObjectType*, std::string*)> getvalue;
  std::function<bool(RootObjectType*, const std::string&)> setvalue;
  std::function<c_camera_intrinsics*(RootObjectType*)> camera_intrinsicts;
  std::function<c_sparse_feature_detector_options*(RootObjectType*)> sparse_feature_detector;
  std::function<c_sparse_feature_descriptor_options*(RootObjectType*)> sparse_feature_descriptor;
  std::function<c_feature2d_matcher_options*(RootObjectType*)> sparse_feature_matcher;
  std::function<c_feature_registration_options *(RootObjectType *)> feature_registration_options;
  std::function<c_ecc_registration_options *(RootObjectType *)> ecc_registration_options;
  std::function<c_eccflow_registration_options *(RootObjectType *)> eccflow_registration_options;
  std::function<c_master_frame_selection_options*(RootObjectType*)> master_frame_selection;
  std::function<c_stereo_input_source_options*(RootObjectType*)> stereo_input_source_options;
  std::function<bool(RootObjectType * obj)> onclick;
  std::function<std::string(RootObjectType * obj)> helpstring;
  std::function<bool(const RootObjectType * obj)> enabled;
  std::vector<MenuItem> menu;
};


template<class RootObjectType>
class c_ctlist :
    public std::vector<c_ctlbind<RootObjectType>>
{
};

template<class StructType, class FieldType>
inline constexpr size_t offset_of(FieldType StructType::*mp)
{
  return (size_t) &((StructType*) (nullptr)->*mp);
}

template<class RootObjectType, class T = RootObjectType>
struct c_ctlbind_context
{
  const size_t offset = 0;

  c_ctlbind_context()
  {
    static_assert(std::is_base_of_v<RootObjectType, T>,
        "RootObjectType must be same base class for T for default constructor");
  }

  template<class FieldType, class U = T, typename = std::enable_if_t<std::is_base_of_v<U, T>>>
  auto operator()(FieldType U::*mp) const
  {
    static_assert(std::is_class_v<U>, "This operator can only be called on classes.!");
    static_assert(std::is_base_of_v<U, T>, "The type of the pointer to member must match the type of the context!");
    return c_ctlbind_context<RootObjectType, FieldType> ( offset + offset_of(mp) );
  }

  template<class Base, typename = std::enable_if_t<std::is_base_of_v<Base, T>>>
  operator c_ctlbind_context<RootObjectType, Base> () const
  {
    static_assert(std::is_base_of_v<Base, T>, "Base must be base class for T");
    return c_ctlbind_context<RootObjectType, Base> (offset);
  }

  template<class Base, typename = std::enable_if_t<std::is_base_of_v<Base, T>>>
  c_ctlbind_context<RootObjectType, Base> as_base() const
  {
    static_assert(std::is_base_of_v<Base, T>, "Base must be base class for T");
    return c_ctlbind_context<RootObjectType, Base> (offset);
  }

private:
  template<class A, class B> friend class c_ctlbind_context;
  c_ctlbind_context(size_t offs) : offset(offs) { }
};

template<class Base, class RootObjectType, class T>
inline auto as_base(const c_ctlbind_context<RootObjectType, T>& ctx)
{
  return ctx.template as_base<Base>();
}



template<class RootObjectType>
void ctlbind_expandable_group(c_ctlist<RootObjectType> & ctls, const std::string & cname, const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BeginExpandableGroup;
  ctls.emplace_back(c);
}

template<class RootObjectType>
void ctlbind_expandable_group(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, bool> & ectx,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BeginExpandableGroup;
  c.enabled = [offset = ectx.offset](const RootObjectType * obj) {
    return obj ? *reinterpret_cast<const bool*>( reinterpret_cast<const uint8_t*>(obj) + offset) : false;
  };

  ctls.emplace_back(c);
}

template<class RootObjectType>
void ctlbind_expandable_group(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const std::function<bool(const RootObjectType * obj)> & eneblefn,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BeginExpandableGroup;
  c.enabled = eneblefn;
  ctls.emplace_back(c);
}


template<class RootObjectType>
void ctlbind_group(c_ctlist<RootObjectType> & ctls)
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.ctype = BindType::CtlType::BeginGroup;
  ctls.emplace_back(c);
}

template<class RootObjectType>
void ctlbind_group(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, bool> & ectx)
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.ctype = BindType::CtlType::BeginGroup;
  c.enabled = [offset = ectx.offset](const RootObjectType * obj) {
    return obj ? *reinterpret_cast<const bool*>( reinterpret_cast<const uint8_t*>(obj) + offset) : false;
  };

  ctls.emplace_back(c);
}



template<class RootObjectType>
void ctlbind_end_group(c_ctlist<RootObjectType> & ctls)
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.ctype = BindType::CtlType::EndGroup;
  ctls.emplace_back(c);
}

template<class RootObjectType, class FieldType>
void ctlbind(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, FieldType> & ctx,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;

  if( std::is_enum_v<FieldType> ) {
    c.ctype = BindType::CtlType::EnumCombobox;
    c.get_enum_members = get_members_of<FieldType>();
  }
  else if( std::is_same_v<FieldType, bool>) {
    c.ctype = BindType::CtlType::Checkbox;
  }
  else if (is_normal_integer_v<FieldType> || std::is_floating_point_v<FieldType> ) {
    c.ctype = BindType::CtlType::NumericBox;
  }
  else {
    c.ctype = BindType::CtlType::Textbox;
  }

  c.getvalue = [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
    return obj ? *s = toString(*reinterpret_cast<const FieldType*>(
        reinterpret_cast<const uint8_t*>(obj) + offset)), true :
        false;
  };

  c.setvalue = [offset = ctx.offset](RootObjectType * obj, const std::string & v) -> bool {
    return obj ? fromString(v, reinterpret_cast<FieldType*>(
        reinterpret_cast<uint8_t*>(obj) + offset)) :
        false;
  };

  ctls.emplace_back(c);
}


template<class RootObjectType, class StructType, class FieldType>
void ctlbind(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    FieldType (StructType::*getv)() const, void (StructType::*setv)(FieldType),
    const std::string & cdesc = "",
    std::string (StructType::*helpstring)() = nullptr)
{
  using BindType = c_ctlbind<RootObjectType>;
  using FT = std::decay_t<FieldType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;

  if( std::is_enum_v<FieldType> ) {
    c.ctype = BindType::CtlType::EnumCombobox;
    c.get_enum_members = get_members_of<FieldType>();
  }
  else if( std::is_same_v<FieldType, bool>) {
    c.ctype = BindType::CtlType::Checkbox;
  }
  else {
    c.ctype = BindType::CtlType::Textbox;
  }

  c.getvalue = [offset = ctx.offset, getv](const RootObjectType * obj, std::string * s) -> bool {
    if ( obj ) {
      const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
      *s = toString((obj2->*getv)());
      return true;
    }
    return false;
  };

  c.setvalue = [offset = ctx.offset, setv](RootObjectType * obj, const std::string & s) -> bool {
    if ( obj ) {
      FT v;
      if ( fromString(s, &v) ) {
        StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
        (obj2->*setv)(v);
        return true;
      }
    }
    return false;
  };

  if( helpstring ) {
    c.helpstring = [offset = ctx.offset, helpstring](RootObjectType * obj) -> std::string {
      if ( obj ) {
        StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
        return (obj2->*helpstring)();
      }
      return "";
    };
  }

  ctls.emplace_back(c);
}


template<class RootObjectType, class FieldType>
void ctlbind_spinbox(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, FieldType> & ctx,
    double minv, double maxv, double step,
    const std::string & cdesc)
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = is_normal_integer_v<FieldType> ? BindType::CtlType::SpinBox : BindType::CtlType::DoubleSpinBox;
  c.range.min = minv;
  c.range.max = maxv;
  c.range.step = step;

  c.getvalue = [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
      return obj ? *s = toString(*reinterpret_cast<const FieldType*>(
          reinterpret_cast<const uint8_t*>(obj) + offset)), true :
          false;
  };

  c.setvalue = [offset = ctx.offset](RootObjectType * obj, const std::string & v) -> bool {
    return obj ? fromString(v, reinterpret_cast<FieldType*>(
        reinterpret_cast<uint8_t*>(obj) + offset)) :
        false;
  };

  ctls.emplace_back(c);
}

template<class RootObjectType, class FieldType>
std::enable_if_t<is_normal_integer_v<FieldType> || std::is_floating_point_v<FieldType>, void>
ctlbind_slider_spinbox(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, FieldType> & ctx,
    double minv, double maxv, double step,
    const std::string & cdesc)
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = is_normal_integer_v<FieldType> ? BindType::CtlType::SpinBox : BindType::CtlType::DoublesliderSpinBox;
  c.range.min = minv;
  c.range.max = maxv;
  c.range.step = step;

  c.getvalue =
      [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
        return obj ? *s = toString(*reinterpret_cast<const FieldType*>(
            reinterpret_cast<const uint8_t*>(obj) + offset)), true :
            false;
      };

  c.setvalue =
      [offset = ctx.offset](RootObjectType * obj, const std::string & v) -> bool {
        return obj ? fromString(v, reinterpret_cast<FieldType*>(
            reinterpret_cast<uint8_t*>(obj) + offset)) :
            false;
      };

  ctls.emplace_back(c);
}


template<class RootObjectType, class StructType, class FieldType>
std::enable_if_t<is_normal_integer_v<FieldType> || std::is_floating_point_v<FieldType>, void>
ctlbind_slider_spinbox(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    FieldType (StructType::*getv)() const, void (StructType::*setv)(FieldType),
    double minv, double maxv, double stepv,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = is_normal_integer_v<FieldType> ? BindType::CtlType::SpinBox : BindType::CtlType::DoublesliderSpinBox;
  c.range.min = minv;
  c.range.max = maxv;
  c.range.step = stepv;

  c.getvalue = [offset = ctx.offset, getv](const RootObjectType * obj, std::string * s) -> bool {
    if ( obj ) {
      const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
      *s = toString((obj2->*getv)());
      return true;
    }
    return false;
  };

  c.setvalue = [offset = ctx.offset, setv](RootObjectType * obj, const std::string & s) -> bool {
    if ( obj ) {
      StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
      FieldType v;
      if ( fromString(s, &v) ) {
        (obj2->*setv)(v);
      }
      return true;
    }
    return false;
  };

  ctls.emplace_back(c);
}


template<class RootObjectType>
void ctlbind_browse_for_directory(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, std::string> & ctx,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BrowseForDirectory;

  const size_t offset = ctx.offset;

  c.getvalue =
      [offset](const RootObjectType * obj, std::string * s) -> bool {
        return obj ? *s = *reinterpret_cast<const std::string*>(reinterpret_cast<const uint8_t*>(obj) + offset), true : false;
      };

  c.setvalue =
      [offset](RootObjectType * obj, const std::string & v) -> bool {
        return obj ? *reinterpret_cast<std::string*>(reinterpret_cast<uint8_t*>(obj) + offset) = v, true : false;
      };

  ctls.emplace_back(c);
}

template<class RootObjectType, class StructType, class StringType>
void ctlbind_browse_for_directory(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    StringType (StructType::*getv)() const, void (StructType::*setv)(StringType),
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BrowseForDirectory;

  const size_t offset = ctx.offset;

  c.getvalue =
      [offset, getv](const RootObjectType * obj, std::string * s) -> bool {
        if ( obj ) {
          const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
          *s = toString((obj2->*getv)());
          return true;
        }
        return false;
      };

  c.setvalue =
      [offset, setv](RootObjectType * obj, const std::string & s) -> bool {
        if ( obj ) {
          StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
          (obj2->*setv)(s);
          return true;
        }
        return false;
      };

  ctls.emplace_back(c);
}


template<class RootObjectType>
void ctlbind_browse_for_file(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, std::string> & ctx,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BrowseForExistingFile;

  const size_t offset = ctx.offset;

  c.getvalue =
      [offset](const RootObjectType * obj, std::string * s) -> bool {
        return obj ? *s = *reinterpret_cast<const std::string*>(reinterpret_cast<const uint8_t*>(obj) + offset), true : false;
      };

  c.setvalue =
      [offset](RootObjectType * obj, const std::string & v) -> bool {
        return obj ? *reinterpret_cast<std::string*>(reinterpret_cast<uint8_t*>(obj) + offset) = v, true : false;
      };

  ctls.emplace_back(c);
}

template<class RootObjectType, class StructType, class StringType>
void ctlbind_browse_for_file(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    StringType (StructType::*getv)() const, void (StructType::*setv)(StringType),
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::BrowseForExistingFile;

  c.getvalue =
      [offset = ctx.offset, getv](const RootObjectType * obj, std::string * s) -> bool {
        if ( obj ) {
          const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
          *s = toString((obj2->*getv)());
          return true;
        }
        return false;
      };

  c.setvalue =
      [offset = ctx.offset, setv](RootObjectType * obj, const std::string & s) -> bool {
        if ( obj ) {
          StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
          (obj2->*setv)(s);
          return true;
        }
        return false;
      };

  ctls.emplace_back(c);
}


template<class EnumType, class RootObjectType>
std::enable_if_t<std::is_enum_v<EnumType>, void>
ctlbind_flags_checkbox(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, int> & ctx,
    const std::string & cdesc )
{
  static_assert(std::is_enum_v<EnumType>, "EnumType must be enum");

  using BindType = c_ctlbind<RootObjectType>;
  using FieldType = int;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::FlagsCheckbox;
  c.get_enum_members = get_members_of<EnumType>();

  c.getvalue =
      [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
        return obj ? *s = toString(*reinterpret_cast<const FieldType*>(
            reinterpret_cast<const uint8_t*>(obj) + offset)), true :
            false;
      };

  c.setvalue =
      [offset = ctx.offset](RootObjectType * obj, const std::string & s) -> bool {
        return obj ? fromString(s, reinterpret_cast<FieldType*>(
            reinterpret_cast<uint8_t*>(obj) + offset)) :
            false;
      };

  ctls.emplace_back(c);

}

template<class EnumType, class RootObjectType, class StructType>
std::enable_if_t<std::is_enum_v<EnumType>, void>
ctlbind_flags_checkbox(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    int (StructType::*getv)() const, void (StructType::*setv)(int),
    const std::string & cdesc )
{
  static_assert(std::is_enum_v<EnumType>, "EnumType must be enum");

  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::FlagsCheckbox;
  c.get_enum_members = get_members_of<EnumType>();

  c.getvalue =
    [offset = ctx.offset, getv](const RootObjectType * obj, std::string * s) -> bool {
      if ( obj ) {
        const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
        *s = toString((obj2->*getv)());
        return true;
      }
      return false;
    };

  c.setvalue =
    [offset = ctx.offset, setv](RootObjectType * obj, const std::string & s) -> bool {
      if ( obj ) {
        int v;
        if ( fromString(s, &v) ) {
          StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
          (obj2->*setv)(v);
          return true;
        }
      }
      return false;
    };

  ctls.emplace_back(c);
}
//
//template<class RootObjectType, class StructType>
//inline void ctlbind_input_source_selection(c_ctlist<RootObjectType> & ctls, const std::string & cname,
//    const c_ctlbind_context<RootObjectType, std::string> & ctx,
//    const std::function<const c_input_sequence*(StructType*)> & get_input_sequence,
//    //const c_input_sequence * (StructType::*get_input_sequence)() const,
//    const std::string & cdesc = "")
//{
//  using BindType = c_ctlbind<RootObjectType>;
//  using FieldType = std::string;
//
//  BindType c;
//  c.cname = cname;
//  c.cdesc = cdesc;
//  c.ctype = BindType::CtlType::InputSourceSelection;
//
//  c.getvalue = [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
//    return obj ? *s = *reinterpret_cast<const std::string*>(reinterpret_cast<const uint8_t*>(obj) + offset), true : false;
//  };
//
//  c.setvalue = [offset = ctx.offset](RootObjectType * obj, const std::string & v) -> bool {
//    return obj ? *reinterpret_cast<std::string*>(reinterpret_cast<uint8_t*>(obj) + offset) = v, true : false;
//  };
//
//  c.input_sequence =
//    [offset = ctx.offset, get_input_sequence](const RootObjectType * obj) -> const c_input_sequence *  {
//      if ( obj ) {
//        const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
//        return get_input_sequence(obj2);
//      }
//      return nullptr;
//    };
//
//  ctls.emplace_back(c);
//}
//

template<class RootObjectType>
inline void ctlbind_math_expression_ctl(c_ctlist<RootObjectType> & ctls,
    const c_ctlbind_context<RootObjectType, std::string> & ctx,
    const std::function<std::string(RootObjectType *)> & helpstring)
{
  using BindType = c_ctlbind<RootObjectType>;
  using FieldType = std::string;

  BindType c;
  c.cname = "";
  c.cdesc = "";
  c.ctype = BindType::CtlType::MathExpression;

  c.getvalue = [offset = ctx.offset](const RootObjectType * obj, std::string * s) -> bool {
    return obj ? *s = *reinterpret_cast<const std::string*>(reinterpret_cast<const uint8_t*>(obj) + offset), true : false;
  };

  c.setvalue = [offset = ctx.offset](RootObjectType * obj, const std::string & v) -> bool {
    return obj ? *reinterpret_cast<std::string*>(reinterpret_cast<uint8_t*>(obj) + offset) = v, true : false;
  };

  ctx.helpstring = helpstring;

  ctls.emplace_back(c);
}


template<class RootObjectType, class StructType, class StringType>
void ctlbind_math_expression_ctl(c_ctlist<RootObjectType> & ctls,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    StringType (StructType::*getv)() const, void (StructType::*setv)(StringType),
    std::string (StructType::*helpstring)())
{
  using BindType = c_ctlbind<RootObjectType>;
  using FieldType = StringType;

  BindType c;
  c.cname = "";
  c.cdesc = "";
  c.ctype = BindType::CtlType::MathExpression;

  c.getvalue = [offset = ctx.offset, getv](const RootObjectType * obj, std::string * s) -> bool {
    if ( obj ) {
      const StructType * obj2 = reinterpret_cast<const StructType*>(reinterpret_cast<const uint8_t*>(obj) + offset);
      *s = toString((obj2->*getv)());
      return true;
    }
    return false;
  };

  c.setvalue = [offset = ctx.offset, setv](RootObjectType * obj, const std::string & s) -> bool {
    if ( obj ) {
      StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
      (obj2->*setv)(s);
      return true;
    }
    return false;
  };

  c.helpstring = [offset = ctx.offset, helpstring](RootObjectType * obj) -> std::string {
    if ( obj ) {
      StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
      return (obj2->*helpstring)();
    }
    return "";
  };

  ctls.emplace_back(c);
}



template<class RootObjectType, class StructType>
inline void ctlbind_menu_button(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  if( !ctls.empty() && ctls.back().ctype == BindType::CtlType::MenuButton ) {
    // Not supported
    return;
  }

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::MenuButton;
  ctls.emplace_back(c);
}


template<class RootObjectType, class StructType>
inline void ctlbind_command_button(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    const std::function<bool(StructType *)> & onclick,
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  if( !ctls.empty() && ctls.back().ctype == BindType::CtlType::MenuButton ) {

    ctls.back().menu.emplace_back();

    auto & item = ctls.back().menu.back();
    item.name = cname;
    item.onclick = [offset = ctx.offset, onclick](RootObjectType * obj) -> bool {
      if ( obj ) {
        StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
        return onclick(obj2);
      }
      return false;
    };

  }
  else {

    BindType c;
    c.cname = cname;
    c.cdesc = cdesc;
    c.ctype = BindType::CtlType::CommandButton;

    c.onclick = [offset = ctx.offset, onclick](RootObjectType * obj) -> bool {
      if ( obj ) {
        StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
        return onclick(obj2);
      }
      return false;
    };

    ctls.emplace_back(c);
  }
}

template<class RootObjectType, class StructType>
inline void ctlbind_command_button(c_ctlist<RootObjectType> & ctls, const std::string & cname,
    const c_ctlbind_context<RootObjectType, StructType> & ctx,
    bool (StructType::*onclick)(),
    const std::string & cdesc = "")
{
  using BindType = c_ctlbind<RootObjectType>;

  BindType c;
  c.cname = cname;
  c.cdesc = cdesc;
  c.ctype = BindType::CtlType::CommandButton;

  c.onclick = [offset = ctx.offset, onclick](RootObjectType * obj) -> bool {
    if ( obj ) {
      StructType * obj2 = reinterpret_cast<StructType*>(reinterpret_cast<uint8_t*>(obj) + offset);
      return (obj2->*onclick)();
    }
    return false;
  };

  ctls.emplace_back(c);
}

// opencv types
#ifdef CV_VERSION

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, cv::TermCriteria> & ctx)
{
  using S = cv::TermCriteria;
  ctlbind_flags_checkbox<cv::TermCriteria::Type>(ctls, "TermCriteria.type", ctx(&S::type), "Termination criteria for Lucas-Kanade optical flow");
  ctlbind(ctls, "TermCriteria.COUNT", ctx(&S::maxCount), "Termination criteria for Lucas-Kanade optical flow");
  ctlbind(ctls, "TermCriteria.epsilon", ctx(&S::epsilon), "Termination criteria for Lucas-Kanade optical flow");
}

#endif // CV_VERSION



#define BIND_CONTROL_MV(ctls, S, M, ctx, desc)
    // ctlbind(ctls, #M, ctx(&S::M), desc)

#define BIND_CONTROL_MF(ctls, ctx, S, M, desc)
    // ctlbind(ctls, #M, ctx, &S::M, &S::set_##M, desc)

#define BIND_CONTROL_MF2(ctls, name, ctx, S, M, desc)
    // ctlbind(ctls, name, ctx, &S::M, &S::set_##M, desc)

#define BIND_FLAGS_CHECKBOX_MF2(ctls, name, ctx, S, M, etype, desc)
    // ctlbind_flags_checkbox<etype>(ctls, name, ctx, &S::M, &S::set_##M, desc);

#define BIND_BROWSE_FOR_FILE_MF(ctls, ctx, S, M, desc)
    // // ctlbind_browse_for_file(ctls, #M, ctx, &S::M, &S::set_##M, desc)

#define BIND_BROWSE_FOR_FILE_MF2(ctls, name, ctx, S, M, desc)
    // // ctlbind_browse_for_file(ctls, name, ctx, &S::M, &S::set_##M, desc)



typedef std::function<void(const std::string&)> ctrlbind_copy_to_clipboard_callback;
void set_ctrlbind_copy_to_clipboard_callback(const ctrlbind_copy_to_clipboard_callback & fn);
const ctrlbind_copy_to_clipboard_callback & get_ctrlbind_copy_to_clipboard_callback();

typedef std::function<std::string()> ctrlbind_get_clipboard_text_callback;
void set_ctrlbind_get_clipboard_text_callback(const ctrlbind_get_clipboard_text_callback & fn);
const ctrlbind_get_clipboard_text_callback & get_ctrlbind_get_clipboard_text_callback();

typedef std::function<void(double x, double y, double w, double h)> ctrlbind_update_roi_callback;
void set_ctrlbind_update_roi_callback(const ctrlbind_update_roi_callback & fn);
const ctrlbind_update_roi_callback & get_ctrlbind_update_roi_callback();

template<class StructType>
bool ctlbind_copy_config_to_clipboard(const std::string & groupName, const StructType & data)
{
  const auto cb = get_ctrlbind_copy_to_clipboard_callback();
  if ( !cb ) {
    return false;
  }

  c_config cfg;
  if ( !save_settings(cfg.root().add_group(groupName), data) ) {
    CF_ERROR("save_settings('%s') fails", groupName.c_str());
    return false;
  }

  cb(cfg.write_string());
  return true;
}

template<class StructType>
bool ctlbind_paste_config_from_clipboard(const std::string & groupName, StructType * data)
{
  const auto cb = get_ctrlbind_get_clipboard_text_callback();
  if ( !cb ) {
    return false;
  }

  const std::string text = cb();
  if ( text.empty() ) {
    CF_ERROR("No clipboard text available");
    return false;
  }

  c_config cfg;
  if ( !cfg.read_string(text.c_str()) ) {
    CF_ERROR("Can not parse clipboard texrt: cfg.read_string() fails");
    return false;
  }

  c_config_setting group = cfg.root()[groupName];
  if ( !group.isGroup() ) {
    CF_ERROR("No group named '%s' is found in config root", groupName.c_str());
    return false;
  }

  if ( !load_settings(group, data) ) {
    CF_ERROR("load_settings() fails for group '%s'", groupName.c_str());
    return false;
  }

  return true;
}

#endif /* __ctrlbind_h__ */
