/*
 * c_stereo_input_options.h
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_input_options_h__
#define __c_stereo_input_options_h__

#include <core/io/c_stereo_input.h>
#include <core/pipeline/c_image_processing_pipeline.h>

struct c_stereo_input_source_options
{
  const c_input_sequence * input_sequence = nullptr;
  std::string left_stereo_source;
  std::string right_stereo_source;
  stereo_input_frame_layout_type layout_type = stereo_frame_layout_horizontal;
  bool swap_cameras = false;
};

struct c_stereo_input_options:
    c_image_processing_pipeline_input_options
{
  c_stereo_input_source_options input_source;
};

bool serialize_base_stereo_input_options(c_config_setting section, bool save, c_stereo_input_options & opts);

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_input_source_options> & ctx)
{
  using BindType = c_ctlbind<RootObjectType>;
  using FieldType = c_stereo_input_source_options;

  BindType c;
  c.cname = "";
  c.cdesc = "";
  c.ctype = BindType::CtlType::StereoInputSourceSelection;

  c.stereo_input_source_options =
      [offset = ctx.offset](RootObjectType * obj) -> FieldType *  {
        return obj ? reinterpret_cast<FieldType*>(reinterpret_cast<uint8_t*>(obj) + offset): nullptr;
      };

  ctls.emplace_back(c);
}


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_input_options> & ctx)
{
  using S = c_stereo_input_options;

  ctlbind(ctls, ctx(&S::input_source));
  ctlbind_expandable_group(ctls, "Input Sequence", "");
    ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
  ctlbind_end_group(ctls);
}

#endif /* __c_stereo_input_options_h__ */
