/*
 * c_master_frame_selection.h
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_master_frame_selection_h__
#define __c_master_frame_selection_h__

#include <core/io/c_input_sequence.h>
#include <core/ctrlbind/ctrlbind.h>

enum master_frame_selection_method
{
  master_frame_specific_index,
  master_frame_middle_index,
  master_frame_best_of_100_in_middle,
};

struct c_master_frame_selection_options
{
  c_input_sequence * input_sequence = nullptr;
  std::string master_fiename;
  master_frame_selection_method master_selection_method = master_frame_specific_index;
  int master_frame_index = 0;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls,const c_ctlbind_context<RootObjectType, c_master_frame_selection_options> & ctx)
{
  using BindType = c_ctlbind<RootObjectType>;
  using FieldType = c_master_frame_selection_options;

  BindType c;
  c.ctype = BindType::CtlType::MasterFrameSelection;
  c.master_frame_selection =
      [offset = ctx.offset](RootObjectType * obj) -> FieldType *  {
        return obj ? reinterpret_cast<FieldType*>(reinterpret_cast<uint8_t*>(obj) + offset): nullptr;
      };

  ctls.emplace_back(c);
}

c_input_sequence::sptr select_master_source(const c_master_frame_selection_options & opts,
    const c_input_sequence::sptr & input_sequence,
    int * master_source_index);


#endif /* __c_master_frame_selection_h__ */
