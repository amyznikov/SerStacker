/*
 * c_master_frame_selection.cc
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */
#include "c_master_frame_selection.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<master_frame_selection_method>()
{
  static const c_enum_member members[] = {
      {master_frame_specific_index, "specific_index", },
      {master_frame_middle_index, "middle_index", },
      {master_frame_best_of_100_in_middle, "best_of_100_in_middle", },
      {master_frame_specific_index },
  };

  return members;
}


c_input_sequence::sptr select_master_source(const c_master_frame_selection_options & opts,
    const c_input_sequence::sptr & input_sequence,
    int * master_source_index)
{
  c_input_sequence::sptr master_sequence;

  * master_source_index = -1;

  std::string master_filename = opts.master_fiename;
  CF_DEBUG("master_filename='%s'", master_filename.c_str());

  if ( master_filename.empty() ) {
    if ( input_sequence && !input_sequence->empty() ) {
      master_filename = input_sequence-> source(*master_source_index = 0)->filename();
      CF_DEBUG("master_filename='%s'", master_filename.c_str());
    }
  }
  else if ( input_sequence && !input_sequence->empty() ) {

    std::vector<c_input_source::sptr>::const_iterator source_pos =
        std::find_if(input_sequence->sources().begin(), input_sequence->sources().end(),
            [&opts](const c_input_source::sptr & s ) -> bool {
              return s->filename() == opts.master_fiename;
            });

    if ( source_pos != input_sequence->sources().end() ) {
      *master_source_index = source_pos - input_sequence->sources().begin();
    }
    CF_DEBUG("*master_source_index ='%d'", *master_source_index );
  }

  if ( *master_source_index >= 0 ) {
    master_sequence = input_sequence;
    CF_DEBUG("master_sequence = input_sequence" );
  }
  else if( !master_filename.empty() && (master_sequence = c_input_sequence::create(master_filename)) ) {
    *master_source_index = 0;
    CF_DEBUG("*master_source_index = 0" );
  }
  else {
    CF_ERROR("ERROR: c_input_sequence::create(master_file_name_='%s') fails", master_filename.c_str());
  }

  return master_sequence;
}

bool serialize_base_master_frame_selection_options(c_config_setting section, bool save,
    c_master_frame_selection_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, master_fiename);
  SERIALIZE_OPTION(section, save, opts, master_selection_method);
  SERIALIZE_OPTION(section, save, opts, master_frame_index);
  SERIALIZE_OPTION(section, save, opts, input_image_preprocessor);
  return true;
}
