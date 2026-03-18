/*
 * c_roi_selection.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_roi_selection.h"
#include "c_roi_rectangle_selection.h"
#include "c_planetary_disk_selection.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<roi_selection_method>()
{
  static const c_enum_member members[] = {
      { roi_selection_none, "none", },
      { roi_selection_rectange_crop, "rectangle", },
      { roi_selection_planetary_disk, "planetary_disk", },
      { roi_selection_none, },
  };
  return members;
}

c_roi_selection::sptr c_roi_selection::create(const c_roi_selection_options & opts)
{
  switch(opts.method)
  {
    case roi_selection_rectange_crop:
      return sptr(new c_roi_rectangle_selection(
          opts.rectangle_roi_selection));

    case roi_selection_planetary_disk:
      return sptr(new c_planetary_disk_selection(
          opts.planetary_disk_crop_size,
          opts.planetary_disk_gbsigma,
          opts.planetary_disk_stdev_factor,
          opts.se_close_size));

    default:
      break;
  }

  return nullptr;
}
