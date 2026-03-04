/*
 * c_edge_preserving_filter_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_edge_preserving_filter_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_edge_preserving_filter_routine::FilterType>()
{
  static const c_enum_member members[] = {
      { c_edge_preserving_filter_routine::FilterType::RECURS_FILTER, "RECURS" },
      { c_edge_preserving_filter_routine::FilterType::NORMCONV_FILTER, "NORMCONV" },
      { c_edge_preserving_filter_routine::FilterType::RECURS_FILTER }  // must  be last
  };

  return members;
}


void c_edge_preserving_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "filterType", ctx(&this_class::filterType),
      "cv::RECURS_FILTER or cv::NORMCONV_FILTER");

  ctlbind(ctls, "sigma_s", ctx(&this_class::sigma_s),
      "Range between 0 to 200.");

  ctlbind(ctls, "sigma_r", ctx(&this_class::sigma_r),
      "Range between 0 to 1");

}

bool c_edge_preserving_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, filterType);
    SERIALIZE_OPTION(settings, save, *this, sigma_s);
    SERIALIZE_OPTION(settings, save, *this, sigma_r);
    return true;
  }
  return false;
}

bool c_edge_preserving_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
#if HAVE_OpenCV_photo
  cv::edgePreservingFilter(image.getMat(), image, filterType, sigma_s, sigma_r);
  return true;
#else
  CF_ERROR("OpenCV module photo is not available. Can not call cv::edgePreservingFilter()");
  (void)(image);
  (void)(mask);
  return false;
#endif
}
