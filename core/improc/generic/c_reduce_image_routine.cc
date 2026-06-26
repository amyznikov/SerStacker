/*
 * c_reduce_image_routine.cc
 *
 *  Created on: Jun 25, 2026
 *      Author: amyznikov
 */

#include "c_reduce_image_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_reduce_image_routine::REDUCE_DIM>()
{
  static const c_enum_member members[] = {
      { c_reduce_image_routine::REDUCE_ROWS, "ROWS", "matrix is reduced to a single row" },
      { c_reduce_image_routine::REDUCE_COLS, "COLS", "matrix is reduced to a single column"},
      { c_reduce_image_routine::REDUCE_ROWS}
  };

  return members;
}


void c_reduce_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "rtype", ctx(&this_class::_rtype), "cv::ReduceType");
  ctlbind(ctls, "dim", ctx(&this_class::_dim), "Reduce Dimension");
  ctlbind(ctls, "Use ROI selection", ctx(&this_class::_useROISelection), "Use ROI selected in GUI");
  ctlbind(ctls, "rect", ctx(&this_class::_rect), "Rectangular ROI to crop, X,Y;WxH");
}

bool c_reduce_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _rect);
    SERIALIZE_OPTION(settings, save, *this, _useROISelection);

    return true;
  }
  return false;
}

bool c_reduce_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();
  const cv::Size size = image.size();

  cv::Rect rc;
  if ( !_useROISelection ) {
    rc = _rect;
  }
  else if ( !ctlbind_get_roi(&rc) ) {
    CF_ERROR("ROI is not selected");
    return false;
  }

  rc &= cv::Rect(0, 0, size.width, size.height);
  if ( rc.empty() ) {
    CF_ERROR("c_reduce_image_routine: adjusted Rect is empty");
    return false;
  }

  cv::Mat dst;
  int dtype = -1;
  if ( _rtype == cv::REDUCE_SUM || _rtype == cv::REDUCE_SUM2 ) {
    dtype = std::max(src.depth(), CV_32F);
  }

  cv::reduce(src(rc), dst, _dim, _rtype, dtype);
  if ( _dim == REDUCE_ROWS ) { // 0 means that the matrix is reduced to a single row
    dst = cv::repeat(dst, rc.height, 1 );
  }
  else { // 1 means that the matrix is reduced to a single column
    dst = cv::repeat(dst, 1, rc.width);
  }

  if ( dst.size() == image.size() ) {
    image.move(dst);
  }
  else {
    if ( image.depth() != dst.depth() ) {
      image.getMat().convertTo(image, dst.depth());
    }
    dst.copyTo(image.getMatRef()(rc));
  }

  return true;
}

