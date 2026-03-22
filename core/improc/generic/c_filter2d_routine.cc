/*
 * c_filter2d_routine.cc
 *
 *  Created on: Mar 21, 2026
 *      Author: amyznikov
 */

#include "c_filter2d_routine.h"
#include <core/ssprintf.h>
#include <core/debug.h>

void c_filter2d_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "anchor", ctx(&this_class::_anchor), "");
  ctlbind(ctls, "delta", ctx(&this_class::_delta), "");
  ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "");
  ctlbind(ctls, "borderType", ctx(&this_class::_borderType), "");
  ctlbind_multiline_textbox(ctls, "kernel", ctx, &this_class::kernel, &this_class::set_kernel);
}

bool c_filter2d_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
     SERIALIZE_OPTION(settings, save, *this, _anchor);
     SERIALIZE_OPTION(settings, save, *this, _delta);
     SERIALIZE_OPTION(settings, save, *this, _ddepth);
     SERIALIZE_OPTION(settings, save, *this, _borderType);
     SERIALIZE_OPTION(settings, save, *this, _kernel);
    return true;
  }
  return false;
}

bool c_filter2d_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
//  CF_DEBUG("kernel=\n%s\n", _kernel.c_str());

  if( _K.empty() && !_kernel.empty() ) {

    cv::Mat1f K;

    size_t num_columns = 0;

    const std::vector<std::string> rows = strsplit(_kernel, "\n\r");
    for( size_t i = 0, nrows = rows.size(); i < nrows; ++i ) {
      const std::vector<std::string> cols = strsplit(rows[i], " \t;,");
      const size_t ncols = cols.size();
      if ( i == 0 ) {
        num_columns = ncols;
        K.create(nrows, ncols);
      }
      else if ( ncols != num_columns ) {
        CF_ERROR("Bad column number in row %zu", i);
        return false;
      }

      for ( size_t j = 0; j < ncols; ++j ) {

        const char * s = cols[j].c_str();
        double v = 0;

        if ( sscanf(s, "%lf", &v) != 1 ) {
          CF_ERROR("Can not parse item '%s' at row=%zu col=%zu", s, i, j );
          return false;
        }

        K(i, j) = v;
      }
    }

    _K = std::move(K);
  }

  if( !_K.empty() ) {
    if( _ddepth < 0 || _ddepth >= image.depth() ) {
      cv::filter2D(image.getMat(), image, _ddepth, _K, _anchor, _delta, _borderType);
    }
    else {
      cv::filter2D(image.getMat(), image, -1, _K, _anchor, _delta, _borderType);
      image.getMat().convertTo(image, _ddepth);
    }
  }

  return true;
}
