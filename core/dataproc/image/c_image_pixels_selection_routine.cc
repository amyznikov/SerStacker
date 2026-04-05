/*
 * c_image_pixels_selection_routine.cc
 *
 *  Created on: Jan 7, 2024
 *      Author: amyznikov
 */

#include "c_image_pixels_selection_routine.h"
#include <core/ssprintf.h>

// TODO: is c_math_parser thread-safe ?
#if HAVE_TBB
 #include <tbb/tbb.h>
 typedef tbb::blocked_range<int> tbb_range;
#endif

static const struct
{
  const char * name;
  const char * tooltip;
} math_args[] = {
    { "cn", "number of image channels" },
    { "y", "y coordinate of a pixel" },
    { "x", "x coordinate of a pixel" },
    { "v0", "pixel value from channel 0" },
    { "v1", "pixel value from channel 1" },
    { "v2", "pixel value from channel 2" },
    { "v3", "pixel value from channel 3" },
};



template<class T1>
static void _process_image(const cv::Mat & _src, cv::Mat1b & dst, const c_math_expression & math)
{
  const int src_rows = _src.rows;
  const int src_cols = _src.cols;
  const int src_channels = _src.channels();

  const cv::Mat_<T1> src = _src;

  dst.create(_src.size());
  dst.setTo(0);


#if HAVE_TBB
  tbb::parallel_for(tbb_range(0, src_rows),
      [&src, &dst, &math, src_rows, src_cols, src_channels](const tbb_range & range) {

        double args[sizeof(math_args) / sizeof(math_args[0])] = {0};
        args[0] = src_channels;

        for( int y = range.begin(); y < range.end(); ++y ) {

#else
          double args[sizeof(math_args) / sizeof(math_args[0])] = {0};
          args[0] = src_channels;

          for( int y = 0; y < src_rows; ++y ) {
#endif

          const T1 *srcp = src[y];
          uint8_t *dstp = dst[y];

          args[1] = y;

          for( int x = 0; x < src_cols; ++x ) {

            args[2] = x;

            for( int c = 0; c < src_channels; ++c ) {
              args[2 + c] = srcp[x * src_channels + c];
            }

            if ( math.eval(args) ) {
              dstp[x] = 255;
            }
          }
#if HAVE_TBB
        }});
#else
    }
#endif
}

static void process_image(const cv::Mat & src, cv::Mat1b & dst, const c_math_expression & math)
{
  switch (src.depth()) {
    case CV_8U:
      return _process_image<uint8_t>(src, dst, math);
    case CV_8S:
      return _process_image<int8_t>(src, dst, math);
    case CV_16U:
      return _process_image<uint16_t>(src, dst, math);
    case CV_16S:
      return _process_image<int16_t>(src, dst, math);
    case CV_32S:
      return _process_image<int32_t>(src, dst, math);
    case CV_32F:
      return _process_image<float>(src, dst, math);
    case CV_64F:
      return _process_image<double>(src, dst, math);
  }
}


bool c_image_pixels_selection_routine::initialize()
{
  if ( _math.arguments().empty() ) {
    for( int i = 0; i < (int) (sizeof(math_args) / sizeof(math_args[0])); ++i ) {
      if( !_math.add_argument(i, math_args[i].name, math_args[i].tooltip) ) {
        CF_ERROR("math_.add_argument('%s') fails", math_args[i].name);
      }
    }
  }

  if ( _helpstring.empty() ) {

    _helpstring = "Apply math formula to pixel values\n";

    _helpstring += "\nArguments:\n";
    for ( const auto & c : _math.arguments() ) {
      _helpstring += ssprintf("%s  : %s\n", c.name.c_str(), c.desc.c_str());
    }

    _helpstring += "\nUnary Operations:\n";
    for ( const auto & c : _math.unary_operations() ) {
      _helpstring += ssprintf("%s  : %s\n", c.name.c_str(), c.desc.c_str());
    }


    _helpstring += "\nBinary Operations:\n";
    for( const auto & c : _math.binary_operations() ) {
      _helpstring += "-------------------------\n";
      for( const auto & op : c ) {
        _helpstring += ssprintf("%s  : %s\n", op.name.c_str(), op.desc.c_str());
      }
    }

    _helpstring += "\nConstants:\n";
    for ( const auto & c : _math.constants() ) {
      _helpstring += ssprintf("%s  : %s\n", c.name.c_str(), c.desc.c_str());
    }

    _helpstring += "\nFunctions:\n";
    for ( const auto & c : _math.functions() ) {
      _helpstring += ssprintf("%s  : %s\n", c.name.c_str(), c.desc.c_str());
    }
  }

  return true;
}

void c_image_pixels_selection_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "expression", ctx, &this_class::expression, &this_class::set_expression, "");
  ctlbind(ctls, "invert_selection", ctx, &this_class::invert_selection, &this_class::set_invert_selection);
  ctlbind(ctls, "mask_mode", ctx, &this_class::mask_mode, &this_class::set_mask_mode, "Selection Combine Mode");
  ctlbind_button_strip(ctls, ctx);
  ctlbind_item(ctls, "Functions...", ctx, [](this_class * _ths) {
    ctlbind_show_info_text("Math Expression", _ths->_helpstring);
    return false;
  });
}

bool c_image_pixels_selection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, expression);
    SERIALIZE_PROPERTY(settings, save, *this, invert_selection);
    SERIALIZE_PROPERTY(settings, save, *this, mask_mode);
    return true;
  }

  return false;
}

bool c_image_pixels_selection_routine::process(c_video_frame * frame)
{
  if( _expression.empty() ) {
    _expression_changed = false;
    return true;
  }

  if( _expression_changed ) {

    if( !_math.parse(_expression.c_str()) ) {
      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", _math.error_message().c_str(),
          _math.pointer_to_syntax_error() ? _math.pointer_to_syntax_error() : "null");

      return false;
    }

    _expression_changed = false;
  }


  frame->get_image("PIXEL_VALUE", _current_image, cv::noArray(), cv::noArray());
  process_image(_current_image, _current_mask, _math);

  if ( _invert_selection ) {
    cv::bitwise_not(_current_mask, _current_mask);
  }

  frame->update_selection(_current_mask, _mask_mode);

  return true;
}
