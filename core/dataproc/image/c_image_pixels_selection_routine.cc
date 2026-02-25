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
  const char * tooldip;
} processor_args[] = {
    { "cn", "number of image channels" },
    { "y", "y coordinate of a pixel" },
    { "x", "x coordinate of a pixel" },
    { "v0", "pixel value from channel 0" },
    { "v1", "pixel value from channel 1" },
    { "v2", "pixel value from channel 2" },
    { "v3", "pixel value from channel 3" },
};



template<class T1>
static void process_image_(const cv::Mat & _src, cv::Mat1b & dst, const c_math_expression & math)
{
  const int src_rows =
      _src.rows;

  const int src_cols =
      _src.cols;

  const int src_channels =
      _src.channels();

  const cv::Mat_<T1> src = _src;

  dst.create(_src.size());
  dst.setTo(0);


#if HAVE_TBB
  tbb::parallel_for(tbb_range(0, src_rows),
      [&src, &dst, &math, src_rows, src_cols, src_channels](const tbb_range & range) {

        double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};
        args[0] = src_channels;

        for( int y = range.begin(); y < range.end(); ++y ) {

#else
          double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};
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
      return process_image_<uint8_t>(src, dst, math);
    case CV_8S:
      return process_image_<int8_t>(src, dst, math);
    case CV_16U:
      return process_image_<uint16_t>(src, dst, math);
    case CV_16S:
      return process_image_<int16_t>(src, dst, math);
    case CV_32S:
      return process_image_<int32_t>(src, dst, math);
    case CV_32F:
      return process_image_<float>(src, dst, math);
    case CV_64F:
      return process_image_<double>(src, dst, math);
  }
}



std::string c_image_pixels_selection_routine::helpstring()
{
  static std::string _helpstring;

  if ( _helpstring.empty() ) {

    _helpstring.append("Arguments:\n");
    for( int i = 0, n = sizeof(processor_args) / sizeof(processor_args[0]); i < n; ++i ) {
      _helpstring.append(ssprintf("%s %s\n", processor_args[i].name, processor_args[i].tooldip));
    }

    _helpstring.append("\nConstants:\n");
    for ( const auto & func: _math.constants() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nUnary operations:\n");
    for ( const auto & func: _math.unary_operations() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nBinary operations:\n");
    for ( int p = 0, np = _math.binary_operations().size(); p < np; ++p ) {
      for ( const auto & func: _math.binary_operations()[p] ) {
        _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
      }
    }

    _helpstring.append("\nFunctions:\n");
    for ( const auto & func: _math.functions() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

  }

  return _helpstring;
}

//void c_image_pixels_selection_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
//{
//  BIND_MATH_EXPRESSION_CTRL(ctls, expression, helpstring, "", "formula for math expression");
//  BIND_CTRL(ctls, invert_selection, "invert_selection", "invert_selection");
//  BIND_CTRL(ctls, mask_mode, "mask_mode", "combine selection mode");
//}

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

    if( !_initialized ) {
      for( int i = 0, n = sizeof(processor_args) / sizeof(processor_args[0]); i < n; ++i ) {
        _math.add_argument(i, processor_args[i].name, processor_args[i].tooldip);
      }
    }

    if ( !_math.parse(_expression.c_str()) ) {

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
