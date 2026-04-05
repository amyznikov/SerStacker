/*
 * c_math_expression_routine.cc
 *
 *  Created on: Oct 24, 2023
 *      Author: amyznikov
 */

#include "c_math_expression_routine.h"
#include <core/ssprintf.h>

#if HAVE_TBB
  #include <tbb/tbb.h>

  template<typename T, typename Func>
  static inline void run_loop(T start, T end, Func && f)
  {
    tbb::parallel_for(tbb::blocked_range<int>(start, end), f, tbb::static_partitioner());
  }
  static inline int rbegin(const tbb::blocked_range<int> & range)
  {
    return range.begin();
  }
  static inline int rend(const tbb::blocked_range<int> & range)
  {
    return range.end();
  }

#else

  template<typename T, typename Func>
  static inline void run_loop(T start, T end, Func && f)
  {
    cv::parallel_for_(cv::Range(start, end), f);
  }
  static inline int rbegin(const cv::Range & range)
  {
    return range.start;
  }
  static inline int rend(const cv::Range & range)
  {
    return range.end;
  }

#endif


static constexpr struct
{
  const char * name;
  const char * tooltip;
} math_args[] = {
    { "y", "y coordinate of a pixel" },
    { "x", "x coordinate of a pixel" },
    { "v0", "pixel value from channel 0" },
    { "v1", "pixel value from channel 1" },
    { "v2", "pixel value from channel 2" },
    { "v3", "pixel value from channel 3" },
    { "cn", "current destination channel" },
    { "v", "pixel value from cn source channel" },
};

static const int arg_y_index = 0;
static const int arg_x_index = 1;
static const int arg_v0_index = 2;
static const int arg_v1_index = 3;
static const int arg_v2_index = 4;
static const int arg_v3_index = 5;
static const int arg_cn_index = 6;
static const int arg_v_index = 7;
static const int num_args = sizeof(math_args) / sizeof(math_args[0]) ;
static_assert(num_args == 8, "APP BUG: invalid argument indexing");

template<class T1, class T2>
static bool _process_image(const cv::Mat & _src, cv::Mat & _dst, const c_math_expression & math, cv::InputArray _dst_mask)
{
  const int src_rows = _src.rows;
  const int src_cols = _src.cols;
  const int src_channels = _src.channels();

  const int dst_rows = _dst.rows;
  const int dst_cols = _dst.cols;
  const int dst_channels = _dst.channels();

  const cv::Mat_<T1> src = _src;
  cv::Mat_<T2> dst = _dst;

  if( _dst_mask.empty() ) {

    run_loop(0, src.rows, [=, &src, &dst, &math](const auto & range) {

      double args[num_args] = {0};

      const int beg = rbegin(range), end = rend(range);
      for ( int y = beg; y < end; ++y ) {

        const T1 *srcp = src[y];
        T2 *dstp = dst[y];

        args[arg_y_index] = y;

        for( int x = 0; x < src_cols; ++x ) {

          args[arg_x_index] = x;

          for( int c = 0; c < src_channels; ++c ) {
            args[arg_v0_index + c] = srcp[x * src_channels + c];
          }

          for( int c = 0; c < dst_channels; ++c ) {
            args[arg_cn_index] = c;
            args[arg_v_index] = srcp[x * src_channels + c];
            dstp[x * dst_channels + c] = cv::saturate_cast<T2>(math.eval(args));
          }
        }
      }
    });
  }
  else if( _dst_mask.channels() == 1 ) {

    const cv::Mat1b mask = _dst_mask.getMat();

    run_loop(0, src.rows, [=, &src, &dst, &mask, &math](const auto & range) {

      double args[num_args] = {0};

      const int beg = rbegin(range), end = rend(range);
      for ( int y = beg; y < end; ++y ) {

        const T1 *srcp = src[y];
        const uint8_t * mskp = mask[y];
        T2 *dstp = dst[y];

        args[arg_y_index] = y;

        for( int x = 0; x < src_cols; ++x ) {
          if( mskp[x] ) {

            args[arg_x_index] = x;

            for( int c = 0; c < src_channels; ++c ) {
              args[arg_v0_index + c] = srcp[x * src_channels + c];
            }

            for( int c = 0; c < dst_channels; ++c ) {
              args[arg_cn_index] = c;
              args[arg_v_index] = srcp[x * src_channels + c];
              dstp[x * dst_channels + c] = cv::saturate_cast<T2>(math.eval(args));
            }
          }
        }
      }
    });
  }

  else if ( _dst_mask.channels() == dst_channels ) {

    const cv::Mat_<uint8_t> mask = _dst_mask.getMat();

    run_loop(0, src.rows, [=, &src, &dst, &mask, &math](const auto & range) {

      double args[num_args] = {0};

      const int beg = rbegin(range), end = rend(range);
      for ( int y = beg; y < end; ++y ) {

        const T1 *srcp = src[y];
        const uint8_t * mskp = mask[y];
        T2 *dstp = dst[y];

        args[arg_y_index] = y;

        for( int x = 0; x < src_cols; ++x ) {

          args[arg_x_index] = x;

          for( int c = 0; c < src_channels; ++c ) {
            args[arg_v0_index + c] = srcp[x * src_channels + c];
          }

          for( int c = 0; c < dst_channels; ++c ) {
            if( mskp[x * dst_channels + c] ) {
              args[arg_cn_index] = c;
              args[arg_v_index] = srcp[x * src_channels + c];
              dstp[x * dst_channels + c] = cv::saturate_cast<T2>(math.eval(args));
            }
          }
        }
      }
    });
  }
  else {
    CF_ERROR("Not supported combination of destination image and mask channels.\n"
        "dst_channels = %d dst_mask.channels=%d", dst_channels, _dst_mask.channels());
    return false;
  }

  return true;
}

static bool process_image(const cv::Mat & src, cv::Mat & dst, const c_math_expression & math, cv::InputArray dst_mask)
{
  if( !dst_mask.empty() && dst_mask.depth() != CV_8U ) {
    CF_ERROR("Not supported dst_mask depth=%d. CV_8U is expected", dst_mask.depth());
    return false;
  }

  switch (src.depth()) {
    case CV_8U:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<uint8_t, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<uint8_t, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<uint8_t, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<uint8_t, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<uint8_t, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<uint8_t, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<uint8_t, double>(src, dst, math, dst_mask);
      }
      break;

    case CV_8S:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<int8_t, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<int8_t, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<int8_t, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<int8_t, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<int8_t, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<int8_t, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<int8_t, double>(src, dst, math, dst_mask);
      }
      break;
    case CV_16U:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<uint16_t, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<uint16_t, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<uint16_t, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<uint16_t, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<uint16_t, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<uint16_t, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<uint16_t, double>(src, dst, math, dst_mask);
      }
      break;
    case CV_16S:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<int16_t, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<int16_t, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<int16_t, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<int16_t, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<int16_t, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<int16_t, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<int16_t, double>(src, dst, math, dst_mask);
      }
      break;
    case CV_32S:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<int32_t, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<int32_t, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<int32_t, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<int32_t, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<int32_t, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<int32_t, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<int32_t, double>(src, dst, math, dst_mask);
      }
      break;
    case CV_32F:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<float, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<float, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<float, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<float, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<float, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<float, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<float, double>(src, dst, math, dst_mask);
      }
      break;
    case CV_64F:
      switch (dst.depth()) {
        case CV_8U:
          return _process_image<double, uint8_t>(src, dst, math, dst_mask);
        case CV_8S:
          return _process_image<double, int8_t>(src, dst, math, dst_mask);
        case CV_16U:
          return _process_image<double, uint16_t>(src, dst, math, dst_mask);
        case CV_16S:
          return _process_image<double, int16_t>(src, dst, math, dst_mask);
        case CV_32S:
          return _process_image<double, int32_t>(src, dst, math, dst_mask);
        case CV_32F:
          return _process_image<double, float>(src, dst, math, dst_mask);
        case CV_64F:
          return _process_image<double, double>(src, dst, math, dst_mask);
      }
      break;
  }

  CF_ERROR("Not supported image depth: src.depth()=%d dst.depth()=%d", src.depth(), dst.depth());
  return false;
}

void c_math_expression_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_multiline_textbox(ctls, "", ctx, &this_class::expression, &this_class::set_expression);
  ctlbind(ctls, "INPUT :", ctx(&this_class::_input_channel), "Specify source data for processing");
  ctlbind(ctls, "OUTPUT:", ctx(&this_class::_output_channel), "Specify destination data for processing");
  ctlbind(ctls, "Ignore Mask:", ctx(&this_class::_ignore_mask), "Set true to ignore input mask");
  ctlbind_button_strip(ctls, ctx);
  ctlbind_item(ctls, "Functions...", ctx, [](this_class * _ths) {
    ctlbind_show_info_text("Math Expression", _ths->_helpstring);
    return false;
  });
}

bool c_math_expression_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, expression);
    SERIALIZE_PROPERTY(settings, save, *this, input_channel);
    SERIALIZE_PROPERTY(settings, save, *this, output_channel);
    return true;
  }
  return false;
}

bool c_math_expression_routine::initialize()
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

bool c_math_expression_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _expression.empty() ) {
    _expression_changed = false;
    return true;
  }

  if( _expression_changed ) {

    if ( !_math.parse(_expression.c_str()) ) {

      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", _math.error_message().c_str(),
          _math.pointer_to_syntax_error() ? _math.pointer_to_syntax_error() : "null");

      return false;
    }

    _expression_changed = false;
  }

  switch (_input_channel) {
    case IMAGE:
      switch (_output_channel) {
        case IMAGE:
          process_image(image.getMat(), image.getMatRef(), _math, _ignore_mask ? cv::noArray() : mask);
          break;

        case MASK: {
          mask.create(image.size(), CV_8UC1);
          process_image(image.getMat(), mask.getMatRef(), _math, cv::noArray());
          break;
        }
      }
      break;

    case MASK:
      switch (_output_channel) {
        case MASK:
          process_image(mask.getMat(), mask.getMatRef(), _math, _ignore_mask ? cv::noArray() : mask.getMat());
          break;

        case IMAGE:
          if( !mask.empty() ) {
            if( image.size() != mask.size() || image.channels() != mask.channels() ) {
              image.create(mask.size(), CV_MAKETYPE(std::max(image.depth(), mask.depth()), mask.channels()));
            }
            process_image(mask.getMat(), image.getMatRef(), _math, _ignore_mask ? cv::noArray() : mask.getMat());
          }
          break;
      }
      break;
  }

  return true;
}
