/*
 * c_math_expression_routine.cc
 *
 *  Created on: Oct 24, 2023
 *      Author: amyznikov
 */

#include "c_math_expression_routine.h"
#include <core/ssprintf.h>

// TODO: is c_math_parser thread-safe ?
#if HAVE_TBB
 #include <tbb/tbb.h>
 typedef tbb::blocked_range<int> tbb_range;
#endif

template<>
const c_enum_member* members_of<c_math_expression_routine::CHANNEL>()
{
  static const c_enum_member members[] = {
      { c_math_expression_routine::IMAGE, "IMAGE", "" },
      { c_math_expression_routine::MASK, "MASK", "" },
      { c_math_expression_routine::IMAGE },
  };

  return members;
}

static constexpr struct
{
  const char * name;
  const char * tooldip;
} processor_args[] = {
    {"y", "y coordinate of a pixel"},
    {"x", "x coordinate of a pixel"},
    {"v0", "pixel value from channel 0"},
    {"v1", "pixel value from channel 1"},
    {"v2", "pixel value from channel 2"},
    {"v3", "pixel value from channel 3"},
    {"cn", "current destination channel"},
    {"v",  "pixel value from cn source channel"},
};

template<class T1, class T2>
static void process_image_(const cv::Mat & _src, cv::Mat & _dst, const c_math_expression & math, cv::InputArray _mask)
{
  const int src_rows =
      _src.rows;

  const int src_cols =
      _src.cols;

  const int src_channels =
      _src.channels();

  const int dst_rows =
      _dst.rows;

  const int dst_cols =
      _dst.cols;

  const int dst_channels =
      _dst.channels();

  const cv::Mat_<T1> src = _src;
  const cv::Mat1b mask = _mask.empty() ? cv::Mat1b() : _mask.getMat();
  cv::Mat_<T2> dst = _dst;


  if ( mask.empty() ) {
#if HAVE_TBB
    tbb::parallel_for(tbb_range(0, src_rows),
        [&src, &dst, &math, src_rows, src_cols, src_channels, dst_channels] (const tbb_range & range) {

      double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};

      for( int y = range.begin(); y < range.end(); ++y ) {

#else
      double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};

      for( int y = 0; y < src_rows; ++y ) {
#endif

      const T1 *srcp = src[y];
      T2 *dstp = dst[y];

      args[0] = y;

      for( int x = 0; x < src_cols; ++x ) {

        args[1] = x;

        for( int c = 0; c < src_channels; ++c ) {
          args[2 + c] = srcp[x * src_channels + c];
        }

        for( int c = 0; c < dst_channels; ++c ) {
          args[6] = c;
          args[7] = srcp[x * src_channels + c];
          dstp[x * dst_channels + c] = math.eval(args);
        }
      }
#if HAVE_TBB
      }});
#else
    }
#endif
  }

  else {

#if HAVE_TBB
    tbb::parallel_for(tbb_range(0, src_rows),
        [&src, &dst, &mask, &math, src_rows, src_cols, src_channels, dst_channels](const tbb_range & range) {

          double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};

          for( int y = range.begin(); y < range.end(); ++y ) {

#else
          double args[sizeof(processor_args) / sizeof(processor_args[0])] = {0};

          for( int y = 0; y < src_rows; ++y ) {
#endif

            const T1 *srcp = src[y];
            T2 *dstp = dst[y];

            args[0] = y;

            for( int x = 0; x < src_cols; ++x ) {
              if( mask[y][x] ) {

                args[1] = x;

                for( int c = 0; c < src_channels; ++c ) {
                  args[2 + c] = srcp[x * src_channels + c];
                }

                for( int c = 0; c < dst_channels; ++c ) {
                  args[6] = c;
                  args[7] = srcp[x * src_channels + c];
                  dstp[x * dst_channels + c] = math.eval(args);
                }
              }
            }
#if HAVE_TBB
          }});
#else
         }
#endif
  }
}

static void process_image(const cv::Mat & src, cv::Mat & dst, const c_math_expression & math, cv::InputArray _mask)
{
  switch (src.depth()) {
    case CV_8U:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<uint8_t,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<uint8_t,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<uint8_t,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<uint8_t,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<uint8_t,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<uint8_t,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<uint8_t,double>(src, dst,math, _mask);
      }
      break;

    case CV_8S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int8_t,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<int8_t,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<int8_t,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<int8_t,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<int8_t,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<int8_t,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<int8_t,double>(src, dst,math, _mask);
      }
      break;
    case CV_16U:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<uint16_t,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<uint16_t,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<uint16_t,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<uint16_t,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<uint16_t,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<uint16_t,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<uint16_t,double>(src, dst,math, _mask);
      }
      break;
    case CV_16S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int16_t,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<int16_t,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<int16_t,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<int16_t,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<int16_t,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<int16_t,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<int16_t,double>(src, dst,math, _mask);
      }
      break;
    case CV_32S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int32_t,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<int32_t,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<int32_t,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<int32_t,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<int32_t,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<int32_t,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<int32_t,double>(src, dst,math, _mask);
      }
      break;
    case CV_32F:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<float,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<float,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<float,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<float,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<float,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<float,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<float,double>(src, dst,math, _mask);
      }
      break;
    case CV_64F:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<double,uint8_t>(src, dst,math, _mask);
        case CV_8S:
          return process_image_<double,int8_t>(src, dst,math, _mask);
        case CV_16U:
          return process_image_<double,uint16_t>(src, dst,math, _mask);
        case CV_16S:
          return process_image_<double,int16_t>(src, dst,math, _mask);
        case CV_32S:
          return process_image_<double,int32_t>(src, dst,math, _mask);
        case CV_32F:
          return process_image_<double,float>(src, dst,math, _mask);
        case CV_64F:
          return process_image_<double,double>(src, dst,math, _mask);
      }
      break;
  }
}

c_math_expression_routine::~c_math_expression_routine()
{
}

void c_math_expression_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL2(ctls, input_channel, "input from:", "");
  ADD_IMAGE_PROCESSOR_CTRL2(ctls, output_channel, "output to:", "");
  ADD_IMAGE_PROCESSOR_CTRL2(ctls, ignore_mask, "ignore mask:", "");
  ADD_IMAGE_PROCESSOR_CTRL_MATH_EXPRESSION(ctls, expression, "", "formula for math expression");
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

bool c_math_expression_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( expression_.empty() ) {
    expression_changed_ = false;
    return true;
  }

  if( !initialized_ ) {

    initialized_ = true;

    for( int i = 0; i < (int) (sizeof(processor_args) / sizeof(processor_args[0])); ++i ) {
      if( !math_.add_argument(i, processor_args[i].name, processor_args[i].tooldip) ) {
        CF_ERROR("math_.add_argument('%s') fails", processor_args[i].name);
      }
    }
  }

  if( expression_changed_ ) {

    if ( !math_.parse(expression_.c_str()) ) {

      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", math_.error_message().c_str(),
          math_.pointer_to_syntax_error() ? math_.pointer_to_syntax_error() : "null");

      return false;
    }

    expression_changed_ = false;
  }

  switch (input_channel_) {
    case IMAGE:
      switch (output_channel_) {

        case IMAGE:
          process_image(image.getMatRef(), image.getMatRef(), math_,
              ignore_mask_ ? cv::noArray() :
                  mask);
          break;

        case MASK: {
          cv::Mat1b input_mask;
          if( !mask.empty() ) {
            if( !ignore_mask_ ) {
              mask.getMat().copyTo(input_mask);
            }
          }
          else {
            if( mask.size() != image.size() ) {
              mask.create(image.size(), CV_8U);
            }
            mask.setTo(0);
          }
          process_image(image.getMatRef(), mask.getMatRef(), math_,
              input_mask);
          break;
        }
      }
      break;

    case MASK:
      switch (output_channel_) {

        case MASK:
          process_image(mask.getMatRef(), mask.getMatRef(), math_,
              ignore_mask_ ? cv::noArray() : mask.getMat());
          break;

        case IMAGE:
          if( !mask.empty() ) {

            if( image.size() != mask.size() || image.channels() != mask.channels() ) {
              image.create(mask.size(), CV_MAKETYPE(std::max(image.depth(), mask.depth()), mask.channels()));
            }
            process_image(mask.getMatRef(), image.getMatRef(), math_,
                ignore_mask_ ? cv::noArray() : mask.getMat());
          }
          break;
      }
      break;
  }

  return true;
}
