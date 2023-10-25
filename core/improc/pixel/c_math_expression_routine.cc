/*
 * c_math_expression_routine.cc
 *
 *  Created on: Oct 24, 2023
 *      Author: amyznikov
 */

#include "c_math_expression_routine.h"
#include <core/ssprintf.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

template<>
const c_enum_member* members_of<c_math_expression_routine::CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { c_math_expression_routine::IMAGE, "IMAGE", "" },
      { c_math_expression_routine::MASK, "MASK", "" },
      { c_math_expression_routine::IMAGE },
  };

  return members;
}


template<class T1, class T2 >
static void process_image_(const cv::Mat & _src, cv::Mat & _dst, c_math_parser * p)
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
  cv::Mat_<T2> dst = _dst;



  double args[8] = {0};

  for ( int y = 0; y < src_rows; ++y ) {

    const T1 * srcp = src[y];
    T2 * dstp = dst[y];

    args[1] = y;

    for ( int x = 0; x < src_cols; ++x ) {

      args[0] = x;
      args[2] = srcp[x];

      for ( int c = 0; c < src_channels; ++c ) {
        args[3 + c] = srcp[x * src_channels + c];
      }

      const double rv =
          c_math_parser_eval(p, args);

      for ( int c = 0; c < dst_channels; ++c ) {
        dstp[x*dst_channels + c] = rv;
      }
    }
  }
}

static void process_image(const cv::Mat & src, cv::Mat & dst, c_math_parser * p)
{
  switch (src.depth()) {
    case CV_8U:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<uint8_t,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<uint8_t,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<uint8_t,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<uint8_t,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<uint8_t,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<uint8_t,float>(src, dst, p);
        case CV_64F:
          return process_image_<uint8_t,double>(src, dst, p);
      }
      break;

    case CV_8S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int8_t,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<int8_t,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<int8_t,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<int8_t,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<int8_t,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<int8_t,float>(src, dst, p);
        case CV_64F:
          return process_image_<int8_t,double>(src, dst, p);
      }
      break;
    case CV_16U:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<uint16_t,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<uint16_t,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<uint16_t,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<uint16_t,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<uint16_t,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<uint16_t,float>(src, dst, p);
        case CV_64F:
          return process_image_<uint16_t,double>(src, dst, p);
      }
      break;
    case CV_16S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int16_t,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<int16_t,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<int16_t,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<int16_t,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<int16_t,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<int16_t,float>(src, dst, p);
        case CV_64F:
          return process_image_<int16_t,double>(src, dst, p);
      }
      break;
    case CV_32S:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<int32_t,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<int32_t,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<int32_t,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<int32_t,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<int32_t,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<int32_t,float>(src, dst, p);
        case CV_64F:
          return process_image_<int32_t,double>(src, dst, p);
      }
      break;
    case CV_32F:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<float,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<float,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<float,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<float,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<float,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<float,float>(src, dst, p);
        case CV_64F:
          return process_image_<float,double>(src, dst, p);
      }
      break;
    case CV_64F:
      switch (dst.depth()) {
        case CV_8U:
          return process_image_<double,uint8_t>(src, dst, p);
        case CV_8S:
          return process_image_<double,int8_t>(src, dst, p);
        case CV_16U:
          return process_image_<double,uint16_t>(src, dst, p);
        case CV_16S:
          return process_image_<double,int16_t>(src, dst, p);
        case CV_32S:
          return process_image_<double,int32_t>(src, dst, p);
        case CV_32F:
          return process_image_<double,float>(src, dst, p);
        case CV_64F:
          return process_image_<double,double>(src, dst, p);
      }
      break;
  }
}

c_math_expression_routine::~c_math_expression_routine()
{
  c_math_parser_destroy(math_parser_);
}

void c_math_expression_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL2(ctls, input_channel, "input from:", "");
  ADD_IMAGE_PROCESSOR_CTRL2(ctls, output_channel, "output to:", "");
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
  CF_DEBUG("expression: %s", expression_.c_str());

  if( expression_.empty() ) {
    expression_changed_ = false;
    return true;
  }

  if( !math_parser_ ) {

    if ( !(math_parser_ = c_math_parser_create_stdc())  ) {
      CF_ERROR("c_math_parser_create_stdc() fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 0, "x", "x coordiante of a pixel") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(x) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 1, "y", "y coordiante of a pixel") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(y) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 2, "v", "pixel value") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(v) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 3, "v0", "pixel value from channel 0") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(v0) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 4, "v1", "pixel value from channel 1") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(v1) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 5, "v2", "pixel value from channel 2") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(v0) fails");
      return false;
    }

    if( c_math_parser_add_argument(math_parser_, 6, "v3", "pixel value from channel 3") < 0 ) {
      CF_ERROR("c_math_parser_add_argument(v0) fails");
      return false;
    }

    expression_changed_ = true;
  }

  if( expression_changed_ ) {

    const char * errmsg =
        c_math_parser_parse(math_parser_,
            expression_.c_str());

    if ( errmsg ) {
      CF_ERROR("c_math_parser_parse(): fails: %s", errmsg);
      return false;
    }

    expression_changed_ = false;
  }

  switch (input_channel_) {
    case IMAGE:
      switch (output_channel_) {
        case IMAGE: {
          process_image(image.getMatRef(), image.getMatRef(), math_parser_);
          break;
        }
        case MASK: {
          if( mask.size() != image.size() ) {
            mask.create(image.size(), CV_8U);
            mask.setTo(0);
          }
          process_image(image.getMatRef(), mask.getMatRef(), math_parser_);
          break;
        }
      }
      break;
    case MASK:
      switch (output_channel_) {
        case IMAGE:
          break;
        case MASK:
          break;
      }
      break;
  }


  return true;
}
