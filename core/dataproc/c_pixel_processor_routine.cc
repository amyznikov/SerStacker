/*
 * c_pixel_processor_routine.cc
 *
 *  Created on: Jan 26, 2024
 *      Author: amyznikov
 */

#include "c_pixel_processor_routine.h"
#include <core/ssprintf.h>
#include <core/debug.h>

// TODO: is c_math_parser thread-safe ?
#if HAVE_TBB
 #include <tbb/tbb.h>
 typedef tbb::blocked_range<int> tbb_range;
#endif

template<>
const c_enum_member* members_of<c_pixel_processor_routine::InputType>()
{
  static const c_enum_member members[] = {
      { c_pixel_processor_routine::InputType_Image, "Image", "Image" },
      { c_pixel_processor_routine::InputType_PointCloud, "PointCloud", "PointCloud" },
      { c_pixel_processor_routine::InputType_Image }
  };

  return members;
}


namespace  {


static const struct {
  const char * arg_name;
  const char * arg_desc;
} math_args [] = {
    {"x", " X coordinate (column index) of current pixel, 0 for not structured point clouds"},
    {"y", " Y coordinate ((row index) of current pixel, 0 for not structured point clouds"},
    {"X", " X coordinate current pixel in point cloud, 0 for 2D images"},
    {"Y", " Y coordinate current pixel in point cloud, 0 for 2D images"},
    {"Z", " Z coordinate current pixel in point cloud, 0 for 2D images"},
    {"c", " Channel index of current pixel"},
    {"cn", "Number of color channels"},
    {"v", " Value of current pixel in currently processing channel"},
    {"v0", " Value of current pixel in channel 0"},
    {"v1", " Value of current pixel in channel 1"},
    {"v2", " Value of current pixel in channel 2"},
    {"v3", " Value of current pixel in channel 3"},
};

static constexpr int arg_x_index = 0;
static constexpr int arg_y_index = 1;
static constexpr int arg_X_index = 2;
static constexpr int arg_Y_index = 3;
static constexpr int arg_Z_index = 4;
static constexpr int arg_c_index = 5;
static constexpr int arg_cn_index = 6;
static constexpr int arg_v_index = 7;
static constexpr int arg_v0_index = 8;
//static constexpr int arg_v1_index = 9;
//static constexpr int arg_v2_index = 10;
//static constexpr int arg_v3_index = 11;

static constexpr int num_args =
    sizeof(math_args) / sizeof(math_args[0]);

static void setup_math_parser(c_math_expression & math)
{
  static bool initialized = false;

  if( !initialized ) {

    math.clear_args();

    for( int i = 0; i < num_args; ++i ) {
      math.add_argument(i, math_args[i].arg_name, math_args[i].arg_desc);
    }

    initialized = true;
  }
}

template<class T1, class T2>
static bool process_image_(const c_math_expression & math, cv::InputArray _src, cv::OutputArray _dst)
{
  const cv::Size src_size =
      _src.size();

  const int width =
      src_size.width;

  const int height =
      src_size.height;

  const int cn =
      _src.channels();

  const cv::Mat_<T1> src =
      _src.getMat();

  cv::Mat_<T2> dst =
      _dst.getMatRef();

  double args[num_args] = { 0 };

  args[arg_cn_index] = cn;

  for( int y = 0; y < height; ++y ) {

    const T1 * srcp = src[y];
    T2 * dstp = dst[y];

    args[arg_y_index] = y;

    for( int x = 0; x < width; ++x ) {

      args[arg_x_index] = x;

      for( int c = 0; c < cn; ++c ) {
        args[arg_v0_index + c] =
            srcp[x * cn + c];
      }

      for( int c = 0; c < cn; ++c ) {

        args[arg_c_index] = c;
        args[arg_v_index] = srcp[x * cn + c];

        dstp[x * cn + c] =
            cv::saturate_cast<T2>(math.eval(args));
      }
    }
  }

  return true;
}

static bool process_image(const c_math_expression & math, cv::InputArray _src, cv::OutputArray _dst, int ddepth)
{
  if( ddepth < 0 || ddepth > CV_64F ) {
    ddepth = _src.depth();
  }

  _dst.create(_src.size(),
      CV_MAKETYPE(ddepth,
          _src.channels()));

  switch (_src.depth()) {
    case CV_8U:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<uint8_t, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<uint8_t, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<uint8_t, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<uint8_t, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<uint8_t, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<uint8_t, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<uint8_t, double>(math, _src, _dst);
      }
      break;

    case CV_8S:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<int8_t, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<int8_t, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<int8_t, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<int8_t, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<int8_t, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<int8_t, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<int8_t, double>(math, _src, _dst);
      }
      break;
    case CV_16U:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<uint16_t, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<uint16_t, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<uint16_t, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<uint16_t, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<uint16_t, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<uint16_t, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<uint16_t, double>(math, _src, _dst);
      }
      break;
    case CV_16S:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<int16_t, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<int16_t, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<int16_t, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<int16_t, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<int16_t, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<int16_t, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<int16_t, double>(math, _src, _dst);
      }
      break;
    case CV_32S:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<int32_t, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<int32_t, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<int32_t, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<int32_t, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<int32_t, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<int32_t, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<int32_t, double>(math, _src, _dst);
      }
      break;
    case CV_32F:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<float, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<float, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<float, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<float, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<float, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<float, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<float, double>(math, _src, _dst);
      }
      break;
    case CV_64F:
      switch (_dst.depth()) {
        case CV_8U:
          return process_image_<double, uint8_t>(math, _src, _dst);
        case CV_8S:
          return process_image_<double, int8_t>(math, _src, _dst);
        case CV_16U:
          return process_image_<double, uint16_t>(math, _src, _dst);
        case CV_16S:
          return process_image_<double, int16_t>(math, _src, _dst);
        case CV_32S:
          return process_image_<double, int32_t>(math, _src, _dst);
        case CV_32F:
          return process_image_<double, float>(math, _src, _dst);
        case CV_64F:
          return process_image_<double, double>(math, _src, _dst);
      }
      break;
  }

  return false;
}


template<class T1, class T2>
static bool process_point_cloud_(const c_math_expression & math,
    cv::InputArray input_points, cv::InputArray input_colors,
    cv::InputArray input_mask,
    cv::OutputArray output_colors)
{
  const cv::Size src_size =
      input_colors.size();

  const int width =
      src_size.width;

  const int height =
      src_size.height;

  const int cn =
      input_points.channels();

  const cv::Mat3f points =
      input_points.getMat();

  const cv::Mat_<T1> src =
      input_colors.getMat();

  cv::Mat_<T2> dst =
      output_colors.getMatRef();

  double args[num_args] = { 0 };

  args[arg_cn_index] = cn;

  if( input_mask.size() == src_size && input_mask.type() == CV_8UC1 ) {

    const cv::Mat1b M =
        input_mask.getMat();

    for( int y = 0; y < height; ++y ) {

      const uint8_t * mskp = M[y];
      const T1 * srcp = src[y];
      T2 * dstp = dst[y];

      args[arg_y_index] = y;

      for( int x = 0; x < width; ++x ) {
        if( !mskp[x] ) {
          for( int c = 0; c < cn; ++c ) {
            dstp[x * cn + c] =
                cv::saturate_cast<T2>(srcp[x * cn + c]);
          }

        }
        else {
          args[arg_x_index] = x;
          args[arg_X_index] = points[y][x][0];
          args[arg_Y_index] = points[y][x][1];
          args[arg_Z_index] = points[y][x][2];

          for( int c = 0; c < cn; ++c ) {
            args[arg_v0_index + c] =
                srcp[x * cn + c];
          }

          for( int c = 0; c < cn; ++c ) {

            args[arg_c_index] = c;
            args[arg_v_index] = srcp[x * cn + c];

            dstp[x * cn + c] =
                cv::saturate_cast<T2>(math.eval(args));
          }
        }
      }
    }
  }
  else if( input_mask.size() == src_size && input_mask.depth() == CV_8U && input_mask.channels() == cn ) {

    const cv::Mat_<uint8_t> M =
        input_mask.getMat();

    for( int y = 0; y < height; ++y ) {

      const uint8_t * mskp = M[y];
      const T1 * srcp = src[y];
      T2 * dstp = dst[y];

      args[arg_y_index] = y;

      for( int x = 0; x < width; ++x ) {

        args[arg_x_index] = x;
        args[arg_X_index] = points[y][x][0];
        args[arg_Y_index] = points[y][x][1];
        args[arg_Z_index] = points[y][x][2];

        for( int c = 0; c < cn; ++c ) {
          args[arg_v0_index + c] =
              srcp[x * cn + c];
        }

        for( int c = 0; c < cn; ++c ) {
          if( !mskp[x * cn + c] ) {
            dstp[x * cn + c] =
                cv::saturate_cast<T2>(srcp[x * cn + c]);
          }
          else {

            args[arg_c_index] = c;
            args[arg_v_index] = srcp[x * cn + c];

            dstp[x * cn + c] =
                cv::saturate_cast<T2>(math.eval(args));
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < height; ++y ) {

      const T1 * srcp = src[y];
      T2 * dstp = dst[y];

      args[arg_y_index] = y;

      for( int x = 0; x < width; ++x ) {

        args[arg_x_index] = x;
        args[arg_X_index] = points[y][x][0];
        args[arg_Y_index] = points[y][x][1];
        args[arg_Z_index] = points[y][x][2];

        for( int c = 0; c < cn; ++c ) {
          args[arg_v0_index + c] =
              srcp[x * cn + c];
        }

        for( int c = 0; c < cn; ++c ) {

          args[arg_c_index] = c;
          args[arg_v_index] = srcp[x * cn + c];

          dstp[x * cn + c] =
              cv::saturate_cast<T2>(math.eval(args));
        }
      }
    }
  }

  return true;
}

static bool process_point_cloud(const c_math_expression & math,
    cv::InputArray input_points, cv::InputArray input_colors,
    cv::InputArray input_mask,
    cv::OutputArray output_colors,
    int ddepth)
{

  if( ddepth < 0 || ddepth > CV_64F ) {
    ddepth = input_colors.depth();
  }

  output_colors.create(input_colors.size(),
      CV_MAKETYPE(ddepth,
          input_colors.channels()));

  switch (input_colors.depth()) {
    case CV_8U:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<uint8_t, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<uint8_t, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<uint8_t, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<uint8_t, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<uint8_t, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<uint8_t, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<uint8_t, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;

    case CV_8S:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<int8_t, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<int8_t, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<int8_t, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<int8_t, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<int8_t, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<int8_t, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<int8_t, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
    case CV_16U:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<uint16_t, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<uint16_t, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<uint16_t, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<uint16_t, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<uint16_t, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<uint16_t, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<uint16_t, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
    case CV_16S:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<int16_t, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<int16_t, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<int16_t, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<int16_t, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<int16_t, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<int16_t, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<int16_t, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
    case CV_32S:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<int32_t, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<int32_t, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<int32_t, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<int32_t, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<int32_t, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<int32_t, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<int32_t, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
    case CV_32F:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<float, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<float, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<float, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<float, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<float, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<float, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<float, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
    case CV_64F:
      switch (output_colors.depth()) {
        case CV_8U:
          return process_point_cloud_<double, uint8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_8S:
          return process_point_cloud_<double, int8_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16U:
          return process_point_cloud_<double, uint16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_16S:
          return process_point_cloud_<double, int16_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32S:
          return process_point_cloud_<double, int32_t>(math, input_points, input_colors, input_mask, output_colors);
        case CV_32F:
          return process_point_cloud_<double, float>(math, input_points, input_colors, input_mask, output_colors);
        case CV_64F:
          return process_point_cloud_<double, double>(math, input_points, input_colors, input_mask, output_colors);
      }
      break;
  }

  return false;
}

}  // namespace



std::string c_pixel_processor_routine::helpstring() const
{
  static std::string _helpstring;

  if( _helpstring.empty() ) {

    setup_math_parser(math_);

    _helpstring.append("Arguments:\n");
    for( const auto & arg : math_.arguments() ) {
      _helpstring.append(ssprintf("%s %s\n", arg.name.c_str(), arg.desc.c_str()));
    }

    _helpstring.append("\nConstants:\n");
    for( const auto & func : math_.constants() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nUnary operations:\n");
    for( const auto & func : math_.unary_operations() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nBinary operations:\n");
    for( int p = 0, np = math_.binary_operations().size(); p < np; ++p ) {
      for( const auto & func : math_.binary_operations()[p] ) {
        _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
      }
    }

    _helpstring.append("\nFunctions:\n");
    for( const auto & func : math_.functions() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

  }

  return _helpstring;
}

void c_pixel_processor_routine::get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls)
{
  ADD_DATA_PROCESSOR_CTRL(ctls, input_type, "input_type", "input view type");
  ADD_DATA_PROCESSOR_CTRL(ctls, input, "input", "input image name");
  ADD_DATA_PROCESSOR_CTRL(ctls, output, "output", "output image name");
  ADD_DATA_PROCESSOR_CTRL(ctls, output_depth, "output_depth", "output image depth");
  ADD_DATA_PROCESSOR_CTRL_MATH_EXPRESSION(ctls, expression, "", "formula for math expression", helpstring);
}

bool c_pixel_processor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, input_type);
    SERIALIZE_PROPERTY(settings, save, *this, input);
    SERIALIZE_PROPERTY(settings, save, *this, output);
    SERIALIZE_PROPERTY(settings, save, *this, output_depth);
    SERIALIZE_PROPERTY(settings, save, *this, expression);
    return true;
  }

  return false;
}

bool c_pixel_processor_routine::process(c_data_frame::sptr & dataframe)
{
  if( expression_.empty() ) {
    expression_changed_ = false;
    return true;
  }

  if ( !dataframe ) {
    CF_ERROR("c_pixel_processor_routine: No data frame provided");
    return false;
  }


  DataViewType v = (DataViewType)input_type_;

  cv::Mat input_image, input_data, input_mask;

  if ( !dataframe->get_data(&v, input_, input_image, input_data, input_mask) ) {
    CF_ERROR("dataframe->get_data('%s') fails", input_);
    return false;
  }

  if( expression_changed_ ) {

    setup_math_parser(math_);

    if( !math_.parse(expression_.c_str()) ) {

      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", math_.error_message().c_str(),
          math_.pointer_to_syntax_error() ? math_.pointer_to_syntax_error() : "null");

      return false;
    }

    expression_changed_ = false;
  }


  switch (v) {
    case DataViewType_Image: {
      cv::Mat output_image;

      if ( !process_image(math_, input_image, output_image, output_depth_) ) {
        CF_ERROR("process_image() fails");
        return false;
      }

      dataframe->set_data(v, output_,
          output_image,
          cv::noArray(),
          cv::noArray());

      break;
    }
    case DataViewType_PointCloud: {

      cv::Mat output_colors;

//      CF_DEBUG("\n"
//          "input_image: %dx%d channels=%d depth=%d\n"
//          "input_data: %dx%d channels=%d depth=%d\n"
//          "input_mask: %dx%d channels=%d depth=%d\n"
//          ,
//          input_image.rows, input_image.cols, input_image.channels(), input_image.depth(),
//          input_data.rows, input_data.cols, input_data.channels(), input_data.depth(),
//          input_mask.rows, input_mask.cols, input_mask.channels(), input_mask.depth());

      if ( !process_point_cloud(math_, input_image, input_data, input_mask, output_colors, output_depth_) ) {
        CF_ERROR("process_point_cloud() fails");
        return false;
      }

      dataframe->set_data(v, output_, input_image, output_colors, cv::noArray());

      break;
    }
    default:
      CF_ERROR("Unsupported view type %d", v);
      return false;
  }

  //dataframe->set_data(v, output_, image, data, mask);

  return true;
}
