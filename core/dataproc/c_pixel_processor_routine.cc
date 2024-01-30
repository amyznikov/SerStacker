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

static bool setup_math_expression(c_math_expression & math, const std::string & expression, DataViewType v)
{
  math.clear_args();

//  switch (v) {
//    case DataViewType_Image:
//      math.add_argument(arg_image_x_index, arg_image_x_name, arg_image_x_desc);
//      math.add_argument(arg_image_y_index, arg_image_y_name, arg_image_y_desc);
//      break;
//    case DataViewType_PointCloud:
//      math.add_argument(arg_pc_x_index, arg_pc_x_name, arg_pc_x_desc);
//      math.add_argument(arg_pc_y_index, arg_pc_y_name, arg_pc_y_desc);
//      math.add_argument(arg_pc_z_index, arg_pc_y_name, arg_pc_y_desc);
//      break;
//    default:
//      CF_ERROR("Not supported DataViewType=%d requested", v);
//      return false;
//  }

//  math.add_argument(arg_cn_index, arg_cn_name, arg_cn_desc);
//  math.add_argument(arg_c_index, arg_c_name, arg_c_desc);

  if ( !math.parse(expression.c_str()) ) {

    CF_ERROR("math_.parse() fails: %s\n"
        "error_pos=%s", math.error_message().c_str(),
        math.pointer_to_syntax_error() ? math.pointer_to_syntax_error() : "null");

    return false;
  }

  return true;
}

static bool process_image(const c_math_expression & math,  cv::InputArray image)
{
  return false;
}

static bool process_point_cloud(const c_math_expression & math,  cv::InputArray points,  cv::InputArray colors)
{
  return false;
}

}  // namespace

void c_pixel_processor_routine::get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls)
{
  ADD_DATA_PROCESSOR_CTRL(ctls, input_type, "input_type", "input view type");
  ADD_DATA_PROCESSOR_CTRL(ctls, input, "input", "input image name");
  ADD_DATA_PROCESSOR_CTRL(ctls, output, "output", "output image name");
}

bool c_pixel_processor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, input_type);
    SERIALIZE_PROPERTY(settings, save, *this, input);
    SERIALIZE_PROPERTY(settings, save, *this, output);
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

  cv::Mat image, data, mask;

  if ( !dataframe->get_data(&v, input_, image, data, mask) ) {
    CF_ERROR("dataframe->get_data('%s') fails", input_);
    return false;
  }

  if( expression_changed_ ) {
    if( !setup_math_expression(math_, expression_, v) ) {
      CF_ERROR("setup_math_expression() fails");
      return false;
    }
    expression_changed_ = false;
  }


  switch (v) {
    case DataViewType_Image: {

      CF_DEBUG("image: %dx%d channels=%d depth=%d\n",
          image.cols, image.rows, image.channels(), image.depth());

      if ( !process_image(math_, image) ) {
        CF_ERROR("process_image() fails");
        return false;
      }

      break;
    }
    case DataViewType_PointCloud: {

      CF_DEBUG("image: %dx%d channels=%d depth=%d\n"
          "data: %dx%d channels=%d depth=%d\n",
          image.cols, image.rows, image.channels(), image.depth(),
          data.cols, data.rows, data.channels(), data.depth());

      if ( !process_point_cloud(math_, image, data) ) {
        CF_ERROR("process_point_cloud() fails");
        return false;
      }

      break;
    }
    default:
      CF_ERROR("Unsupported view type %d", v);
      return false;
  }

  dataframe->set_data(v, output_, image, data, mask);

  return true;
}
