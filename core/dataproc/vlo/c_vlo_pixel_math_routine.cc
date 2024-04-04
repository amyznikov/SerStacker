/*
 * c_vlo_pixel_math_routine.cc
 *
 *  Created on: Feb 10, 2024
 *      Author: amyznikov
 */

#include "c_vlo_pixel_math_routine.h"


namespace {

static constexpr int arg_l_index = 0;
static constexpr const char * arg_l_name = "l";
static constexpr const char * arg_l_desc = "layer index";

static constexpr int arg_s_index = 1;
static constexpr const char * arg_s_name = "s";
static constexpr const char * arg_s_desc = "slot index";

static constexpr int arg_e_index = 2;
static constexpr const char * arg_e_name = "e";
static constexpr const char * arg_e_desc = "echo index";

static constexpr int arg_d_index = 3;
static constexpr const char * arg_d_name = "d";
static constexpr const char * arg_d_desc = "echo distance";

static constexpr int arg_a_index = 4;
static constexpr const char * arg_a_name = "a";
static constexpr const char * arg_a_desc = "echo area";

static constexpr int arg_p_index = 5;
static constexpr const char * arg_p_name = "p";
static constexpr const char * arg_p_desc = "echo peak";

static constexpr int arg_w_index = 6;
static constexpr const char * arg_w_name = "w";
static constexpr const char * arg_w_desc = "echo width";

static constexpr int arg_x_index = 7;
static constexpr const char * arg_x_name = "x";
static constexpr const char * arg_x_desc = "X coordinate of 3D point in [cm]";

static constexpr int arg_y_index = 8;
static constexpr const char * arg_y_name = "y";
static constexpr const char * arg_y_desc = "Y coordinate of 3D point in [cm]";

static constexpr int arg_z_index = 9;
static constexpr const char * arg_z_name = "z";
static constexpr const char * arg_z_desc = "Z coordinate of 3D point in [cm]";

static constexpr int num_args = 10;

static void init_math_parser(c_math_expression & m)
{
  static bool initialized = false;
  if( !initialized ) {

    m.clear_args();

    m.add_argument(arg_l_index, arg_l_name, arg_l_desc);
    m.add_argument(arg_s_index, arg_s_name, arg_s_desc);
    m.add_argument(arg_e_index, arg_e_name, arg_e_desc);
    m.add_argument(arg_d_index, arg_d_name, arg_d_desc);
    m.add_argument(arg_a_index, arg_a_name, arg_a_desc);
    m.add_argument(arg_p_index, arg_p_name, arg_p_desc);
    m.add_argument(arg_w_index, arg_w_name, arg_w_desc);
    m.add_argument(arg_x_index, arg_x_name, arg_x_desc);
    m.add_argument(arg_y_index, arg_y_name, arg_y_desc);
    m.add_argument(arg_z_index, arg_z_name, arg_z_desc);

    initialized = true;
  }
}

static bool process_data(const c_vlo_scan & scan, const c_math_expression & math, cv::Mat3f & output_image)
{
  output_image.create(scan.size);
  output_image.setTo(cv::Scalar::all(0));

  for( int l = 0; l < scan.size.height; ++l ) {
    for( int s = 0; s < scan.size.width; ++s ) {
      for( int e = 0; e < 3; ++e ) {

        double args[num_args];

        args[arg_l_index] = l;
        args[arg_s_index] = s;
        args[arg_e_index] = e;
        args[arg_d_index] = scan.distances.empty() ? 0 : scan.distances[l][s][e];
        args[arg_a_index] = scan.area.empty() ? 0 : scan.area[l][s][e];
        args[arg_p_index] = scan.peak.empty() ? 0 : scan.peak[l][s][e];
        args[arg_w_index] = scan.width.empty() ? 0 : scan.width[l][s][e];
        args[arg_x_index] = scan.clouds[e][l][s][0];
        args[arg_y_index] = scan.clouds[e][l][s][1];
        args[arg_z_index] = scan.clouds[e][l][s][2];

        output_image[l][s][e] = math.eval(args);
      }

    }
  }

  return true;
}

} // namespace



std::string c_vlo_pixel_math_routine::helpstring() const
{
  static std::string _helpstring;

  if ( _helpstring.empty() ) {

    init_math_parser(math_);

    _helpstring.append("Arguments:\n");
    _helpstring.append(ssprintf("%s : %s\n", arg_l_name, arg_l_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_s_name, arg_s_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_e_name, arg_e_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_d_name, arg_d_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_a_name, arg_a_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_p_name, arg_p_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_w_name, arg_w_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_x_name, arg_x_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_y_name, arg_y_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_z_name, arg_z_desc));

    _helpstring.append("\nConstants:\n");
    for ( const auto & func: math_.constants() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nUnary operations:\n");
    for ( const auto & func: math_.unary_operations() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

    _helpstring.append("\nBinary operations:\n");
    for ( int p = 0, np = math_.binary_operations().size(); p < np; ++p ) {
      for ( const auto & func: math_.binary_operations()[p] ) {
        _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
      }
    }

    _helpstring.append("\nFunctions:\n");
    for ( const auto & func: math_.functions() ) {
      _helpstring.append(ssprintf("%s %s\n", func.name.c_str(), func.desc.c_str()));
    }

  }

  return _helpstring;
}

void c_vlo_pixel_math_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_MATH_EXPRESSION_CTRL(ctls, expression, helpstring, "", "formula for math expression");
  BIND_CTRL(ctls, output_name, "Output To:", "Output channel name");
}

bool c_vlo_pixel_math_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, output_name);
    SERIALIZE_PROPERTY(settings, save, *this, expression);
    return true;
  }

  return false;
}

bool c_vlo_pixel_math_routine::process(c_vlo_data_frame * vlo)
{
  if( expression_.empty() ) {
    expression_changed_ = false;
    return true;
  }

  if( expression_changed_ ) {

    init_math_parser(math_);

    if ( !math_.parse(expression_.c_str()) ) {

      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", math_.error_message().c_str(),
          math_.pointer_to_syntax_error() ? math_.pointer_to_syntax_error() : "null");

      return false;
    }

    expression_changed_ = false;
  }

  if ( !process_data(vlo->current_scan_, math_, output_image_) ) {
    CF_ERROR("c_vlo_pixel_math_routine: process_data() fails");
    return false;
  }

  vlo->set_data(DataViewType_Image,
      output_name_.empty() ? "PROCESSED_IMAGE" : output_name_,
      output_image_,
      cv::noArray(),
      cv::noArray());

  return true;
}
