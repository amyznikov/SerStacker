/*
 * c_vlo_pixel_selection_routine.cc
 *
 *  Created on: Jan 3, 2024
 *      Author: amyznikov
 */

#include "c_vlo_pixel_selection_routine.h"
#include <core/ssprintf.h>

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

//
static constexpr int arg_d0_index = 10;
static constexpr const char * arg_d0_name = "d0";
static constexpr const char * arg_d0_desc = "echo0 distance";

static constexpr int arg_d1_index = 11;
static constexpr const char * arg_d1_name = "d1";
static constexpr const char * arg_d1_desc = "echo1 distance";

static constexpr int arg_d2_index = 12;
static constexpr const char * arg_d2_name = "d2";
static constexpr const char * arg_d2_desc = "echo2 distance";

//
static constexpr int arg_p0_index = 13;
static constexpr const char * arg_p0_name = "p0";
static constexpr const char * arg_p0_desc = "echo0 peak";

static constexpr int arg_p1_index = 14;
static constexpr const char * arg_p1_name = "p1";
static constexpr const char * arg_p1_desc = "echo1 peak";

static constexpr int arg_p2_index = 15;
static constexpr const char * arg_p2_name = "p2";
static constexpr const char * arg_p2_desc = "echo2 peak";

//
static constexpr int arg_a0_index = 16;
static constexpr const char * arg_a0_name = "a0";
static constexpr const char * arg_a0_desc = "echo0 area";

static constexpr int arg_a1_index = 17;
static constexpr const char * arg_a1_name = "p1";
static constexpr const char * arg_a1_desc = "echo1 area";

static constexpr int arg_a2_index = 18;
static constexpr const char * arg_a2_name = "p2";
static constexpr const char * arg_a2_desc = "echo2 area";

//
static constexpr int arg_w0_index = 19;
static constexpr const char * arg_w0_name = "w0";
static constexpr const char * arg_w0_desc = "echo0 width";

static constexpr int arg_w1_index = 20;
static constexpr const char * arg_w1_name = "w1";
static constexpr const char * arg_w1_desc = "echo1 width";

static constexpr int arg_w2_index = 21;
static constexpr const char * arg_w2_name = "w2";
static constexpr const char * arg_w2_desc = "echo2 width";


static constexpr int num_args = 22;

static bool process_data(c_vlo_scan & scan, const c_math_expression & math, cv::Mat3b & output_selection)
{
  output_selection.create(scan.size);
  output_selection.setTo(cv::Scalar::all(0));

  double args[num_args] = { 0 };

  for ( int l = 0; l < scan.size.height; ++l ) {
    for ( int s = 0; s < scan.size.width; ++s ) {
      for ( int e = 0; e < 3; ++e ) {

        args[arg_l_index] = l;
        args[arg_s_index] = s;
        args[arg_e_index] = e;
        args[arg_d_index] = scan.distances[l][s][e];
        args[arg_a_index] = scan.area[l][s][e];
        args[arg_w_index] = scan.width[l][s][e];
        args[arg_x_index] = scan.clouds[e][l][s][0];
        args[arg_y_index] = scan.clouds[e][l][s][1];
        args[arg_z_index] = scan.clouds[e][l][s][2];

        args[arg_d0_index] = scan.distances[l][s][0];
        args[arg_d1_index] = scan.distances[l][s][1];
        args[arg_d2_index] = scan.distances[l][s][2];

        if ( !scan.peak.empty() ) {
          args[arg_p_index] =  scan.peak[l][s][e];
          args[arg_p0_index] = scan.peak[l][s][0];
          args[arg_p1_index] = scan.peak[l][s][1];
          args[arg_p2_index] = scan.peak[l][s][2];
        }

        args[arg_a0_index] = scan.area[l][s][0];
        args[arg_a1_index] = scan.area[l][s][1];
        args[arg_a2_index] = scan.area[l][s][2];

        args[arg_w0_index] = scan.width[l][s][0];
        args[arg_w1_index] = scan.width[l][s][1];
        args[arg_w2_index] = scan.width[l][s][2];

        if( math.eval(args) ) {
          output_selection[l][s][e] = 255;
        }

      }

    }
  }

  return true;
}

} // namespace


std::string c_vlo_pixel_selection_routine::helpstring() const
{
  static std::string _helpstring;

  if ( _helpstring.empty() ) {

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
    _helpstring.append(ssprintf("%s : %s\n", arg_d0_name, arg_d0_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_d1_name, arg_d1_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_d2_name, arg_d2_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_p0_name, arg_p0_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_p1_name, arg_p1_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_p2_name, arg_p2_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_a0_name, arg_a0_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_a1_name, arg_a1_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_a2_name, arg_a2_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_w0_name, arg_w0_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_w1_name, arg_w1_desc));
    _helpstring.append(ssprintf("%s : %s\n", arg_w2_name, arg_w2_desc));


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

void c_vlo_pixel_selection_routine::get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls)
{
  ADD_DATA_PROCESSOR_CTRL_MATH_EXPRESSION(ctls, expression, "", "formula for math expression", helpstring);
  ADD_DATA_PROCESSOR_CTRL(ctls, invert_selection, "invert_selection", "invert_selection");
  ADD_DATA_PROCESSOR_CTRL(ctls, mask_mode, "mask_mode", "combine selection mode");
}

bool c_vlo_pixel_selection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, expression);
    SERIALIZE_PROPERTY(settings, save, *this, invert_selection);
    SERIALIZE_PROPERTY(settings, save, *this, mask_mode);
    return true;
  }

  return false;
}

bool c_vlo_pixel_selection_routine::process(c_vlo_data_frame * vlo)
{
  if( expression_.empty() ) {
    expression_changed_ = false;
    return true;
  }

  if( expression_changed_ || vlo->current_scan_.version != previous_vlo_scan_version_ ) {

    math_.clear_args();

    math_.add_argument(arg_l_index, arg_l_name, arg_l_desc);
    math_.add_argument(arg_s_index, arg_s_name, arg_s_desc);
    math_.add_argument(arg_e_index, arg_e_name, arg_e_desc);
    math_.add_argument(arg_d_index, arg_d_name, arg_d_desc);
    math_.add_argument(arg_a_index, arg_a_name, arg_a_desc);
    math_.add_argument(arg_p_index, arg_p_name, arg_p_desc);
    math_.add_argument(arg_w_index, arg_w_name, arg_w_desc);
    math_.add_argument(arg_x_index, arg_x_name, arg_x_desc);
    math_.add_argument(arg_y_index, arg_y_name, arg_y_desc);
    math_.add_argument(arg_z_index, arg_z_name, arg_z_desc);
    math_.add_argument(arg_d0_index, arg_d0_name, arg_d0_desc);
    math_.add_argument(arg_d1_index, arg_d1_name, arg_d1_desc);
    math_.add_argument(arg_d2_index, arg_d2_name, arg_d2_desc);
    math_.add_argument(arg_p0_index, arg_p0_name, arg_p0_desc);
    math_.add_argument(arg_p1_index, arg_p1_name, arg_p1_desc);
    math_.add_argument(arg_p2_index, arg_p2_name, arg_p2_desc);
    math_.add_argument(arg_a0_index, arg_a0_name, arg_a0_desc);
    math_.add_argument(arg_a1_index, arg_a1_name, arg_a1_desc);
    math_.add_argument(arg_a2_index, arg_a2_name, arg_a2_desc);
    math_.add_argument(arg_w0_index, arg_w0_name, arg_w0_desc);
    math_.add_argument(arg_w1_index, arg_w1_name, arg_w1_desc);
    math_.add_argument(arg_w2_index, arg_w2_name, arg_w2_desc);

    if ( !math_.parse(expression_.c_str()) ) {

      CF_ERROR("math_.parse() fails: %s\n"
          "error_pos=%s", math_.error_message().c_str(),
          math_.pointer_to_syntax_error() ? math_.pointer_to_syntax_error() : "null");

      return false;
    }

    expression_changed_ = false;
  }

  if ( !process_data(vlo->current_scan_, math_, selection_mask) ) {
    CF_ERROR("c_vlo_pixel_selection_routine: process_data() fails");
    return false;
  }

  if ( invert_selection_ ) {
    cv::bitwise_not(selection_mask, selection_mask);
  }

  vlo->update_selection(selection_mask, mask_mode_);

  return true;
}

