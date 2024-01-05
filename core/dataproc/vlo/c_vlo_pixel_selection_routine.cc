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

static constexpr int num_args = 7;

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> process_data_(const ScanType & scan, const c_math_expression & math, cv::Mat3b & output_selection)
{
  get_vlo_points2d(scan, cv::noArray(),
      [&](int l, int s, int e, const auto & echo) {

        double args[num_args];

        args[arg_l_index] = l;
        args[arg_s_index] = s;
        args[arg_e_index] = e;
        args[arg_d_index] = echo.dist;
        args[arg_a_index] = echo.area;
        args[arg_p_index] = echo.peak;
        args[arg_w_index] = echo.width;

        if ( math.eval(args) ) {
          output_selection[l][s][e] = 255;
        }

      });


  return true;
}

bool process_data_(const c_vlo_scan6_slm & scan, const c_math_expression & math, cv::Mat3b & output_selection)
{
  get_vlo_points2d(scan, cv::noArray(),
      [&](int l, int s, int e, const auto & echo) {

        double args[num_args];

        args[arg_l_index] = l;
        args[arg_s_index] = s;
        args[arg_e_index] = e;
        args[arg_d_index] = echo.dist;
        args[arg_a_index] = echo.area;

        if ( math.eval(args) ) {
          output_selection[l][s][e] = 255;
        }

      });

  return true;
}

static bool process_data(c_vlo_scan & scan, const c_math_expression & math, cv::Mat3b & output_selection)
{
  output_selection.create(vlo_scan_size(scan));
  output_selection.setTo(cv::Scalar::all(0));

  switch (scan.version)
  {
    case VLO_VERSION_1:
      return process_data_(scan.scan1, math, output_selection);
    case VLO_VERSION_3:
      return process_data_(scan.scan3, math, output_selection);
    case VLO_VERSION_5:
      return process_data_(scan.scan5, math, output_selection);
    case VLO_VERSION_6_SLM:
      return process_data_(scan.scan6_slm, math, output_selection);
    default:
      break;
  }

  CF_ERROR("Unsupported VLO scan version encountered: %d", scan.version);
  return false;
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

bool c_vlo_pixel_selection_routine::process(c_vlo_frame * vlo)
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

    if( (previous_vlo_scan_version_ = vlo->current_scan_.version) != VLO_VERSION_6_SLM ) {
      math_.add_argument(arg_p_index, arg_p_name, arg_p_desc);
      math_.add_argument(arg_w_index, arg_w_name, arg_w_desc);
    }

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

