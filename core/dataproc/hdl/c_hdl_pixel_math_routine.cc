/*
 * c_hdl_pixel_math_routine.cc
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#include "c_hdl_pixel_math_routine.h"
#include <core/debug.h>


static const struct {
  const char * name;
  const char * desc;
} math_args[] = {
    // Single lidar point as received from packet.
    {"lid", "laser_id"},
    {"lring", "laser_ring"},
    {"pkt", ""},
    {"datablock", ""},
    {"flags", ""},
    {"azimuth", ""},
    {"elevation", ""},
    {"distance", ""},
    {"intensity", ""},
    {"timestamp", ""},

    // c_hdl_specification
    {"sensor", ""},
    {"distance_resolution", ""},

    // c_hdl_lasers_table
    {"num_lasers", ""},
};

static const int lid_arg_index = 0;
static const int lring_arg_index = 1;
static const int pkt_arg_index = 2;
static const int datablock_arg_index = 3;
static const int flags_arg_index = 4;
static const int azimuth_arg_index = 5;
static const int elevation_arg_index = 6;
static const int distance_arg_index = 7;
static const int intensity_arg_index = 8;
static const int timestamp_arg_index = 9;
static const int sensor_arg_index = 10;
static const int distance_resolution_arg_index = 11;
static const int num_lasers_arg_index = 12;
static constexpr int num_args = sizeof(math_args)/sizeof(math_args[0]);
static_assert(num_args == 13, "APP BUG: Array size if invalid");


static void ensure_math_initialized(c_math_expression & m)
{
  if ( m.arguments().empty() ) {

    for ( int i = 0; i < num_args; ++i ) {
      const auto & arg = math_args[i];
      m.add_argument(i, arg.name, arg.desc);
    }

  }
}

void c_hdl_pixel_math_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Output:", ctx(&this_class::_output_name), "Output channel name");
  ctlbind_multiline_textbox(ctls, "", ctx, &this_class::expression, &this_class::set_expression);

}
bool c_hdl_pixel_math_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _output_name);
    SERIALIZE_OPTION(settings, save, *this, _expression);
    return true;
  }
  return false;
}

bool c_hdl_pixel_math_routine::process(c_hdl_data_frame * hdl)
{
  if ( !_expression.empty() ) {

    ensure_math_initialized(_math);

    if ( _math.empty() && !_math.parse(_expression) ) {
      CF_ERROR("_math.parse() fails. Syntax error at: '%s'", _math.pointer_to_syntax_error());
      _math.clear();
      return false;
    }

    const c_hdl_specification & lidar = hdl->current_lidar();
    const c_hdl_range_image & range_image = hdl->range_image();

    cv::Mat1f outputImage;
    cv::Mat1b outputMask;
    range_image.create(outputImage, &outputMask);

    double args[num_args] = {0};
    args[sensor_arg_index] = lidar.sensor;
    args[distance_resolution_arg_index] = lidar.distance_resolution;
    args[num_lasers_arg_index] = lidar.lasers.size();

    for( const auto & p : hdl->current_frame()->points ) {

      int r, c;
      if( range_image.project(p, &r, &c) ) {
        args[lid_arg_index] = p.laser_id;
        args[lring_arg_index] = p.laser_ring;
        args[pkt_arg_index] = p.pkt;
        args[datablock_arg_index] = p.datablock;
        args[flags_arg_index] = p.flags;
        args[azimuth_arg_index] = p.azimuth;
        args[elevation_arg_index] = p.elevation;
        args[distance_arg_index] = p.distance;
        args[intensity_arg_index] = p.intensity;
        args[timestamp_arg_index] = p.timestamp;

        outputImage(r, c) = _math.eval(args);
        outputMask(r, c) = 255;
      }
    }

    cv::rotate(outputImage, outputImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(outputMask, outputMask, cv::ROTATE_90_COUNTERCLOCKWISE);

    const std::string output_name = _output_name.empty() ? "HDL_MATH" : _output_name;
    hdl->add_image(output_name, outputImage, outputMask);
  }

  return true;
}
