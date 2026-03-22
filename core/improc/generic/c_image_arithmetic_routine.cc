/*
 * c_image_arithmetic_routine.cc
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#include "c_image_arithmetic_routine.h"

//static const struct {
//  const char * name;
//  const char * desc;
//} math_args[] = {
//    {"y", "y coordinate of a current image pixel"},
//    {"x", "x coordinate of a current image pixel"},
//    {"v0", "pixel value from current image channel 0"},
//    {"v1", "pixel value from current image channel 1"},
//    {"v2", "pixel value from current image channel 2"},
//    {"v3", "pixel value from current image channel 3"},
//    {"v",  "pixel value from cc source channel of current image"},
//
//    {"arg.y", "y coordinate of a current image pixel"},
//    {"arg.x", "x coordinate of a current image pixel"},
//    {"arg.v0", "pixel value from current image channel 0"},
//    {"arg.v1", "pixel value from current image channel 1"},
//    {"arg.v2", "pixel value from current image channel 2"},
//    {"arg.v3", "pixel value from current image channel 3"},
//    {"arg.v",  "pixel value from cc source channel of current image"},
//
//    {"cc", "current destination channel index"},
//};
//

//static const int y_arg_index = 0;
//static const int x_arg_index = 1;
//static const int v0_arg_index = 2;
//static const int v1_arg_index = 3;
//static const int v2_arg_index = 4;
//static const int v3_arg_index = 5;
//static const int v_arg_index = 6;
//
//static const int arg_y_arg_index = 7;
//static const int arg_x_arg_index = 8;
//static const int arg_v0_arg_index = 9;
//static const int arg_v1_arg_index = 10;
//static const int arg_v2_arg_index = 11;
//static const int arg_v3_arg_index = 12;
//static const int arg_v_arg_index = 13;
//
//static const int cc_arg_index = 14;
//
//static constexpr int num_args = sizeof(math_args)/sizeof(math_args[0]);
//static_assert(num_args == 15, "APP BUG: Array size if invalid");
//
//
//static void ensure_math_initialized(c_math_expression & m)
//{
//  if ( m.arguments().empty() ) {
//
//    for ( int i = 0; i < num_args; ++i ) {
//      const auto & arg = math_args[i];
//      m.add_argument(i, arg.name, arg.desc);
//    }
//
//  }
//}


void c_image_arithmetic_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Arg image:", ctx(&this_class::_argname), "Argument image name");
  ctlbind(ctls, "Out image:", ctx(&this_class::_outname), "Output image name");
  ctlbind_multiline_textbox(ctls, "", ctx, &this_class::expression, &this_class::set_expression);
}

bool c_image_arithmetic_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _argname);
    SERIALIZE_OPTION(settings, save, *this, _outname);
    SERIALIZE_OPTION(settings, save, *this, _expression);
    return true;
  }
  return false;
}

bool c_image_arithmetic_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const auto & glb = globals();
  CF_DEBUG("glb.size=%zu", glb.size());

  for ( auto ii = glb.begin(); ii != glb.end(); ++ii ) {
    CF_DEBUG("glb: '%s'", ii->first.c_str());
  }


  //get_global();

//  cv::Mat src1, src2, dst;
//  image.getMat().convertTo(src1, CV_32F);
//
//  if ( !_expression.empty() ) {
//
//    ensure_math_initialized(_math);
//
//    if ( _math.empty() && !_math.parse(_expression) ) {
//      CF_ERROR("_math.parse() fails. Syntax error at: '%s'", _math.pointer_to_syntax_error());
//      _math.clear();
//      return false;
//    }
//  }

  return true;
}

