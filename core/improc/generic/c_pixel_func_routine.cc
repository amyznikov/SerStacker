/*
 * c_pixel_func_routine.cc
 *
 *  Created on: May 4, 2023
 *      Author: amyznikov
 */

#include "c_pixel_func_routine.h"
#include <tbb/tbb.h>
#include <core/ssprintf.h>


template<>
const c_enum_member * members_of<c_pixel_func_routine::Function>()
{
  static const c_enum_member members[] = {
      { c_pixel_func_routine::Function_None, "None", " No function, return just pixel value "},
      { c_pixel_func_routine::Function_sqrt, "sqrt", ""},
      { c_pixel_func_routine::Function_sqr, "sqr", ""},
      { c_pixel_func_routine::Function_abs, "abs", ""},
      { c_pixel_func_routine::Function_log, "log", ""},
      { c_pixel_func_routine::Function_exp, "exp", ""},
      { c_pixel_func_routine::Function_inv, "inv", " Inverse (reciprocal) function: V' = 1/V "},

      { c_pixel_func_routine::Function_sin, "sin", ""},
      { c_pixel_func_routine::Function_cos, "cos", ""},
      { c_pixel_func_routine::Function_asin, "asin", ""},
      { c_pixel_func_routine::Function_acos, "acos", ""},

      { c_pixel_func_routine::Function_asinh, "asinh", ""},
      { c_pixel_func_routine::Function_acosh, "acosh", ""},

      { c_pixel_func_routine::Function_pow, "pow",
          "Power function:\n"
              "  V' =  pow(V, p0)" },

      { c_pixel_func_routine::Function_poly, "poly",
          "Polynomial function:\n"
              " p0 + p1 * x + p2 * x^2 + .. pn * x^n" },

//      { c_pixel_func_routine::Function_, "", ""},
//      { c_pixel_func_routine::Function_, "", ""},
//      { c_pixel_func_routine::Function_, "", ""},


      {c_pixel_func_routine::Function_None}
  };

  return members;
}

static inline double sqr( double x)
{
  return x * x;
}

template<class T, class Op>
static void forEach_(cv::Mat & image, const Op & op)
{
  typedef tbb::blocked_range<int> range;

  cv::Mat_<T> img = image;

  const int rows = img.rows;

  tbb::parallel_for(range(0, img.rows, 128),
      [&img, op](const range & r) {

        const int cols = img.cols;
        const int cn = img.channels();

        for( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          T *imgp = img[y];

          for( int x = 0; x < cols; ++x ) {
            for( int c = 0; c < cn; ++c ) {
              imgp[x * cn + c] =
                  cv::saturate_cast<T>(op(imgp[x * cn + c]));
            }
          }
        }
      });
}

template<class Op>
static void forEachPixel(cv::Mat & image, const Op & op)
{
  switch (image.depth()) {
    case CV_8U:
      forEach_<uint8_t>(image, op);
      break;
    case CV_8S:
      forEach_<int8_t>(image, op);
      break;
    case CV_16U:
      forEach_<uint16_t>(image, op);
      break;
    case CV_16S:
      forEach_<int16_t>(image, op);
      break;
    case CV_32S:
      forEach_<int32_t>(image, op);
      break;
    case CV_32F:
      forEach_<float>(image, op);
      break;
    case CV_64F:
      forEach_<double>(image, op);
      break;
  }
}

void c_pixel_func_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "function", ctx(&this_class::_function), "output = c4 + c3 * func((input - c1) * c2)");
   ctlbind(ctls, "c1", ctx(&this_class::_c1), "");
   ctlbind(ctls, "c2", ctx(&this_class::_c2), "");
   ctlbind(ctls, "c3", ctx(&this_class::_c3), "");
   ctlbind(ctls, "c4", ctx(&this_class::_c4), "");
   ctlbind(ctls, "params", ctx(&this_class::_params), "");
}

bool c_pixel_func_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _function);
    SERIALIZE_OPTION(settings, save, *this, _c1);
    SERIALIZE_OPTION(settings, save, *this, _c2);
    SERIALIZE_OPTION(settings, save, *this, _c3);
    SERIALIZE_OPTION(settings, save, *this, _c4);
    SERIALIZE_OPTION(settings, save, *this, _params);
    return true;
  }
  return false;
}

bool c_pixel_func_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  switch (_function) {
    case Function_None:
      if( _c1 != 0 || _c2 != 1 || _c3 != 1 || _c4 != 0 ) {
        forEachPixel(image.getMatRef(),
            [this](double v) {
              return _c4 + _c3 * (v - _c1) * _c2;
            });
      }
      break;

    case Function_sqrt:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 * std::sqrt(std::abs( (v - _c1) * _c2));
          });
      break;

    case Function_sqr:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 * sqr((v - _c1) * _c2);
          });
      break;

    case Function_abs:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 * std::abs((v - _c1) * _c2);
          });
      break;

    case Function_log:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 * std::log((v - _c1) * _c2);
          });
      break;

    case Function_exp:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 * std::exp((v - _c1) * _c2);
          });
      break;

    case Function_inv:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c4 + _c3 / ((v - _c1) * _c2);
          });
      break;

    case Function_sin:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::sin((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_cos:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::cos((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_asin:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::asin((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_acos:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::acos((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_asinh:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::asinh((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_acosh:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return _c3 * std::acosh((v - _c1) * _c2) + _c4;
          });
      break;

    case Function_pow:
      if ( _params.size() == 1 ) {

        const double p =
            _params[0] ;

        forEachPixel(image.getMatRef(),
            [this, p](double v) {
              return _c3 * std::pow((v - _c1) * _c2, p) + _c4;
            });
      }
      break;

    case Function_poly:
      if( _params.size() > 0 ) {
        forEachPixel(image.getMatRef(),
            [this](double v) {

              double s = _params[0];

              if ( _params.size() > 1 ) {

                const double x = (v - _c1) * _c2;
                s += _params[1] * x;

                double xx = x;
                for ( int i = 2, n = _params.size(); i < n; ++i ) {
                  xx *= x;
                  s += _params[i] * xx;
                }
              }

              return _c3 * s + _c4;
            });
      }
      break;
  }

  return true;
}
