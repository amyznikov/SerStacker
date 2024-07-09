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



void c_pixel_func_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, function,
      "Select function to apply to pixel values\n");

  BIND_PCTRL(ctls, c1, "c1");
  BIND_PCTRL(ctls, c2, "c2");
  BIND_PCTRL(ctls, c3, "c3");
  BIND_PCTRL(ctls, c4, "c4");
  BIND_PCTRL(ctls, params, "params");
}

bool c_pixel_func_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, function);
    SERIALIZE_PROPERTY(settings, save, *this, c1);
    SERIALIZE_PROPERTY(settings, save, *this, c2);
    SERIALIZE_PROPERTY(settings, save, *this, c3);
    SERIALIZE_PROPERTY(settings, save, *this, c4);
    SERIALIZE_PROPERTY(settings, save, *this, params);
    return true;
  }
  return false;
}

bool c_pixel_func_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  switch (function_) {
    case Function_None:
      if( c1_ != 0 || c2_ != 1 || c3_ != 1 || c4_ != 0 ) {
        forEachPixel(image.getMatRef(),
            [this](double v) {
              return c4_ + c3_ * (v - c1_) * c2_;
            });
      }
      break;

    case Function_sqrt:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ * std::sqrt(std::abs( (v - c1_) * c2_));
          });
      break;

    case Function_sqr:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ * sqr((v - c1_) * c2_);
          });
      break;

    case Function_abs:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ * std::abs((v - c1_) * c2_);
          });
      break;

    case Function_log:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ * std::log((v - c1_) * c2_);
          });
      break;

    case Function_exp:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ * std::exp((v - c1_) * c2_);
          });
      break;

    case Function_inv:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c4_ + c3_ / ((v - c1_) * c2_);
          });
      break;

    case Function_sin:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::sin((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_cos:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::cos((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_asin:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::asin((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_acos:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::acos((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_asinh:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::asinh((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_acosh:
      forEachPixel(image.getMatRef(),
          [this](double v) {
            return c3_ * std::acosh((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_pow:
      if ( params_.size() == 1 ) {

        const double p =
            params_[0] ;

        forEachPixel(image.getMatRef(),
            [this, p](double v) {
              return c3_ * std::pow((v - c1_) * c2_, p) + c4_;
            });
      }
      break;

    case Function_poly:
      if( params_.size() > 0 ) {
        forEachPixel(image.getMatRef(),
            [this](double v) {

              double s = params_[0];

              if ( params_.size() > 1 ) {

                const double x = (v - c1_) * c2_;
                s += params_[1] * x;

                double xx = x;
                for ( int i = 2, n = params_.size(); i < n; ++i ) {
                  xx *= x;
                  s += params_[i] * xx;
                }
              }

              return c3_ * s + c4_;
            });
      }
      break;
  }

  return true;
}


