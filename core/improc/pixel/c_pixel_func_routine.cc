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
  static constexpr c_enum_member members[] = {
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

//      { c_pixel_func_routine::Function_, "", ""},
//      { c_pixel_func_routine::Function_, "", ""},
//      { c_pixel_func_routine::Function_, "", ""},


      {c_pixel_func_routine::Function_None}
  };

  return members;
}

static inline float sqr( float x)
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



void c_pixel_func_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, function,
      "Select function to apply to pixel values\n");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, c1, "c1");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, c2, "c2");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, c3, "c3");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, c4, "c4");
}

bool c_pixel_func_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, function);
    SERIALIZE_PROPERTY(settings, save, *this, c1);
    SERIALIZE_PROPERTY(settings, save, *this, c2);
    SERIALIZE_PROPERTY(settings, save, *this, c3);
    SERIALIZE_PROPERTY(settings, save, *this, c4);
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
            [this](float v) {
              return c4_ + c3_ * (v - c1_) * c2_;
            });
      }
      break;

    case Function_sqrt:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ * std::sqrt(std::abs( (v - c1_) * c2_));
          });
      break;

    case Function_sqr:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ * sqr((v - c1_) * c2_);
          });
      break;

    case Function_abs:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ * std::abs((v - c1_) * c2_);
          });
      break;

    case Function_log:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ * std::log((v - c1_) * c2_);
          });
      break;

    case Function_exp:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ * std::exp((v - c1_) * c2_);
          });
      break;

    case Function_inv:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c4_ + c3_ / ((v - c1_) * c2_);
          });
      break;

    case Function_sin:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::sin((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_cos:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::cos((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_asin:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::asin((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_acos:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::acos((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_asinh:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::asinh((v - c1_) * c2_) + c4_;
          });
      break;

    case Function_acosh:
      forEachPixel(image.getMatRef(),
          [this](float v) {
            return c3_ * std::acosh((v - c1_) * c2_) + c4_;
          });
      break;
  }

  return true;
}


