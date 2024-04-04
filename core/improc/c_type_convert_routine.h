/*
 * c_type_convert_routine.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_type_convert_routine_h__
#define __c_type_convert_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_type_convert_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_type_convert_routine,
      "convertTo",
      "Calls <strong>image.convertTo(ddepth, alpha, beta)</strong>")  ;


  void set_ddepth(enum PIXEL_DEPTH v)
  {
    ddepth_ = v;
  }

  enum PIXEL_DEPTH ddepth() const
  {
    return ddepth_;
  }

  void set_alpha(double  v)
  {
    alpha_ = v;
  }

  double alpha() const
  {
    return alpha_;
  }

  void set_beta(double  v)
  {
    beta_ = v;
  }

  double beta() const
  {
    return beta_;
  }

  void set_auto_scale(bool v)
  {
    auto_scale_ = v;
  }

  bool auto_scale() const
  {
    return auto_scale_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, ddepth, "OpenCV pixel depth");
    BIND_PCTRL(ctls, auto_scale, "auto_scale");
    BIND_PCTRL(ctls, alpha, "scale");
    BIND_PCTRL(ctls, beta, "offset");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, ddepth);
      SERIALIZE_PROPERTY(settings, save, *this, alpha);
      SERIALIZE_PROPERTY(settings, save, *this, beta);
      SERIALIZE_PROPERTY(settings, save, *this, auto_scale);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    const int ddepth = (ddepth_ == PIXEL_DEPTH_NO_CHANGE || image.fixedType()) ?
        image.depth() : ddepth_;

    double alpha, beta;

    if ( !auto_scale_ ) {
      alpha = alpha_;
      beta = beta_;
    }
    else {

      double src_max = 1;
      double dst_max = 1;

      switch (image.depth()) {
      case CV_8U:
        src_max = UINT8_MAX;
        break;
      case CV_8S:
        src_max = INT8_MAX;
        break;
      case CV_16U:
        src_max = UINT16_MAX;
        break;
      case CV_16S:
        src_max = INT16_MAX;
        break;
      case CV_32S:
        src_max = INT32_MAX;
        break;
      case CV_32F:
        src_max = 1;
        break;
      case CV_64F:
        src_max = 1;
        break;
      }

      switch (ddepth) {
      case CV_8U:
        dst_max = UINT8_MAX;
        break;
      case CV_8S:
        dst_max = INT8_MAX;
        break;
      case CV_16U:
        dst_max = UINT16_MAX;
        break;
      case CV_16S:
        dst_max = INT16_MAX;
        break;
      case CV_32S:
        dst_max = INT32_MAX;
        break;
      case CV_32F:
        dst_max = 1;
        break;
      case CV_64F:
        dst_max = 1;
        break;
      }

      alpha = dst_max / src_max;
      beta = 0;
    }

    if( ddepth != image.depth() || alpha != 1 || beta != 0 ) {

      image.getMat().convertTo(image, ddepth, alpha, beta);
    }

    return true;
  }

protected:
  double alpha_ = 1.0;
  double beta_ = 0.0;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_32F;
  bool auto_scale_ = true;
};



#endif /* __c_type_convert_routine_h__ */

