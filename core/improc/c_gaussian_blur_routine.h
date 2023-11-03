/*
 * c_gaussian_filter_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#ifndef __c_gaussian_filter_routine_h__
#define __c_gaussian_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_gaussian_filter.h>

class c_gaussian_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_blur_routine,
      "gaussian_blur", "Gaussian Blur");

  enum StereoMode {
    StereoNone,
    StereoHLayout,
    StereoVLayout,
  };

  void set_sigmax(double v)
  {
    sigmax_ = v;
  }

  double sigmax() const
  {
    return sigmax_;
  }

  void set_sigmay(double v)
  {
    sigmay_ = v;
  }

  double sigmay() const
  {
    return sigmay_;
  }

  int ksizex() const
  {
    return ksizex_;
  }

  void set_ksizex(int v)
  {
    ksizex_ = v;
  }

  int ksizey() const
  {
    return ksizey_;
  }

  void set_ksizey(int v)
  {
    ksizey_ = v;
  }

  void set_border_type(cv::BorderTypes v)
  {
    border_type_ = v;
  }

  cv::BorderTypes border_type() const
  {
    return border_type_;
  }

  void set_border_value(const cv::Scalar & v)
  {
    border_value_ = v;
  }

  const cv::Scalar & border_value() const
  {
    return border_value_;
  }

  StereoMode stereo_mode() const
  {
    return stereo_mode_;
  }

  void set_stereo_mode(StereoMode v)
  {
    stereo_mode_ = v;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigmax, "Gaussian kernel sigma along x direction");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigmay, "Gaussian kernel sigma along y direction");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ksizex, "Kernel size along x direction");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ksizey, "Kernel size along y direction");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, border_type, "Border mode");
//    ADD_IMAGE_PROCESSOR_CTRL(ctls, border_value, "border value for cv::BORDER_CONSTANT");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ignore_mask, "Ignore alpha mask");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stereo_mode, "stereo frame layout");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, sigmax);
      SERIALIZE_PROPERTY(settings, save, *this, sigmay);
      SERIALIZE_PROPERTY(settings, save, *this, ksizex);
      SERIALIZE_PROPERTY(settings, save, *this, ksizey);
      SERIALIZE_PROPERTY(settings, save, *this, border_type);
//      SERIALIZE_PROPERTY(settings, save, *this, border_value);
      SERIALIZE_PROPERTY(settings, save, *this, stereo_mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    switch (stereo_mode_) {
      case StereoNone:
        if( ignore_mask_ || mask.empty() || cv::countNonZero(mask) == mask.size().area() ) {
          c_gaussian_filter(sigmax_, sigmay_, cv::Size(ksizex_, ksizey_)).
              apply(image.getMat(), cv::noArray(), image, border_type_);
        }
        else {
          cv::Mat tmp;
          image.getMat().copyTo(tmp);
          tmp.setTo(0, ~mask.getMat());
          c_gaussian_filter(sigmax_, sigmay_, cv::Size(ksizex_, ksizey_)).
              apply(tmp, mask, image, border_type_);
        }
        break;

      case StereoHLayout: {

        const cv::Mat src_image =
            image.getMat();

        const cv::Mat src_mask =
            mask.getMat();

        const cv::Size size =
            src_image.size();

        cv::Mat frames[2];
        cv::Mat masks[2];

        src_image(cv::Rect(0, 0, size.width / 2, size.height)).copyTo(frames[0]);
        src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)).copyTo(frames[1]);

        if ( !ignore_mask_ && !src_mask.empty() ) {
          src_mask(cv::Rect(0, 0, size.width / 2, size.height)).copyTo(masks[0]);
          src_mask(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)).copyTo(masks[1]);
        }

        for( int i = 0; i < 2; ++i ) {
          if( !masks[i].empty() ) {
            frames[i].setTo(0, ~masks[i]);
          }
          c_gaussian_filter(sigmax_, sigmay_, cv::Size(ksizex_, ksizey_)).
              apply(frames[i], masks[i], frames[i], border_type_);
        }

        frames[0].copyTo(src_image(cv::Rect(0, 0, size.width / 2, size.height)));
        frames[1].copyTo(src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)));

        break;
      }

      case StereoVLayout: {

        const cv::Mat src_image =
            image.getMat();

        const cv::Mat src_mask =
            mask.getMat();

        const cv::Size size =
            src_image.size();

        cv::Mat frames[2];
        cv::Mat masks[2];

        src_image(cv::Rect(0, 0, size.width, size.height / 2)).copyTo(frames[0]);
        src_image(cv::Rect(0, size.height / 2, size.width, size.height - size.width / 2)).copyTo(frames[1]);

        if( !ignore_mask_ && !src_mask.empty() ) {
          src_mask(cv::Rect(0, 0, size.width, size.height / 2)).copyTo(masks[0]);
          src_mask(cv::Rect(0, size.height / 2, size.width, size.height - size.width / 2)).copyTo(masks[1]);
        }

        for( int i = 0; i < 2; ++i ) {
          if( !masks[i].empty() ) {
            frames[i].setTo(0, ~masks[i]);
          }
          c_gaussian_filter(sigmax_, sigmay_, cv::Size(ksizex_, ksizey_)).
              apply(frames[i], masks[i], frames[i], border_type_);
        }

        frames[0].copyTo(src_image(cv::Rect(0, 0, size.width / 2, size.height)));
        frames[1].copyTo(src_image(cv::Rect(size.width / 2, 0, size.width - size.width / 2, size.height)));

        break;
      }

      default:
        break;
    }

    return true;
  }

protected:
  double sigmax_ = 1;
  double sigmay_ = -1;
  int ksizex_ = -1;
  int ksizey_ = -1;
  StereoMode stereo_mode_ = StereoNone;
  cv::BorderTypes border_type_ = cv::BORDER_REFLECT101;
  cv::Scalar border_value_ = cv::Scalar::all(0);
};

#endif /* __c_gaussian_filter_routine_h__ */
