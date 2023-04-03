/*
 * c_stereo_rectification_routine.h
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 *
 *  Apply stereo rectification to horizontally laid out stereo frame.
 *
 *  This routine will apply stereo rectification to horizontally laid out input stereo frame.
 *
 *  The rectification remap is constructed from data read from user-provided calibration YML files
 *  created by OpenCV or stereo calibration pipeline.
 *  For actual YML file format see c_stereo_calibration::save_current_camera_parameters()
 *
 */

#pragma once
#ifndef __c_stereo_rectification_routine_h__
#define __c_stereo_rectification_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>

class c_stereo_rectification_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_stereo_rectification_routine,
      "stereo_rectification",
      "Apply stereo rectification to horizontal layout stereo frame.<br>");

  enum OverlayMode {
    OverlayNone,
    OverlayAdd,
    OverlayAbsdiff,
  };


  void set_intrinsics_filename(const std::string & v)
  {
    stereo_intrinsics_filename_  = v;
    have_stereo_calibration_ = false;
  }

  const std::string & intrinsics_filename() const
  {
    return stereo_intrinsics_filename_;
  }

  void set_extrinsics_filename(const std::string & v)
  {
    stereo_extrinsics_filename_  = v;
    have_stereo_calibration_ = false;
  }

  const std::string & extrinsics_filename() const
  {
    return stereo_extrinsics_filename_;
  }

  void set_overlay_mode(OverlayMode v)
  {
    overlay_mode_ = v;
  }

  OverlayMode overlay_mode() const
  {
    return overlay_mode_;
  }

  void set_overlay_offset(int v)
  {
    overlay_offset_ = v;
  }

  int overlay_offset() const
  {
    return overlay_offset_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, intrinsics_filename, "Stereo intrinsics YML file");
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, extrinsics_filename, "Stereo extrinsics YML file");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, overlay_mode, "Overlay two stereo frames into one frame");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, overlay_offset, "Shift left image before overlay");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, extrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, overlay_mode);
      SERIALIZE_PROPERTY(settings, save, *this, overlay_offset);

      return true;
    }

    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if( !have_stereo_calibration_ ) {

      bool have_initrinsics = false;
      bool have_extrinsics = false;

      if( !stereo_intrinsics_filename_.empty() ) {
        if( !read_stereo_camera_intrinsics_yml(&intrinsics_, stereo_intrinsics_filename_) ) {
          CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails",
              stereo_intrinsics_filename_.c_str());
          return false;
        }
        have_initrinsics = true;
      }

      if( !stereo_extrinsics_filename_.empty() ) {
        if( !read_stereo_camera_extrinsics_yml(&extrinsics_, stereo_extrinsics_filename_) ) {
          CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails",
              stereo_extrinsics_filename_.c_str());
          return false;
        }
        have_extrinsics = true;
      }

      if( have_initrinsics && have_extrinsics ) {

        bool fOk =
            create_stereo_rectification(cv::Size(image.cols() / 2, image.rows()),
                intrinsics_,
                extrinsics_,
                -1,
                rmaps);

        if( !fOk ) {
          CF_ERROR("create_stereo_rectification() fails");
          return false;
        }

        have_stereo_calibration_ = true;
      }
    }

    const cv::Rect roi[2] = {
        cv::Rect(0, 0, image.cols() / 2, image.rows()),
        cv::Rect(image.cols() / 2, 0, image.cols() / 2, image.rows()),
    };

    cv::Mat images[2] = {};
    cv::Mat masks[2] = {};

    if( !have_stereo_calibration_ ) {

      const cv::Mat src = image.getMat();
      const cv::Mat msk = mask.getMat();

      for( int i = 0; i < 2; ++i ) {
        src(roi[i]).copyTo(images[i]);
        if( mask.needed() && !mask.empty() ) {
          msk(roi[i]).copyTo(masks[i]);
        }
      }
    }

    else {

      const cv::Mat src = image.getMat();
      const cv::Mat msk = mask.getMat();

      for( int i = 0; i < 2; ++i ) {

        cv::remap(src(roi[i]), images[i],
            rmaps[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

        if( mask.needed() && !mask.empty() ) {

          cv::remap(msk(roi[i]), masks[i],
              rmaps[i], cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);

          cv::compare(masks[i], 254, masks[i],
              cv::CMP_GE);

        }

      }
    }

    switch (overlay_mode_) {

      case OverlayNone: {

        image.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), images[0].type());
        cv::Mat &dst = image.getMatRef();
        for( int i = 0; i < 2; ++i ) {
          images[i].copyTo(dst(roi[i]));
        }

        if( mask.needed() && !mask.empty() ) {
          mask.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), masks[0].type());
          cv::Mat &m = mask.getMatRef();
          for( int i = 0; i < 2; ++i ) {
            masks[i].copyTo(m(roi[i]));
          }
        }
        break;
      }

      case OverlayAdd: {

        const cv::Mat &left_image =
            images[0];

        const cv::Mat &right_image =
            images[1];

        image.create(right_image.size(),
            right_image.type());

        cv::Mat &dst_image =
            image.getMatRef();

        dst_image.setTo(0);

        cv::addWeighted(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            0.5,
            right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)), 0.5,
            0, dst_image);

        if( mask.needed() && !mask.empty() ) {

          const cv::Mat &left_mask =
              masks[0];

          const cv::Mat &right_mask =
              masks[1];

          mask.create(right_mask.size(),
              right_mask.type());

          cv::Mat &dst_mask =
              mask.getMatRef();

          dst_mask.setTo(0);

          cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
              right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
              dst_mask);
        }

        break;
      }

      case OverlayAbsdiff: {

        const cv::Mat &left_image =
            images[0];

        const cv::Mat &right_image =
            images[1];

        image.create(right_image.size(),
            right_image.type());

        cv::Mat &dst_image =
            image.getMatRef();

        dst_image.setTo(0);

        cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
            dst_image);

        if( mask.needed() && !mask.empty() ) {

          const cv::Mat &left_mask =
              masks[0];

          const cv::Mat &right_mask =
              masks[1];

          mask.create(right_mask.size(),
              right_mask.type());

          cv::Mat &dst_mask =
              mask.getMatRef();

          dst_mask.setTo(0);

          cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
              right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
              dst_mask);
        }

        break;
      }

      default:
        break;
    }

    return true;
  }

protected:
  std::string stereo_intrinsics_filename_;
  std::string stereo_extrinsics_filename_;
  OverlayMode overlay_mode_ = OverlayNone;
  int overlay_offset_ = 0;

  c_stereo_camera_intrinsics intrinsics_;
  c_stereo_camera_extrinsics extrinsics_;

  bool have_stereo_calibration_ = false;
  cv::Mat2f rmaps[2];
};

#endif /* __c_stereo_rectification_routine_h__ */
