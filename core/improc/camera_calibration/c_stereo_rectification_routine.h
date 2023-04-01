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

  void set_overlay_stereo_frames(bool v)
  {
    overlay_stereo_frames_ = v;
  }

  bool overlay_stereo_frames() const
  {
    return overlay_stereo_frames_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, intrinsics_filename, "Stereo intrinsics YML file");
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, extrinsics_filename, "Stereo extrinsics YML file");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, overlay_stereo_frames, "Overlay two stereo frames in one image with cv::addWeighted()");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, extrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, overlay_stereo_frames);

      return true;
    }

    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if ( !have_stereo_calibration_ ) {

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

    if ( !have_stereo_calibration_ ) {

      if ( overlay_stereo_frames_ ) {

        const cv::Mat src =
            image.getMat();

        cv::addWeighted(src(roi[0]), 0.5,
            src(roi[1]), 0.5,
            0, image);

        if ( mask.needed() && !mask.empty() ) {

          const cv::Mat msk =
              mask.getMat();

          cv::bitwise_and(msk(roi[0]),
              msk(roi[1]),
              mask);
        }
      }

    }

    else {

      for( int i = 0; i < 2; ++i ) {
        if( rmaps[i].size() != roi[i].size() ) {
          CF_ERROR("Frame size not match to intrinsics");
          return false;
        }
      }

      if ( !overlay_stereo_frames_ ) {

        cv::Mat & img =
            image.getMatRef();

        for( int i = 0; i < 2; ++i ) {
          cv::remap(img(roi[i]), img(roi[i]),
              rmaps[i], cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
        }

        if( mask.needed() ) {

          if( mask.empty() ) {
            mask.create(image.size(), CV_8UC1);
            mask.setTo(255);
          }

          cv::Mat &msk =
              mask.getMatRef();

          for( int i = 0; i < 2; ++i ) {
            cv::remap(msk(roi[i]), msk(roi[i]),
                rmaps[i], cv::noArray(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
          }

          cv::compare(msk, 254, mask, cv::CMP_GE);
        }
      }
      else {

        const cv::Mat & src_image =
            image.getMatRef();

        cv::Mat dst_image[2];

        for( int i = 0; i < 2; ++i ) {
          cv::remap(src_image(roi[i]), dst_image[i],
              rmaps[i], cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
        }

        cv::addWeighted(dst_image[0], 0.5,
            dst_image[1], 0.5,
            0, image);


        if( mask.needed() ) {

          cv::Mat src_mask[2];

          if( mask.empty() ) {
            for( int i = 0; i < 2; ++i ) {
              src_mask[i] = cv::Mat1b(roi[i].size(), 255);
            }
          }
          else {

            cv::Mat &msk =
                mask.getMatRef();

            for( int i = 0; i < 2; ++i ) {
              cv::remap(msk(roi[i]), src_mask[i],
                  rmaps[i], cv::noArray(),
                  cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
              cv::compare(src_mask[i], 254, src_mask[i], cv::CMP_GE);
            }

          }

          cv::bitwise_and(src_mask[0], src_mask[1], mask);
        }
      }
    }

    return true;
  }

protected:
  std::string stereo_intrinsics_filename_;
  std::string stereo_extrinsics_filename_;
  bool overlay_stereo_frames_ = false;

  c_stereo_camera_intrinsics intrinsics_;
  c_stereo_camera_extrinsics extrinsics_;

  bool have_stereo_calibration_ = false;
  cv::Mat2f rmaps[2];
};

#endif /* __c_stereo_rectification_routine_h__ */
