/*
 * c_regular_stereo.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_regular_stereo.h"
#include <core/proc/camera_calibration/stereo_calibrate.h>
#include <core/proc/colormap.h>
#include <core/debug.h>

void c_regular_stereo::set_enable_stereo_rectification(bool v)
{
  enable_stereo_rectification_ = v;
}

bool c_regular_stereo::enable_stereo_rectification() const
{
  return enable_stereo_rectification_;
}

void c_regular_stereo::set_camera_intrinsics_yml(const std::string & v)
{
  camera_intrinsics_yml_ = v;
}

const std::string& c_regular_stereo::camera_intrinsics_yml() const
{
  return camera_intrinsics_yml_;
}

void c_regular_stereo::set_camera_extrinsics_yml(const std::string & v)
{
  camera_extrinsics_yml_ = v;
}

const std::string& c_regular_stereo::camera_extrinsics_yml() const
{
  return camera_extrinsics_yml_;
}

c_regular_stereo_matcher& c_regular_stereo::stereo_matcher()
{
  return stereo_matcher_;
}

const c_regular_stereo_matcher& c_regular_stereo::stereo_matcher() const
{
  return stereo_matcher_;
}

c_regular_stereo_image_processing_options & c_regular_stereo::image_processing_options()
{
  return image_processing_options_;
}

const c_regular_stereo_image_processing_options & c_regular_stereo::image_processing_options() const
{
  return image_processing_options_;
}

c_stereo_output_options & c_regular_stereo::output_options()
{
  return output_options_;
}

const c_stereo_output_options & c_regular_stereo::output_options() const
{
  return output_options_;
}

bool c_regular_stereo::canceled() const
{
  return false;
}

bool c_regular_stereo::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  SERIALIZE_PROPERTY(settings, save, *this, enable_stereo_rectification );
  SERIALIZE_PROPERTY(settings, save, *this, camera_intrinsics_yml );
  SERIALIZE_PROPERTY(settings, save, *this, camera_extrinsics_yml );

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matcher")) ) {
    stereo_matcher_.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, input_image_processor);
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, remapped_image_processor);
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, output_image_processor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, depthmap_filename);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_image_filename);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_ply_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, save_depthmaps);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_image);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_ply);
  }

  return true;
}


bool c_regular_stereo::initialize()
{
  rmaps_[0].release();
  rmaps_[1].release();

  if( enable_stereo_rectification_ ) {

    if( !camera_intrinsics_yml_.empty() ) {
      CF_ERROR("camera_intrinsics_yml not set");
      return false;
    }

    if( camera_extrinsics_yml_.empty() ) {
      CF_ERROR("camera_extrinsics_yml not set");
      return false;
    }

    if( !read_stereo_camera_intrinsics_yml(&stereo_intrinsics_, camera_intrinsics_yml_) ) {
      CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails", camera_intrinsics_yml_.c_str());
      return false;
    }

    if( !read_stereo_camera_extrinsics_yml(&stereo_extrinsics_, camera_extrinsics_yml_) ) {
      CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails", camera_extrinsics_yml_.c_str());
      return false;
    }

    bool fOK =
        create_stereo_rectification(stereo_intrinsics_.camera[0].image_size,
            stereo_intrinsics_,
            stereo_extrinsics_,
            -1,
            rmaps_,
            &new_intrinsics_,
            &new_extrinsics_,
            R_,
            P_,
            &Q_,
            validRoi_);

    if( !fOK ) {
      CF_ERROR("create_stereo_rectification() fails");
      return false;
    }
  }

  return true;
}

void c_regular_stereo::cleanup()
{
  rmaps_[0].release();
  rmaps_[1].release();

  return;
}

bool c_regular_stereo::process_stereo_frame(const cv::Mat images[2], const cv::Mat masks[2])
{
  for( int i = 0; i < 2; ++i ) {

    if ( rmaps_[i].empty() ) {

      images[i].copyTo(current_images_[i]);
      masks[i].copyTo(current_masks_[i]);

      if( image_processing_options_.input_image_processor ) {
        if( !image_processing_options_.input_image_processor->process(current_images_[i], current_masks_[i]) ) {
          CF_ERROR("input_image_processor->process(frame_index=%d) fails", i);
          return false;
        }
      }
    }
    else if( !image_processing_options_.input_image_processor ) {

      cv::remap(images[i], current_images_[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( masks[i].empty() ) {
        current_masks_[i].release();
      }
      else {
        cv::remap(masks[i], current_masks_[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(current_masks_[i], 250,
            current_masks_[i],
            cv::CMP_GT);
      }
    }
    else {

      images[i].copyTo(current_images_[i]);
      masks[i].copyTo(current_masks_[i]);

      if( !image_processing_options_.input_image_processor->process(current_images_[i], current_masks_[i]) ) {
        CF_ERROR("input_image_processor->process(frame_index=%d) fails", i);
        return false;
      }

      cv::remap(current_images_[i], current_images_[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( !current_masks_[i].empty() ) {
        cv::remap(current_masks_[i], current_masks_[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(current_masks_[i], 250,
            current_masks_[i],
            cv::CMP_GT);
      }
    }

    if ( image_processing_options_.remapped_image_processor ) {
      if( !image_processing_options_.remapped_image_processor->process(current_images_[i], current_masks_[i]) ) {
        CF_ERROR("remapped_image_processor->process(frame_index=%d) fails", i);
        return false;
      }
    }
  }

  if ( !stereo_matcher_.compute(current_images_[0], current_images_[1], current_disparity_) ) {
    CF_ERROR("stereo_matcher_.compute() fails");
    return false;
  }

  return true;
}


bool c_regular_stereo::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if( current_images_[0].empty() || current_images_[1].empty() ) {
    return false;
  }

  const cv::Size sizes[2] = {
       current_images_[0].size(),
       current_images_[1].size(),
   };

   const cv::Size totalSize(sizes[0].width + sizes[1].width,
       2 * std::max(sizes[0].height, sizes[1].height));

   const cv::Rect roi[4] = {
       cv::Rect(0, 0, sizes[0].width, sizes[0].height),
       cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height),
       cv::Rect(0, std::max(sizes[0].height, sizes[1].height), sizes[0].width, sizes[0].height),
       cv::Rect(sizes[0].width, std::max(sizes[0].height, sizes[1].height), sizes[1].width, sizes[1].height),
   };

   display_frame.create(totalSize,
       CV_MAKETYPE(current_images_[0].depth(), 3));

   cv::Mat & display_frame_ =
       display_frame.getMatRef();

   for( int i = 0; i < 2; ++i ) {
     if( current_images_[i].channels() == display_frame_.channels() ) {
       current_images_[i].copyTo(display_frame_(roi[i]));
     }
     else {
       cv::cvtColor(current_images_[i], display_frame_(roi[i]),
           cv::COLOR_GRAY2BGR);
     }
   }

   if( display_frame_.depth() != CV_8U ) {
     display_frame_.convertTo(display_frame_, CV_8U);
   }

   if ( !current_disparity_.empty() ) {

     cv::Mat disp;


     current_disparity_.convertTo(disp, CV_8U,
         255 / std::max(1., stereo_matcher_.currentMaxDisparity()));

     apply_colormap(disp, disp, COLORMAP_TURBO);

     const int r =
         stereo_matcher_.currentReferenceImageIndex();

     const cv::Rect &blend_roi =
         roi[2 + r];

     const cv::Rect &disp_roi =
         roi[2 + !r];

     disp.copyTo(display_frame_(disp_roi));

     if ( current_images_[r].type() == display_frame_.type() ) {
       cv::addWeighted(disp, 0.5, current_images_[r], 0.5, 0, display_frame_(blend_roi));
     }
     else if ( current_images_[r].depth() == display_frame_.depth() ) {
       cv::Mat tmp;
       cv::cvtColor(current_images_[r], tmp, cv::COLOR_GRAY2BGR);
       cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));
     }
     else if ( current_images_[r].channels() == display_frame_.channels() ) {
       cv::Mat tmp;
       current_images_[r].convertTo(tmp, display_frame_.depth());
       cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));

     }
     else {
       cv::Mat tmp;
       cv::cvtColor(current_images_[r], tmp, cv::COLOR_GRAY2BGR);
       tmp.convertTo(tmp, display_frame_.depth());
       cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));
     }
   }

  if ( display_frame.needed() ) {
    display_frame_.copyTo(display_frame);
  }

  if ( display_mask.needed() ) {
    display_mask.release();
  }

  return true;
}

