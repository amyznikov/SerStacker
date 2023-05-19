/*
 * c_stereo_rectification_routine.cc
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 */

#include "c_stereo_rectification_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/lpg.h>


template<>
const c_enum_member* members_of<c_stereo_rectification_routine::OverlayMode>()
{
  static constexpr c_enum_member members[] = {
      { c_stereo_rectification_routine::OverlayNone, "None", "No overlay" },
      { c_stereo_rectification_routine::OverlayBlend, "Blend", "cv::addWeighted(left, right)" },
      { c_stereo_rectification_routine::OverlayAbsdiff, "Absdiff", "cv::absdiff(left, right)" },
      { c_stereo_rectification_routine::OverlayNCC, "NCC", "NCC" },
      { c_stereo_rectification_routine::OverlayDisplaySSA, "DisplaySSA", "DisplaySSA" },
      { c_stereo_rectification_routine::OverlayBlendSSA, "BlendSSA", "BlendSSA" },
      { c_stereo_rectification_routine::OverlaySSA, "SSA", "SSA" },
      // { c_stereo_rectification_routine::OverlayDAISY, "DAISY", "DAISY distance" },
      { c_stereo_rectification_routine::OverlayNone },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_stereo_rectification_routine::SwapFramesMode>()
{
  static constexpr c_enum_member members[] = {
      { c_stereo_rectification_routine::SwapFramesNone, "None", "Don't swap frames" },
      { c_stereo_rectification_routine::SwapFramesAfterRectification, "After",
          "Swap camera frames after rectification)" },
      { c_stereo_rectification_routine::SwapFramesBeforeRectification, "Before",
          "Swap camera frames before rectification" },
      { c_stereo_rectification_routine::SwapFramesNone },
  };

  return members;
}

bool c_stereo_rectification_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( enable_rectification_ && !have_stereo_calibration_ ) {

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

  if( !enable_rectification_ || !have_stereo_calibration_ ) {

    const cv::Mat src = image.getMat();
    const cv::Mat msk = mask.getMat();

    if ( swap_frames_ == SwapFramesBeforeRectification ) {
      for( int i = 0; i < 2; ++i ) {
        src(roi[i]).copyTo(images[!i]);
        if( mask.needed() && !mask.empty() ) {
          msk(roi[i]).copyTo(masks[!i]);
        }
      }
    }
    else {

      for( int i = 0; i < 2; ++i ) {
        src(roi[i]).copyTo(images[i]);
        if( mask.needed() && !mask.empty() ) {
          msk(roi[i]).copyTo(masks[i]);
        }
      }
    }
  }

  else {

    const cv::Mat src = image.getMat();
    const cv::Mat msk = mask.getMat();

    if ( swap_frames_ == SwapFramesBeforeRectification ) {

      for( int i = 0; i < 2; ++i ) {

        cv::remap(src(roi[i]), images[!i],
            rmaps[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

        if( mask.needed() && !mask.empty() ) {

          cv::remap(msk(roi[i]), masks[!i],
              rmaps[i], cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);

          cv::compare(masks[!i], 254, masks[!i],
              cv::CMP_GE);

        }
      }
    }
    else {

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
  }

  switch (overlay_mode_) {

    case OverlayNone: {

      image.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), images[0].type());
      cv::Mat &dst = image.getMatRef();

      if( swap_frames_ == SwapFramesAfterRectification ) {
        for( int i = 0; i < 2; ++i ) {
          images[i].copyTo(dst(roi[!i]));
        }
      }
      else {
        for( int i = 0; i < 2; ++i ) {
          images[i].copyTo(dst(roi[i]));
        }
      }

      if( mask.needed() && !mask.empty() ) {

        mask.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), masks[0].type());
        cv::Mat &m = mask.getMatRef();

        if( swap_frames_ == SwapFramesAfterRectification ) {
          for( int i = 0; i < 2; ++i ) {
            masks[i].copyTo(m(roi[!i]));
          }
        }
        else {
          for( int i = 0; i < 2; ++i ) {
            masks[i].copyTo(m(roi[i]));
          }
        }
      }

      break;
    }

    case OverlayBlend: {

      const cv::Mat &left_image =
          images[0];

      const cv::Mat &right_image =
          images[1];

      image.create(left_image.size(),
          left_image.type());

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

//      cv::addWeighted(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)), 0.5,
//          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)), 0.5,
//          0,
//      dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));

      cv::addWeighted(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)), 0.5,
          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)), 0.5,
          0,
          dst_image(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));

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

//        cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
//            right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
//            dst_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
        cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
            dst_mask(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));

      }

      break;
    }

    case OverlayAbsdiff: {

      const cv::Mat &left_image =
          images[0];

      const cv::Mat &right_image =
          images[1];

      image.create(left_image.size(),
          left_image.type());

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

//      cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
//          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
//          dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
      cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
          dst_image(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));


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

//        cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
//            right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
//            dst_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
        cv::bitwise_and(left_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            right_mask(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
            dst_mask(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));
      }

      break;
    }


    case OverlayNCC: {

      static const thread_local cv::Mat1f G =
          cv::getGaussianKernel(21, 5.0, CV_32F);

      cv::Mat m[2];

      {
        INSTRUMENT_REGION("SUBTRACT_MEAN");

        for( int i = 0; i < 2; ++i ) {
          cv::sepFilter2D(images[i], m[i], CV_32F, G, G);
          cv::subtract(images[i], m[i], m[i], cv::noArray(), CV_32F);
        }
      }

      const cv::Mat &left_image =
          m[0];

      const cv::Mat &right_image =
          m[1];

      image.create(left_image.size(),
                CV_MAKETYPE(CV_32F, left_image.channels()));

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

      {
        INSTRUMENT_REGION("ABSDIFF");

//        cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
//            right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
//            dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
        cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
            dst_image(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));

      }

      if( mask.needed() && !mask.empty() ) {
        mask.release();
      }

      break;
    }

    case OverlayDisplaySSA : {

      const cv::Mat left_image = images[0];
      const cv::Mat right_image = images[1];

      std::vector<c_ssarray> pdescs[2];
      cv::Mat desc_images[2];

      ssa_pyramid(left_image, pdescs[0], ss_maxlvl_, ss_flags_);
      ssa_pyramid(right_image, pdescs[1], ss_maxlvl_, ss_flags_);

      image.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), CV_32F);
      cv::Mat &dst = image.getMatRef();

      for( int i = 0; i < 2; ++i ) {
        if( swap_frames_ == SwapFramesAfterRectification ) {
          ssa_cvtfp32(pdescs[!i].back(), desc_images[i], ss_flags_);
        }
        else {
          ssa_cvtfp32(pdescs[i].back(), desc_images[i], ss_flags_);
        }

        //cv::resize(desc_images[i], dst(roi[i]), roi[i].size(), 0, 0, cv::INTER_AREA);
        desc_images[i].copyTo(dst(roi[i]));
      }

      break;
    }

    case OverlayBlendSSA: {

      const cv::Mat left_image =
          images[0];

      const cv::Mat right_image =
          images[1];

      std::vector<c_ssarray> descs[2];
      cv::Mat desc_images[2];

      ssa_pyramid(left_image, descs[0], ss_maxlvl_, ss_flags_);
      ssa_pyramid(right_image, descs[1], ss_maxlvl_, ss_flags_);

      for( int i = 0; i < 2; ++i ) {
        if( swap_frames_ == SwapFramesAfterRectification ) {
          ssa_cvtfp32(descs[!i].back(), desc_images[i], ss_flags_);
        }
        else {
          ssa_cvtfp32(descs[i].back(), desc_images[i], ss_flags_);
        }
        // xx
        //cv::resize(desc_images[i], desc_images[i], roi[i].size(), 0, 0, cv::INTER_AREA);
      }

      image.create(right_image.size(), CV_32F);
      cv::Mat &dst = image.getMatRef();
      dst.setTo(0);

      cv::addWeighted(desc_images[0](cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)), 0.5,
          desc_images[1](cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)), 0.5,
          0,
          dst(cv::Rect(0, 0, left_image.cols - overlay_offset_, left_image.rows)));


      break;
    }

    case OverlaySSA: {

      const cv::Mat src_images[2] = {
          images[0],
          images[1]
      };

      std::vector<c_ssarray> descs[2];

      {
        INSTRUMENT_REGION("ssa_pyramid");

        for ( int i = 0; i < 2; ++i ) {
          ssa_pyramid(src_images[i], descs[i], ss_maxlvl_, ss_flags_);
        }
      }

      image.create(src_images[0].size(),
          CV_MAKETYPE(CV_32F, 1));

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

//      ssa_cvtfp32(descs[1].back(),
//          dst_image(cv::Rect(0, 0, descs[1].back().cols(), descs[1].back().rows())),
//          ss_flags_);

//      ssa_cvtfp32(descs[0].back(),
//          dst_image(cv::Rect(0, 0, descs[0].back().cols(), descs[0].back().rows())),
//          ss_flags_);

      {
        INSTRUMENT_REGION("ssa_compare");

        ssa_compare(descs[0], cv::Rect(overlay_offset_, 0, images[0].cols - overlay_offset_, images[0].rows),
            descs[1], cv::Rect(0, 0, images[1].cols - overlay_offset_, images[1].rows),
            dst_image(cv::Rect(0, 0, images[0].cols - overlay_offset_, images[0].rows)));

        cv::Mat1b m;

        ssa_mask(descs[1][0], m);
        dst_image.setTo(0, ~m);
      }

      if( mask.needed() && !mask.empty() ) {
        mask.release();
      }

      break;
    }


    default:
      break;
  }

  return true;
}

