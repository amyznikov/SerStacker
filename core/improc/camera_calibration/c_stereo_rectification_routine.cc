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
const c_enum_member* members_of<c_stereo_rectification_routine::DisplayMode>()
{
  static const c_enum_member members[] = {
      { c_stereo_rectification_routine::DisplayHLayout, "HLayout", "Horizontal Layout" },
      { c_stereo_rectification_routine::DisplayVLayout, "VLayout", "Vertical Layout" },
      { c_stereo_rectification_routine::DisplayBlend, "Blend", "cv::addWeighted(left, right)" },
      { c_stereo_rectification_routine::DisplayAbsdiff, "Absdiff", "cv::absdiff(left, right)" },
      { c_stereo_rectification_routine::DisplaySSA, "DisplaySSA", "DisplaySSA" },
      { c_stereo_rectification_routine::DisplayBlendSSA, "BlendSSA", "BlendSSA" },
      { c_stereo_rectification_routine::DisplayDiffSSA, "SSA", "SSA" },
      { c_stereo_rectification_routine::DisplayHLayout },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_stereo_rectification_routine::SwapFramesMode>()
{
  static const c_enum_member members[] = {
      { c_stereo_rectification_routine::SwapFramesNone, "None", "Don't swap frames" },
      { c_stereo_rectification_routine::SwapFramesAfterRectification, "After",
          "Swap camera frames after rectification)" },
      { c_stereo_rectification_routine::SwapFramesBeforeRectification, "Before",
          "Swap camera frames before rectification" },
      { c_stereo_rectification_routine::SwapFramesNone },
  };

  return members;
}


void c_stereo_rectification_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Enable rectification", ctx, &this_class::enable_rectification, &this_class::set_enable_rectification, "Enable image rectification");
  ctlbind_browse_for_file(ctls, "intrinsics", ctx, &this_class::intrinsics_filename, &this_class::set_intrinsics_filename, "Stereo intrinsics YML file");
  ctlbind_browse_for_file(ctls, "extrinsics", ctx, &this_class::extrinsics_filename, &this_class::set_extrinsics_filename, "Stereo extrinsics YML file");
  ctlbind(ctls, "swap_frames", ctx, &this_class::swap_frames, &this_class::set_swap_frames, "Swap Left and Right frames");
  ctlbind(ctls, "display_mode", ctx, &this_class::display_mode, &this_class::set_display_mode, "Overlay two stereo frames into one frame");
  ctlbind(ctls, "ss_sigma", ctx, &this_class::ss_sigma, &this_class::set_ss_sigma, "ss_sigma");
  ctlbind(ctls, "ss_radius", ctx, &this_class::ss_radius, &this_class::set_ss_radius, "ss_radius");
  ctlbind(ctls, "ss_maxlvl", ctx, &this_class::ss_maxlvl, &this_class::set_ss_maxlvl, "ss_maxlvl");
  ctlbind_spinbox(ctls, "overlay_offset", ctx(&this_class::_overlay_offset), 0, 511, 1, "Shift left image before overlay");
}

bool c_stereo_rectification_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    c_config_setting section;

    SERIALIZE_PROPERTY(settings, save, *this, enable_rectification);
    SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);
    SERIALIZE_PROPERTY(settings, save, *this, extrinsics_filename);
    SERIALIZE_PROPERTY(settings, save, *this, swap_frames);
    SERIALIZE_PROPERTY(settings, save, *this, display_mode);
    SERIALIZE_PROPERTY(settings, save, *this, overlay_offset);
    SERIALIZE_PROPERTY(settings, save, *this, ss_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, ss_radius);
    SERIALIZE_PROPERTY(settings, save, *this, ss_maxlvl);

    return true;
  }

  return false;
}


bool c_stereo_rectification_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _enable_rectification && !_have_stereo_calibration ) {

    bool have_initrinsics = false;
    bool have_extrinsics = false;

    if( !_stereo_intrinsics_filename.empty() ) {
      if( !read_stereo_camera_intrinsics_yml(&_intrinsics, _stereo_intrinsics_filename) ) {
        CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails",
            _stereo_intrinsics_filename.c_str());
        return false;
      }
      have_initrinsics = true;
    }

    if( !_stereo_extrinsics_filename.empty() ) {
      if( !read_stereo_camera_extrinsics_yml(&_extrinsics, _stereo_extrinsics_filename) ) {
        CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails",
            _stereo_extrinsics_filename.c_str());
        return false;
      }
      have_extrinsics = true;
    }

    if( have_initrinsics && have_extrinsics ) {

      bool fOk =
          create_stereo_rectification(cv::Size(image.cols() / 2, image.rows()),
              _intrinsics,
              _extrinsics,
              -1,
              rmaps);

      if( !fOk ) {
        CF_ERROR("create_stereo_rectification() fails");
        return false;
      }

      _have_stereo_calibration = true;
    }
  }

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, image.cols() / 2, image.rows()),
      cv::Rect(image.cols() / 2, 0, image.cols() / 2, image.rows()),
  };

  cv::Mat images[2] = {};
  cv::Mat masks[2] = {};

  if( !_enable_rectification || !_have_stereo_calibration ) {

    const cv::Mat src = image.getMat();
    const cv::Mat msk = mask.getMat();

    if ( _swap_frames == SwapFramesBeforeRectification ) {
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

    if ( _swap_frames == SwapFramesBeforeRectification ) {

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

  switch (_display_mode) {

    case DisplayHLayout: {

      image.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), images[0].type());
      cv::Mat &dst = image.getMatRef();

      if( _swap_frames == SwapFramesAfterRectification ) {
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

        if( _swap_frames == SwapFramesAfterRectification ) {
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

    case DisplayVLayout: {

      image.create(cv::Size(std::max(roi[0].width, roi[1].width), roi[0].height + roi[1].height), images[0].type());
      cv::Mat &dst = image.getMatRef();

      if( _swap_frames == SwapFramesAfterRectification ) {
        images[1].copyTo(dst(cv::Rect(0, 0, roi[1].width, roi[1].height)));
        images[0].copyTo(dst(cv::Rect(0, roi[1].height, roi[0].width, roi[0].height)));
      }
      else {
        images[0].copyTo(dst(cv::Rect(0, 0, roi[0].width, roi[0].height)));
        images[1].copyTo(dst(cv::Rect(0, roi[0].height, roi[1].width, roi[1].height)));
      }

      if( mask.needed() && !mask.empty() ) {

        mask.create(cv::Size(std::max(roi[0].width, roi[1].width), roi[0].height + roi[1].height), images[0].type());
        cv::Mat &m = mask.getMatRef();

        if( _swap_frames == SwapFramesAfterRectification ) {
          masks[1].copyTo(m(cv::Rect(0, 0, roi[1].width, roi[1].height)));
          masks[0].copyTo(m(cv::Rect(0, roi[1].height, roi[0].width, roi[0].height)));
        }
        else {
          masks[0].copyTo(m(cv::Rect(0, 0, roi[0].width, roi[0].height)));
          masks[1].copyTo(m(cv::Rect(0, roi[0].height, roi[1].width, roi[1].height)));
        }
      }

      break;
    }

    case DisplayBlend: {

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

      cv::addWeighted(left_image(cv::Rect(_overlay_offset, 0, left_image.cols - _overlay_offset, left_image.rows)), 0.5,
          right_image(cv::Rect(0, 0, right_image.cols - _overlay_offset, right_image.rows)), 0.5,
          0,
          dst_image(cv::Rect(0, 0, left_image.cols - _overlay_offset, left_image.rows)));

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
        cv::bitwise_and(left_mask(cv::Rect(_overlay_offset, 0, left_image.cols - _overlay_offset, left_image.rows)),
            right_mask(cv::Rect(0, 0, right_image.cols - _overlay_offset, right_image.rows)),
            dst_mask(cv::Rect(0, 0, left_image.cols - _overlay_offset, left_image.rows)));

      }

      break;
    }

    case DisplayAbsdiff: {

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
      cv::absdiff(left_image(cv::Rect(_overlay_offset, 0, left_image.cols - _overlay_offset, left_image.rows)),
          right_image(cv::Rect(0, 0, right_image.cols - _overlay_offset, right_image.rows)),
          dst_image(cv::Rect(0, 0, left_image.cols - _overlay_offset, left_image.rows)));


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
        cv::bitwise_and(left_mask(cv::Rect(_overlay_offset, 0, left_image.cols - _overlay_offset, left_image.rows)),
            right_mask(cv::Rect(0, 0, right_image.cols - _overlay_offset, right_image.rows)),
            dst_mask(cv::Rect(0, 0, left_image.cols - _overlay_offset, left_image.rows)));
      }

      break;
    }

    case DisplaySSA : {

      const cv::Mat left_image = images[0];
      const cv::Mat right_image = images[1];

      std::vector<c_ssarray> pdescs[2];
      cv::Mat desc_images[2];

      if ( !ssa_pyramid(left_image, pdescs[0], _ss_maxlvl) ) {
        CF_ERROR("ssa_pyramid() fails");
        break;
      }

      if ( !ssa_pyramid(right_image, pdescs[1], _ss_maxlvl) ) {
        CF_ERROR("ssa_pyramid() fails");
        break;
      }

      image.create(cv::Size(roi[0].width + roi[1].width, std::max(roi[0].height, roi[1].height)), CV_32F);
      cv::Mat &dst = image.getMatRef();

      for( int i = 0; i < 2; ++i ) {
        if( _swap_frames == SwapFramesAfterRectification ) {
          ssa_cvtfp32(pdescs[!i].back(), desc_images[i]);
        }
        else {
          ssa_cvtfp32(pdescs[i].back(), desc_images[i]);
        }

        //cv::resize(desc_images[i], dst(roi[i]), roi[i].size(), 0, 0, cv::INTER_AREA);
        desc_images[i].copyTo(dst(roi[i]));
      }

      break;
    }

    case DisplayBlendSSA: {

      const cv::Mat left_image =
          images[0];

      const cv::Mat right_image =
          images[1];

      std::vector<c_ssarray> descs[2];
      cv::Mat desc_images[2];

      if ( !ssa_pyramid(left_image, descs[0], _ss_maxlvl) ) {
        CF_ERROR("ssa_pyramid() fails");
        break;
      }

      if( !ssa_pyramid(right_image, descs[1], _ss_maxlvl) ) {
        CF_ERROR("ssa_pyramid() fails");
        break;
      }

      for( int i = 0; i < 2; ++i ) {
        if( _swap_frames == SwapFramesAfterRectification ) {
          ssa_cvtfp32(descs[!i].back(), desc_images[i]);
        }
        else {
          ssa_cvtfp32(descs[i].back(), desc_images[i]);
        }
        // xx
        //cv::resize(desc_images[i], desc_images[i], roi[i].size(), 0, 0, cv::INTER_AREA);
      }

      image.create(right_image.size(), CV_32F);
      cv::Mat &dst = image.getMatRef();
      dst.setTo(0);

      cv::addWeighted(desc_images[0](cv::Rect(_overlay_offset, 0, left_image.cols - _overlay_offset, left_image.rows)), 0.5,
          desc_images[1](cv::Rect(0, 0, right_image.cols - _overlay_offset, right_image.rows)), 0.5,
          0,
          dst(cv::Rect(0, 0, left_image.cols - _overlay_offset, left_image.rows)));


      break;
    }

    case DisplayDiffSSA: {

      const cv::Mat src_images[2] = {
          images[0],
          images[1]
      };

      std::vector<c_ssarray> descs[2];

      {
        INSTRUMENT_REGION("ssa_pyramid");

        for ( int i = 0; i < 2; ++i ) {
          if ( !ssa_pyramid(src_images[i], descs[i], _ss_maxlvl) ) {
            CF_ERROR("ssa_pyramid() fails");
            break;
          }
        }

        if ( descs[0].empty() || descs[1].empty() ) {
          break;
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

        ssa_compare(descs[0], cv::Rect(_overlay_offset, 0, images[0].cols - _overlay_offset, images[0].rows),
            descs[1], cv::Rect(0, 0, images[1].cols - _overlay_offset, images[1].rows),
            dst_image(cv::Rect(0, 0, images[0].cols - _overlay_offset, images[0].rows)));

        //cv::Mat1b m;
        //ssa_mask(descs[1][0], m);
        //dst_image.setTo(0, ~m);
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

