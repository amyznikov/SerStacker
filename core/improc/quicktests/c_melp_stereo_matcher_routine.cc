/*
 * c_melp_stereo_matcher_routine.cc
 *
 *  Created on: Jun 10, 2023
 *      Author: amyznikov
 */

#include "c_melp_stereo_matcher_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_melp_stereo_matcher_routine::DisplayType>()
{
  static constexpr c_enum_member members[] = {
      { c_melp_stereo_matcher_routine::DisplayDisparity, "Disparity", "" },
      { c_melp_stereo_matcher_routine::DisplayHlayout, "Hlayout", "" },
      { c_melp_stereo_matcher_routine::DisplayVlayout, "Vlayout", "" },
      { c_melp_stereo_matcher_routine::DisplayBlend, "Blend", "" },
      { c_melp_stereo_matcher_routine::DisplayAbsdiff, "Absdiff", "" },
      { c_melp_stereo_matcher_routine::DisplaySAD, "SAD", "" },
      { c_melp_stereo_matcher_routine::DisplayDisparity },
  };

  return members;
}

void c_melp_stereo_matcher_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, displayType, "displayType");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, minimum_image_size, "minimum_image_size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, displaypos, "displaypos");
  ADD_IMAGE_PROCESSOR_SPINBOX_CTRL(ctls, overlay_offset, 0, 511, 1, "Shift left image before overlay");
}

bool c_melp_stereo_matcher_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, displayType);
    SERIALIZE_PROPERTY(settings, save, *this, minimum_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, displaypos);
    SERIALIZE_PROPERTY(settings, save, *this, overlay_offset);
    return true;
  }
  return false;
}

bool c_melp_stereo_matcher_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  using c_blockarray = c_melp_stereo_matcher::c_blockarray;
  using c_blockdesc = c_melp_stereo_matcher::c_blockdesc;
  using c_block_pyramid = c_melp_stereo_matcher::c_block_pyramid;


  static const auto recurse =
      [](c_block_pyramid::sptr p, int h, int v) -> c_block_pyramid::sptr {

        for ( int i = 0; i < h && p; ++i ) {
          p = p->g;
        }

        for ( int i = 0; i < v && p; ++i ) {
          p = p->m;
        }

        return p;
      };

  static const auto select_display_node =
      [](c_block_pyramid::sptr p, const std::vector<int> & displaypos) -> c_block_pyramid::sptr {

        for( int i = 0, n = displaypos.size(); i < n && p; i += 2 ) {
          const int h = displaypos[i];
          const int v = i < n - 1 ? displaypos[i + 1] : 0;
          p = recurse(p, h, v);
        }

        return p;
      };


  cv::Mat images[2];

  cv::Mat src_image =
      image.getMat();

  cv::Mat src_mask =
      mask.getMat();

  const cv::Size src_size =
      src_image.size();

  const cv::Size s(src_size.width / 2, src_size.height);

  const cv::Rect ROI[2] = {
      cv::Rect(0, 0, s.width, s.height),
      cv::Rect(s.width, 0, s.width, s.height),
  };

  for( int i = 0; i < 2; ++i ) {
    images[i] = src_image(ROI[i]);
  }

  if( !m.compute(images[0], images[1], cv::noArray()) ) {
    CF_ERROR("m.compute() fails");
    return false;
  }

  if ( mask.needed() ) {
    mask.release();
  }

  switch (displayType_) {
    case DisplayDisparity: {
      c_block_pyramid::sptr p = select_display_node(m.rp(), displaypos_);
      if( p ) {

        image.create(p->M.size(), CV_32FC1);

        cv::Mat1f dst = image.getMatRef();

        for( int y = 0; y < dst.rows; ++y ) {
          for( int x = 0; x < dst.cols; ++x ) {
            dst[y][x] = p->M[y][x] - x;
          }
        }
      }
      break;
    }
    case DisplayHlayout: {
      c_block_pyramid::sptr rp = select_display_node(m.rp(), displaypos_);
      c_block_pyramid::sptr lp = select_display_node(m.lp(), displaypos_);
      if( rp && lp ) {

        image.create(cv::Size(lp->image.cols + rp->image.cols,
            std::max(lp->image.rows, rp->image.rows)), rp->image.type());

        cv::Mat &dst = image.getMatRef();

        rp->image.copyTo(dst(cv::Rect(0, 0, rp->image.cols, rp->image.rows)));
        lp->image.copyTo(dst(cv::Rect(rp->image.cols, 0, lp->image.cols, lp->image.rows)));
      }
      break;
    }
    case DisplayVlayout:{
      c_block_pyramid::sptr rp = select_display_node(m.rp(), displaypos_);
      c_block_pyramid::sptr lp = select_display_node(m.lp(), displaypos_);
      if( rp && lp ) {

        image.create(cv::Size(std::max(lp->image.cols, rp->image.cols),
            lp->image.rows + rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();

        rp->image.copyTo(dst(cv::Rect(0, 0, rp->image.cols, rp->image.rows)));
        lp->image.copyTo(dst(cv::Rect(0, rp->image.rows, lp->image.cols, lp->image.rows)));
      }
      break;
    }
    case DisplayBlend: {
      c_block_pyramid::sptr rp = select_display_node(m.rp(), displaypos_);
      c_block_pyramid::sptr lp = select_display_node(m.lp(), displaypos_);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();
        dst.setTo(0);

//        cv::addWeighted(lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)), 0.5,
//            rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)), 0.5,
//            0,
//            dst(cv::Rect(0, 0, lp->image.cols - overlay_offset_, lp->image.rows)));

        cv::addWeighted(rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)), 0.5,
            lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)), 0.5,
            0,
            dst(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)));

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }
      break;
    }

    case DisplayAbsdiff: {
      c_block_pyramid::sptr rp = select_display_node(m.rp(), displaypos_);
      c_block_pyramid::sptr lp = select_display_node(m.lp(), displaypos_);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();
        dst.setTo(0);

//        cv::absdiff(lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)),
//            rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)),
//            dst(cv::Rect(0, 0, lp->image.cols - overlay_offset_, lp->image.rows)));
        cv::absdiff(rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)),
            lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)),
            dst(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)));

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }


      break;
    }

    case DisplaySAD: {
      c_block_pyramid::sptr rp = select_display_node(m.rp(), displaypos_);
      c_block_pyramid::sptr lp = select_display_node(m.lp(), displaypos_);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), CV_32FC1);

        cv::Mat1f dst = image.getMatRef();

        c_melp_stereo_matcher::sad(overlay_offset_, lp, rp, dst);

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }
      break;
    }
  }



  return true;
}
