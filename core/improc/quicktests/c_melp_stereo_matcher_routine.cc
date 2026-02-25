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
  static const c_enum_member members[] = {
      { c_melp_stereo_matcher_routine::DisplayDisparity, "Disparity", "" },
      { c_melp_stereo_matcher_routine::DisplayHlayout, "Hlayout", "" },
      { c_melp_stereo_matcher_routine::DisplayVlayout, "Vlayout", "" },
      { c_melp_stereo_matcher_routine::DisplayMM0, "MM0", ""  },
      { c_melp_stereo_matcher_routine::DisplayMM1, "MM1", ""  },
      { c_melp_stereo_matcher_routine::DisplayBlend, "Blend", "" },
      { c_melp_stereo_matcher_routine::DisplayAbsdiff, "Absdiff", "" },
      { c_melp_stereo_matcher_routine::DisplaySAD, "SAD", "" },
      { c_melp_stereo_matcher_routine::DisplayTextureMap,"TextureMap", ""  },
      { c_melp_stereo_matcher_routine::DisplayTextureMask,"TextureMask", ""  },
      { c_melp_stereo_matcher_routine::DisplayDisparity },

  };

  return members;
}

void c_melp_stereo_matcher_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "display", ctx, &this_class::displayType, &this_class::set_displayType, "displayType");
   ctlbind(ctls, "minimum_image_size", ctx, &this_class::minimum_image_size, &this_class::set_minimum_image_size, "minimum_image_size");
   ctlbind(ctls, "texture_threshold", ctx, &this_class::texture_threshold, &this_class::set_texture_threshold, "texture_threshold");
   ctlbind(ctls, "displaypos", ctx, &this_class::displaypos, &this_class::set_displaypos, "displaypos");
   ctlbind_spinbox(ctls, "overlay_offset", ctx(&this_class::_overlay_offset), 0, 511, 1, "Shift left image before overlay");
}

bool c_melp_stereo_matcher_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, displayType);
    SERIALIZE_PROPERTY(settings, save, *this, minimum_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, texture_threshold);
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

  if( !_m.compute(images[0], images[1], cv::noArray()) ) {
    CF_ERROR("m.compute() fails");
    return false;
  }

  switch (_displayType) {

    case DisplayTextureMap: {
      _m.texture_map().copyTo(image);
      if( mask.needed() ) {
        _m.texture_mask().copyTo(mask);
      }
      break;
    }

    case DisplayTextureMask: {
      _m.texture_mask().copyTo(image);
      if( mask.needed() ) {
        mask.release();
      }
      break;
    }

    case DisplayDisparity: {
      c_block_pyramid::sptr p = select_display_node(_m.rp(), _displaypos);
      if( p ) {
        p->M.convertTo(image, CV_32F);
      }
      if ( mask.needed() ) {
        if ( p->M.size() == _m.texture_mask().size() ) {
          _m.texture_mask().copyTo(mask);
        }
        else {
          mask.release();
        }
      }
      break;
    }

    case DisplayMM0:{
      c_block_pyramid::sptr p = select_display_node(_m.rp(), _displaypos);
      if( p ) {
        p->MM[0].convertTo(image, CV_32F);
      }
      if ( mask.needed() ) {
        mask.release();
      }
      break;
    }

    case DisplayMM1:{
      c_block_pyramid::sptr p = select_display_node(_m.rp(), _displaypos);
      if( p ) {
        p->MM[1].convertTo(image, CV_32F);
      }
      if ( mask.needed() ) {
        mask.release();
      }
      break;
    }

//    case DisplayMM2:{
//      c_block_pyramid::sptr p = select_display_node(m.rp(), displaypos_);
//      if( p ) {
//        p->MM[2].convertTo(image, CV_32F);
//      }
//      if ( mask.needed() ) {
//        mask.release();
//      }
//      break;
//    }
//
//    case DisplayMM3:{
//      c_block_pyramid::sptr p = select_display_node(m.rp(), displaypos_);
//      if( p ) {
//        p->MM[3].convertTo(image, CV_32F);
//      }
//      if ( mask.needed() ) {
//        mask.release();
//      }
//      break;
//    }

    case DisplayHlayout: {
      c_block_pyramid::sptr rp = select_display_node(_m.rp(), _displaypos);
      c_block_pyramid::sptr lp = select_display_node(_m.lp(), _displaypos);
      if( rp && lp ) {

        image.create(cv::Size(lp->image.cols + rp->image.cols,
            std::max(lp->image.rows, rp->image.rows)), rp->image.type());

        //image.create(cv::Size(m.lmelp()->image.cols + m.rmelp()->image.cols,
        //    std::max(m.lmelp()->image.rows, m.rmelp()->image.rows)), m.rmelp()->image.type());

        cv::Mat &dst = image.getMatRef();

        rp->image.copyTo(dst(cv::Rect(0, 0, rp->image.cols, rp->image.rows)));
        lp->image.copyTo(dst(cv::Rect(rp->image.cols, 0, lp->image.cols, lp->image.rows)));
        //m.rmelp()->image.copyTo(dst(cv::Rect(0, 0, m.rmelp()->image.cols, m.rmelp()->image.rows)));
        //m.lmelp()->image.copyTo(dst(cv::Rect(m.rmelp()->image.cols, 0, m.lmelp()->image.cols, m.lmelp()->image.rows)));
      }

      if ( mask.needed() ) {
        mask.release();
      }

      break;
    }

    case DisplayVlayout:{
      c_block_pyramid::sptr rp = select_display_node(_m.rp(), _displaypos);
      c_block_pyramid::sptr lp = select_display_node(_m.lp(), _displaypos);
      if( rp && lp ) {

        image.create(cv::Size(std::max(lp->image.cols, rp->image.cols),
            lp->image.rows + rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();

        rp->image.copyTo(dst(cv::Rect(0, 0, rp->image.cols, rp->image.rows)));
        lp->image.copyTo(dst(cv::Rect(0, rp->image.rows, lp->image.cols, lp->image.rows)));
      }

      if ( mask.needed() ) {
        mask.release();
      }

      break;
    }

    case DisplayBlend: {
      c_block_pyramid::sptr rp = select_display_node(_m.rp(), _displaypos);
      c_block_pyramid::sptr lp = select_display_node(_m.lp(), _displaypos);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();
        dst.setTo(0);

//        cv::addWeighted(lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)), 0.5,
//            rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)), 0.5,
//            0,
//            dst(cv::Rect(0, 0, lp->image.cols - overlay_offset_, lp->image.rows)));

        cv::addWeighted(rp->image(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)), 0.5,
            lp->image(cv::Rect(_overlay_offset, 0, lp->image.cols - _overlay_offset, lp->image.rows)), 0.5,
            0,
            dst(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)));

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }
      break;
    }

    case DisplayAbsdiff: {
      c_block_pyramid::sptr rp = select_display_node(_m.rp(), _displaypos);
      c_block_pyramid::sptr lp = select_display_node(_m.lp(), _displaypos);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), rp->image.type());

        cv::Mat &dst = image.getMatRef();
        dst.setTo(0);

//        cv::absdiff(lp->image(cv::Rect(overlay_offset_, 0, lp->image.cols - overlay_offset_, lp->image.rows)),
//            rp->image(cv::Rect(0, 0, rp->image.cols - overlay_offset_, rp->image.rows)),
//            dst(cv::Rect(0, 0, lp->image.cols - overlay_offset_, lp->image.rows)));
        cv::absdiff(rp->image(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)),
            lp->image(cv::Rect(_overlay_offset, 0, lp->image.cols - _overlay_offset, lp->image.rows)),
            dst(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)));

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }

      break;
    }

    case DisplaySAD: {
      c_block_pyramid::sptr rp = select_display_node(_m.rp(), _displaypos);
      c_block_pyramid::sptr lp = select_display_node(_m.lp(), _displaypos);
      if( rp && lp ) {

        image.create(cv::Size(rp->image.cols, rp->image.rows), CV_32FC1);

        cv::Mat1f dst = image.getMatRef();

        c_melp_stereo_matcher::sad(_overlay_offset, lp, rp, dst);

        if ( mask.needed() ) {
          cv::Mat1b m(rp->image.size(), 0);
          m(cv::Rect(0, 0, rp->image.cols - _overlay_offset, rp->image.rows)).setTo(255);
          m.copyTo(mask);
        }

      }
      break;
    }
  }



  return true;
}
