/*
 * c_stereo_rectification_routine.cc
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 */

#include "c_stereo_rectification_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/lpg.h>

namespace cv {

  typedef Mat_<uint32_t> Mat1u;

//  template<>
//  class DataType<uint32_t>
//  {
//  public:
//      typedef uint32_t    value_type;
//      typedef value_type  work_type;
//      typedef value_type  channel_type;
//      typedef value_type  vec_type;
//      enum { generic_type = 0,
//             depth        = CV_32S,
//             channels     = 1,
//             fmt          = (int)'i',
//             type         = CV_MAKETYPE(depth, channels)
//           };
//  };
}


template<>
const c_enum_member* members_of<c_stereo_rectification_routine::OverlayMode>()
{
  static constexpr c_enum_member members[] = {
      { c_stereo_rectification_routine::OverlayNone, "None", "No overlay" },
      { c_stereo_rectification_routine::OverlayAddWeighted, "addWeighted", "cv::addWeighted(left, right)" },
      { c_stereo_rectification_routine::OverlayAbsdiff, "Absdiff", "cv::absdiff(left, right)" },
      { c_stereo_rectification_routine::OverlayContrast, "Contrast", "cv::absdiff(left, right)/cv::addWeighted(left, right)" },
      { c_stereo_rectification_routine::OverlayNCC, "NCC", "NCC" },
      { c_stereo_rectification_routine::OverlaySSD, "SSD", "SSD" },

      { c_stereo_rectification_routine::OverlayNone },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_stereo_rectification_routine::SwapFramesMode>()
{
  static constexpr c_enum_member members[] = {
      { c_stereo_rectification_routine::SwapFramesNone, "None", "Don't swap frames" },
      { c_stereo_rectification_routine::SwapFramesBeforeRectification, "Before",
          "Swap camera frames before rectification" },
      { c_stereo_rectification_routine::SwapFramesAfterRectification, "After",
          "Swap camera frames after rectification)" },
      { c_stereo_rectification_routine::SwapFramesNone },
  };

  return members;
}

namespace {

static void compute_gradient(const cv::Mat & src, cv::Mat & gx, cv::Mat & gy)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::filter2D(src, gx, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  gx.convertTo(gx, CV_8U, 10, 128);

  cv::filter2D(src, gy, CV_32F, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  gy.convertTo(gy, CV_8U, 10, 128);
}

static void compute_descriptors(const cv::Mat3b & image, cv::Mat1f & desc)
{
  cv::Mat s, gx, gy;
  std::vector<cv::Mat> lab;

  const double sigma = 1.5;

  cv::GaussianBlur(image, s, cv::Size(11, 11), sigma, sigma, cv::BORDER_REPLICATE);
  cv::cvtColor(s, s, cv::COLOR_BGR2Lab);
  cv::split(s, lab);

  const cv::Mat1b l = lab[0];
  const cv::Mat1b a = lab[1];
  const cv::Mat1b b = lab[2];

  compute_gradient(l, gx, gy);

  desc.create(s.size());

  const cv::Mat1b ggx = gx;
  const cv::Mat1b ggy = gy;

  for( int y = 0; y < s.rows; ++y ) {
    for( int x = 0; x < s.cols; ++x ) {

      const uint32_t lv = 0; //l[y][x];
      const uint32_t av = std::max(0U, std::min(127U, (uint32_t) ((float) a[y][x] - 63.f))) / 16;
      const uint32_t bv = std::max(0U, std::min(127U, (uint32_t) ((float) b[y][x] - 63.f))) / 16;
      const uint32_t gxv = 0; //ggx[y][x];
      const uint32_t gyv = 0;//ggy[y][x];

      const uint32_t v = (lv << 24) | (gxv << 16) | (gyv << 8) |
          ((av << 4) & 0xF0);// | ((bv) & 0xF);

//              (gyh << (19 - 8)) |
//              (gcv << (12 - 8)) |
//              (gsl << 3 | (gxl << 1) | (gyl << 0))
//              ;
//
//
// 32 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5  4  3    2     1
// [    gsh        ] [      gxh         ] [  gyh             ] [   gc            ] [gsl] [gxl] [gyl]
//
//
//
//      const uint32_t v =
//          ((uint32_t) gs[y][x] << 24) |
//              ((uint32_t) ggx[y][x] << 18) |
//              ((uint32_t) ggy[y][x] << 12) |
//              ((uint32_t) gc[y][x][0] << 8);

      desc[y][x] = v;
    }
  }
}

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

    case OverlayAddWeighted: {

      const cv::Mat &left_image =
          images[0];

      const cv::Mat &right_image =
          images[1];

      image.create(left_image.size(),
          left_image.type());

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

      cv::addWeighted(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
          0.5,
          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)), 0.5,
          0, dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));

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
            dst_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
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

      cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
          dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));

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
            dst_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
      }

      break;
    }

    case OverlayContrast: {

      const cv::Mat &left_image =
          images[0];

      const cv::Mat &right_image =
          images[1];


      cv::Mat summ;
      cv::Mat diff;

      image.create(left_image.size(),
          CV_MAKETYPE(CV_32F, left_image.channels()));

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

      cv::addWeighted(cv::abs(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows))),
          0.5,
          cv::abs(right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows))), 0.5,
          1, summ,
          CV_MAKETYPE(CV_32F, left_image.channels()));

      cv::GaussianBlur(summ, summ, cv::Size(5, 5), 0, 0, cv::BORDER_REPLICATE);

      cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
          right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
          diff);

      cv::divide(diff, summ,
          dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
          1, CV_MAKETYPE(CV_32F, left_image.channels()));

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
            dst_mask(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
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

        cv::absdiff(left_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)),
            right_image(cv::Rect(0, 0, right_image.cols - overlay_offset_, right_image.rows)),
            dst_image(cv::Rect(overlay_offset_, 0, left_image.cols - overlay_offset_, left_image.rows)));
      }

      if( mask.needed() && !mask.empty() ) {
        mask.release();
      }

      break;
    }

    case OverlaySSD: {

      const cv::Mat3b left_image = images[0];
      const cv::Mat3b right_image = images[1];

      cv::Mat1f left_desc, right_desc;

      compute_descriptors(left_image, left_desc);
      compute_descriptors(right_image, right_desc);

      image.create(left_image.size(),
                CV_MAKETYPE(CV_32F, 1));

      cv::Mat &dst_image =
          image.getMatRef();

      dst_image.setTo(0);

      {
        INSTRUMENT_REGION("ABSDIFF");

        cv::absdiff(left_desc(cv::Rect(overlay_offset_, 0, left_desc.cols - overlay_offset_, left_desc.rows)),
            right_desc(cv::Rect(0, 0, right_desc.cols - overlay_offset_, right_desc.rows)),
            dst_image(cv::Rect(overlay_offset_, 0, left_desc.cols - overlay_offset_, left_desc.rows)));
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

