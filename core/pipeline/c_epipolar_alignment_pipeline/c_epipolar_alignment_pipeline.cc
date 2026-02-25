/*
 * c_epipolar_alignment_pipeline.cc
 *
 *  Created on: Dec 22, 2023
 *      Author: amyznikov
 */

#include "c_epipolar_alignment_pipeline.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/morphology.h>
#include <core/proc/stereo/scale_sweep.h>
#include <core/proc/colormap.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

template<>
const c_enum_member * members_of<c_epipolar_alignment_feature2d_type>()
{
  static const c_enum_member members[] = {
      { c_epipolar_alignment_feature2d_sparse, "sparse", },
      { c_epipolar_alignment_feature2d_dense, "dense", },
      { c_epipolar_alignment_feature2d_sparse},
  };
  return members;
}



namespace {

template<class T>
static double distance_between_points(const cv::Point_<T> & p1, const cv::Point_<T> & p2 )
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

template<class T>
static double distance_between_points(const cv::Vec<T, 2> & p1, const cv::Vec<T, 2> & p2 )
{
  return sqrt( (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
}

template<class T>
static inline bool IS_INSIDE_IMAGE(T x, T y, const cv::Size & image_size)
{
  return (x >= 0) && (y >= 0) && (x < image_size.width) && (y < image_size.height);
}

template<class T>
static inline bool IS_INSIDE_IMAGE(const cv::Point_<T> & point, const cv::Size & image_size)
{
  return IS_INSIDE_IMAGE(point.x, point.y, image_size);
}

template<class T>
static inline bool IS_INSIDE_IMAGE(const cv::Point_<T> & point, const cv::Mat & image)
{
  return IS_INSIDE_IMAGE(point.x, point.y, image.size());
}


static void correlate(const cv::Mat & src1, const cv::Mat & src2, cv::Mat & dst, const cv::Mat1f & G, double eps = 255)
{
  INSTRUMENT_REGION("");

  cv::Mat s1, s2, cov, norm;

  cv::sepFilter2D(src1, s1, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(src1, s1, s1, cv::noArray(), CV_32F);

  cv::sepFilter2D(src2, s2, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(src2, s2, s2, cv::noArray(), CV_32F);


  cv::sepFilter2D(s1.mul(s2), cov, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::sepFilter2D(s1.mul(s1), s1, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::sepFilter2D(s2.mul(s2), s2, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::sqrt(s1.mul(s2), norm);

  cv::divide(cov, norm, dst);
}


static void extract_pixel_matches(const cv::Mat2f & rmap, const cv::Mat1b & mask,
    std::vector<cv::Point2f> & dstpts, std::vector<cv::Point2f> & srcpts)
{
  INSTRUMENT_REGION("");

  for ( int y = 0; y < mask.rows; ++y ) {
    for ( int x = 0; x < mask.cols; ++x ) {
      if ( mask[y][x] ) {
        dstpts.emplace_back(x,y);
        srcpts.emplace_back(rmap[y][x]);
      }
    }
  }

}


static void draw_matched_positions(cv::Mat & image,
    const std::vector<cv::Point2f> & current_positions,
    const std::vector<cv::Point2f> & previous_positions,
    const cv::Mat1b & inliers)
{

  const int N = current_positions.size();
  const cv::Scalar inlierColor = CV_RGB(220, 220, 0);
  const cv::Scalar outlierColor = CV_RGB(220, 0, 0);

  for( int i = 0; i < N; ++i ) {

    const cv::Point2f &cp =
        current_positions[i];

    const cv::Point2f &pp =
        previous_positions[i];

    const cv::Scalar color =
        inliers.empty() || inliers[i][0] ?
            inlierColor :
            outlierColor;

    cv::line(image, pp, cp, color, 1, cv::LINE_4);
    //cv::rectangle(image, cv::Point(cp.x - 1, cp.y - 1), cv::Point(cp.x + 1, cp.y + 1), color, 1, cv::LINE_4);
  }
}

}

c_epipolar_alignment_pipeline::c_epipolar_alignment_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}


bool c_epipolar_alignment_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_options")) ) {
    SERIALIZE_OPTION(section, save, _camera_options, camera_intrinsics);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {

    SERIALIZE_OPTION(section, save, _feature2d_options, feature2d_type);

    if( (subsection = SERIALIZE_GROUP(section, save, "detector")) ) {
      SERIALIZE_OPTION(section, save, _feature2d_options, sparse_detector_options);
    }

    if( (subsection = SERIALIZE_GROUP(section, save, "optflowpyrlk")) ) {
      SERIALIZE_OPTION(section, save, _feature2d_options, optflowpyrlk_options);
    }
  }


  if( (section = SERIALIZE_GROUP(settings, save, "eccflow")) ) {
    SERIALIZE_PROPERTY(section, save, _eccflow, support_scale);
    SERIALIZE_PROPERTY(section, save, _eccflow, max_iterations);
    SERIALIZE_PROPERTY(section, save, _eccflow, update_multiplier);
    SERIALIZE_PROPERTY(section, save, _eccflow, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _eccflow, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _eccflow, downscale_method);
    SERIALIZE_PROPERTY(section, save, _eccflow, scale_factor);
    SERIALIZE_PROPERTY(section, save, _eccflow, min_image_size);
    SERIALIZE_PROPERTY(section, save, _eccflow, max_pyramid_level);
    SERIALIZE_PROPERTY(section, save, _eccflow, noise_level);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_pose")) ) {

    SERIALIZE_OPTION(section, save, *this, correlation_threshold);
    SERIALIZE_OPTION(section, save, *this, correlation_eps);
    SERIALIZE_OPTION(section, save, *this, correlation_kernel_radius);

    SERIALIZE_OPTION(section, save, _camera_pose_options, direction);
    SERIALIZE_OPTION(section, save, _camera_pose_options, max_iterations);
    SERIALIZE_OPTION(section, save, _camera_pose_options, robust_threshold);
    SERIALIZE_OPTION(section, save, _camera_pose_options, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, _camera_pose_options, epsx);
    SERIALIZE_OPTION(section, save, _camera_pose_options, epsf);
    SERIALIZE_OPTION(section, save, _camera_pose_options, lm);

  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {

    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "progress_video_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, progress_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_optflow_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "save_optflow_video")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, optflow_output_options);
    }


    SERIALIZE_OPTION(section, save, _output_options, save_matches_csv);

  }

  return true;
}


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_epipolar_alignment_input_options> & ctx)
{
  using S = c_epipolar_alignment_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
}


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_epipolar_alignment_camera_options> & ctx)
{
  using S = c_epipolar_alignment_camera_options;
  ctlbind(ctls, "camera_intrinsics", ctx(&S::camera_intrinsics), "");
}


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_epipolar_alignment_feature2d_options> & ctx)
{
  using S = c_epipolar_alignment_feature2d_options;

  ctlbind(ctls, "feature2d_type", ctx(&S::feature2d_type), "Select feature2d type used for odometry estimation");

  ctlbind_expandable_group(ctls, "Sparse feature2d Options", "");
    ctlbind(ctls, "feature2d detector",  ctx(&S::sparse_detector_options), "");
    ctlbind_expandable_group(ctls, "optflowpyrlk_options", "");
      ctlbind(ctls, ctx(&S::optflowpyrlk_options));
    ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls,
    const c_ctlbind_context<RootObjectType, c_epipolar_alignment_output_options> & ctx)
{
  using S = c_epipolar_alignment_output_options;

  ctlbind(ctls, "save_matches_csv", ctx(&S::save_matches_csv), "");

  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
  ctlbind(ctls, ctx(&S::progress_output_options));

  ctlbind(ctls, "save_optflow_video", ctx(&S::save_optflow_video), "");
  ctlbind(ctls, ctx(&S::optflow_output_options));

  ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));
}

const c_ctlist<c_epipolar_alignment_pipeline> & c_epipolar_alignment_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

     ctlbind_expandable_group(ctls, "1. Input options", "");
     ctlbind(ctls, ctx(&this_class::_input_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "2. Camera parameters", "");
     ctlbind(ctls, ctx(&this_class::_camera_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "3. Odometry", "");
     ctlbind(ctls, ctx(&this_class::_feature2d_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "4. Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");
     ctlbind(ctls, ctx(&this_class::_camera_pose_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "5. Output options", "");
     ctlbind(ctls, ctx(&this_class::_output_options));
     ctlbind_end_group(ctls);
  }

  return ctls;
}

//
//const std::vector<c_image_processing_pipeline_ctrl>& c_epipolar_alignment_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
////
////  if( ctrls.empty() ) {
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Camera parameters", "");
////      PIPELINE_CTL_CAMERA_INTRINSICS(ctrls, _camera_options.camera_intrinsics);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Odometry", "");
////
////      PIPELINE_CTL(ctrls, _feature2d_options.feature2d_type, "type",
////        "Select feature2d type used for odometry estimation");
////
////      PIPELINE_CTL_GROUPC(ctrls, "Sparse feature2d Options", "", (_this->_feature2d_options.feature2d_type == c_epipolar_alignment_feature2d_sparse));
////
////        PIPELINE_CTL_GROUP(ctrls, "feature2d detector", "");
////          PIPELINE_CTL_FEATURE2D_DETECTOR_OPTIONS(ctrls, _feature2d_options.sparse_detector_options);
////        PIPELINE_CTL_END_GROUP(ctrls);
////
////        PIPELINE_CTL_GROUP(ctrls, "optflowpyrlk", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.maxLevel, "maxLevel", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.winSize, "winSize", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.maxIterations, "maxIterations", "");
////          // PIPELINE_CTL(ctrls, feature2d_options_.optflowpyrlk_options.flags, "flags", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.eps, "eps", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.minEigThreshold, "minEigThreshold", "");
////          PIPELINE_CTL(ctrls, _feature2d_options.optflowpyrlk_options.maxErr, "maxErr", "");
////        PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUPC(ctrls, "EccFlow Options", "", (_this->_feature2d_options.feature2d_type == c_epipolar_alignment_feature2d_dense));
////        PIPELINE_CTLP2(ctrls, _eccflow, support_scale, "support_scale", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, max_iterations, "max_iterations", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, update_multiplier, "update_multiplier", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, input_smooth_sigma, "input_smooth_sigma", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, reference_smooth_sigma, "reference_smooth_sigma", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, downscale_method, "downscale_method", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, scale_factor, "scale_factor", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, min_image_size, "min_image_size", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, max_pyramid_level, "max_pyramid_level", "");
////        PIPELINE_CTLP2(ctrls, _eccflow, noise_level, "noise_level", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    //    c_sparse_feature_detector_options sparse_detector_options;
////    //    c_optflowpyrlk_feature2d_matcher_options optflowpyrlk_options;
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");
////
////      PIPELINE_CTL(ctrls, correlation_threshold, "corr. threshold", "");
////      PIPELINE_CTL(ctrls, correlation_eps, "corr. eps", "");
////      PIPELINE_CTL(ctrls, correlation_kernel_radius, "corr. radius", "");
////
////      PIPELINE_CTL(ctrls, _camera_pose_options.direction, "Motion direction", "Optimize assuming a priori known motion direction");
////      PIPELINE_CTL(ctrls, _camera_pose_options.max_iterations, "max iterations", "Number of iterations for outliers removal");
////      PIPELINE_CTL(ctrls, _camera_pose_options.robust_threshold, "robust threshold", "Parameter for robust function in pixels");
////      PIPELINE_CTL(ctrls, _camera_pose_options.max_levmar_iterations, "max levmar iterations", "Number of levmar iterations");
////      PIPELINE_CTL(ctrls, _camera_pose_options.epsx, "levmar epsx", "levmar epsx parameter");
////      PIPELINE_CTL(ctrls, _camera_pose_options.epsf, "levmar epsf", "levmar epsf parameter");
////      PIPELINE_CTL(ctrls, _camera_pose_options.lm, "lm", "lm method");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////      PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////
////      PIPELINE_CTL(ctrls, _output_options.save_matches_csv, "save_matches_csv", "");
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Progress video", "");
////        PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.progress_output_options,
////            _this->_output_options.save_progress_video);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Optflow video", "");
////        PIPELINE_CTL(ctrls, _output_options.save_optflow_video, "save_optflow_video", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.optflow_output_options,
////            _this->_output_options.save_optflow_video);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////    ////////
////  }
//
//  return ctrls;
//}

bool c_epipolar_alignment_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_epipolar_alignment_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->_camera_options = this->_camera_options;
  p->_camera_pose_options = this->_camera_pose_options;
  p->_output_options = this->_output_options;
  p->correlation_threshold  = this->correlation_threshold;
  p->correlation_eps = this->correlation_eps;
  p->correlation_kernel_radius = this->correlation_kernel_radius;

  return true;
}

bool c_epipolar_alignment_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( current_mg.empty() || previous_mg.empty() ) {
    return false;
  }

  cv::Mat current_frame, previous_frame, remapped_current_frame;

  if ( current_mg.channels() == 3 ) {
    current_frame = current_mg;
  }
  else {
    cv::cvtColor(current_mg, current_frame, cv::COLOR_GRAY2BGR);
  }

  if ( previous_mg.channels() == 3 ) {
    previous_frame = previous_mg;
  }
  else {
    cv::cvtColor(previous_mg, previous_frame, cv::COLOR_GRAY2BGR);
  }

  if( !remapped_current_mg.empty() ) {
    if( remapped_current_mg.channels() == 3 ) {
      remapped_current_frame = remapped_current_mg;
    }
    else {
      cv::cvtColor(remapped_current_mg, remapped_current_frame, cv::COLOR_GRAY2BGR);
    }
  }


  const cv::Size size =
      current_frame.size();

  const cv::Size display_size(3 * size.width, 3 * size.height);

  const cv::Rect roi[3][3] = {
      { cv::Rect(0, 0, size.width, size.height), cv::Rect(1 * size.width, 0, size.width, size.height), cv::Rect(
          2 * size.width, 0, size.width, size.height) },

      { cv::Rect(0, 1 * size.height, size.width, size.height), cv::Rect(1 * size.width, 1 * size.height, size.width,
          size.height), cv::Rect(2 * size.width, 1 * size.height, size.width, size.height) },

      { cv::Rect(0, 2 * size.height, size.width, size.height), cv::Rect(1 * size.width, 2 * size.height, size.width,
          size.height), cv::Rect(2 * size.width, 2 * size.height, size.width, size.height) },
  };

  if ( display_mask.needed() ) {
    display_mask.release();
  }

  if( display_frame.needed() ) {

    cv::Mat tmp;

    display_frame.create(display_size,
       current_frame.type());

    cv::Mat & display =
        display_frame.getMatRef();

    display.setTo(cv::Scalar::all(0));

    // [0][0]
    previous_frame.copyTo(display(roi[0][0]));
    // [0][1]
    current_frame.copyTo(display(roi[0][1]));
    // [0][2]
    if( !remapped_current_frame.empty() ) {
      remapped_current_frame.copyTo(display(roi[0][2]));
    }

    // [1][0]
    if ( !_current_correlation.empty() ) {
      if( display.channels() == 1 ) {
        _current_correlation.convertTo(display(roi[1][0]), display.depth(), 100);
      }
      else {
        cv::Mat tmp;
        _current_correlation.convertTo(tmp, display.depth(), 100);
        cv::cvtColor(tmp, display(roi[1][0]),
            cv::COLOR_GRAY2BGR);
      }
    }
    else {

      cv::Mat dsp = display(roi[1][0]);

      for( int i = 0, n = _warped_current_positions.size(); i < n; ++i ) {

        const auto & p1 = _matched_previous_positions[i];

        const auto & p2 = _warped_current_positions[i];

        const cv::Scalar c =
            _current_inliers[i][0] ? CV_RGB(255, 255, 16) :
                CV_RGB(255, 16, 16);

        cv::line(dsp, p1, p2, c, 1, cv::LINE_8);

      }
    }


    // [1][1]
    cv::warpPerspective(current_frame, tmp,
        _currentDerotationHomography,
        size,
        cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
        cv::BORDER_CONSTANT);

    cv::addWeighted(previous_frame, 0.5, tmp, 0.5, 0,
        display(roi[1][1]));

    // [1][2]
    if( !remapped_current_frame.empty() ) {
      cv::addWeighted(remapped_current_frame, 0.5, previous_frame, 0.5, 0,
          display(roi[1][2]));
    }
    else {
    }


    if ( !_warped_current_positions.empty() ) {

      cv::Mat1f radial(size, 0.0f);
      cv::Mat1f tangential(size, 0.0f);
      cv::Mat1b mask(size, (uint8_t)(255));

      for ( int i = 0, n = _warped_current_positions.size(); i < n; ++i ) {

        const float x1 = _matched_previous_positions[i].x;
        const float y1 = _matched_previous_positions[i].y;

        const float x2 = _warped_current_positions[i].x;
        const float y2 = _warped_current_positions[i].y;

        const int ix = (int)(x1);
        const int iy = (int)(y1);

        if( ix >= 0 && ix < size.width && iy >= 0 && iy < size.height ) {

          const float dX = x1 - _currentEpipole.x;
          const float dY = y1 - _currentEpipole.y;
          const float L = cv::norm(cv::Vec2f(dX, dY));

          const cv::Vec2f radius_vector( dX / L, dY / L);
          const cv::Vec2f tangential_vector( -dY / L, dX / L);

          const cv::Vec2f flow(x2 - x1, y2 - y1);

          radial[iy][ix] = flow.dot(radius_vector);
          tangential[iy][ix] = std::abs(flow.dot(tangential_vector));
          mask[iy][ix] = 0;
        }
      }

      // [2][0]
      radial.convertTo(tmp, CV_8U, 10, 0);
      if ( false ) {
        cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);
      }
      else {
        apply_colormap(tmp, tmp, COLORMAP_TURBO);
      }

      tmp.setTo(cv::Scalar::all(0), mask);
      tmp.convertTo(display(roi[2][0]), display.depth());

      // [2][1]
      tangential.convertTo(tmp, CV_8U, 10, 0);
      if ( false ) {
        cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);
      }
      else {
        apply_colormap(tmp, tmp, COLORMAP_TURBO);
      }
      tmp.setTo(cv::Scalar::all(0), mask);
      tmp.convertTo(display(roi[2][1]), display.depth());
    }


    // [2][2]
    if ( _current_inliers.rows == _matched_previous_positions.size() ) {

      cv::Mat3b m(roi[2][2].size(), cv::Vec3b::all(0));

      for ( int i = 0, n = _matched_previous_positions.size(); i < n; ++i ) {

        const cv::Point2f & p =
            _matched_previous_positions[i];

        const int x = (int)(p.x);
        const int y = (int)(p.y);

        if ( _current_inliers[i][0] ) {
          m[y][x][0] = 255;
          m[y][x][1] = 255;
          m[y][x][2] = 255;
        }
        else {
          m[y][x][0] = 0;
          m[y][x][1] = 0;
          m[y][x][2] = 255;
        }
      }

      m.convertTo(display(roi[2][2]), display.depth());
    }

    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {

        cv::Mat pane =
            display(roi[i][j]);

        cv::rectangle(pane, cv::Point(0, 0), cv::Point(pane.cols - 1, pane.rows - 1),
            CV_RGB(140, 128, 64), 1,
            cv::LINE_4);
      }
    }

    if( IS_INSIDE_IMAGE(_currentEpipole, size) ) {
      for( int i = 0; i < 3; ++i ) {
        for( int j = 0; j < 3; ++j ) {

          cv::Mat pane =
              display(roi[i][j]);

          const cv::Point2f E(_currentEpipole.x, _currentEpipole.y);

          cv::ellipse(pane, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
          cv::line(pane, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
          cv::line(pane, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
        }
      }
    }
  }

  return true;
}

bool c_epipolar_alignment_pipeline::initialize()
{
  _feature2d_options.sparse_detector_options.type =
      SPARSE_FEATURE_DETECTOR_FAST;

  _eccflow.set_support_scale(3);
  _eccflow.set_noise_level(1e-3);
  _eccflow.set_min_image_size(4);
  _eccflow.set_scale_factor(0.75);
  _eccflow.set_max_iterations(3);


  return true;
}


bool c_epipolar_alignment_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _current_frame.release();
  _current_mask.release();
  _previous_frame.release();
  _previous_mask.release();
  _current_remap.release();
  _current_inliers.release();
  _current_correlation_mask.release();;
  current_mg.release();
  previous_mg.release();
  remapped_current_mg.release();;

  _matched_current_positions.clear();
  _matched_previous_positions.clear();
  _current_keypoints.clear();
  _previous_keypoints.clear();
  _matched_current_positions.clear();
  _matched_previous_positions.clear();
  _matched_fused_positions.clear();

  _output_path =
      create_output_path(_output_options.output_directory);

  if( _feature2d_options.feature2d_type != c_epipolar_alignment_feature2d_sparse ) {
    _sparse_feature_detector.reset();
  }
  else if( !(_sparse_feature_detector = create_sparse_feature_detector(_feature2d_options.sparse_detector_options)) ) {
    CF_ERROR("create_sparse_feature_detector() fails");
    return false;
  }

  G =
      cv::getGaussianKernel(2 * std::max(correlation_kernel_radius, 1) + 1,
          0, CV_32F);

  _current_euler_anges = cv::Vec3d(0, 0, 0);
  _current_translation_vector = cv::Vec3d(0, 0, 1); // camera_pose_options_.direction == EPIPOLAR_MOTION_FORWARD ? 1 : -1

  return true;
}

void c_epipolar_alignment_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();
  _sparse_feature_detector.reset();
}

bool c_epipolar_alignment_pipeline::run_pipeline()
{

  if ( !start_pipeline(_input_options.start_frame_index, _input_options.max_input_frames) ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  _processed_frames = 0;
  _accumulated_frames = 0;
  for( ; _processed_frames < _total_frames;  ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !_input_sequence->read(_current_frame, &_current_mask) ) {
        CF_ERROR("input_sequence_->read() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }


    if( _input_options.input_image_processor ) {

      lock_guard lock(mutex());

      if( !_input_options.input_image_processor->process(_current_frame, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }

    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( true ) {
      lock_guard lock(mutex());

      _accumulated_frames = _processed_frames;

      std::swap(_current_frame, _previous_frame);
      std::swap(_current_mask, _previous_mask);
      std::swap(_current_keypoints, _previous_keypoints);
    }

    //if( !is_live_sequence )
    {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}


bool c_epipolar_alignment_pipeline::process_current_frame()
{
  if ( !extract_and_match_keypoints() ) {
    CF_ERROR("extract_keypoints() fails");
    return false;
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
    return false;
  }

  if( !fuse_matches() ) {
    CF_ERROR("fuse_matches() fails");
    return false;
  }

  if ( !save_progess_videos() ) {
    CF_ERROR("save_epipolar_stereo_frames() fails");
    return false;
  }

  if ( !save_matches_csv() ) {
    CF_ERROR("save_matches_csv() fails");
    return false;
  }

  // output_options_.save_matches_csv

  return true;
}


bool c_epipolar_alignment_pipeline::extract_and_match_keypoints_sparse()
{
  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  if ( _current_frame.empty() ) {
    // silently ignore if still wait for next frame
    return true;
  }

  _current_keypoints.clear();
  _sparse_feature_detector->detect(_current_frame, _current_keypoints, _current_mask);
  if ( _current_keypoints.size() < 10 ) { // ignore if fail
    CF_ERROR("sparse_feature_detector_->detect(current_frame_) detects only %zu key points, skip frame", _current_keypoints.size());
    return true;
  }

  if ( _previous_frame.empty() || _previous_keypoints.empty() ) {
    // silently ignore if still wait for next frame
    return true;
  }

  const bool fOK =
      match_optflowpyrlk(_previous_frame, _current_frame,
          _previous_keypoints,
          _feature2d_options.optflowpyrlk_options,
          _matched_previous_positions,
          _matched_current_positions);

  if( !fOK ) {
    CF_ERROR("match_optflowpyrlk() fails");
    return false;
  }

  current_mg = _current_frame;
  previous_mg = _previous_frame;

  return true;
}

bool c_epipolar_alignment_pipeline::extract_and_match_keypoints_dense()
{
  if ( _previous_frame.empty() ) {
    // silently ignore if still wait for next frame
    return true;
  }

  static const cv::Mat1b SE(3, 3, (uint8_t) 255);

  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  if( _current_frame.channels() == 1 ) {
    cv::morphologyEx(_current_frame, current_mg, cv::MORPH_GRADIENT,
        SE,
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }
  else {
    cv::cvtColor(_current_frame, current_mg,
        cv::COLOR_BGR2GRAY);
    cv::morphologyEx(current_mg, current_mg, cv::MORPH_GRADIENT,
        SE,
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }

  if( _previous_frame.channels() == 1 ) {
    cv::morphologyEx(_previous_frame, previous_mg, cv::MORPH_GRADIENT,
        SE,
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }
  else {
    cv::cvtColor(_previous_frame, previous_mg,
        cv::COLOR_BGR2GRAY);
    cv::morphologyEx(previous_mg, previous_mg, cv::MORPH_GRADIENT,
        SE,
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }


  _current_remap.release();
  if( !_eccflow.compute(current_mg, previous_mg, _current_remap, _current_mask, _previous_mask) ) {
    CF_ERROR("eccflow_.compute() fails");
    return false;
  }

  // remap current -> previous
  cv::remap(current_mg, remapped_current_mg, _current_remap, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT);

  correlate(previous_mg, remapped_current_mg, _current_correlation, G,
      correlation_eps);

  cv::compare(_current_correlation,
      cv::Scalar::all(correlation_threshold),
      _current_correlation_mask,
      cv::CMP_GE);

  _matched_current_positions.clear();
  _matched_previous_positions.clear();

  // match previous -> current
  extract_pixel_matches(_current_remap,
      _current_correlation_mask,
      _matched_previous_positions,
      _matched_current_positions);

  return true;
}

bool c_epipolar_alignment_pipeline::extract_and_match_keypoints()
{
  switch (_feature2d_options.feature2d_type) {
    case c_epipolar_alignment_feature2d_sparse:
      return extract_and_match_keypoints_sparse();
    case c_epipolar_alignment_feature2d_dense:
      return extract_and_match_keypoints_dense();
  }
  CF_ERROR("APP BUG: Invalid feature2d_type=%d requested", _feature2d_options.feature2d_type);
  return false;
}


bool c_epipolar_alignment_pipeline::estmate_camera_pose()
{
  if( _matched_current_positions.size() < 6 || _matched_previous_positions.size() < 6 ) {
    return true; // ignore
  }

  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  _current_inliers.create(_matched_current_positions.size(), 1);
  _current_inliers.setTo(255);


//  CF_DEBUG("Initial A=(%+g %+g %+g) T=(%+g %+g %+g)",
//      current_euler_anges_(0) * 180/CV_PI,
//      current_euler_anges_(1) * 180/CV_PI,
//      current_euler_anges_(2) * 180/CV_PI,
//      current_translation_vector_(0),
//      current_translation_vector_(1),
//      current_translation_vector_(2));

  _current_euler_anges = cv::Vec3d(0, 0, 0);
  _current_translation_vector = cv::Vec3d(0, 0, 1); // camera_pose_options_.direction == EPIPOLAR_MOTION_FORWARD ? +1 : -1

  bool fOk =
      lm_camera_pose_and_derotation_homography(
          _camera_options.camera_intrinsics.camera_matrix,
          _matched_current_positions,
          _matched_previous_positions,
          _current_euler_anges,
          _current_translation_vector,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
          _current_inliers,
          &_camera_pose_options);

  if ( !fOk ) {
    CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
  }

  const cv::Matx33d & camera_matrix =
      _camera_options.camera_intrinsics.camera_matrix;

  const cv::Vec3d A =
      _current_euler_anges;

  const cv::Vec3d T =
      _current_translation_vector;

  _currentRotationMatrix =
      build_rotation(A);

  _currentDerotationHomography =
      camera_matrix * _currentRotationMatrix * camera_matrix.inv();

//  currentEssentialMatrix_ =
//      compose_essential_matrix(currentRotationMatrix_, T);
//
//  currentFundamentalMatrix_ =
//      compose_fundamental_matrix(currentEssentialMatrix_, camera_matrix) * currentDerotationHomography_.inv();
//
//
//  compute_epipoles(currentFundamentalMatrix_,
//      currentEpipoles_);

//  currentEpipole_ =
//      0.5 * (currentEpipoles_[0] + currentEpipoles_[1]);

  _currentEpipole =
      compute_epipole(camera_matrix, T);

    CF_DEBUG("A: (%g %g %g)", _current_euler_anges(0) * 180 / CV_PI, _current_euler_anges(1) * 180 / CV_PI, _current_euler_anges(2) * 180 / CV_PI);
    CF_DEBUG("T: (%g %g %g)", _current_translation_vector(0), _current_translation_vector(1), _current_translation_vector(2));
//    CF_DEBUG("E: (%g %g) EE: {%+g %+g} {%+g %+g}", currentEpipole_.x, currentEpipole_.y,
//        currentEpipoles_[0].x, currentEpipoles_[0].y, currentEpipoles_[1].x, currentEpipoles_[1].y);
    CF_DEBUG("E: (%g %g) ", _currentEpipole.x, _currentEpipole.y);

//  if ( distance_between_points(currentEpipoles_[0], currentEpipoles_[1]) > 1 ) {
//    CF_WARNING("\nWARNING!\n"
//        "Something looks POOR: Computed epipoles differ after derotation:\n"
//        "E0 = {%g %g}\n"
//        "E1 = {%g %g}\n",
//        currentEpipoles_[0].x, currentEpipoles_[0].y,
//        currentEpipoles_[1].x, currentEpipoles_[1].y);
//  }

  cv::perspectiveTransform(_matched_current_positions, _warped_current_positions,
      _currentDerotationHomography);

  return true;
}

bool c_epipolar_alignment_pipeline::fuse_matches()
{
//  if ( !matched_fused_positions_.empty() ) {
//    //matched_fused_positions_ = matched_p
//  }
//
//  for ( int i = 0, n = warped_previous_positions_.size(); i < n; ++i ) {
//
//    const float x1 = matched_current_positions_[i].x;
//    const float y1 = matched_current_positions_[i].y;
//
//    const float x2 = warped_previous_positions_[i].x;
//    const float y2 = warped_previous_positions_[i].y;
//
//  }

//  //matched_fused_positions_

  return true;
}


bool c_epipolar_alignment_pipeline::save_progess_videos()
{
  if( _output_options.save_progress_video ) {

    cv::Mat display;

    if ( !get_display_image(display, cv::noArray()) ) {
      CF_WARNING("get_display_image() fails");
      return true;
    }

    bool fOk =
        add_output_writer(_progress_writer,
              _output_options.progress_output_options,
              "progress",
              ".avi");

    if( !fOk ) {
      CF_ERROR("add_output_writer('%s') fails",
          _progress_writer.filename().c_str());
      return false;
    }

    if( !_progress_writer.write(display) ) {
      CF_ERROR("progress_writer_.write('%s') fails.",
          _progress_writer.filename().c_str());
      return false;
    }
  }

  if( _output_options.save_optflow_video && !_current_remap.empty() ) {

    bool fOk =
        add_output_writer(_optflow_writer,
              _output_options.optflow_output_options,
              "optflow",
              ".ser");

    if( !fOk ) {
      CF_ERROR("add_output_writer('%s') fails",
          _optflow_writer.filename().c_str());
      return false;
    }


    cv::Mat channels[2], mag;
    cv::Mat2f flow;

    ecc_remap_to_optflow(_current_remap, flow);
    cv::split(flow, channels);
    cv::magnitude(channels[0], channels[1], mag);

    if( !_optflow_writer.write(mag) ) {
      CF_ERROR("optflow_writer_.write('%s') fails.",
          _optflow_writer.filename().c_str());
      return false;
    }

  }

  return true;
}

bool c_epipolar_alignment_pipeline::save_matches_csv()
{
  if( !_output_options.save_matches_csv ) {
    return true;
  }

  if ( _matched_current_positions.empty() || _matched_previous_positions.empty() ) {
    return true;
  }

  std::string output_directory =
      ssprintf("%s/matches_csv",
          _output_path.c_str());

  if ( !create_path(output_directory) ) {
    CF_ERROR("create_path('%s') fails.",
        output_directory.c_str());
    return false;
  }

  std::string output_filename =
      ssprintf("%s/matches.%06d.txt",
          output_directory.c_str(),
          _input_sequence->current_pos()-1);

  FILE * fp = fopen(output_filename.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen('%s') fails.",
        output_filename.c_str());
    return false;
  }

  fprintf(fp, "x1\ty1\tx2\ty2\n");
  for ( int i = 0, n = _matched_current_positions.size(); i < n; ++i ) {

    const cv::Point2f & cp =
        _matched_current_positions[i];

    const cv::Point2f & pp =
        _matched_previous_positions[i];

    fprintf(fp, "%+12.1f\t%+12.1f\t%+12.1f\t%+12.1f\n",
        cp.x, cp.y, pp.x, pp.y);

  }

  fclose(fp);

  return true;
}



