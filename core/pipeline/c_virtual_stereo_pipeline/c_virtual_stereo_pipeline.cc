/*
 * c_virtual_stereo_pipeline.cc
 *
 *  Created on: Mar 9, 2023
 *      Author: amyznikov
 */

#include "c_virtual_stereo_pipeline.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/polar_trasform.h>
#include <core/proc/stereo/triangulate.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/colormap.h>
#include <core/readdir.h>
#include <core/io/save_ply.h>
#include <chrono>
#include <thread>

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

c_virtual_stereo_pipeline::c_virtual_stereo_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
  _feature2d_options.detector.type = SPARSE_FEATURE_DETECTOR_AKAZE;
  _feature2d_options.matcher.type = FEATURE2D_MATCHER_HAMMING;
  _feature2d_options.matcher.hamming.max_acceptable_distance = 20;
  _feature2d_options.matcher.hamming.octavedif = -1;


  //pyrflowlk_options_.detector.type = SPARSE_FEATURE_DETECTOR_FAST;
}

c_virtual_stereo_input_options & c_virtual_stereo_pipeline::input_options()
{
  return _input_options;
}

const c_virtual_stereo_input_options & c_virtual_stereo_pipeline::input_options() const
{
  return _input_options;
}

c_virtual_stereo_camera_options & c_virtual_stereo_pipeline::camera_options()
{
  return _camera_options;
}

const c_virtual_stereo_camera_options & c_virtual_stereo_pipeline::camera_options() const
{
  return _camera_options;
}

c_virtual_stereo_image_processing_options & c_virtual_stereo_pipeline::image_processing_options()
{
  return _image_processing_options;
}

const c_virtual_stereo_image_processing_options & c_virtual_stereo_pipeline::image_processing_options() const
{
  return _image_processing_options;
}

c_virtual_stereo_feature2d_options & c_virtual_stereo_pipeline::feature2d_options()
{
  return _feature2d_options;
}

const c_virtual_stereo_feature2d_options & c_virtual_stereo_pipeline::feature2d_options() const
{
  return _feature2d_options;
}

c_lm_camera_pose_options & c_virtual_stereo_pipeline::camera_pose_options()
{
  return _camera_pose_options;
}

const c_lm_camera_pose_options & c_virtual_stereo_pipeline::camera_pose_options() const
{
  return _camera_pose_options;
}

c_virtual_stereo_output_options & c_virtual_stereo_pipeline::output_options()
{
  return _output_options;
}

const c_virtual_stereo_output_options & c_virtual_stereo_pipeline::output_options() const
{
  return _output_options;
}

bool c_virtual_stereo_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

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
    SERIALIZE_OPTION(section, save, _feature2d_options, detector);
    SERIALIZE_OPTION(section, save, _feature2d_options, descriptor);
    SERIALIZE_OPTION(section, save, _feature2d_options, matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_pose")) ) {
    SERIALIZE_OPTION(section, save, _camera_pose_options, direction);
    SERIALIZE_OPTION(section, save, _camera_pose_options, max_iterations);
    SERIALIZE_OPTION(section, save, _camera_pose_options, robust_threshold);
    SERIALIZE_OPTION(section, save, _camera_pose_options, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, _camera_pose_options, epsx);
    SERIALIZE_OPTION(section, save, _camera_pose_options, epsf);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matcher")) ) {
    SERIALIZE_OPTION(section, save, _stereo_matcher_options, enable_stereo_matcher);
    _stereo_matcher.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "triangulation")) ) {
    SERIALIZE_OPTION(section, save, _triangulation_options, enable_triangulation);
    SERIALIZE_OPTION(section, save, _triangulation_options, baseline);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "epipolar_matcher")) ) {
    _epipolar_matcher.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "epipolar_flow")) ) {
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, enabled);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, enable_debug);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, support_scale);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, max_iterations);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, normalization_scale);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, max_pyramid_level);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, noise_level);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, update_multiplier);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, input_smooth_sigma);
    SERIALIZE_OPTION(section, save, _epipolar_flow_options, reference_smooth_sigma);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "morph_gradient_flow")) ) {
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, enabled);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, max_pyramid_level);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, block_radius);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, search_radius);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, alpha);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, beta);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, gradient_threshold);
    SERIALIZE_OPTION(section, save, _morph_gradient_flow_options, enable_debug);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    // SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, input_processor);
    SERIALIZE_IMAGE_PROCESSOR(section, save, _image_processing_options, feature2d_preprocessor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    SERIALIZE_OPTION(section, save, _output_options, progress_video_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_polar_frames);
    SERIALIZE_OPTION(section, save, _output_options, polar_frames_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_disparity_frames);
    SERIALIZE_OPTION(section, save, _output_options, disparity_frames_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_homography_video);
    SERIALIZE_OPTION(section, save, _output_options, homography_video_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_epipolar_flow_debug_images);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_input_options> & ctx)
{
  using S = c_virtual_stereo_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_camera_options> & ctx)
{
  using S = c_virtual_stereo_camera_options;
  ctlbind(ctls, "camera_intrinsics", ctx(&S::camera_intrinsics), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_image_processing_options> & ctx)
{
  using S = c_virtual_stereo_image_processing_options;
  ctlbind(ctls, "feature2d_preprocessor", ctx(&S::feature2d_preprocessor), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_feature2d_options> & ctx)
{
  using S = c_virtual_stereo_feature2d_options;
  ctlbind(ctls, as_base<c_sparse_feature_extractor_and_matcher_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_matcher_options> & ctx)
{
  using S = c_virtual_stereo_matcher_options;
  ctlbind(ctls, "enable_stereo_matcher", ctx(&S::enable_stereo_matcher), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_triangulation_options> & ctx)
{
  using S = c_virtual_stereo_triangulation_options;
  ctlbind(ctls, "enable_triangulation", ctx(&S::enable_triangulation), "");
  ctlbind(ctls, "baseline", ctx(&S::baseline), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,c_virtual_stereo_epipolar_flow_options > & ctx)
{
  using S = c_virtual_stereo_epipolar_flow_options;

  ctlbind(ctls, "enabled", ctx(&S::enabled), "");
  ctlbind(ctls, "support_scale", ctx(&S::support_scale), "");
  ctlbind(ctls, "max_pyramid_level", ctx(&S::max_pyramid_level), "");
  ctlbind(ctls, "max_iterations", ctx(&S::max_iterations), "");
  ctlbind(ctls, "normalization_scale", ctx(&S::normalization_scale), "");
  ctlbind(ctls, "noise_level", ctx(&S::noise_level), "");
  ctlbind(ctls, "update_multiplier", ctx(&S::update_multiplier), "");
  ctlbind(ctls, "input_smooth_sigma", ctx(&S::input_smooth_sigma), "");
  ctlbind(ctls, "reference_smooth_sigma", ctx(&S::reference_smooth_sigma), "");
  ctlbind(ctls, "enable_debug", ctx(&S::enable_debug), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_morph_gradient_flow_options> & ctx)
{
  using S = c_virtual_stereo_morph_gradient_flow_options;

  ctlbind(ctls, "enabled", ctx(&S::enabled), "");
  ctlbind(ctls, "max_pyramid_level", ctx(&S::max_pyramid_level), "");
  ctlbind(ctls, "block_radius", ctx(&S::block_radius), "");
  ctlbind(ctls, "search_radius", ctx(&S::search_radius), "");
  ctlbind(ctls, "alpha", ctx(&S::alpha), "");
  ctlbind(ctls, "beta", ctx(&S::beta), "");
  ctlbind(ctls, "gradient_threshold", ctx(&S::gradient_threshold), "");
  ctlbind(ctls, "enable_debug", ctx(&S::enable_debug), "");

}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_virtual_stereo_output_options> & ctx)
{
  using S = c_virtual_stereo_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "output_directory", ctx(&S::output_directory), "");
  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
  ctlbind(ctls, "progress_video_filename", ctx(&S::progress_video_filename), "");
  ctlbind(ctls, "save_polar_frames", ctx(&S::save_polar_frames), "");
  ctlbind(ctls, "polar_frames_filename", ctx(&S::polar_frames_filename), "");
  ctlbind(ctls, "save_disparity_frames", ctx(&S::save_disparity_frames), "");
  ctlbind(ctls, "disparity_frames_filename", ctx(&S::disparity_frames_filename), "");
  ctlbind(ctls, "save_homography_video", ctx(&S::save_homography_video), "");
  ctlbind(ctls, "homography_video_filename", ctx(&S::homography_video_filename), "");
  ctlbind(ctls, "save_epipolar_flow_debug_images", ctx(&S::save_epipolar_flow_debug_images), "");
}

const c_ctlist<c_virtual_stereo_pipeline> & c_virtual_stereo_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "Input options", "");
    ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Camera parameters", "");
    ctlbind(ctls, ctx(&this_class::_camera_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Feature2D options", "");
    ctlbind(ctls, ctx(&this_class::_feature2d_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");
    ctlbind(ctls, ctx(&this_class::_camera_pose_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Stereo Matcher Options", "");
    ctlbind(ctls, ctx(&this_class::_stereo_matcher_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Triangulation", "");
    ctlbind(ctls, ctx(&this_class::_triangulation_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Epipolar Flow", "");
    ctlbind(ctls, ctx(&this_class::_epipolar_flow_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "morph_gradient_flow", "");
    ctlbind(ctls, ctx(&this_class::_morph_gradient_flow_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Epipolar Matcher", "");
    ctlbind(ctls, ctx(&this_class::_epipolar_matcher_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Image processing", "");
    ctlbind(ctls, ctx(&this_class::_image_processing_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;

}
//
//const std::vector<c_image_processing_pipeline_ctrl> & c_virtual_stereo_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
////
////
////  if( ctrls.empty() ) {
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////      ////////
////    PIPELINE_CTL_GROUP(ctrls, "Camera parameters", "");
////      PIPELINE_CTL_CAMERA_INTRINSICS(ctrls, _camera_options.camera_intrinsics);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Feature2D options", "");
////      PIPELINE_CTL_GROUP(ctrls, "Feature2D Detector Options", "");
////      PIPELINE_CTL_FEATURE2D_DETECTOR_OPTIONS(ctrls, _feature2d_options.detector);
////      PIPELINE_CTL_END_GROUP(ctrls);
////      PIPELINE_CTL_GROUP(ctrls, "Feature2D Descriptor Options", "");
////      PIPELINE_CTL_FEATURE2D_DESCRIPTOR_OPTIONS(ctrls, _feature2d_options.descriptor);
////      PIPELINE_CTL_END_GROUP(ctrls);
////      PIPELINE_CTL_GROUP(ctrls, "Feature2D Matcher Options", "");
////      PIPELINE_CTL_FEATURE2D_MATCHER_OPTIONS(ctrls, _feature2d_options.matcher);
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");
////      PIPELINE_CTL(ctrls, _camera_pose_options.direction, "Motion direction", "Optimize assuming a priori known motion direction");
////      PIPELINE_CTL(ctrls, _camera_pose_options.max_iterations, "max iterations", "Number of iterations for outliers removal");
////      PIPELINE_CTL(ctrls, _camera_pose_options.robust_threshold, "robust threshold", "Parameter for robust function in pixels");
////      PIPELINE_CTL(ctrls, _camera_pose_options.max_levmar_iterations, "max levmar iterations", "Number of levmar iterations");
////      PIPELINE_CTL(ctrls, _camera_pose_options.epsx, "levmar epsx", "levmar epsx parameter");
////      PIPELINE_CTL(ctrls, _camera_pose_options.epsf, "levmar epsf", "levmar epsf parameter");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Stereo Matcher Options", "");
////      PIPELINE_CTL_STEREO_MATCHER_OPTIONS(ctrls, _stereo_matcher);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Triangulation", "");
////      PIPELINE_CTL(ctrls, _triangulation_options.enable_triangulation, "enable triangulation", "Set checked to enable triangulation");
////      PIPELINE_CTL(ctrls, _triangulation_options.baseline, "baseline", "Set baseline for triangulation");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Epipolar Flow", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.enabled, "Enable epipolar_flow", "Check to enable c_epipolar_flow testing");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.enable_debug, "Enable Debug images", "Check to enable c_epipolar_flow to save debg images");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.support_scale, "support_scale", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.max_pyramid_level, "max_pyramid_level", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.update_multiplier, "update_multiplier", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.input_smooth_sigma, "input_smooth_sigma", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.reference_smooth_sigma, "reference_smooth_sigma", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.normalization_scale, "normalization_scale", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.noise_level, "noise_level", "");
////      PIPELINE_CTL(ctrls, _epipolar_flow_options.max_iterations, "max_iterations", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "morph_gradient_flow", "");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.enabled, "Enable pyrflowlk", "Check to enable pyrflowlk testing");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.max_pyramid_level, "max_pyramid_level", "max_pyramid_level");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.block_radius, "block_radius", "block_radius");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.search_radius, "search_radius", "search_radius");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.alpha, "alpha", "alpha");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.beta, "beta", "beta");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.gradient_threshold, "gradient_threshold", "gradient_threshold");
////      PIPELINE_CTL(ctrls, _morph_gradient_flow_options.enable_debug, "Enable debug images", "Check to save pyrflowlk debug images");
////    PIPELINE_CTL_END_GROUP(ctrls);
////    ////////
////
////    PIPELINE_CTL_GROUP(ctrls, "Epipolar Matcher", "");
////      PIPELINE_CTL(ctrls, _epipolar_matcher.options().enabled, "enable epipolar matcher", "");
////      PIPELINE_CTL_GROUP(ctrls, "Epipolar Matcher Options", "");
////        PIPELINE_CTL(ctrls, _epipolar_matcher.options().max_disparity, "max_disparity", "");
////        PIPELINE_CTL(ctrls, _epipolar_matcher.options().diff_threshold, "diff_threshold", "");
////        PIPELINE_CTL(ctrls, _epipolar_matcher.options().avg_scale, "avg_scale", "");
////        PIPELINE_CTL(ctrls, _epipolar_matcher.options().enable_debug, "enable_debug", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////    ////////
////
////    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
////      // PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.input_processor, "Input image preprocessor", "");
////      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _image_processing_options.feature2d_preprocessor, "Feature2D image preprocessor", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////      PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
////      PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////      PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
////      PIPELINE_CTL(ctrls, _output_options.progress_video_filename, "progress_video_filename", "");
////      PIPELINE_CTL(ctrls, _output_options.save_polar_frames, "save_polar_frames", "");
////      PIPELINE_CTL(ctrls, _output_options.polar_frames_filename, "polar_frames_filename", "");
////      PIPELINE_CTL(ctrls, _output_options.save_disparity_frames, "save_disparity_frames", "");
////      PIPELINE_CTL(ctrls, _output_options.disparity_frames_filename, "disparity_frames_filename", "");
////      PIPELINE_CTL(ctrls, _output_options.save_homography_video, "save_homography_video", "");
////      PIPELINE_CTL(ctrls, _output_options.homography_video_filename, "homography_video_filename", "");
////      PIPELINE_CTL(ctrls, _output_options.save_epipolar_flow_debug_images, "save_epipolar_flow_debug_images", "");
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    ////////
////  }
//
//  return ctrls;
//}

bool c_virtual_stereo_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_image_stacking_pipeline::base::copyParameters() fails");
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
  p->_image_processing_options = this->_image_processing_options;
  p->_feature2d_options = this->_feature2d_options;
  p->_camera_pose_options  = this->_camera_pose_options;
  p->_stereo_matcher_options = this->_stereo_matcher_options;
  p->_output_options = this->_output_options;

  return true;
}


bool c_virtual_stereo_pipeline::initialize_pipeline()
{
  CF_DEBUG("Enter");

  lock_guard lock(mutex());


  if ( !base::initialize_pipeline() ) {
    CF_ERROR("Base::initialize_pipeline() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  _output_path =
      create_output_path(output_options().output_directory);


  /////////////////////////////////////////////////////////////////////////////

  if( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }


  /////////////////////////////////////////////////////////////////////////////

  if ( !(_keypoints_extractor = create_keypoints_extractor() ) ) {
    CF_ERROR("create_keypoints_extractor() fails");
    return false;
  }

//  if( feature2d_options_.matcher.type == FEATURE2D_MATCHER_OptFlowPyrLK ) {
//    keypoints_matcher_.reset();
//  }
//  else if( !(keypoints_matcher_ = create_sparse_feature_matcher(feature2d_options_.matcher)) ) {
//    CF_ERROR("create_sparse_feature_matcher() fails");
//    return false;
//  }

  /////////////////////////////////////////////////////////////////////////////

  _currentDisparity.release();

  _current_image.release();
  _previous_image.release();

  _current_mask.release();
  _previous_mask.release();

  _current_keypoints.clear();
  _previous_keypoints.clear();

  _current_descriptors.release();
  _previous_descriptors.release();

  /////////////////////////////////////////////////////////////////////////////

  _matched_current_positions.clear();
  _matched_previous_positions.clear();

  /////////////////////////////////////////////////////////////////////////////

  if( _epipolar_flow_options.enabled ) {
//    epipolar_flow_.set_support_scale(epipolar_flow_options_.support_scale);
//    epipolar_flow_.set_max_iterations(epipolar_flow_options_.max_iterations);
//    epipolar_flow_.set_update_multiplier(epipolar_flow_options_.update_multiplier);
//    epipolar_flow_.set_normalization_scale(epipolar_flow_options_.normalization_scale);
//    epipolar_flow_.set_input_smooth_sigma(epipolar_flow_options_.input_smooth_sigma);
//    epipolar_flow_.set_reference_smooth_sigma(epipolar_flow_options_.reference_smooth_sigma);
//    epipolar_flow_.set_max_pyramid_level(epipolar_flow_options_.max_pyramid_level);
//    epipolar_flow_.set_noise_level(epipolar_flow_options_.noise_level);
  }

  /////////////////////////////////////////////////////////////////////////////

  CF_DEBUG("Leave");

  return true;
}

void c_virtual_stereo_pipeline::cleanup_pipeline()
{
  lock_guard lock(mutex());

  close_input_sequence();

  if ( _progress_video_writer.is_open() ) {
    _progress_video_writer.close();
    CF_DEBUG("SAVED '%s'", _progress_video_writer.filename().c_str());
  }

  if ( _polar_frames_writer.is_open() ) {
    _polar_frames_writer.close();
    CF_DEBUG("SAVED '%s'", _polar_frames_writer.filename().c_str());
  }

  if ( _disparity_frames_writer.is_open() ) {
    _disparity_frames_writer.close();
    CF_DEBUG("SAVED '%s'", _disparity_frames_writer.filename().c_str());
  }

  if ( _homography_video_writer.is_open() ) {
    _homography_video_writer.close();
    CF_DEBUG("SAVED '%s'", _homography_video_writer.filename().c_str());
  }

//  if ( median_hat_video_writer_.is_open() ) {
//    median_hat_video_writer_.close();
//    CF_DEBUG("SAVED '%s'", median_hat_video_writer_.filename().c_str());
//  }

  // pyrflowlk_keypoints_detector_.reset();

}

//bool c_virtual_stereo_pipeline::open_input_sequence()
//{
//  if ( !input_sequence_ ) {
//    CF_ERROR("ERROR: input_sequence_ not set");
//    return false;
//  }
//
//  if (  !input_sequence_->open() ) {
//    CF_ERROR("ERROR: input_sequence->open() fails");
//    return false;
//  }
//
//  return true;
//}
//
//void c_virtual_stereo_pipeline::close_input_sequence()
//{
//  if ( input_sequence_ ) {
//    input_sequence_->close();
//  }
//}
//
//
//bool c_virtual_stereo_pipeline::seek_input_sequence(int pos)
//{
//  if ( !input_sequence_->seek(pos) ) {
//    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", pos);
//    return false;
//  }
//  return true;
//}

bool c_virtual_stereo_pipeline::read_input_frame(cv::Mat & output_image, cv::Mat & output_mask)
{
  // lock_guard lock(mutex());

  if( !base::read_input_frame(_input_sequence, _input_options, output_image, output_mask, false, false) ) {
    CF_DEBUG("base::read_input_frame() fails");
    return false;
  }

  if( !output_image.empty() ) {
    // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.

    if( output_image.depth() == CV_32F || output_image.depth() == CV_64F ) {
      output_image.convertTo(output_image, CV_8U, 255);
    }
    else if( output_image.depth() == CV_16U ) {
      output_image.convertTo(output_image, CV_8U, 255. / 65535.);
    }
  }

  return true;
}

c_sparse_feature_extractor_and_matcher::sptr c_virtual_stereo_pipeline::create_keypoints_extractor() const
{
  return c_sparse_feature_extractor_and_matcher::create(_feature2d_options);
}

bool c_virtual_stereo_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  ///////////////////////////////////

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  const bool is_live_sequence =
      _input_sequence->is_live();

  if ( is_live_sequence ) {

    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;

  }
  else {

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);


    _total_frames = end_pos - start_pos;
    _processed_frames = 0;
    _accumulated_frames = 0;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
          _total_frames);
      return false;
    }

    if( !seek_input_sequence(start_pos) ) {
      CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
      return false;
    }
  }


  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if ( true ) {

      lock_guard lock(mutex());

      if( !read_input_frame(_current_image, _current_mask) ) {
        CF_DEBUG("read_input_frame() fails");
        break;
      }

      if ( _current_image.empty() ) {
        // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
        continue;
      }


      if( _input_options.input_image_processor ) {
        if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
          CF_ERROR("ERROR: input_processor->process() fails");
          return false;
        }
      }

      if( canceled() ) {
        break;
      }
    }

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( !write_progress_video() ) {
      CF_DEBUG("write_progress_video() fails");
      return false;
    }

    if ( !write_homography_video() ) {
      CF_DEBUG("write_homography_video() fails");
      return false;
    }

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( true ) {
      lock_guard lock(mutex());

      _accumulated_frames = _processed_frames;

      std::swap(_current_image, _previous_image);
      std::swap(_current_mask, _previous_mask);
      std::swap(_current_keypoints, _previous_keypoints);
      std::swap(_current_descriptors, _previous_descriptors);
    }
  }


  return true;
}

bool c_virtual_stereo_pipeline::process_current_frame()
{
  INSTRUMENT_REGION("");

  if ( !extract_keypoints() ) {
    CF_ERROR("extract_keypoints() fails");
    return false;
  }

  if ( !match_keypoints() ) {
    CF_ERROR("match_keypoints() fails");
    return false;
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
  }

  if ( !run_polar_stereo() ) {
    CF_ERROR("run_stereo_matcher() fails");
    return false;
  }

  if ( !run_epipolar_stereo() ) {
    CF_ERROR("run_epipolar_matcher() fails");
    return false;
  }

//  if ( !run_epipolar_flow() ) {
//    CF_ERROR("run_epipolar_flow() fails");
//    return false;
//  }

  if ( !run_morph_gradient_flow() ) {
    CF_ERROR("run_pyrflowlk2() fails");
    return false;
  }

  return true;
}

bool c_virtual_stereo_pipeline::extract_keypoints()
{
  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  if( _feature2d_options.matcher.type == FEATURE2D_MATCHER_OptFlowPyrLK ) {
    _keypoints_extractor->detect(_current_image, _current_keypoints, _current_mask);
  }
  else {
    _keypoints_extractor->detectAndCompute(_current_image, _current_mask,
        _current_keypoints, _current_descriptors);
  }


//  CF_DEBUG("current_keypoints_: %zu current_descriptors_: %dx%d depth=%d",
//      current_keypoints_.size(),
//      current_descriptors_.rows, current_descriptors_.cols,
//      current_descriptors_.depth());

  return true;
}


bool c_virtual_stereo_pipeline::match_keypoints()
{
  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  _matched_current_positions.clear();
  _matched_previous_positions.clear();

  if( !_keypoints_extractor->matcher() ) {

    if( !_current_keypoints.empty() && !_previous_image.empty() ) {

      const bool fOK =
          match_optflowpyrlk(_previous_image, _current_image,
              _previous_keypoints,
              _feature2d_options.matcher.optflowpyrlk,
              _matched_previous_positions,
              _matched_current_positions);

      if( !fOK ) {
        CF_ERROR("match_optflowpyrlk() fails");
        return false;
      }
    }
  }
  else if( !_current_keypoints.empty() && !_previous_keypoints.empty() ) {

    std::vector<cv::Point2f> cps, pps;

    const size_t num_matches =
        ::match_keypoints(_keypoints_extractor->matcher(),
            _current_keypoints,
            _current_descriptors,
            _previous_keypoints,
            _previous_descriptors,
            nullptr,
            &cps,
            &pps);

    for( int i = 0; i < num_matches; ++i ) {

      const cv::Point2f &cp = cps[i];
      const cv::Point2f &pp = pps[i];
//      const float d2 = (cp.x - pp.x) * (cp.x - pp.x) + (cp.y - pp.y) * (cp.y - pp.y);
//      if( d2 > 0 ) {
        _matched_current_positions.emplace_back(cp);
        _matched_previous_positions.emplace_back(pp);
//      }
    }
  }

  CF_DEBUG("%zu / %zu matches", _matched_current_positions.size(), _current_keypoints.size());

  return true;
}

bool c_virtual_stereo_pipeline::estmate_camera_pose()
{
  if( _matched_current_positions.size() < 6 || _matched_previous_positions.size() < 6 ) {
    return true; // ignore
  }

  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  _currentInliers.release();

  _currentEulerAnges = cv::Vec3d(0, 0, 0);
  _currentTranslationVector = cv::Vec3d(0, 0, 1);

  bool fOk =
      lm_camera_pose_and_derotation_homography(
          _camera_options.camera_intrinsics.camera_matrix,
          _matched_current_positions,
          _matched_previous_positions,
          _currentEulerAnges,
          _currentTranslationVector,
          &_currentRotationMatrix,
          &_currentEssentialMatrix,
          &_currentFundamentalMatrix,
          &_currentDerotationHomography,
          _currentInliers,
          &_camera_pose_options);

  if ( !fOk ) {
    CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
  }

//  CF_DEBUG("currentInliers_.size=%dx%d nz=%d",
//      currentInliers_.rows, currentInliers_.cols,
//      cv::countNonZero(currentInliers_));

  compute_epipoles(_currentFundamentalMatrix,
      _currentEpipoles);

  _currentEpipole =
      0.5 * (_currentEpipoles[0] + _currentEpipoles[1]);

  //  CF_DEBUG("A: (%g %g %g)", currentEulerAnges_(0) * 180 / CV_PI, currentEulerAnges_(1) * 180 / CV_PI, currentEulerAnges_(2) * 180 / CV_PI);
  //  CF_DEBUG("T: (%g %g %g)", currentTranslationVector_(0), currentTranslationVector_(1), currentTranslationVector_(2));
  //  CF_DEBUG("E: (%g %g) EE: {%+g %+g} {%+g %+g}", currentEpipole_.x, currentEpipole_.y,
  //      currentEpipoles_[0].x, currentEpipoles_[0].y, currentEpipoles_[1].x, currentEpipoles_[1].y);

  if ( distance_between_points(_currentEpipoles[0], _currentEpipoles[1]) > 1 ) {
    CF_WARNING("\nWARNING!\n"
        "Something looks POOR: Computed epipoles differ after derotation:\n"
        "E0 = {%g %g}\n"
        "E1 = {%g %g}\n",
        _currentEpipoles[0].x, _currentEpipoles[0].y,
        _currentEpipoles[1].x, _currentEpipoles[1].y);
  }

  return true;
}

bool c_virtual_stereo_pipeline::run_polar_stereo()
{
  if( !_output_options.save_polar_frames && !_stereo_matcher.enabled() ) {
    return true; // ignore
  }

  cv::Mat frames[2];
  cv::Mat masks[2];
  cv::Mat2f inverse_remap;

  if( !create_stereo_frames(frames, masks, &inverse_remap) ) {
    return true; // ignore
  }

  // Optionally save polar frames if requested by user
  if( _output_options.save_polar_frames ) {

    cv::Mat display;

    const cv::Size dst_size(frames[0].cols + frames[1].cols,
        std::max(frames[0].rows, frames[1].rows));

    const cv::Rect dst_roi[2] = {
        cv::Rect(0, 0, frames[0].cols, frames[0].rows),
        cv::Rect(frames[0].cols, 0, frames[1].cols, frames[1].rows),
    };

    display.create(dst_size, frames[0].type());
    frames[0].copyTo(display(dst_roi[0]));
    frames[1].copyTo(display(dst_roi[1]));

    if( !_polar_frames_writer.is_open() ) {

      const std::string output_video_filename =
          generate_output_filename(_output_options.polar_frames_filename,
              "polar",
              ".png");

      bool fOK =
          _polar_frames_writer.open(output_video_filename);

      if( !fOK ) {
        CF_ERROR("polar_frames_writer_.open('%s') fails",
            output_video_filename.c_str());
        return false;
      }

      CF_DEBUG("Created '%s' display.size()=%dx%d",
          output_video_filename.c_str(),
          display.cols,
          display.rows);
    }

    if( !_polar_frames_writer.write(display, cv::noArray(), false, 0) ) {
      CF_ERROR("polar_frames_writer_.write() fails: %s",
          _progress_video_writer.filename().c_str());
      return false;
    }
  }

  // compute stereo disparity if requested by user
  if( _stereo_matcher.enabled() ) {

    if( !_stereo_matcher.compute(frames[0], frames[1], _currentDisparity) ) {
      CF_ERROR("stereo_matcher_.compute() fails");
      return false;
    }

    cv::remap(_currentDisparity, _currentDisparity,
        inverse_remap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(-1));

    if( _output_options.save_disparity_frames && !_currentDisparity.empty() ) {

      if( !_disparity_frames_writer.is_open() ) {

        const std::string output_video_filename =
            generate_output_filename(_output_options.disparity_frames_filename,
                "disparity",
                ".tiff");

        bool fOK =
            _disparity_frames_writer.open(output_video_filename);

        if( !fOK ) {
          CF_ERROR("disparity_frames_writer_.open('%s') fails",
              output_video_filename.c_str());
          return false;
        }

        CF_DEBUG("Created '%s' display.size()=%dx%d",
            output_video_filename.c_str(),
            _currentDisparity.cols,
            _currentDisparity.rows);
      }

      if( !_disparity_frames_writer.write(_currentDisparity, cv::noArray(), false, 0) ) {
        CF_ERROR("disparity_frames_writer_.write() fails: %s",
            _disparity_frames_writer.filename().c_str());
        return false;
      }
    }

    if( !run_triangulation(ssprintf("%s/polar_stereo", _output_path.c_str())) ) {
      CF_ERROR("run_triangulation() fails");
      return false;
    }

  }

  return true;
}

bool c_virtual_stereo_pipeline::run_triangulation(const std::string & output_directory)
{
  if ( !_triangulation_options.enable_triangulation ) {
    return true; // disabled, silently ignore
  }

  if ( _currentDisparity.empty() ) {
    // probably disabled by stereo matcher options, silently ignore
    return true;
  }

//  CF_DEBUG("current_disparity_: %dx%d depth=%d",
//      current_disparity_.cols,
//      current_disparity_.rows,
//      current_disparity_.depth());

  const cv::Matx33d inverted_camera_matrix =
      _camera_options.camera_intrinsics.camera_matrix.inv();

  // current epipole location in pixel coordinates on reference frame
  const cv::Point2d & E =
      _currentEpipole;

  // Unit vector to epipole direction in camera coordinates
  const cv::Vec3d epipole_direction_in_camera_coordinates =
      pix2cam(E.x, E.y,
          inverted_camera_matrix,
          true);

  const cv::Mat1f disparity =
      _currentDisparity;

  double depth;
  cv::Vec3d pos3d;
  std::vector<cv::Vec3d> cloud3d;
  std::vector<cv::Vec3b> colors3d;
  cv::Mat3b refimg;


  if ( _previous_image.type() == CV_8UC3 ) {
    refimg = _previous_image;
  }
  else if ( _previous_image.type() == CV_8UC1 ) {
    cv::cvtColor(_previous_image, refimg, cv::COLOR_BGR2GRAY);
  }
  else {
    _previous_image.convertTo(refimg, refimg.depth());
    cv::cvtColor(refimg, refimg, cv::COLOR_BGR2GRAY);
  }

  cloud3d.reserve(_currentDisparity.size().area());

  for ( int y = 0; y < _currentDisparity.rows; ++y ) {
    for ( int x = 0; x < _currentDisparity.cols; ++x ) {
      if ( disparity[y][x] > 0 ) {

        const double epipolar_distance = hypot(x - E.x, y - E.y);
        const double sina = (y - E.y ) / epipolar_distance;
        const double cosa = (x - E.x ) / epipolar_distance;
        const double current_x = (epipolar_distance + disparity[y][x]) * cosa + E.x;
        const double current_y = (epipolar_distance + disparity[y][x]) * sina + E.y;

        bool fOk =
            triangulate_point(x, y,
                current_x, current_y,
                inverted_camera_matrix,
                epipole_direction_in_camera_coordinates,
                _triangulation_options.baseline,
                &depth,
                &pos3d);

        if ( fOk ) {
          cloud3d.emplace_back(pos3d);
          colors3d.emplace_back(refimg[y][x]);
        }
      }
    }
  }

  CF_DEBUG("cloud3d.size=%zu", cloud3d.size());

  const std::string output_filename =
      ssprintf("%s/cloud3d/cloud3d.%05d.ply",
          output_directory.c_str(),
          _input_sequence->current_pos()-1);

  if( !save_ply(cloud3d, colors3d, output_filename) ) {
    CF_ERROR("save_ply('%s') fails", output_filename.c_str());
    return false;
  }

  return true;
}


bool c_virtual_stereo_pipeline::run_epipolar_stereo()
{
  if ( !_epipolar_matcher.enabled() ||  _current_image.empty() || _previous_image.empty() ) {
    return true; // ignore
  }

  if( !_epipolar_matcher.options().enable_debug ) {
    _epipolar_matcher.set_debug_path("");
  }
  else {
    _epipolar_matcher.set_debug_path(ssprintf("%s/epipolar_debug/frame%05d", _output_path.c_str(),
        _input_sequence->current_pos() - 1));
  }

  bool fOK =
      _epipolar_matcher.match(_current_image, _current_mask,
          _previous_image, _previous_mask,
          _currentDerotationHomography,
          _currentEpipole);

  if( !fOK ) {
    CF_ERROR("epipolar_matcher_.match() fails");
    return false;
  }

  return true;
}
//
//bool c_virtual_stereo_pipeline::run_epipolar_flow()
//{
//  if ( !epipolar_flow_options_.enabled ||  current_image_.empty() || previous_image_.empty() ) {
//    return true; // ignore
//  }
//
//  if( !epipolar_flow_options_.enable_debug ) {
//    epipolar_flow_.set_debug_path("");
//  }
//  else {
//    epipolar_flow_.set_debug_path(ssprintf("%s/epipolar_flow/frame%05d", output_path_.c_str(),
//        input_sequence_->current_pos() - 1));
//  }
//
//
//
//
//  cv::Mat current_image = current_image_;
//  cv::Mat reference_image = previous_image_;
//  cv::Mat input_mask = current_mask_;
//  cv::Mat reference_mask = previous_mask_;
//  cv::Mat2f rmap;
//  cv::Mat current_derotated_image;
//
//  c_homography_image_transform image_transform;
//  image_transform.set_homography_matrix(currentDerotationHomography_.inv());
//  image_transform.create_remap(rmap, reference_image.size());
//
//  cv::remap(current_image_, current_derotated_image, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//
//  if ( output_options_.save_epipolar_flow_debug_images ) {
//
//    const std::string outputpath =
//        ssprintf("%s/epipolar_flow/remaps%03d",
//            output_path_.c_str(),
//            input_sequence_->current_pos() - 1);
//
//    std::string filename;
//
//    if( !save_image(current_image, filename = ssprintf("%s/current_image.png", outputpath.c_str())) ) {
//      CF_ERROR("save_image('%s') fails", filename.c_str());
//      return false;
//    }
//
//    if( !save_image(reference_image, filename = ssprintf("%s/reference_image.png", outputpath.c_str())) ) {
//      CF_ERROR("save_image('%s') fails", filename.c_str());
//      return false;
//    }
//
//    if( !save_image(current_derotated_image, filename = ssprintf("%s/current_derotated_image.png", outputpath.c_str())) ) {
//      CF_ERROR("save_image('%s') fails", filename.c_str());
//      return false;
//    }
//
//  }
//
//  epipolar_flow_.set_epipole(currentEpipole_);
//  rmap.release();
//
//  bool fOK =
//    epipolar_flow_.compute(current_derotated_image, reference_image, rmap,
//        input_mask, reference_mask);
//
//  if( !fOK ) {
//    CF_ERROR("epipolar_flow_.compute() fails");
//    return false;
//  }
//
//  if ( output_options_.save_epipolar_flow_debug_images ) {
//
//    const std::string outputpath =
//        ssprintf("%s/epipolar_flow/remaps%03d",
//            output_path_.c_str(),
//            input_sequence_->current_pos() - 1);
//
//    std::string filename;
//
//    cv::Mat current_derotated_image_remapped_to_reference;
//    cv::remap(current_derotated_image, current_derotated_image_remapped_to_reference, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//
//    if( !save_image(current_derotated_image_remapped_to_reference,
//        filename = ssprintf("%s/current_derotated_image_remapped_to_reference.png", outputpath.c_str())) ) {
//      CF_ERROR("save_image('%s') fails", filename.c_str());
//      return false;
//    }
//
//  }
//
//  return true;
//}

bool c_virtual_stereo_pipeline::run_morph_gradient_flow()
{
  if ( !_morph_gradient_flow_options.enabled ||  _current_image.empty() || _previous_image.empty() ) {
    return true; // ignore
  }

  cv::Mat current_image, current_mask;
  cv::Mat previous_image, previous_mask;
  cv::Mat derotated_current_image, derotated_current_mask;
  cv::Mat1f disp;
  cv::Mat1f cost;

  const std::string debug_path =
      _morph_gradient_flow_options.enable_debug ?
          ssprintf("%s/morph_gradient_flow/%03d",
              _output_path.c_str(), _input_sequence->current_pos() - 1) :
          "";


  current_image = _current_image;
  current_mask = _current_mask;
  previous_image = _previous_image;
  previous_mask = _previous_mask;

  cv::warpPerspective(current_image, derotated_current_image,
      _currentDerotationHomography,
      current_image.size(),
      cv::INTER_LINEAR,
      cv::BORDER_REPLICATE);

  if ( !current_mask.empty() ) {

    cv::warpPerspective(current_mask, derotated_current_mask,
        _currentDerotationHomography,
        current_image.size(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

    cv::compare(derotated_current_mask, 255,
        derotated_current_mask,
        cv::CMP_GE);
  }

  bool fOK =
      morph_gradient_flow(derotated_current_image, derotated_current_mask,
          previous_image, previous_mask,
          _morph_gradient_flow_options.max_pyramid_level,
          _morph_gradient_flow_options.block_radius,
          _morph_gradient_flow_options.search_radius,
          _morph_gradient_flow_options.alpha,
          _morph_gradient_flow_options.beta,
          _morph_gradient_flow_options.gradient_threshold,
          _currentEpipole,
          disp,
          cost,
          debug_path);

  if( !fOK ) {
    CF_ERROR("morph_gradient_flow() fails");
    return false;
  }

  _currentDisparity = disp;

  if( !run_triangulation(ssprintf("%s/morph_gradient_flow", _output_path.c_str())) ) {
    CF_ERROR("run_triangulation() fails");
    return false;
  }

  return true;
}


bool c_virtual_stereo_pipeline::create_stereo_frames(cv::Mat frames[2], cv::Mat masks[2], cv::Mat2f * inverse_remap)
{
  if( _current_image.empty() || _previous_image.empty() ) {
    return false;
  }

  INSTRUMENT_REGION("");

  cv::Mat2f rmaps[2];

  const cv::Size src_size =
      _current_image.size();

  create_epipolar_remaps(src_size,
      _currentEpipole,
      _currentDerotationHomography.inv(),
      rmaps[0],
      rmaps[1],
      nullptr,
      inverse_remap);

  const cv::Size dst_size(rmaps[0].cols * 2,
      rmaps[0].rows);

  cv::remap(_current_image, frames[0],
      rmaps[0], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  cv::remap(_previous_image, frames[1],
      rmaps[1], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  const cv::Mat current_mask =
      _current_mask.empty() ?
          cv::Mat1b(_current_image.size(), 255) :
          _current_mask;

  const cv::Mat previous_mask =
      _previous_mask.empty() ?
          cv::Mat1b(_previous_image.size(), 255) :
          _previous_mask;

  cv::remap(current_mask, masks[0],
      rmaps[0], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  cv::remap(previous_mask, masks[1],
      rmaps[1], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  return true;
}


bool c_virtual_stereo_pipeline::create_homography_display(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( _current_image.empty() || _previous_image.empty() ) {
    return false;
  }

  c_homography_image_transform homography;
  //c_homography_ecc_motion_model model(&homography);
  //c_ecc_forward_additive ecc(&homography);
  c_ecch ecch(&homography);

  cv::Mat cimg, pimg;

  if ( _current_image.channels() == 1 ) {
    cimg = _current_image;
  }
  else {
    cv::cvtColor(_current_image, cimg, cv::COLOR_BGR2GRAY);
  }

  if ( _previous_image.channels() == 1 ) {
    pimg = _previous_image;
  }
  else {
    cv::cvtColor(_previous_image, pimg, cv::COLOR_BGR2GRAY);
  }

  ecch.set_minimum_image_size(64);
  ecch.set_maxlevel(-1);

  if ( !estimate_image_transform(&homography, _matched_current_positions, _matched_previous_positions) ) {
    CF_ERROR("estimate_image_transform() fails");
  }

  if ( !ecch.set_reference_image(pimg, _previous_mask) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  if ( !ecch.align(cimg, _current_mask) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  display_frame.create(_previous_image.size(), _previous_image.type());

  cv::Mat & dst_image = display_frame.getMatRef();
  cv::remap(_current_image, dst_image, ecch.create_remap(), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::addWeighted(dst_image, 0.5, _previous_image, 0.5, 0, dst_image);

  return true;
}


void c_virtual_stereo_pipeline::draw_matched_positions(cv::Mat & image,
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
    cv::rectangle(image, cv::Point(cp.x - 1, cp.y - 1), cv::Point(cp.x + 1, cp.y + 1), color, 1, cv::LINE_4);
  }
}
;


bool c_virtual_stereo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  //////////////////

  lock_guard lock(mutex());

  if( _current_image.empty() || _previous_image.empty() ) {
    return false;
  }

  const cv::Size size = _current_image.size();
  const cv::Size dst_size(2 * size.width, 2 * size.height);

  const cv::Rect cRoi(0, 0, size.width, size.height);
  const cv::Rect wRoi(0, size.height, size.width, size.height);
  const cv::Rect pRoi(size.width, 0, size.width, size.height);
  const cv::Rect dRoi(size.width, size.height, size.width, size.height);

  cv::Mat3b cimg(dst_size);

  if ( _current_image.channels() == 3 ) {
    _current_image.copyTo(cimg(cRoi));
  }
  else {
    cv::cvtColor(_current_image, cimg(cRoi), cv::COLOR_GRAY2BGR);
  }

  if ( !_matched_previous_positions.empty() ) {

    cv::Mat img = cimg(cRoi);
    draw_matched_positions(img,
        _matched_current_positions,
        _matched_previous_positions,
        _currentInliers);
  }

  cv::Mat tmp1, tmp2;
  std::vector<cv::Point2f> warped_current_positions_;

  /////////////////
  if( !_matched_current_positions.empty() ) {
    cv::perspectiveTransform(_matched_current_positions, warped_current_positions_,
        _currentDerotationHomography);
  }

  cv::warpPerspective(_current_image, tmp1,
      _currentDerotationHomography,
      pRoi.size(),
      cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
      cv::BORDER_CONSTANT);

  if ( tmp1.channels() == 1 ) {
    cv::cvtColor(tmp1, tmp1, cv::COLOR_GRAY2BGR);
  }


  /////////////////

  if ( _previous_image.channels() == 3 ) {
    _previous_image.copyTo(tmp2);
  }
  else {
    cv::cvtColor(_previous_image, tmp2, cv::COLOR_GRAY2BGR);
  }

  cv::addWeighted(tmp1, 0.5, tmp2, 0.5, 0, tmp2);


  /////////////////


  if( !warped_current_positions_.empty() && !_matched_previous_positions.empty() ) {

    draw_matched_positions(tmp1,
        warped_current_positions_,
        _matched_previous_positions,
        _currentInliers);
  }

  if( IS_INSIDE_IMAGE(_currentEpipole, tmp1.size()) ) {

    const cv::Point2f E(_currentEpipole.x, _currentEpipole.y);

    cv::ellipse(tmp1, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
    cv::line(tmp1, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(tmp1, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }

  if( IS_INSIDE_IMAGE(_currentEpipole, tmp2.size()) ) {

    const cv::Point2f E(_currentEpipole.x, _currentEpipole.y);

    cv::ellipse(tmp2, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
    cv::line(tmp2, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(tmp2, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }

  /////////////////


  tmp1.copyTo(cimg(wRoi));
  tmp2.copyTo(cimg(pRoi));

  if ( _currentDisparity.empty() ) {
    cimg(dRoi).setTo(0);
  }
  else {
    _currentDisparity.convertTo(tmp1, CV_8U, 255. / _stereo_matcher.currentMaxDisparity());
    apply_colormap(tmp1, cimg(dRoi), COLORMAP_TURBO);
  }


  /////////////////

  display_frame.move(cimg);

  if ( display_mask.needed() ) {
    _current_mask.copyTo(display_mask);
  }

  return true;
}

bool c_virtual_stereo_pipeline::write_progress_video()
{
  if ( !_output_options.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !_progress_video_writer.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(_output_options.progress_video_filename,
            "progress",
            ".avi");

    bool fOK =
        _progress_video_writer.open(output_video_filename);

    if( !fOK ) {
      CF_ERROR("progress_video_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !_progress_video_writer.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("progress_video_writer_.write() fails: %s",
        _progress_video_writer.filename().c_str());
    return false;
  }

  return true;
}


bool c_virtual_stereo_pipeline::write_homography_video()
{
  if ( !_output_options.save_homography_video ) {
    return true;
  }

  cv::Mat display;

  if ( !create_homography_display(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !_homography_video_writer.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(_output_options.homography_video_filename,
            "homography",
            ".avi");

    bool fOK =
        _homography_video_writer.open(output_video_filename);

    if( !fOK ) {
      CF_ERROR("homography_video_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !_homography_video_writer.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("homography_video_writer_.write() fails: %s",
        _homography_video_writer.filename().c_str());
    return false;
  }

  return true;
}


