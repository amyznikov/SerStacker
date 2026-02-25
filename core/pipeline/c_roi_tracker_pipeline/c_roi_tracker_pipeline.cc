/*
 * c_roi_tracker_pipeline.cc
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#include "c_roi_tracker_pipeline.h"
#include <core/ssprintf.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

c_roi_tracker_pipeline::c_roi_tracker_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options() const
{
  return _input_options;
}

c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options()
{
  return _input_options;
}

const c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options() const
{
  return _tracker_options;
}

c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options()
{
  return _tracker_options;
}

const c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options() const
{
  return _output_options;
}

c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options()
{
  return _output_options;
}


bool c_roi_tracker_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "tracker_options")) ) {
    SERIALIZE_OPTION(section, save, _tracker_options, roi);
    ::serialize(_tracker_options.tracker, settings, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    SERIALIZE_OPTION(section, save, _output_options, progress_video_filename);
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_input_options> & ctx)
{
  using S = c_roi_tracker_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_options> & ctx)
{
  using S = c_roi_tracker_options;

  ctlbind(ctls, "Enable tracking", ctx(&S::enabled), "");

  ctlbind_group(ctls, ctx(&S::enabled));

    ctlbind(ctls, "Tracker type", ctx(&S::tracker_type),  "Select tracker type to use");

    if ( true ) {
      using S1 = std::decay_t<decltype(S::boosting)>;
      const auto gctx = ctx(&S::boosting);

      ctlbind_expandable_group(ctls, "BOOSTING", "");
        ctlbind(ctls, "numClassifiers", gctx(&S1::numClassifiers), "the number of classifiers to use in a OnlineBoosting algorithm");
        ctlbind(ctls, "samplerOverlap", gctx(&S1::samplerOverlap), "search region parameters to use in a OnlineBoosting algorithm");
        ctlbind(ctls, "samplerSearchFactor", gctx(&S1::samplerSearchFactor), "search region parameters to use in a OnlineBoosting algorithm");
        ctlbind(ctls, "iterationInit", gctx(&S1::iterationInit), "the initial iterations");
        ctlbind(ctls, "featureSetNumFeatures", gctx(&S1::featureSetNumFeatures), "# features");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      using S1 = std::decay_t<decltype(S::mil)>;
      const auto gctx = ctx(&S::mil);

      ctlbind_expandable_group(ctls, "MIL", "");
        ctlbind(ctls, "samplerInitInRadius", gctx(&S1::samplerInitInRadius), "radius for gathering positive instances during init");
        ctlbind(ctls, "samplerInitMaxNegNum", gctx(&S1::samplerInitMaxNegNum), "# negative samples to use during init");
        ctlbind(ctls, "samplerSearchWinSize", gctx(&S1::samplerSearchWinSize), "size of search window");
        ctlbind(ctls, "samplerTrackInRadius", gctx(&S1::samplerTrackInRadius), "radius for gathering positive instances during tracking");
        ctlbind(ctls, "samplerTrackMaxPosNum", gctx(&S1::samplerTrackMaxPosNum), "# positive samples to use during tracking");
        ctlbind(ctls, "samplerTrackMaxNegNum", gctx(&S1::samplerTrackMaxNegNum), "# negative samples to use during tracking");
        ctlbind(ctls, "featureSetNumFeatures", gctx(&S1::featureSetNumFeatures), "# features");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      using S1 = std::decay_t<decltype(S::kcf)>;
      const auto gctx = ctx(&S::kcf);

      ctlbind_expandable_group(ctls, "KCF", "");
        ctlbind(ctls, "detect_thresh", gctx(&S1::detect_thresh), "detection confidence threshold");
        ctlbind(ctls, "sigma", gctx(&S1::sigma), "gaussian kernel bandwidth");
        ctlbind(ctls, "lambda", gctx(&S1::lambda), "regularization");
        ctlbind(ctls, "interp_factor", gctx(&S1::interp_factor), "linear interpolation factor for adaptation");
        ctlbind(ctls, "output_sigma_factor", gctx(&S1::output_sigma_factor), "spatial bandwidth (proportional to target)");
        ctlbind(ctls, "pca_learning_rate", gctx(&S1::pca_learning_rate), "compression learning rate");
        ctlbind(ctls, "resize", gctx(&S1::resize), "activate the resize feature to improve the processing speed");
        ctlbind(ctls, "split_coeff", gctx(&S1::split_coeff), "split the training coefficients into two matrices");
        ctlbind(ctls, "wrap_kernel", gctx(&S1::wrap_kernel), "wrap around the kernel values");
        ctlbind(ctls, "compress_feature", gctx(&S1::compress_feature), "activate the pca method to compress the features");
        ctlbind(ctls, "max_patch_size", gctx(&S1::max_patch_size), "threshold for the ROI size");
        ctlbind(ctls, "compressed_size", gctx(&S1::compressed_size), "feature size after compression");
        ctlbind(ctls, "desc_pca", gctx(&S1::desc_pca), " compressed descriptors of TrackerKCF::MODE");
        ctlbind(ctls, "desc_npca", gctx(&S1::desc_npca), "non-compressed descriptors of TrackerKCF::MODE");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      using S1 = std::decay_t<decltype(S::tld)>;
      const auto gctx = ctx(&S::tld);
      ctlbind_expandable_group(ctls, "TLD", "");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      using S1 = std::decay_t<decltype(S::medianflow)>;
      const auto gctx = ctx(&S::medianflow);

      ctlbind_expandable_group(ctls, "MEDIANFLOW", "");
        ctlbind(ctls, "pointsInGrid", gctx(&S1::pointsInGrid), "square root of number of keypoints used; increase it to trade accurateness for speed");
        ctlbind(ctls, "winSize", gctx(&S1::winSize), "window size parameter for Lucas-Kanade optical flow");
        ctlbind(ctls, "maxLevel", gctx(&S1::maxLevel), "maximal pyramid level number for Lucas-Kanade optical flow");
        ctlbind(ctls, gctx(&S1::termCriteria));
        ctlbind(ctls, "winSizeNCC", gctx(&S1::winSizeNCC), "window size around a point for normalized cross-correlation check");
        ctlbind(ctls, "maxMedianLengthOfDisplacementDifference", gctx(&S1::maxMedianLengthOfDisplacementDifference), "criterion for loosing the tracked object");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      ctlbind_expandable_group(ctls, "MOSSE", "");
      ctlbind_end_group(ctls);
    }

    if ( true ) {
      using S1 = std::decay_t<decltype(S::csrt)>;
      const auto gctx = ctx(&S::csrt);
      ctlbind_expandable_group(ctls, "CSRT", "");
      ctlbind(ctls, "use_hog", gctx(&S1::use_hog), "");
      ctlbind(ctls, "use_color_names", gctx(&S1::use_color_names), "");
      ctlbind(ctls, "use_gray", gctx(&S1::use_gray), "");
      ctlbind(ctls, "use_rgb", gctx(&S1::use_rgb),  "");
      ctlbind(ctls, "use_channel_weights", gctx(&S1::use_channel_weights),  "");
      ctlbind(ctls, "use_segmentation", gctx(&S1::use_segmentation),  "");
      ctlbind(ctls, "window_function", gctx(&S1::window_function),  "Window function: hann, cheb, kaiser");
      ctlbind(ctls, "kaiser_alpha", gctx(&S1::kaiser_alpha),  "");
      ctlbind(ctls, "cheb_attenuation", gctx(&S1::cheb_attenuation),  "");
      ctlbind(ctls, "template_size", gctx(&S1::template_size),  "");
      ctlbind(ctls, "gsl_sigma", gctx(&S1::gsl_sigma),  "");
      ctlbind(ctls, "hog_orientations", gctx(&S1::hog_orientations),  "");
      ctlbind(ctls, "hog_clip", gctx(&S1::hog_clip),  "");
      ctlbind(ctls, "padding", gctx(&S1::padding),  "");
      ctlbind(ctls, "filter_lr", gctx(&S1::filter_lr),  "");
      ctlbind(ctls, "weights_lr", gctx(&S1::weights_lr),  "");
      ctlbind(ctls, "num_hog_channels_used", gctx(&S1::num_hog_channels_used),  "");
      ctlbind(ctls, "admm_iterations", gctx(&S1::admm_iterations),  "");
      ctlbind(ctls, "histogram_bins", gctx(&S1::histogram_bins),  "");
      ctlbind(ctls, "histogram_lr", gctx(&S1::histogram_lr),  "");
      ctlbind(ctls, "background_ratio", gctx(&S1::background_ratio),  "");
      ctlbind(ctls, "number_of_scales", gctx(&S1::number_of_scales),  "");
      ctlbind(ctls, "scale_sigma_factor", gctx(&S1::scale_sigma_factor),  "");
      ctlbind(ctls, "scale_model_max_area", gctx(&S1::scale_model_max_area),  "");
      ctlbind(ctls, "scale_lr", gctx(&S1::scale_lr),  "");
      ctlbind(ctls, "scale_step", gctx(&S1::scale_step),  "");
      ctlbind(ctls, "psr_threshold", gctx(&S1::psr_threshold),  "we lost the target, if the psr is lower than this");
      ctlbind_end_group(ctls);
    }

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
    if ( true ) {
      using S1 = std::decay_t<decltype(S::goturn)>;
      const auto gctx = ctx(&S::goturn);
      ctlbind_expandable_group(ctls, "GOTURN", "");
      ctlbind_end_group(ctls);
    }
#endif

  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_pipeline_options> & ctx)
{
  using S = c_roi_tracker_pipeline_options;

  ctlbind(ctls, "ROI Rect X;Y;WxH;", ctx(&S::roi), "");
  ctlbind(ctls, ctx(&S::tracker));
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_output_options> & ctx)
{
  using S = c_roi_tracker_output_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));

  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
  ctlbind_group(ctls, ctx(&S::save_progress_video));
    ctlbind_browse_for_file(ctls, "progress_video_filename", ctx(&S::progress_video_filename), "");
  ctlbind_end_group(ctls);
}



const c_ctlist<c_roi_tracker_pipeline> & c_roi_tracker_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "2. ROI tracking", "");
      ctlbind(ctls, ctx(&this_class::_tracker_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "3. Output options", "");
    ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}


//const std::vector<c_image_processing_pipeline_ctrl> & c_roi_tracker_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
//
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "ROI tracking", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.enabled, "enable tracker", "");
////      PIPELINE_CTL(ctrls, _tracker_options.roi, "ROI Rect X;Y;WxH;", "");
////
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.tracker_type,  "Tracker type", "Select tarcker type to use");
////
////      PIPELINE_CTL_GROUP(ctrls, "BOOSTING", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.boosting.numClassifiers, "numClassifiers", "the number of classifiers to use in a OnlineBoosting algorithm");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.boosting.samplerOverlap, "samplerOverlap", "search region parameters to use in a OnlineBoosting algorithm");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.boosting.samplerSearchFactor, "samplerSearchFactor", "search region parameters to use in a OnlineBoosting algorithm");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.boosting.iterationInit, "iterationInit", "the initial iterations");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.boosting.featureSetNumFeatures, "featureSetNumFeatures", "# features");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "MIL", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerInitInRadius, "samplerInitInRadius", "radius for gathering positive instances during init");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerInitMaxNegNum, "samplerInitMaxNegNum", "# negative samples to use during init");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerSearchWinSize, "samplerSearchWinSize", "size of search window");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerTrackInRadius, "samplerTrackInRadius", "radius for gathering positive instances during tracking");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerTrackMaxPosNum, "samplerTrackMaxPosNum", "# positive samples to use during tracking");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.samplerTrackMaxNegNum, "samplerTrackMaxNegNum", "# negative samples to use during tracking");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.mil.featureSetNumFeatures, "featureSetNumFeatures", "# features");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "KCF", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.detect_thresh, "detect_thresh", "detection confidence threshold");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.sigma, "sigma", "gaussian kernel bandwidth");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.lambda, "lambda", "regularization");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.interp_factor, "interp_factor", "linear interpolation factor for adaptation");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.output_sigma_factor, "output_sigma_factor", "spatial bandwidth (proportional to target)");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.pca_learning_rate, "pca_learning_rate", "compression learning rate");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.resize, "resize", "activate the resize feature to improve the processing speed");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.split_coeff, "split_coeff", "split the training coefficients into two matrices");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.wrap_kernel, "wrap_kernel", "wrap around the kernel values");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.compress_feature, "compress_feature", "activate the pca method to compress the features");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.max_patch_size, "max_patch_size", "threshold for the ROI size");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.compressed_size, "compressed_size", "feature size after compression");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.desc_pca, "desc_pca", " compressed descriptors of TrackerKCF::MODE");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.kcf.desc_npca, "desc_npca", "non-compressed descriptors of TrackerKCF::MODE");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "TLD", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "MEDIANFLOW", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.pointsInGrid, "pointsInGrid", "square root of number of keypoints used; increase it to trade accurateness for speed");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.winSize, "winSize", "window size parameter for Lucas-Kanade optical flow");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.maxLevel, "maxLevel", "maximal pyramid level number for Lucas-Kanade optical flow");
////      PIPELINE_CTL_BITFLAGS(ctrls, _tracker_options.tracker.medianflow.termCriteria.type, cv::TermCriteria::Type, "termCriteria.type", "termination criteria for Lucas-Kanade optical flow");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.termCriteria.maxCount, "termCriteria.COUNT", "termination criteria for Lucas-Kanade optical flow");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.termCriteria.epsilon, "termCriteria.epsilon", "termination criteria for Lucas-Kanade optical flow");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.winSizeNCC, "winSizeNCC", "window size around a point for normalized cross-correlation check");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.medianflow.maxMedianLengthOfDisplacementDifference, "maxMedianLengthOfDisplacementDifference", "criterion for loosing the tracked object");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "MOSSE", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "CSRT", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_hog, "use_hog", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_color_names, "use_color_names", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_gray, "use_gray", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_rgb, "use_rgb", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_channel_weights, "use_channel_weights", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.use_segmentation, "use_segmentation", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.window_function, "window_function", "Window function: hann, cheb, kaiser");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.kaiser_alpha, "kaiser_alpha", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.cheb_attenuation, "cheb_attenuation", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.template_size, "template_size", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.gsl_sigma, "gsl_sigma", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.hog_orientations, "hog_orientations", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.hog_clip, "hog_clip", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.padding, "padding", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.filter_lr, "filter_lr", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.weights_lr, "weights_lr", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.num_hog_channels_used, "num_hog_channels_used", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.admm_iterations, "admm_iterations", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.histogram_bins, "histogram_bins", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.histogram_lr, "histogram_lr", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.background_ratio, "background_ratio", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.number_of_scales, "number_of_scales", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.scale_sigma_factor, "scale_sigma_factor", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.scale_model_max_area, "scale_model_max_area", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.scale_lr, "scale_lr", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.scale_step, "scale_step", "");
////      PIPELINE_CTL(ctrls, _tracker_options.tracker.csrt.psr_threshold, "psr_threshold", "we lost the target, if the psr is lower than this");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
////      PIPELINE_CTL_GROUP(ctrls, "GOTURN", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////#endif
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////      PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, _output_options.output_directory, "output_directory", "");
////      PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
////      PIPELINE_CTLC(ctrls, _output_options.progress_video_filename, "progress_video_filename", "", _this->_output_options.save_progress_video);
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}
//
bool c_roi_tracker_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _output_path =
      create_output_path(_output_options.output_directory);

  if( _tracker_options.tracker.enabled ) {

    if( (_objbox = _tracker_options.roi).empty() ) {
      CF_ERROR("INPUT ERROR: Tracking ROI is empty: x=%d y=%d w=%d h=%d ",
          _objbox.x, _objbox.y,
          _objbox.width, _objbox.height);
      return false;
    }

    if( !_tracker.initialize(_tracker_options.tracker) ) {
      CF_ERROR("tracker_.initialize() fails");
      return false;
    }
  }

  return true;
}

void c_roi_tracker_pipeline::cleanup_pipeline()
{
  if ( _input_sequence ) {
    _input_sequence->close();
  }

  if ( _progress_writer.is_open() ) {
    CF_DEBUG("Closing '%s'", _progress_writer.filename().c_str());
    _progress_writer.close();
  }

  _tracker.release();
}

bool c_roi_tracker_pipeline::run_pipeline()
{
  if( !_input_sequence ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if ( !_input_sequence->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      _input_sequence->is_live();

  if( is_live_sequence ) {
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
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          _total_frames,
          start_pos,
          end_pos,
          _input_sequence->size(),
          _input_options.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !_input_sequence->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !_input_sequence->read(_current_image, &_current_mask) ) {
        CF_DEBUG("input_sequence_->read() fails");
        break;
      }
    }

    if( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {
      lock_guard lock(mutex());
      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++_accumulated_frames;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}




bool c_roi_tracker_pipeline::process_current_frame()
{
  if ( _tracker_options.tracker.enabled ) {

    bool hasUpdates = false;

    if ( !_tracker.track(_current_image, _objbox, &hasUpdates) ) {
      CF_ERROR("tracker_.track() fails");
      return false;
    }


    CF_DEBUG("hasUpdates: %d objbox_: %d;%d %dx%d", hasUpdates,
        _objbox.x, _objbox.y, _objbox.width, _objbox.height);

  }

  if ( !write_progress_video() ) {
    CF_ERROR("write_progress_video() fails");
    return false;
  }

  return true;
}

bool c_roi_tracker_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( _current_image.empty() ) {
    return false;
  }

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, display_frame, cv::COLOR_GRAY2BGR);
  }
  else {
    _current_image.copyTo(display_frame);
  }

  if ( _tracker_options.tracker.enabled ) {
    cv::rectangle(display_frame.getMatRef(), _objbox,
        CV_RGB(32, 255, 64), 1, cv::LINE_8);
  }

  return true;
}

bool c_roi_tracker_pipeline::write_progress_video()
{
  if ( !_output_options.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !_progress_writer.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(_output_options.progress_video_filename,
            "progress",
            ".avi");

    bool fOK =
        _progress_writer.open(output_video_filename);

    if( !fOK ) {
      CF_ERROR("progress_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !_progress_writer.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("homography_video_writer_.write() fails: %s",
        _progress_writer.filename().c_str());
    return false;
  }


  return true;
}

