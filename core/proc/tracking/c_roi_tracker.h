/*
 * c_roi_tracker.h
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_roi_tracker_h__
#define __c_roi_tracker_h__

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <core/settings/opencv_settings.h>
#include <core/ctrlbind/ctrlbind.h>

#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) \
    ((a)<<16 | (b)<<8 | (c))
#endif
#ifndef CV_VERSION_CURRRENT
#define CV_VERSION_CURRRENT \
    CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif



#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 5, 1)
  using TrackerBoosting = cv::legacy::tracking::TrackerBoosting;
  using TrackerMIL = cv::TrackerMIL;
  using TrackerKCF = cv::tracking::TrackerKCF;
  using TrackerTLD = cv::legacy::tracking::TrackerTLD;
  using TrackerMedianFlow = cv::legacy::tracking::TrackerMedianFlow;
  using TrackerMOSSE = cv::legacy::tracking::TrackerMOSSE;
  using TrackerCSRT = cv::tracking::TrackerCSRT;
  // using TrackerGOTURN = cv::legacy::tracking::TrackerGOTURN;
#else
  using TrackerBoosting = cv::TrackerBoosting;
  using TrackerMIL = cv::TrackerMIL;
  using TrackerKCF = cv::TrackerKCF;
  using TrackerTLD = cv::TrackerTLD;
  using TrackerMedianFlow = cv::TrackerMedianFlow;
  using TrackerMOSSE = cv::TrackerMOSSE;
  using TrackerCSRT = cv::TrackerCSRT;
  using TrackerGOTURN = cv::TrackerGOTURN;
#endif

enum ROI_TRACKER_TYPE
{
  ROI_TRACKER_BOOSTING,
  ROI_TRACKER_MIL,
  ROI_TRACKER_KCF,
  ROI_TRACKER_TLD,
  ROI_TRACKER_MEDIANFLOW,
  ROI_TRACKER_MOSSE,
  ROI_TRACKER_CSRT,

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  ROI_TRACKER_GOTURN,
#endif
};


struct c_roi_tracker_options
{
  ROI_TRACKER_TYPE tracker_type = ROI_TRACKER_MEDIANFLOW;
  bool enabled = true;

  TrackerBoosting::Params boosting;
  TrackerMIL::Params mil;
  TrackerKCF::Params kcf;
  TrackerTLD::Params tld;
  TrackerMedianFlow::Params medianflow;
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  TrackerGOTURN::Params goturn;
#endif
  TrackerCSRT::Params csrt;
};

bool serialize(c_roi_tracker_options & options,
    c_config_setting settings,
    bool save);


class c_roi_tracker
{
public:
  c_roi_tracker();

  bool initialize(const c_roi_tracker_options & options);
  void release();
  bool initialized() const;

  bool track(cv::InputArray image, CV_OUT cv::Rect & boundingBox, bool * updated);

protected:
  bool _initialized = false;

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  cv::Ptr<cv::Tracker> _tracker;
#else
  cv::Ptr<cv::Tracker> _tracker;
  cv::Ptr<cv::legacy::tracking::Tracker> _legacy_tracker;
#endif
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_options> & ctx)
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

#endif /* __c_roi_tracker_h__ */
