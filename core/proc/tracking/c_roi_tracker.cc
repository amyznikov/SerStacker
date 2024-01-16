/*
 * c_roi_tracker.cc
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#include "c_roi_tracker.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<ROI_TRACKER_TYPE>()
{
  static const c_enum_member members[] = {
      { ROI_TRACKER_BOOSTING, "BOOSTING", "" },
      { ROI_TRACKER_MIL, "MIL", "" },
      { ROI_TRACKER_KCF, "KCF", "" },
      { ROI_TRACKER_TLD, "TLD", "" },
      { ROI_TRACKER_MEDIANFLOW, "MEDIANFLOW", "" },
      { ROI_TRACKER_MOSSE, "MOSSE", "" },
      { ROI_TRACKER_CSRT, "CSRT", "" },
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      { ROI_TRACKER_GOTURN, "GOTURN", "" },
#endif
      { ROI_TRACKER_MIL},
  };

  return members;
}

bool serialize(c_roi_tracker_options & options, c_config_setting settings, bool save)
{
  c_config_setting section;

  SERIALIZE_OPTION(settings, save, options, enabled);
  SERIALIZE_OPTION(settings, save, options, tracker_type);

  if( (section = SERIALIZE_GROUP(settings, save, "BOOSTING")) ) {
    SERIALIZE_OPTION(section, save, options.boosting, numClassifiers);
    SERIALIZE_OPTION(section, save, options.boosting, samplerOverlap);
    SERIALIZE_OPTION(section, save, options.boosting, samplerSearchFactor);
    SERIALIZE_OPTION(section, save, options.boosting, iterationInit);
    SERIALIZE_OPTION(section, save, options.boosting, featureSetNumFeatures);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "MIL")) ) {
    SERIALIZE_OPTION(section, save, options.mil, samplerInitInRadius);
    SERIALIZE_OPTION(section, save, options.mil, samplerInitMaxNegNum);
    SERIALIZE_OPTION(section, save, options.mil, samplerSearchWinSize);
    SERIALIZE_OPTION(section, save, options.mil, samplerTrackInRadius);
    SERIALIZE_OPTION(section, save, options.mil, samplerTrackMaxPosNum);
    SERIALIZE_OPTION(section, save, options.mil, samplerTrackMaxNegNum);
    SERIALIZE_OPTION(section, save, options.mil, featureSetNumFeatures);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "KCF")) ) {
    SERIALIZE_OPTION(section, save, options.kcf, detect_thresh);
    SERIALIZE_OPTION(section, save, options.kcf, sigma);
    SERIALIZE_OPTION(section, save, options.kcf, lambda);
    SERIALIZE_OPTION(section, save, options.kcf, interp_factor);
    SERIALIZE_OPTION(section, save, options.kcf, output_sigma_factor);
    SERIALIZE_OPTION(section, save, options.kcf, pca_learning_rate);
    SERIALIZE_OPTION(section, save, options.kcf, resize);
    SERIALIZE_OPTION(section, save, options.kcf, split_coeff);
    SERIALIZE_OPTION(section, save, options.kcf, wrap_kernel);
    SERIALIZE_OPTION(section, save, options.kcf, compress_feature);
    SERIALIZE_OPTION(section, save, options.kcf, max_patch_size);
    SERIALIZE_OPTION(section, save, options.kcf, compressed_size);
    SERIALIZE_OPTION(section, save, options.kcf, desc_pca);
    SERIALIZE_OPTION(section, save, options.kcf, desc_npca);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "TLD")) ) {
    // SERIALIZE_OPTION(section, save, options.tld, );
  }
  if( (section = SERIALIZE_GROUP(settings, save, "MEDIANFLOW")) ) {
    SERIALIZE_OPTION(section, save, options.medianflow, pointsInGrid);
    SERIALIZE_OPTION(section, save, options.medianflow, winSize);
    SERIALIZE_OPTION(section, save, options.medianflow, maxLevel);
    SERIALIZE_OPTION(section, save, options.medianflow, termCriteria);
    SERIALIZE_OPTION(section, save, options.medianflow, winSizeNCC);
    SERIALIZE_OPTION(section, save, options.medianflow, maxMedianLengthOfDisplacementDifference);
  }
  if( (section = SERIALIZE_GROUP(settings, save, "MOSSE")) ) {
    // SERIALIZE_OPTION(section, save, options.mosse, );
  }
  if( (section = SERIALIZE_GROUP(settings, save, "CSRT")) ) {
    SERIALIZE_OPTION(section, save, options.csrt, use_hog);
    SERIALIZE_OPTION(section, save, options.csrt, use_color_names);
    SERIALIZE_OPTION(section, save, options.csrt, use_gray);
    SERIALIZE_OPTION(section, save, options.csrt, use_rgb);
    SERIALIZE_OPTION(section, save, options.csrt, use_channel_weights);
    SERIALIZE_OPTION(section, save, options.csrt, use_segmentation);

    SERIALIZE_OPTION(section, save, options.csrt, window_function); //!<  Window function: "hann", "cheb", "kaiser"
    SERIALIZE_OPTION(section, save, options.csrt, kaiser_alpha);
    SERIALIZE_OPTION(section, save, options.csrt, cheb_attenuation);

    SERIALIZE_OPTION(section, save, options.csrt, template_size);
    SERIALIZE_OPTION(section, save, options.csrt, gsl_sigma);
    SERIALIZE_OPTION(section, save, options.csrt, hog_orientations);
    SERIALIZE_OPTION(section, save, options.csrt, hog_clip);
    SERIALIZE_OPTION(section, save, options.csrt, padding);
    SERIALIZE_OPTION(section, save, options.csrt, filter_lr);
    SERIALIZE_OPTION(section, save, options.csrt, weights_lr);
    SERIALIZE_OPTION(section, save, options.csrt, num_hog_channels_used);
    SERIALIZE_OPTION(section, save, options.csrt, admm_iterations);
    SERIALIZE_OPTION(section, save, options.csrt, histogram_bins);
    SERIALIZE_OPTION(section, save, options.csrt, histogram_lr);
    SERIALIZE_OPTION(section, save, options.csrt, background_ratio);
    SERIALIZE_OPTION(section, save, options.csrt, number_of_scales);
    SERIALIZE_OPTION(section, save, options.csrt, scale_sigma_factor);
    SERIALIZE_OPTION(section, save, options.csrt, scale_model_max_area);
    SERIALIZE_OPTION(section, save, options.csrt, scale_lr);
    SERIALIZE_OPTION(section, save, options.csrt, scale_step);
    SERIALIZE_OPTION(section, save, options.csrt, psr_threshold); //!< we lost the target, if the psr is lower than this.
  }

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  if( (section = SERIALIZE_GROUP(settings, save, "GOTURN")) ) {
    // SERIALIZE_OPTION(section, save, options.goturn, );
  }
#endif

  return true;
}


c_roi_tracker::c_roi_tracker()
{
}

//c_roi_tracker::c_roi_tracker(const c_roi_tracker_options & options) :
//    options_(options)
//{
//}
//
//const c_roi_tracker_options & c_roi_tracker::options() const
//{
//  return options_;
//}
//
//c_roi_tracker_options & c_roi_tracker::options()
//{
//  return options_;
//}
//
//bool c_roi_tracker::serialize(c_config_setting settings, bool save)
//{
//  return ::serialize(options_, settings, save);
//}

void c_roi_tracker::release()
{
  initialized = false;

  tracker_.reset();
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 5, 1)
  legacy_tracker_.reset();
#endif
}

bool c_roi_tracker::initialize(const c_roi_tracker_options & options)
{
  release();

  switch (options.tracker_type) {
    case ROI_TRACKER_MIL:
      tracker_ = TrackerMIL::create(options.mil);
      break;

    case ROI_TRACKER_KCF:
      tracker_ = TrackerKCF::create(options.kcf);
      break;

    case ROI_TRACKER_CSRT:
      tracker_ = TrackerCSRT::create(options.csrt);
      break;

    case ROI_TRACKER_BOOSTING:
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      tracker_ = TrackerBoosting::create(options.boosting);
#else
      legacy_tracker_ = TrackerBoosting::create(options.boosting);
#endif
      break;

    case ROI_TRACKER_TLD:
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      tracker_ = TrackerTLD::create(options.tld);
#else
      legacy_tracker_ = TrackerTLD::create(options.tld);
#endif
      break;

    case ROI_TRACKER_MEDIANFLOW:
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      tracker_ = TrackerMedianFlow::create(options.medianflow);
#else
      legacy_tracker_ = TrackerMedianFlow::create(options.medianflow);
#endif
      break;

    case ROI_TRACKER_MOSSE:
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      tracker_ = TrackerMOSSE::create();
#else
      legacy_tracker_ = TrackerMOSSE::create();
#endif
      break;

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      case ROI_TRACKER_GOTURN:
        tracker_ = TrackerGOTURN::create(options_.goturn);
      break;
#endif

    default:
      CF_ERROR("Not supported tracker type %d requested",
          (int )(options.tracker_type));
      break;
  }

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)

  if( !tracker_ ) {
    CF_ERROR("Tracker create fails");
    return false;
  }

#else
  if( !tracker_ && !legacy_tracker_ ) {
    CF_ERROR("Tracker create fails");
    return false;
  }
#endif

  return true;
}

bool c_roi_tracker::track(cv::InputArray image, CV_OUT cv::Rect & boundingBox, bool * updated)
{
#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  if( tracker_ ) {

    if( !initialized ) {
      tracker_->init(image, cv::Rect2d(boundingBox));
      initialized = true;
      *updated = true;
    }
    else {
      cv::Rect2d rc2d =
          boundingBox;

      *updated =
          tracker_->update(image,
              rc2d);

      boundingBox =
          rc2d;
    }

    return true;
  }

#else

  if( tracker_ ) {
    if( !initialized ) {
      tracker_->init(image, boundingBox);
      initialized = true;
      *updated = true;
    }
    else {
      *updated =
          tracker_->update(image,
              boundingBox);
    }
    return true;
  }

  if( legacy_tracker_ ) {

    if( !initialized ) {
      legacy_tracker_->init(image, cv::Rect2d(boundingBox));
      initialized = true;
      *updated = true;
    }
    else {

      cv::Rect2d rc2d =
          boundingBox;

      *updated =
          legacy_tracker_->update(image,
              rc2d);

      boundingBox =
          rc2d;
    }

    return true;
  }
#endif

  CF_ERROR("tracker was not initialized properly, "
      "pointer is null");
  return false;
}

