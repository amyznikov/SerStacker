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

//  const c_roi_tracker_options & options() const;
//  c_roi_tracker_options & options();

  bool initialize(const c_roi_tracker_options & options);
  void release();

  bool track(cv::InputArray image, CV_OUT cv::Rect & boundingBox, bool * updated);

  // bool serialize(c_config_setting settings, bool save);


protected:
  //c_roi_tracker_options options_;
  bool initialized = false;

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
  cv::Ptr<cv::Tracker> tracker_;
#else
  cv::Ptr<cv::Tracker> tracker_;
  cv::Ptr<cv::legacy::tracking::Tracker> legacy_tracker_;
#endif
};

#endif /* __c_roi_tracker_h__ */
