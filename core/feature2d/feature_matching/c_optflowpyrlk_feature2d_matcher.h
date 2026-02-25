/*
 * c_optflowpyrlk_feature2d_matcher.h
 *
 *  Created on: Sep 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_optflowpyrlk_feature2d_matcher_h__
#define __c_optflowpyrlk_feature2d_matcher_h__

#include "c_feature2d_matcher.h"


struct c_optflowpyrlk_feature2d_matcher_options:
    c_feature2d_matcher_base_options
{
  int maxLevel = 3;
  cv::Size winSize = cv::Size(21, 21);
  int maxIterations = 30;
  int flags = 0;
  double eps = 0.01;
  double minEigThreshold = 1e-4;
  double maxErr = 0;
};

bool match_optflowpyrlk(cv::InputArray previous_image, cv::InputArray next_image,
    const std::vector<cv::KeyPoint> & previous_keypoints,
    const c_optflowpyrlk_feature2d_matcher_options & opts,
    /* out */ std::vector<cv::Point2f> & matched_previous_positions,
    /* out */ std::vector<cv::Point2f> & matched_predicted_previous_positions);


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_optflowpyrlk_feature2d_matcher_options> & ctx)
{
  using S = c_optflowpyrlk_feature2d_matcher_options;

  ctlbind(ctls, as_base<c_feature2d_matcher_base_options>(ctx));

  ctlbind(ctls, "maxLevel", ctx(&S::maxLevel), "");
  ctlbind(ctls, "winSize", ctx(&S::winSize), "");
  ctlbind(ctls, "maxIterations", ctx(&S::maxIterations), "");
  //ctlbind(ctls, "flags", ctx)&S::flags), "");
  ctlbind(ctls, "eps", ctx(&S::eps), "");
  ctlbind(ctls, "minEigThreshold", ctx(&S::minEigThreshold), "");
  ctlbind(ctls, "maxErr", ctx(&S::maxErr), "");
}

#endif /* __c_optflowpyrlk_feature2d_matcher_h__ */
