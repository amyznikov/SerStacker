/*
 * flow-color.h
 *
 *  Created on: May 14, 2018
 *      Author: amyznikov
 */

#pragma once
#ifndef __flow_color_h__
#define __flow_color_h__

#include <opencv2/opencv.hpp>

//
// Color encoding of flow vectors
//
// Ported to OpenCV from original optical flow input/output code
// provided as part of the Sintel dataset
//  http://sintel.is.tue.mpg.de/
//  http://members.shaw.ca/quadibloc/other/colint.htm
//


// The "official" threshold - if the absolute value of either
// flow component is greater, it's considered unknown
#define UNKNOWN_FLOW_THRESH 1e9

// value to use to represent unknown flow
#define UNKNOWN_FLOW 1e10


/*
 * Return whether flow vector is unknown
 */
static inline bool unknown_flow(double u, double v) {
  return (fabs(u) > UNKNOWN_FLOW_THRESH) || (fabs(v) > UNKNOWN_FLOW_THRESH) || std::isnan(u) || std::isnan(v);
}

/*
 * Return whether flow vector is unknown
 */
template<class T> static inline
bool unknown_flow(const cv::Point_<T> & f ) {
  return  unknown_flow(f.x, f.y);
}

/*
 * Return whether flow vector is unknown
 */
template<class T> static inline
bool unknown_flow(const cv::Vec<T,2> & f ) {
  return  unknown_flow(f.val[0], f.val[1]);
}


/*
 * Compute Sintel BGR color for given flow vector
 * */
void compute_flow_color(double fx, double fy, uint8_t pix[3]);


/*
 * Compute Sintel BGR color for given flow vector
 * */
template<class T> static inline
void compute_flow_color(const cv::Point_<T> & f, uint8_t pix[3]) {
  return compute_flow_color(f.x, f.y, pix);
}

/*
 * Compute Sintel BGR color for given flow vector
 * */
template<class T> static inline
void compute_flow_color(const cv::Vec<T, 2> & f, uint8_t pix[3]) {
  return compute_flow_color(f.val[0], f.val[1], pix);
}


/*
 * Create flow BGR image using Sintel palette
 *  flow: 2-channel input flow matrix
 *  dst : output BRG image
 * */
bool flow2color(cv::InputArray src, cv::Mat & dst, double maxmotion = -1);

/*
 * Create flow BGR image using HSV color space
 *  flow: 2-channel input flow matrix
 *  dst : output BRG image
 *
 *  The code is extracted from OpenCV example dis_opticalflow.cpp
 * */
bool flow2colorHSV(cv::InputArray src, cv::Mat & dst,
    double maxmotion = -1,
    bool invert_y = false);

/*
 * Create flow BGR image using HSV color space
 *  fx: 1-channel input flow matrix
 *  fy: 1-channel input flow matrix
 *  dst : output BRG image
 *
 *  The code is extracted from OpenCV example dis_opticalflow.cpp
 * */
bool flow2colorHSV(cv::InputArray fx, cv::InputArray fy, cv::Mat & dst,
    double maxmotion = -1,
    bool invert_y = false);


void drawHsvRing(int radius,  /*out*/ cv::Mat & dst,
    /*out, opt*/ cv::Mat * omask = NULL);


/*
 * just for a case
 * */
inline void pasteHsvRing(cv::Mat & dst, cv::InputArray _R, cv::InputArray mask = cv::noArray(),
    cv::Point pos = cv::Point(0, 0))
{
  const cv::Mat R = _R.getMat();
  R.copyTo(dst(cv::Rect(pos.x, pos.y, R.cols, R.rows)), mask);
}

inline void pasteHsvRing(cv::Mat & dst, int radius = 30, cv::Point pos = cv::Point(0, 0))
{
  cv::Mat R, M;
  drawHsvRing(radius, R, &M);
  pasteHsvRing(dst, R, M, pos);
}

#endif /* __glv_flow_color_h__ */
