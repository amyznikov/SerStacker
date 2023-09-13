/*
 * downstrike.h
 *
 *  Created on: May 26, 2018
 *      Author: amyznikov
 */

#ifndef __downstrike_h__
#define __downstrike_h__

#include <opencv2/opencv.hpp>

/*
 * 2x downsampling step by rejecting each EVEN row and column,
 *    keep only uneven
 * */
void downstrike_even(cv::InputArray src, cv::Mat & dst);

/*
 * 2x upsampling step by injecting EVEN ZERO rows and columns ...
 * */
void upject_even(cv::InputArray _src, cv::Mat & dst, cv::Size dstSize,
    cv::Mat * _zmask = NULL, int zmdepth = -1);


/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 *    keep only even
 * */
void downstrike_uneven(cv::InputArray _src, cv::Mat & dst);


/*
 * 2x upsampling step by injecting UNEVEN ZERO rows and columns ...
 * */
void upject_uneven(cv::InputArray _src, cv::Mat & dst, cv::Size dstSize,
    cv::Mat * _zmask = NULL, int zmdepth = -1);


enum DOWNSTRIKE_MODE {
  DOWNSTRIKE_EVEN, // Reject each EVEN row and column: 0,2,4,6...
  DOWNSTRIKE_UNEVEN, // Reject each UNEVEN row and column: 1,3,5,7 ...
};

inline void downstrike_pixels(cv::InputArray src, cv::Mat & dst, DOWNSTRIKE_MODE mode)
{
  switch (mode) {
    case DOWNSTRIKE_EVEN:
      downstrike_even(src, dst);
      break;
    case DOWNSTRIKE_UNEVEN:
      downstrike_uneven(src, dst);
      break;
  }
}

#endif /* __downstrike_h__ */
