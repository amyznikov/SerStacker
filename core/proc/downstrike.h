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
bool downstrike_even(cv::InputArray src, cv::OutputArray dst,
    cv::Size size = cv::Size());

/*
 * 2x downsampling step by rejecting each UNEVEN row and column,
 *    keep only even (0, 2, 4...)
 * */
bool downstrike_uneven(cv::InputArray src, cv::OutputArray dst,
    cv::Size size = cv::Size());

/*
 * 2x upsampling step by injecting EVEN ZERO-VALUED rows and columns ...
 * If optionalOutputZmask output is requested,
 * it will contain single-channel image of requested zmaskDepth (CV_8U by default)
 * with non-zero values for pixels copied rom src and zeros for empty (injected) pixels
 * */
bool upject_even(cv::InputArray _src, cv::OutputArray dst, cv::Size dstSize,
    cv::OutputArray zmask = cv::noArray(),
    int zdepth = -1);

/*
 * 2x upsampling step by injecting UNEVEN ZERO-VALUED rows and columns ...
 * Keep only EVEN (0, 2, 4...) positions from src.
 * */
bool upject_uneven(cv::InputArray src, cv::OutputArray dst, cv::Size dstSize,
    cv::OutputArray zmask = cv::noArray(),
    int zdepth = -1);


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
