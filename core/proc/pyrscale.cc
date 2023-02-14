/*
 * pyrscale.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */
#include "pyrscale.h"
#include <core/debug.h>


/*
 * Pyramid down to specific level
 */
bool pyramid_downscale(cv::InputArray src, cv::Mat & dst, int max_levels, int border_mode)
{
  if( std::min(src.cols(), src.rows()) < 4 ) {
    src.copyTo(dst);
  }
  else {

    cv::pyrDown(src, dst, cv::Size(), border_mode);

    if( (std::min)(dst.cols, dst.rows) >= 4 ) {

      for( int l = 1; l < max_levels; ++l ) {

        cv::pyrDown(dst, dst, cv::Size(), border_mode);

        if( (std::min)(dst.cols, dst.rows) < 4 ) {
          break;
        }
      }
    }
  }

  return true;
}


bool pyramid_upscale(cv::Mat & image, const cv::Size & dstSize, int borderType)
{
  const cv::Size inputSize =
      image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {

      const cv::Size nextSize((sizes.back().width + 1) / 2,
          (sizes.back().height + 1) / 2);

      if( nextSize == inputSize ) {
        break;
      }

      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {

        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);

        return false;
      }

      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i], borderType);
    }
  }

  return true;
}
