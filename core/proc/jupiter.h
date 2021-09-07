/*
 * jupiter.h
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __jupiter_h__
#define __jupiter_h__

#include <opencv2/opencv.hpp>

/*
 Detect planetary disk on given image, fit jovian ellipse
 and return final enclosing ellipse in cv::RotatedRect structure.

 Example:

  @code
  cv::RotatedRect rc;

  if ( !fit_jovian_ellipse(image, &rc) ) {
    CF_ERROR("fit_jovian_ellipse() fails");
    return;
  }

  cv::ellipse(image, rc, CV_RGB(0, 1, 0), 1, cv::LINE_8);

  // draw major semi-axis
  cv::line(image, rc.center, rc.center +
      0.5 * cv::Point2f(rc.size.width * cos(rc.angle * CV_PI / 180),
          rc.size.width * sin(rc.angle * CV_PI / 180)),
          CV_RGB(1,1,0));

  // draw minor semi-axis
  cv::line(image, rc.center, rc.center +
      0.5 * cv::Point2f(rc.size.height * sin(rc.angle * CV_PI / 180),
          -rc.size.height * cos(rc.angle * CV_PI / 180)),
          CV_RGB(1,1,0));

  save_image(image, "fit_jovian_ellipse.tiff");

 @endcode
 */
bool fit_jovian_ellipse(cv::InputArray _image,
    cv::RotatedRect * output_ellipse_rect);

#endif /* __jupiter_h__ */
