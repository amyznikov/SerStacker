/*
 * load_image.h
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef ___load_image_h___
#define _____load_image_h___

#include <opencv2/opencv.hpp>

bool load_image(cv::Mat & dst,
    const std::string & filename);



#endif /* _____load_image_h___ */
