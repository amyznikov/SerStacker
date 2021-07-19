/*
 * save_image.h
 *
 *  Created on: Nov 16, 2019
 *      Author: amyznikov
 */

#ifndef __save_image_h__
#define __save_image_h__

#include <opencv2/opencv.hpp>

bool save_image(cv::InputArray _image, const std::string & fname,
    const std::vector<int>& params = std::vector<int>());



#endif /* __save_image_h__ */
