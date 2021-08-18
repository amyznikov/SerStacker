/*
 * save_image.h
 *
 *  Created on: Nov 16, 2019
 *      Author: amyznikov
 */

#ifndef __save_image_h__
#define __save_image_h__

#include <opencv2/opencv.hpp>


void set_default_tiff_compression(int compression);
int default_tiff_compression();

bool save_image(cv::InputArray _image, const std::string & fname,
    const std::vector<int>& params = std::vector<int>());

bool save_image(cv::InputArray _image, cv::InputArray _mask, const std::string & fname,
    const std::vector<int>& params = std::vector<int>());

// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_bgr_image, const cv::Mat & input_alpha_mask,
    cv::Mat & output_bgra_image);


#endif /* __save_image_h__ */
