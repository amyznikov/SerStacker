/*
 * save_image.h
 *
 *  Created on: Nov 16, 2019
 *      Author: amyznikov
 */

#ifndef __save_image_h__
#define __save_image_h__

#include <core/io/debayer.h>

bool save_image(cv::InputArray image, cv::InputArray mask, const std::string & fname,
    const std::vector<int>& params = std::vector<int>(),
    enum COLORID colorid = COLORID_UNKNOWN);

inline bool save_image(cv::InputArray image, const std::string & fname,
    const std::vector<int> & params = std::vector<int>(),
    enum COLORID colorid = COLORID_UNKNOWN)
{
  return save_image(image, cv::noArray(), fname, params, colorid);
}

void set_default_tiff_compression(int compression);
int default_tiff_compression();

// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_bgr_image, const cv::Mat & input_alpha_mask, cv::Mat & output_bgra_image);


#endif /* __save_image_h__ */
