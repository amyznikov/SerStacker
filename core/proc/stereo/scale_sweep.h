/*
 * scale_sweep.h
 *
 *  Created on: Sep 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __scale_sweep_h__
#define __scale_sweep_h__

#include <opencv2/opencv.hpp>

/*
 * create_scale_compression_remap()
 *
 * This routine creates the map used to cv::remap() the train image
 * for given stereo matching iteration.
 */
void create_scale_compression_remap(int iteration,
    const cv::Size & image_size,
    const cv::Point2d & epipole_location,
    cv::Mat2f & cmap,
    cv::InputArray src_mask = cv::noArray(),
    cv::OutputArray dst_mask = cv::noArray());




#endif /* __scale_sweep_h__ */
