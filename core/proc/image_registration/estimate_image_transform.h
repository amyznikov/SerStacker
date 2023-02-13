/*
 * estimate_image_transform.h
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __estimate_image_transform_h__
#define __estimate_image_transform_h__

#include "c_image_transform.h"


bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_);


#endif /* __estimate_image_transform_h__ */
