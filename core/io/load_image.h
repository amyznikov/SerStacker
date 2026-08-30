/*
 * load_image.h
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef ___load_image_h___
#define _____load_image_h___

#include <core/io/debayer.h>

bool load_image(const std::string & filename,
    cv::OutputArray output_image,
    cv::OutputArray output_mask/* = cv::noArray()*/,
    enum COLORID * output_colorid = nullptr);

//// FIXME: Hack Split BGRA to BGR and mask, this must be handled by format readers
bool splitbgra(const cv::Mat & input_bgra_image,
    cv::Mat & output_bgr_image,
    cv::Mat * output_alpha_mask = nullptr);

#endif /* _____load_image_h___ */
