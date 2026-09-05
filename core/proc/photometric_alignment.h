/*
 * photometric_alignment.h
 *
 *  Created on: Sep 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __photometric_alignment_h__
#define __photometric_alignment_h__

#include <opencv2/opencv.hpp>


/**
 * Estimate correction to current image to photometricaly match referenceImage
 *
 * Generic model is:
 *  referenceImage = outputBrightness + outputContrast * currentImage
 *
 * includeBrightness : if false then reduce model to referenceImage = outputContrast * currentImage
 * includeContrast   : if false then reduce model to referenceImage = outputBrightness + currentImage
 * separateChannels  : if false then compute based on grayscale levels only (average channels)
 *
 */

bool estimatePhotometricAlignment(cv::InputArray currentImage,
    cv::InputArray referenceImage, cv::InputArray commonPixelsMask,
    cv::Scalar & outputBrightness,
    cv::Scalar & outputContrast,
    bool includeBrightness = true,
    bool includeContrast = true,
    bool separateChannels = true,
    double eps = 1e-6);




#endif /* __photometric_alignment_h__ */
