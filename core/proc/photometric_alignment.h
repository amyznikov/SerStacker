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

bool applyPhotometricAlignment(cv::InputArray current_image,
    cv::OutputArray updated_current_image,
    const cv::Scalar brightness,
    const cv::Scalar & contrast,
    bool includeBrightness = true,
    bool includeContrast = true);

bool estimateAndApplyPhotometricAlignment(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const cv::Rect & commonCurrentROI, const cv::Rect & commonReferenceROI,
    cv::OutputArray updated_current_image,
    bool includeBrightness,
    bool includeContrast,
    bool separateChannels,
    double eps,
    cv::Scalar * outputBrightness = nullptr,
    cv::Scalar * outputContrast = nullptr);


#endif /* __photometric_alignment_h__ */
