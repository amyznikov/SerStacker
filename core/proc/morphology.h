/*
 * morphology.h
 *
 *  Created on: Mar 20, 2021
 *      Author: amyznikov
 */
#pragma once
#ifndef __morphology_h__
#define __morphology_h__

#include <opencv2/opencv.hpp>

/** @brief Morphological smoothing

 @see <http://www.mif.vu.lt/atpazinimas/dip/FIP/fip-Morpholo.html>
 * */
void morphological_smooth_close(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological smoothing
 @see <http://www.mif.vu.lt/atpazinimas/dip/FIP/fip-Morpholo.html>
 * */
void morphological_smooth_open(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological gradient using cv::morphologyEx().
    It is the difference between dilation and erosion of an image.

 @see <http://www.mif.vu.lt/atpazinimas/dip/FIP/fip-Morpholo.html>
 * */
void morphological_gradient(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological internal gradient
 The morphological internal gradient is obtained by from the difference of original image with the result of an erosion.

 @see <https://github.com/ijpb/MorphoLibJ/blob/master/src/main/java/inra/ijpb/morphology/Morphology.java>
 * */
void morphological_internal_gradient(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological external gradient.
 The morphological external gradient is obtained by from the difference of the result of a dilation and of the original image.

 @see <https://github.com/ijpb/MorphoLibJ/blob/master/src/main/java/inra/ijpb/morphology/Morphology.java>
 * */
void morphological_external_gradient(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological Laplacian

 @see <http://www.mif.vu.lt/atpazinimas/dip/FIP/fip-Morpholo.html>
 * */
void morphological_laplacian(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

void morphological_laplacian_abs(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological Edge Detection : Ramp Lee

RAL(src) = min( dilate(src) − close(src), open(src) − erode(src) )

Elementary grayscale morphological techniques can be used to distinguish
smooth “ramp” edges from ripple “texture” edges.
  – Non-ramp edges are texture or noise.

@see Michael A. Wirth,  "Grayscale Morphological Analysis",
  <http://www.cyto.purdue.edu/cdroms/micro2/content/education/wirth08.pdf>
 * */
void rampLee(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/** @brief Morphological Edge Detection : Texture Lee

TEL(src ) = min( close(f) - f , f- open(f))

Elementary grayscale morphological techniques can be used to distinguish
smooth “ramp” edges from ripple “texture” edges.
  – Non-ramp edges are texture or noise.

@see Michael A. Wirth,  "Grayscale Morphological Analysis",
  <http://www.cyto.purdue.edu/cdroms/micro2/content/education/wirth08.pdf>
 * */

void texLee(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray SE = cv::Mat1b(3, 3, 255),
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());


/** @brief Morphological line detection.
 */
void generate_morphological_line_filter_bank(std::vector<cv::Mat> & K,
    int ksize = 13,
    int thickness = 1);

/** @brief Morphological directional filter.
 */
void apply_morphological_filter_bank(const std::vector<cv::Mat> & K,
    cv::InputArray src, cv::OutputArray dst,
    int borderType = cv::BORDER_REPLICATE,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

/***/
void build_morph_gradient_pyramid(cv::InputArray image, cv::InputArray mask,
    std::vector<cv::Mat> & layers,
    int max_level);

/***/
void build_morph_laplacian_pyramid(cv::InputArray image, cv::InputArray mask,
    std::vector<cv::Mat> & layers,
    int max_level);

#endif /* __morphology_h__ */
