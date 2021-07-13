/*
 * estimate_noise.h
 *
 *  Created on: Feb 15, 2018
 *      Author: amyznikov
 */

#ifndef __glv_noisemap_h__
#define __glv_noisemap_h__

#include <opencv2/opencv.hpp>

/*
 * Estimate the standard deviation of the gaussian noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 *
 *  Note that for photon-counting (Poisson) nose the variance-stabilizing transform
 *  is requiredd before noise estimation.
 *
 *  For large counts the Anscombe transform is usually recommended,
 *  but for image data normalized to range [0..1] I found simple sqrt(x) may work better
 *
 */
void create_noise_map(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray _mask = cv::noArray());

cv::Scalar estimate_noise(cv::InputArray src,
    cv::OutputArray dst = cv::noArray(),
    cv::InputArray mask = cv::noArray());

int extract_channel_with_max_sn_ratio(cv::InputArray src,
    cv::OutputArray dst,
    cv::InputArray srcmsk = cv::noArray(),
    double * s = nullptr,
    double * n = nullptr);


#endif /* __glv_noisemap_h__ */
