/*
 * fft_transfer_spectrum_profile.h
 *
 *  Created on: Oct 27, 2020
 *      Author: amyznikov
 */

#ifndef __fft_transfer_spectrum_profile_h__
#define __fft_transfer_spectrum_profile_h__


#include <opencv2/opencv.hpp>

bool fft_transfer_spectrum_profile(cv::InputArray from_image,
    cv::InputArray to_image,
    cv::OutputArray dst);

bool accumulate_fft_spectrum_power(const cv::Mat & src,
    cv::Mat & acc,
    float & cnt);

bool max_fft_spectrum_power(const cv::Mat & src,
    cv::Mat & acc);

bool swap_fft_power_spectrum(const cv::Mat & src,
    const cv::Mat & acc,
    cv::Mat & dst);

#endif /* __fft_transfer_spectrum_profile_h__ */
