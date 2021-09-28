/*
 * fft.h
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#ifndef __fft_h__
#define __fft_h__

#include <opencv2/opencv.hpp>

cv::Size fftGetOptimalSize(cv::Size imageSize,
    cv::Size psfSize = cv::Size(0,0),
    bool forceEvenSize = true);

bool fftCopyMakeBorder(cv::InputArray src,
    cv::OutputArray dst,
    cv::Size fftSize,
    cv::Rect * outrc = nullptr);

void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels);

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst);

void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels,
    const cv::Size & psfSize,
    cv::Rect * rc);

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst,
    const cv::Rect & rc);


void fftSwapQuadrants(cv::InputOutputArray spec);

bool fftSpectrumPower(cv::InputArray src,
    cv::OutputArray dst);

bool fftSpectrumModule(cv::InputArray src,
    cv::OutputArray dst);

bool fftSpectrumToPolar(const cv::Mat & src,
    cv::Mat & magnitude,
    cv::Mat & phase);

bool fftSpectrumFromPolar(const cv::Mat & magnitude, const cv::Mat & phase,
    cv::Mat & dst );

void fftSharpenR1(cv::InputArray image, cv::OutputArray dst,
    double scale,
    bool preserve_l2_norm = true);

bool fftAccumulatePowerSpectrum(const cv::Mat & src,
    cv::Mat & acc,
    float & cnt);

bool fftMaxPowerSpectrum(const cv::Mat & src,
    cv::Mat & acc);

bool fftRadialPowerProfile(cv::InputArray src,
    std::vector<double> * fftprofile);

void fftRadialPolySharp(cv::InputArray src, cv::OutputArray dst,
    const std::vector<double> & coeffs,
    std::vector<double> & output_profile_before,
    std::vector<double> & output_profile_after,
    std::vector<double> & output_profile_poly);

#endif /* __fft_h__ */
