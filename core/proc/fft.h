/*
 * fft.h
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#ifndef __fft_h__
#define __fft_h__

#include <opencv2/opencv.hpp>

cv::Size getOptimalFFTSize(cv::Size imageSize,
    cv::Size psfSize = cv::Size(0,0),
    bool forceEvenSize = true);

bool copyMakeFFTBorder(cv::InputArray src,
    cv::OutputArray dst,
    cv::Size fftSize,
    cv::Rect * outrc = nullptr);

void imageToSpectrum(const cv::Mat & src,
    std::vector<cv::Mat2f> & complex_channels);

void imageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::Mat & dst);

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

void fftRadialPowerProfile(const cv::Mat1f & magnitude,
    std::vector<double> & output_profile,
    std::vector<int> & output_counts);

void fftMultiplyRadialPowerProfile(cv::Mat1f & magnitude,
    const std::vector<double> & profile,
    double scale = 1);

void fftSharpen(cv::InputArray src, cv::OutputArray dst,
    const std::vector<double> & coeffs);

#endif /* __fft_h__ */
