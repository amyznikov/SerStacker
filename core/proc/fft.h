/*
 * fft.h
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#ifndef __fft_h__
#define __fft_h__

#include <opencv2/opencv.hpp>

cv::Size fftGetOptimalSize(const cv::Size & imageSize,
    cv::Size psfSize = cv::Size(0, 0),
    cv::Rect * roirc  = nullptr,
    bool forceEvenSize = true);

bool fftCopyMakeBorder(cv::InputArray src,
    cv::OutputArray dst,
    const cv::Size & fftSize,
    cv::Rect * outrc = nullptr);

bool fftImageToSpectrum(cv::InputArray _src, cv::OutputArray _dst,
    const cv::Size & fftSize,
    bool swapQuadrants = true);

void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels);

void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels,
    const cv::Size & psfSize,
    cv::Rect * rc);

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst);


void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst,
    const cv::Rect & rc);

void fftSwapQuadrants(cv::InputOutputArray spec);
void fftSwapQuadrants(cv::InputArray src, cv::OutputArray dst);

/* Power = Re^2 + Im^2 */
bool fftSpectrumPower(cv::InputArray src,
    cv::OutputArray dst);

/* Module = sqrt(Re^2 + Im^2) */
bool fftSpectrumModule(cv::InputArray src,
    cv::OutputArray dst);

bool fftSpectrumPhase(cv::InputArray src,
    cv::OutputArray dst);

bool fftRadialProfile(const cv::Mat1f & spectrum,
    cv::Mat1f & output_profile,
    bool includeCorners = false);

void fftRadialProfileToImage(const cv::Mat1f & radialProfile,
    const cv::Size & outputImageSize,
    bool cornersIncluded,
    cv::Mat1f & outputImage);

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

void fftRadialPolySharp(cv::InputArray src, cv::OutputArray dst,
    const std::vector<double> & coeffs,
    std::vector<double> & output_profile_before,
    std::vector<double> & output_profile_after,
    std::vector<double> & output_profile_poly);

void fftComputeAutoCorrelation(cv::InputArray src,
    cv::OutputArray dst,
    bool logscale = true);

// Space Isotropic Gaussian
cv::Mat1f fftGenerateGaussianFilter(const cv::Size & fftSize,
    double sigma_space = 2.0, double gain = 1.0,
    bool centerDC = true);

cv::Mat1f fftGenerateLaplacianFilter(const cv::Size & fftSize,
    double gain = 1.0, bool squareRoot = false,
    bool centerDC = true);

// Discrete Laplacian Filter for Periodic+Smooth Decomposition
cv::Mat1f fftGenerateDiscreteLaplacianFilter(const cv::Size & fftSize,
    bool centerDC = true);


// Butterworth's formula: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthFilter(const cv::Size & fftSize,
    double cutoff, int order = 2, double gain = 1,
    bool swapQuadrants = false);

// Space Isotropic Gaussian-Based unsharp mask filter
cv::Mat1f fftGenerateGaussianUnsharpFilter(const cv::Size & fftSize,
    double sigma_space, double gain, bool centerDC = true);

// Space Isotropic Butterworth-Based unsharp mask filter
// Butterworth's formula: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthUnsharpFilter(const cv::Size & fftSize,
    double rc, double order, double gain, bool centerDC = true);

void fftCreateVMatrix(cv::InputArray _src, cv::OutputArray _dst);

#endif /* __fft_h__ */
