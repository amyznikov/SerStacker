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
    cv::Size psfRadius = cv::Size(0, 0),
    cv::Rect * roirc  = nullptr,
    bool forceEvenSize = true);

// Adjust rectangular ROI of the planet to an optimal square shape for FFT
cv::Rect fftGetOptimalSquaredROI(const cv::Size & imageSize,
    const cv::Rect & rawROI);

bool fftCopyMakeBorder(cv::InputArray src,
    cv::OutputArray dst,
    const cv::Size & fftSize,
    cv::Rect * outrc = nullptr,
    cv::BorderTypes borderType = cv::BORDER_REFLECT101);

bool fftImageToSpectrum(cv::InputArray _src, cv::OutputArray _dst,
    const cv::Size & fftSize,
    bool centerDC = true);

bool fftImageToSpectrum(cv::InputArray image, std::vector<cv::Mat2f> & output_complex_channels,
    const cv::Size & fftSize = cv::Size(0, 0),
    bool centerDC = true);

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

bool fftRadialProfile(const cv::Mat1f & spectrumModule,
    cv::Mat1f & output_profile);

// DFT Radial Profile for packed OpenCV CCS format.
bool fftRadialProfileCCS(const cv::Mat1f & ccsSpectrum,
    cv::Mat1f & outputProfile);

void fftRadialProfileToImage(const cv::Mat1f & radialProfile,
    const cv::Size & outputImageSize,
    cv::Mat1f & outputImage);

bool dctRadialProfile(const cv::Mat1f & dctSpectrum,
    cv::Mat1f & outputProfile);

void dctRadialProfileToImage(const cv::Mat1f & radialProfile,
    const cv::Size & outputImageSize,
    cv::Mat1f & outputImage);

bool fftSpectrumToPolar(const cv::Mat & src,
    cv::Mat & magnitude,
    cv::Mat & phase);

void fftSpectrumToPolar(cv::Mat2f & spec);
bool fftSpectrumToPolar(cv::InputArray spectrumCart, cv::OutputArray spectrumPolar);

bool fftSpectrumFromPolar(const cv::Mat & magnitude, const cv::Mat & phase,
    cv::Mat & dst );

bool fftAccumulatePowerSpectrum(const cv::Mat & src,
    cv::Mat & acc,
    float & cnt);

bool fftMaxPowerSpectrum(const cv::Mat & src,
    cv::Mat & acc);

// Space Isotropic Gaussian
cv::Mat1f fftGenerateGaussianFilter(const cv::Size & fftSize,
    double sigma_space = 2.0, double gain = 1.0,
    bool centerDC = true);

cv::Mat1f fftGenerateLaplacianFilter(const cv::Size & fftSize, double gain = 1.0,
    bool centerDC = true);

cv::Mat1f fftGenerateLaplacianUnsharpFilter(const cv::Size & fftSize, double gain = 1.0,
    double bwrc = 1, int bworder = 4,
    bool centerDC = true);

cv::Mat1f fftGenerateRampFilter(const cv::Size & fftSize, double gain = 1.0,
    bool centerDC = true);

cv::Mat1f dctGenerateRampFilter(const cv::Size & dctSize, double gain = 1);

// Butterworth's formula: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthFilter(const cv::Size & fftSize,
    double cutoff, int order = 2, double gain = 1,
    bool centerDC = true);

// Space Isotropic Gaussian-Based unsharp mask filter
cv::Mat1f fftGenerateGaussianUnsharpFilter(const cv::Size & fftSize,
    double sigma_space, double gain, bool centerDC = true);

// Space Isotropic Butterworth-Based unsharp mask filter
// Butterworth's formula: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthUnsharpFilter(const cv::Size & fftSize,
    double rc, double order, double gain, bool centerDC = true);

// Space Isotropic Butterworth Band-Pass / Band-Reject Filter.
cv::Mat1f fftGenerateButterworthBandFilter(const cv::Size & fftSize,
    double grain_size, double grain_band, int order, double gain,
    bool inverse, bool centerDC);

// Discrete Laplacian Filter for Periodic+Smooth Decomposition
cv::Mat1f fftGenerateDiscreteLaplacianFilter(const cv::Size & fftSize,
    bool centerDC = true);

bool fftMulSpectrum(cv::InputArray complexSpectrum,
    const cv::Mat1f & filter,
    cv::OutputArray dst);

// Create V-Matrix for Periodic+Smooth Decomposition
void fftCreateVMatrix(cv::InputArray _src, cv::OutputArray _dst);

// Create smooth circular cosine window to mask the corners of a planetary disk ROIs
cv::Mat1f fftCreateCircularApodizationWindow(const cv::Size & size);

/**
 * @brief Generates an inverse frequency-domain cross-shaped filter to suppress rectangular window artifacts.
 *
 * This function constructs a 2D weighting matrix in the frequency domain designed to mitigate
 * spectral leakage ("cross-shaped" or "ray" artifacts) caused by the rectangular boundaries
 * of the input ROI (Region of Interest) during FFT-based cross- or phase correlation.
 * It models the vertical and horizontal leakage profiles and applies Tikhonov regularization
 * to invert the artifact field safely without noise amplification.
 *
 * @param[in]  fftSize   The size of the FFT grid (total dimensions of the spectrum matrix).
 * @param[in]  rectSize  The dimensions of the source rectangular window/ROI causing the leakage.
 * @param[out] dst       Output 2D single-channel matrix (`CV_32FC1`) containing the generated filter.
 * @param[in]  _csigma   Characteristic scale factor controlling the width/decay of the artifact rays.
 * @param[in]  _calpha   Tikhonov regularization parameter. Controls the stability of the inversion
 *                       near the high-amplitude artifact zones (prevents division by zero).
 * @param[in]  centerDC  If true, the DC component (zero frequency) is assumed to be in the center
 *                       of the matrix (shifted FFT). If false, DC is at (0,0).
 *
 * @note Implements multi-threaded generation using `parallel_for` for optimal performance.
 */
void fftGenerateInverseCrossFilter(const cv::Size & fftSize, const cv::Size & rectSize, cv::OutputArray dst,
    double _csigma = 0.5, double _calpha = 0.01, bool centerDC = false);


/**
 * Analytical computation of the 2D Complex CV_32FC2 spectrum V via 1D DFT of rows and columns.
 * Implements Virginie Moizan decomposition directly into fft domain avoiding extra call to cv::dft().
 * The src must be singke-channel real image
 *
 * The classic way to get the same output is to use the cv::dft():
 *   cv::Mat V;
 *   fftCreateVMatrix(SRC, V);
 *   cv::dft(V, V_SPECTRUM, cv::DFT_COMPLEX_OUTPUT);
 *
 * TODO: Check if it has sense to combine fftComputeVSpectrumComplex() with fftMulSpectrum() into single function
 */
bool fftComputeVSpectrumComplex(cv::InputArray _src,
    cv::OutputArray _complexSpectrum);

// DFT with Periodic + Smooth Decomposition.
// The Inverse Discrete Laplacian Filter VLAP must be prepared before this call.
// const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
// The target fftSize (FFT padding) is defined by the VLAP.size()
// TODO: Check if it has sense to combine fftComputeVSpectrumComplex() with fftMulSpectrum() into single function
bool fftPPSDecomposition(cv::InputArray src_image, const cv::Mat1f & VLAP,
    cv::OutputArray P_SPECTRUM, cv::OutputArray S_SPECTRUM,
    cv::OutputArray V_SPECTRUM = cv::noArray());

bool fftPPSDecomposition(cv::InputArray src_image, const cv::Mat1f & VLAP,
    std::vector<cv::Mat2f> * P_SPECTRUMS, std::vector<cv::Mat2f> * S_SPECTRUMS);

bool fftPPSDecompositionPlanes(const std::vector<cv::Mat> & planes, const cv::Mat1f & VLAP,
    std::vector<cv::Mat2f> * P_SPECTRUMS, std::vector<cv::Mat2f> * S_SPECTRUMS,
    std::vector<cv::Mat2f> * V_SPECTRUMS = nullptr);

/**
* @brief Function for automatically determining the position angle from the FFT spectrum module
* @param fftSpectrum Cleaned FFT spectrum (after ppsDecomposition and morphological smoothing)
* @return double Polar axis position angle in degrees [0, 180)
*/
double fftEstimateRadonOrientation(const cv::Mat1f & fftSpectrum,
    cv::OutputArray outputDebugHistogram = cv::noArray());

/**
 * CV_32FC1 CCS input -> CV_32FC2 Complex output
 * */
bool fftUnpackCCSSpectrum(cv::InputArray ccsSpectrum,
    cv::OutputArray _complexSpectrum);

/**
 * CV_32FC1 CCS input -> CV_32FC2 Complex output with sign alternating
 * */
bool fftUnpackCCSSpectrumAlternateSign(cv::InputArray _ccsSpectrum,
    cv::OutputArray _complexSpectrum);

/**
 * CV_32FC1 CCS input -> CV_32FC2 Polar (mag/phase) output
 * */
bool fftCCSSpectrumToPolar(cv::InputArray _ccsSpectrum, cv::OutputArray _polarSpectrum,
    bool centerDC = false);

/**
 * CV_32FC2 Complex input -> CV_32FC1 CCS packed output
 * Pack full complex spectrum of the signal into OpenCV CCS format.
 **/
bool fftPackCCSSpectrum(cv::InputArray _complexSpectrum,
    cv::OutputArray _ccsSpectrum);

/**
* @brief Performs element-wise multiplication of a general-purpose real filter by a
*        complex spectrum in OpenCV CCS format.
* @param[in] filter Real filter (size M x N, type CV_32FC1). Each pixel corresponds to a frequency.
* @param[in] ccsSpectrum Input spectrum in OpenCV CCS format (size M x N, type CV_32FC1).
* @param[out] ccsOutputSpectrum Output spectrum resulting from the multiplication, in OpenCV CCS format.
*/
bool fftMulSpectrumCCS(cv::InputArray ccsSpectrum, const cv::Mat1f & filter,
    cv::OutputArray ccsOutputSpectrum);


/*
 * Analytical computation of the 2D CCS spectrum V via 1D DFT of rows and columns.
 * Implements Virginie Moizan decomposition.
 * Saves ~2.0 ms from ~15 ms on a 1024x1024 grayscale frame by eliminating the 2D DFT.
 * // The src must be singke-channel real image
 */
bool fftComputeVSpectrumCCS(cv::InputArray _src,
    cv::OutputArray ccsOutputVSpectrum);


/*
 * DFT with Periodic + Smooth Decomposition with CCS output.
 * Uses Virginie Moizan decomposition.
 * The Inverse Discrete Laplacian Filter VLAP must be prepared before this call with centerDC=false.
 *   const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
 * The target fftSize (FFT padding) is defined by the VLAP.size()
 * The inputImage must be single-channel of any depth
 **/
bool fftPPSDecompositionCCS(cv::InputArray inputImage, const cv::Mat1f & VLAP,
    cv::OutputArray P_SPECTRUM, cv::OutputArray S_SPECTRUM,
    cv::OutputArray V_SPECTRUM = cv::noArray());

bool fftPPSDecompositionCCS(cv::InputArray inputImage, const cv::Mat1f & VLAP,
    std::vector<cv::Mat1f> * P_SPECTRUMS, std::vector<cv::Mat1f> * S_SPECTRUMS);

bool fftPPSDecompositionCCSPlanes(const std::vector<cv::Mat> & planes, const cv::Mat1f & VLAP,
    std::vector<cv::Mat1f> * P_SPECTRUMS, std::vector<cv::Mat1f> * S_SPECTRUMS,
    std::vector<cv::Mat1f> * V_SPECTRUMS = nullptr);

/**
 * @brief Computes the weighted phase correlation cross of two spectra packed in OpenCV CCS format.
 *   Because of CCS is packed format the both vertical and horizontal sizes of spectrums
 *   must be even if filter embeds alternating sign, otherwise incorrect complex conjugation
 *   may happen because the filter becomes not symmetrical.
 *
 * This function performs element-wise cross-multiplication of two spectra with conjugation of the
 * second spectrum, followed by phase whitening (amplitude normalization) and application of a real bandpass filter.
 * Mathematically, for each frequency it computes: \f$ DST = filter \cdot \frac{S_1 \cdot S_2^*}{|S_1 \cdot S_2^*|} \f$
 *
 * @note The algorithm is optimized for multi-threaded execution (via parallel_for) and operates directly
 * on the packed OpenCV CCS (Complex Conjugate Symmetrical) format. This eliminates redundant memory
 * allocations for full complex matrices.
 *
 * @note **Normalization & Correlation Peak Mechanics:**
 * If the input `filter` is pre-normalized using the L1-norm to unity (i.e., sum(gw) = 1), then the subsequent
 * call to inverse Fourier transform `cv::idft(..., cv::DFT_REAL_OUTPUT)` WITHOUT the `cv::DFT_SCALE` flag
 * will yield a peak value of strictly **1.0** on the autocorrelation map (given a perfect match). This occurs
 * because the \f$1/N\f$ scale introduced by the filter's L1-normalization perfectly cancels out the internal \f$N\f$
 * scaling factor inherent to OpenCV's unscaled IDFT.
 *
 * @param[in] ccsSpectrum1 First input image spectrum in OpenCV CCS format (CV_32FC1, real matrix).
 * @param[in] ccsSpectrum2 Second input image spectrum in OpenCV CCS format (CV_32FC1, real matrix).
 * @param[in] filter Real bandpass filter matrix (frequency weights) matching the size of the input spectra.
 * @param[out] _crossSpectrum Output filtered cross-spectrum in CCS format (CV_32FC1).
 *
 * @return Returns false in case of a size mismatch error.
 */
bool fftCrossSpectrumPhaseCorrelateWeightedCCS(cv::InputArray _ccsSpectrum1, cv::InputArray _ccsSpectrum2,
    const cv::Mat1f & filter, cv::OutputArray _crossSpectrum);

/**
 * @brief Computes the bandpass-filtered autocorrelation spectrum (energy map) in CCS format.
 *
 * This function performs an in-place-like multiplication of a complex CCS spectrum by a real-valued
 * amplitude filter. It automatically handles the internal layout symmetry of the OpenCV CCS format
 * and eliminates the imaginary components, as the autocorrelation of a real signal is strictly real.
 *
 * @param[in] ccsSpectrum Input spectrum matrix of type CV_32FC1, packed in OpenCV's CCS
 *                        (Complex Conjugate Symmetric) format (e.g., generated by cv::dft with DFT_REAL_OUTPUT).
 *
 * @param[in] filter Precomputed amplitude filter matrix of type CV_32FC1 and of the EXACT SAME SIZE
 *                   as ccsSpectrum.
 *                   CRITICAL LAYOUT REQUIREMENTS:
 *                   - Must be in NORMAL UNPACKED 2D FFT layout (NOT a CCS matrix).
 *                   - Contains only the REAL PLANE (magnitude/amplitude weights).
 *                   - DC component (zero frequency) must be located strictly at the top-left pixel (0,0).
 *                   - High frequencies (Nyquist) must converge toward the center of the matrix (rows/2, cols/2).
 *                   - All 4 quadrants must be explicitly present and symmetric relative to the Nyquist axes.
 *                   - To bake in an embedded fftShift, multiply the filter values by the alternating
 *                     sign mask beforehand during its generation.
 *
 * @param[out] _autoCrossSpectrum Output filtered autocorrelation spectrum matrix of type CV_32FC1
 *                                in CCS format.
 *
 * @return double The total integrated bandpass energy of the filtered spectrum,
 */
double fftAutoCrossSpectrumWeightedCCS(cv::InputArray ccsSpectrum, const cv::Mat1f & filter,
    cv::OutputArray _autoCrossSpectrum);




#endif /* __fft_h__ */
