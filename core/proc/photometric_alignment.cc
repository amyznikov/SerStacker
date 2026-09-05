/*
 * photometric_alignment.cc
 *
 *  Created on: Sep 4, 2026
 *      Author: amyznikov
 */
#include <core/proc/photometric_alignment.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>
#include <atomic>
#include <cmath>

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
template<class _Tp>
static bool _estimatePhotometricAlignment(cv::InputArray currentImage,
    cv::InputArray referenceImage, cv::InputArray commonPixelsMask,
    cv::Scalar & outputBrightness,
    cv::Scalar & outputContrast,
    bool includeBrightness,
    bool includeContrast,
    bool separateChannels,
    double eps /*= 1e-6*/)
{

  const cv::Mat cur = currentImage.getMat();
  const cv::Mat ref = referenceImage.getMat();
  const cv::Mat mask = commonPixelsMask.getMat();

  const int rows = cur.rows;
  const int cols = cur.cols;
  const int cur_channels = cur.channels();
  const int target_channels = separateChannels ? cur_channels : 1;

  const uint8_t * cur_base = cur.ptr();
  const uint8_t * ref_base = ref.ptr();
  const uint8_t * mask_base = mask.empty() ? nullptr : mask.ptr();

  const size_t cur_stride = cur.step;
  const size_t ref_stride = ref.step;
  const size_t mask_stride = mask.step;

  std::atomic<double> sumC[4]  = { {0.0}, {0.0}, {0.0}, {0.0} };
  std::atomic<double> sumR[4]  = { {0.0}, {0.0}, {0.0}, {0.0} };
  std::atomic<double> sumCR[4] = { {0.0}, {0.0}, {0.0}, {0.0} };
  std::atomic<double> sumCC[4] = { {0.0}, {0.0}, {0.0}, {0.0} };
  std::atomic<int> counts[4] = { {0}, {0}, {0}, {0} };

  parallel_for(0, rows, [=, &sumC, &sumR, &sumCR, &sumCC, &counts](const auto & range) {

    const int y0 = rbegin(range);
    const int y1 = rend(range);

    const uint8_t* cp_base = cur_base + y0 * cur_stride;
    const uint8_t* rp_base = ref_base + y0 * ref_stride;

    double loc_sumC[4] = {0.0};
    double loc_sumR[4] = {0.0};
    double loc_sumCR[4] = {0.0};
    double loc_sumCC[4] = {0.0};
    int loc_count[4] = {0};

    if (!mask_base) { // NO MASK GIVEN, USE ALL PIXELS

      for (int y = y0; y < y1; ++y, cp_base += cur_stride, rp_base += ref_stride) {
        const _Tp* cp = (const _Tp*)cp_base;
        const _Tp* rp = (const _Tp*)rp_base;

        if (separateChannels) {
          const int total_elements = cols * cur_channels;
          for (int i = 0; i < total_elements; ++i) {
            const double valC = cp[i];
            const double valR = rp[i];
            // For channels 1-4 the compiler optimizes the remainder of the division ?
            const int ch = i % cur_channels;
            loc_count[ch]++;
            if (includeBrightness) {
              loc_sumC[ch] += valC;
              loc_sumR[ch] += valR;
            }
            if (includeContrast)   {
              loc_sumCC[ch] += valC * valC;
              loc_sumCR[ch] += valC * valR;
            }
          }
          cp += total_elements;
          rp += total_elements;
        }
        else if (cur_channels == 1) {
          for (int x = 0; x < cols; ++x) {
            const double valC = *cp++;
            const double valR = *rp++;
            loc_count[0]++;
            if (includeBrightness) {
              loc_sumC[0] += valC;
              loc_sumR[0] += valR; }
            if (includeContrast)   {
              loc_sumCC[0] += valC * valC;
              loc_sumCR[0] += valC * valR;
            }
          }
        }
        else {
          for (int x = 0; x < cols; ++x) {
            double valC = 0.0, valR = 0.0;
            for (int ch = 0; ch < cur_channels; ++ch) {
              valC += *cp++;
              valR += *rp++;
            }
            valC /= cur_channels;
            valR /= cur_channels;

            loc_count[0]++;
            if (includeBrightness) {
              loc_sumC[0] += valC;
              loc_sumR[0] += valR;
            }
            if (includeContrast)   {
              loc_sumCC[0] += valC * valC;
              loc_sumCR[0] += valC * valR;
            }
          }
        }
      }
    }
    else { // MASK IS GIVEN
      const uint8_t* mp_base = mask_base + y0 * mask_stride;

      for (int y = y0; y < y1; ++y, cp_base += cur_stride, rp_base += ref_stride, mp_base += mask_stride) {
        const _Tp* cp = (const _Tp*)cp_base;
        const _Tp* rp = (const _Tp*)rp_base;
        const uchar* mp = mp_base;

        if (separateChannels) { // BGR + Mask
          for (int x = 0; x < cols; ++x) {
            if (!mp[x]) {
              cp += cur_channels;
              rp += cur_channels;
            }
            else {
              for (int ch = 0; ch < cur_channels; ++ch) {
                const double valC = *cp++;
                const double valR = *rp++;
                loc_count[ch]++;
                if (includeBrightness) {
                  loc_sumC[ch] += valC;
                  loc_sumR[ch] += valR;
                }
                if (includeContrast) {
                  loc_sumCC[ch] += valC * valC;
                  loc_sumCR[ch] += valC * valR;
                }
              }
            }
          }
        }
        else { // Grayscale + Mask
          for (int x = 0; x < cols; ++x) {
            if (!mp[x]) {
              cp += cur_channels;
              rp += cur_channels;
            }
            else {
              double valC = 0.0, valR = 0.0;
              for (int ch = 0; ch < cur_channels; ++ch) {
                  valC += *cp++;
                  valR += *rp++;
              }
              if (cur_channels > 1) {
                valC /= cur_channels;
                valR /= cur_channels;
              }
              loc_count[0]++;
              if (includeBrightness) {
                loc_sumC[0] += valC;
                loc_sumR[0] += valR;
              }
              if (includeContrast)   {
                loc_sumCC[0] += valC * valC;
                loc_sumCR[0] += valC * valR;
              }
            }
          }
        }
      }
    }

    for (int ch = 0; ch < target_channels; ++ch) {
      if (loc_count[ch] > 0) {
        counts[ch].fetch_add(loc_count[ch], std::memory_order_relaxed);

        if (includeBrightness) {
          double currentC = sumC[ch].load(std::memory_order_relaxed);
          while (!sumC[ch].compare_exchange_weak(currentC, currentC + loc_sumC[ch],
              std::memory_order_relaxed));

          double currentR = sumR[ch].load(std::memory_order_relaxed);
          while (!sumR[ch].compare_exchange_weak(currentR, currentR + loc_sumR[ch],
              std::memory_order_relaxed));
        }
        if (includeContrast) {
          double currentCC = sumCC[ch].load(std::memory_order_relaxed);
          while (!sumCC[ch].compare_exchange_weak(currentCC, currentCC + loc_sumCC[ch],
              std::memory_order_relaxed));

          double currentCR = sumCR[ch].load(std::memory_order_relaxed);
          while (!sumCR[ch].compare_exchange_weak(currentCR, currentCR + loc_sumCR[ch],
              std::memory_order_relaxed));
        }
      }
    }
  });


  for( int ch = 0; ch < std::min(4, target_channels); ++ch ) {
    const int N = counts[ch].load(std::memory_order_relaxed);
    if( N < 1 ) {
      return false;
    }

    if( includeBrightness && includeContrast ) {
      const double meanC = sumC[ch].load(std::memory_order_relaxed) / N;
      const double meanRef = sumR[ch].load(std::memory_order_relaxed) / N;
      const double meanCC = sumCC[ch].load(std::memory_order_relaxed) / N;
      const double meanCR = sumCR[ch].load(std::memory_order_relaxed) / N;

      const double varC = meanCC - (meanC * meanC);
      if( varC < eps ) {
        outputContrast[ch] = 1.0;
        outputBrightness[ch] = meanRef - meanC;
      }
      else {
        const double cov = meanCR - (meanC * meanRef);
        outputContrast[ch] = cov / varC;
        outputBrightness[ch] = meanRef - (outputContrast[ch] * meanC);
      }
    }
    else if( !includeBrightness && includeContrast ) {
      const double CC = sumCC[ch].load(std::memory_order_relaxed);
      if( std::abs(CC) < eps ) {
        outputContrast[ch] = 1.0;
        outputBrightness[ch] = 0.0;
      }
      else {
        outputContrast[ch] = sumCR[ch].load(std::memory_order_relaxed) / CC;
        outputBrightness[ch] = 0.0;
      }
    }
    else if( includeBrightness && !includeContrast ) {
      outputContrast[ch] = 1.0;
      outputBrightness[ch] = (sumR[ch].load(std::memory_order_relaxed)
          - sumC[ch].load(std::memory_order_relaxed)) / N;
    }
  }

  if( !separateChannels && cur_channels > 1 ) {
    for( int ch = 1; ch < cur_channels; ++ch ) {
      outputContrast[ch] = outputContrast[0];
      outputBrightness[ch] = outputBrightness[0];
    }
  }

  return true;
}

bool estimatePhotometricAlignment(cv::InputArray currentImage,
    cv::InputArray referenceImage, cv::InputArray commonPixelsMask,
    cv::Scalar & outputBrightness,
    cv::Scalar & outputContrast,
    bool includeBrightness,
    bool includeContrast,
    bool separateChannels,
    double eps /*= 1e-6*/)
{
  if( currentImage.size() != referenceImage.size() ) {
    CF_ERROR("currentImage.size() = %dx%d not match to referenceImage.size()=%dx%d",
        currentImage.cols(), currentImage.rows(),
        referenceImage.cols(), referenceImage.rows());
    return false;
  }

  if( currentImage.depth() != referenceImage.depth() ) {
    CF_ERROR("currentImage.depth() = %d not match to referenceImage.depth()=%d",
        currentImage.depth(), referenceImage.depth());
    return false;
  }

  if( currentImage.channels() != referenceImage.channels() ) {
    CF_ERROR("currentImage.channels() = %d not match to referenceImage.channels()=%d",
        currentImage.channels(), referenceImage.channels());
    return false;
  }

  if( !commonPixelsMask.empty() ) {
    if ( commonPixelsMask.type() != CV_8UC1 ) {
      CF_ERROR("Bad commonPixelsMask.type = %d (depth=%d channels=%d). Nust be CV_8UC1",
          commonPixelsMask.type(), commonPixelsMask.depth(), commonPixelsMask.channels());
      return false;
    }

    if ( commonPixelsMask.size() != currentImage.size()) {
      CF_ERROR("currentImage.size() = %dx%d not match to commonPixelsMask.size()=%dx%d",
          currentImage.cols(), currentImage.rows(),
          commonPixelsMask.cols(), commonPixelsMask.rows());
      return false;
    }
  }

  CV_DISPATCH(currentImage.depth(), _estimatePhotometricAlignment,
      currentImage, referenceImage, commonPixelsMask,
      outputBrightness,
      outputContrast,
      includeBrightness,
      includeContrast,
      separateChannels,
      eps);

  CF_ERROR("Not supported image depth = %d", currentImage.depth());
  return false;
}
