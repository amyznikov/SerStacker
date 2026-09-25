/*
 * c_fft_autosharp.h
 *
 *  Created on: Sep 25, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_autosharp_h__
#define __c_fft_autosharp_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

enum FFT_AUTOSHARP_INPAINT_METHOD {
  FFT_AUTOSHARP_INPAINT_DISABLED = 0,
  FFT_AUTOSHARP_LINEAR_INTERPOLATION_INPAINT,
  FFT_AUTOSHARP_AVERAGE_PYRAMID_INPAINT
};

enum FFT_AUTOSHARP_OUTPUT_DISPLAY {
  FFT_AUTOSHARP_DISPLAY_SRC_IMAGE = 0,
  FFT_AUTOSHARP_DISPLAY_RESTORED_IMAGE,
  FFT_AUTOSHARP_DISPLAY_P_SPECTRUM,
  FFT_AUTOSHARP_DISPLAY_S_SPECTRUM,
  FFT_AUTOSHARP_DISPLAY_V_SPECTRUM,
  FFT_AUTOSHARP_DISPLAY_INVERSE_FILTER,
  FFT_AUTOSHARP_DISPLAY_RESTORED_SPECTRUM,
};

/**
 * @struct c_fft_autosharp_options
 * @brief Configuration parameters for the FFT auto-sharpening pipeline.
 */
struct c_fft_autosharp_options
{
  double S1_target = -1.2;
  double macroStructSizePx = 150;
  enum FFT_AUTOSHARP_INPAINT_METHOD mask_inpaint_method = FFT_AUTOSHARP_INPAINT_DISABLED;
  bool autoS1_target = true;
};

struct c_fft_autosharp_debug_options
{
  std::string debug_file_name = "/home/projects/temp/fft_autosharp_profile.txt";
  bool print_debug_info = false;
  bool write_file = false;
};

bool serialize_fft_autosharp_options(c_config_setting section, bool save,
    c_fft_autosharp_options & opts);

inline bool save_settings(c_config_setting section, const c_fft_autosharp_options & opts)
{
  return serialize_fft_autosharp_options(section, true,
      const_cast<c_fft_autosharp_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_fft_autosharp_options * opts)
{
  return serialize_fft_autosharp_options(section, false, *opts);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls,
    const c_ctlbind_context<RootObjectType, c_fft_autosharp_options> & ctx)
{
  using S = c_fft_autosharp_options;
  ctlbind(ctls, "mask_inpaint_method", ctx(&S::mask_inpaint_method), "Mask inpaint method");
  ctlbind(ctls, "autoS1_target", ctx(&S::autoS1_target), "Try top auto estimate S1 target slope based on Macro structure size");
  ctlbind(ctls, "S1_target", ctx(&S::S1_target), "Target Slope of Restored Spectrum ");
  ctlbind(ctls, "macroStructSizePx", ctx(&S::macroStructSizePx), "Macro structure size in pixels");
}

/**
 * @class c_fft_autosharp
 * @brief FFT-based raw stack auto-sharpening.
 */
class c_fft_autosharp
{
public:
  bool compute(const c_fft_autosharp_options & opts, cv::InputArray srcImage, cv::InputArray srcMask,
      cv::OutputArray dstImage, cv::OutputArray dstMask,
      FFT_AUTOSHARP_OUTPUT_DISPLAY outputDisplay = FFT_AUTOSHARP_DISPLAY_RESTORED_IMAGE,
      const c_fft_autosharp_debug_options * debugOpts = nullptr);

  void clearCachedData();

public: // Direct access to internal cache data for debug and advaced visualization
  const std::vector<cv::Mat>& srcPlanes() const
  {
    return _src_planes;
  }
  const std::vector<cv::Mat1f>& srcP() const
  {
    return _src_p;
  }
  const std::vector<cv::Mat1f>& srcS() const
  {
    return _src_s;
  }
  const std::vector<cv::Mat1f>& srcChannelsRestored() const
  {
    return _src_channels_restored;
  }
  const cv::Mat1f& radialProfile() const
  {
    return _radial_profile;
  }
  const cv::Mat1f& inverseFilter() const
  {
    return _inverse_filter;
  }
  const cv::Mat1f& vlapFilter() const
  {
    return _vlap_filter;
  }

protected:
  cv::Mat _src_mask;
  std::vector<cv::Mat> _src_planes;
  std::vector<cv::Mat1f> _src_p, _src_s, _src_v;
  cv::Mat1f _radial_profile;
  cv::Mat1f _inverse_filter;
  std::vector<cv::Mat1f> _src_channels_restored;
  cv::Mat1f _vlap_filter;
};
#endif /* __c_fft_autosharp_h__ */
