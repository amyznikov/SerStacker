/*
 * extract_channel.cc
 *
 *  Created on: Jan 14, 2020
 *      Author: amyznikov
 */

#include "extract_channel.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>
#include <type_traits>
#include <core/debug.h>

namespace {

enum REDUCE_TYPES
{
  REDUCE_SUM = cv::REDUCE_SUM, //!< the output is the sum of all rows/columns of the matrix.
  REDUCE_AVG =  cv::REDUCE_AVG, //!< the output is the mean vector of all rows/columns of the matrix.
  REDUCE_MAX = cv::REDUCE_MAX, //!< the output is the maximum (column/row-wise) of all rows/columns of the matrix.
  REDUCE_MIN = cv::REDUCE_MIN,  //!< the output is the minimum (column/row-wise) of all rows/columns of the matrix.
  REDUCE_SUM2 = cv::REDUCE_SUM2, //!< the output is the sum of all squared rows/columns of the matrix.
  REDUCE_ABSMAX = cv::REDUCE_SUM2 + 1,
  REDUCE_FIRST_NON_ZERO,
  REDUCE_MAX_COLOR
};

}

template<>
const c_enum_member * members_of<color_channel_type>()
{
  static const c_enum_member members[] = {
      { color_channel_0, "0", "cv::extractChannel(0)" },
      { color_channel_1, "1", "cv::extractChannel(1)" },
      { color_channel_2, "2", "cv::extractChannel(2)" },
      { color_channel_3, "3", "cv::extractChannel(3)" },
      //{ color_channel_4, "4", "cv::extractChannel(4)" },

      { color_channel_dont_change, "dont_change", "Don't change color channels"},

      { color_channel_red, "red", "cv::extractChannel(2)"},
      { color_channel_green, "green", "cv::extractChannel(1)" },
      { color_channel_blue, "blue", "cv::extractChannel(0)"},

      { color_channel_gray, "gray", "cv::cvtColor(cv::COLOR_BGR2GRAY)"},
      { color_channel_luminance_YCrCb, "luminance_YCrCb", "cv::cvtColor(cv::COLOR_BGR2YCRCB) -> cv::extractChannel(0)"},
      { color_channel_luminance_lab, "luminance_lab", "cv::cvtColor(cv::COLOR_BGR2LAB)->cv::extractChannel(0)"},
      { color_channel_luminance_luv, "luminance_luv", "cv::cvtColor(cv::COLOR_BGR2Luv)->cv::extractChannel(0)"},
      { color_channel_luminance_hsv, "luminance_hsv", "cv::cvtColor(cv::COLOR_BGR2HSV)->cv::extractChannel(2)"},
      { color_channel_luminance_hls, "luminance_hls", "cv::cvtColor(cv::COLOR_BGR2HLS)->cv::extractChannel(0)"},


      { color_channel_min_inensity, "min", "cv::reduce(cv::REDUCE_MIN)"},
      { color_channel_max_intensity, "max", "cv::reduce(cv::REDUCE_MAX)"},
      { color_channel_avg_intensity, "avg", "cv::reduce(cv::REDUCE_AVG)"},
      { color_channel_sum_intensity, "sum", "cv::reduce(cv::REDUCE_SUM)"},
      { color_channel_sum2_intensity, "sum2", "cv::reduce(cv::REDUCE_SUM2)"},

      { color_channel_absmax , "absmax",  "max of absolute pixel value "},
      { color_channel_first_nonzero, "first_nonzero", "first nonzero value"},

      { color_channel_max_color, "max_color", "max - min"},
      //{ color_channel_max_gradient, "max_gradient", "max gradient"},

      { color_channel_unknown}
  };
  return members;
}

static inline double square(double x)
{
  return x * x;
}

template<class _Tp>
static bool _reduceChannels(cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _srcm, cv::OutputArray _dstm, REDUCE_TYPES rtype, int ddepth, bool autoscale)
{
  if ( _src.empty() || _src.channels() < 2 ) {
    // single-channel image, nothing to reduce, just copy to destination
    if ( _dst.needed() ) {
      convertScaleDepth(_src, _dst, ddepth, autoscale);
    }
    if ( _dstm.needed() ) {
      if ( _srcm.empty() || _srcm.channels() == 1 ) {
        _srcm.copyTo(_dstm);
      }
      else { // all mask channels are active
        reduce_color_channels(_srcm, _dstm, cv::REDUCE_MAX);
      }
    }

    return true;
  }

  if( !_srcm.empty() && _srcm.channels() != 1 && _srcm.channels() != _src.channels() ) { // Bad mask
    CF_ERROR("Invalid combination of image and masj channels: src.chaannels=%d mask.channels=%d",
        _src.channels(), _srcm.channels());
    return false;
  }

  const cv::Mat src = _src.getMat();
  const cv::Mat srcm = _srcm.getMat();
  const int src_depth = src.depth();
  const int src_channels = src.channels();

  // multi-channel image
  switch(rtype) {
    case REDUCE_SUM:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        reduce_color_channels(src, _dst, cv::REDUCE_SUM, src_depth == CV_64F ? CV_64F : CV_32F);
      }
      else {
        std::vector<cv::Mat> src_channels;
        std::vector<cv::Mat> mask_channels;
        cv::Mat dst = cv::Mat::zeros(src.size(), src_depth == CV_64F ? CV_64F : CV_32F);

        cv::split(src, src_channels);
        cv::split(srcm, mask_channels);

        for (int i = 0, cn = src.channels(); i < cn; ++i) {
          cv::add(dst, src_channels[i], dst, mask_channels[i], dst.depth());
        }
        _dst.move(dst);
        if ( _dstm.needed() ) {
          reduce_color_channels(srcm, _dstm, cv::REDUCE_MAX);
        }
      }

      convertScaleDepth(_dst, _dst, ddepth, autoscale, 1. / ( src_channels * getMaxValForPixelDepth(src_depth)) );
      break;

    case REDUCE_SUM2:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        reduce_color_channels(src, _dst, cv::REDUCE_SUM2, src_depth == CV_64F ? CV_64F : CV_32F);
      }
      else {
        using _dTp = std::conditional_t<std::is_same_v<_Tp, double>, double, float>;

        cv::Mat_<_dTp> dst(src.size(), _dTp(0)); // must be zero by design
        cv::Mat1b dstm(src.size(), uint8_t(0));

        const int cols = src.cols;
        const int cn = src.channels();

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols, cn](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            _dTp * dstp = dst[y];

            const uint8_t * mskp = srcm.ptr<const uint8_t>(y);
            uint8_t* dstmp = dstm.ptr<uint8_t>(y);

            for (int x = 0; x < cols; ++x) {
              _dTp sum = _dTp(0);
              bool found = false;

              for (int c = 0; c < cn; ++c) {
                if ( mskp[x * cn + c] ) {
                  const _dTp val = srcp[x * cn + c];
                  sum += val * val;
                  found = true;
                }
              }
              if ( found ) {
                dstp[x] = sum;
                dstmp[x] = uint8_t(255U);
              }
            }
          }});

        if( _dst.needed() ) {
          if( std::is_same_v<_Tp, double> ) {
            _dst.move(dst);
          }
          else {
            dst.convertTo(_dst, CV_32F);
          }
        }
        if( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }

      convertScaleDepth(_dst, _dst, ddepth, autoscale, 1. / (square(getMaxValForPixelDepth(src_depth)) * src_channels));
      break;

    case REDUCE_AVG:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        reduce_color_channels(src, _dst, cv::REDUCE_AVG, src_depth == CV_64F ? CV_64F : CV_32F);
      }
      else {
        std::vector<cv::Mat> src_channels;
        std::vector<cv::Mat> mask_channels;
        cv::Mat dst = cv::Mat::zeros(src.size(), src_depth == CV_64F ? CV_64F : CV_32F);
        cv::Mat counts = cv::Mat::zeros(src.size(), src_depth == CV_64F ? CV_64F : CV_32F);

        cv::split(src, src_channels);
        cv::split(srcm, mask_channels);

        for (int i = 0, cn = src.channels(); i < cn; ++i) {
          cv::add(dst, src_channels[i], dst, mask_channels[i], dst.depth());
          cv::add(counts, 1, counts, mask_channels[i], counts.depth());
        }

        cv::max(counts, cv::Scalar(1.0), counts);
        cv::divide(dst, counts, _dst );
        if ( _dstm.needed() ) {
          reduce_color_channels(srcm, _dstm, cv::REDUCE_MAX);
        }
      }
      convertScaleDepth(_dst, _dst, ddepth, autoscale, 1. / getMaxValForPixelDepth(src_depth));
      break;

    case REDUCE_MAX:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        reduce_color_channels(src, _dst, cv::REDUCE_MAX);
      }
      else {
        cv::Mat_<_Tp> dst(src.size(), _Tp(0));
        cv::Mat1b dstm(src.size(), uint8_t(0));

        const int cols = src.cols;
        const int cn = src.channels();

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols, cn](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            _Tp * dstp = dst[y];

            const uint8_t * mskp = srcm.ptr<const uint8_t>(y);
            uint8_t* dstmp = dstm.ptr<uint8_t>(y);

            for (int x = 0; x < cols; ++x) {
              _Tp max_val = _Tp(0);
              bool found = false;

              for (int c = 0; c < cn; ++c) {
                if ( mskp[x * cn + c] ) {
                  const _Tp val = srcp[x * cn + c];
                  if (!found || val > max_val) {
                    max_val = val;
                    found = true;
                  }
                }
              }
              if ( found ) {
                dstp[x] = max_val;
                dstmp[x] = uint8_t(255U);
              }
            }
          }});

        if ( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      convertScaleDepth(_dst, _dst, ddepth, autoscale);
      break;

    case REDUCE_MIN:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        reduce_color_channels(src, _dst, cv::REDUCE_MIN);
      }
      else {
        cv::Mat_<_Tp> dst(src.size(), _Tp(0)); // must be zero by design
        cv::Mat1b dstm(src.size(), uint8_t(0));

        const int cols = src.cols;
        const int cn = src.channels();

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols, cn](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            _Tp * dstp = dst[y];

            const uint8_t * mskp = srcm.ptr<const uint8_t>(y);
            uint8_t* dstmp = dstm.ptr<uint8_t>(y);

            for (int x = 0; x < cols; ++x) {
              _Tp min_val = _Tp(0);
              bool found = false;

              for (int c = 0; c < cn; ++c) {
                if ( mskp[x * cn + c] ) {
                  const _Tp val = srcp[x * cn + c];
                  if (!found || val < min_val) {
                    min_val = val;
                    found = true;
                  }
                }
              }
              if ( found ) {
                dstp[x] = min_val;
                dstmp[x] = uint8_t(255U);
              }
            }
          }});

        if ( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      convertScaleDepth(_dst, _dst, ddepth, autoscale);
      break;


    case REDUCE_MAX_COLOR:
      if ( srcm.empty() || srcm.channels() == 1 ) {
        cv::Mat cmin, cmax;
        reduce_color_channels(src, cmin, cv::REDUCE_MIN);
        reduce_color_channels(src, cmax, cv::REDUCE_MAX);
        cv::subtract(cmax, cmin, _dst);
      }
      else {
        cv::Mat_<_Tp> dst = cv::Mat_<_Tp>::zeros(src.size());
        cv::Mat1b dstm = cv::Mat1b::zeros(src.size());

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols = src.cols, cn = src.channels()](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            _Tp * dstp = dst[y];

            const uint8_t* mskp = srcm.ptr<const uint8_t>(y);
            uint8_t* dstmp = dstm[y];

            for (int x = 0; x < cols; ++x) {

              _Tp v_min = 0, v_max = 0;
              bool found = false;

              for (int c = 0; c < cn; ++c) {
                if ( mskp[x * cn + c] ) {
                  const _Tp val = srcp[x * cn + c];
                  if (!found) {
                    v_min = v_max = val;
                    found = true;
                  }
                  else {
                    v_min = std::min(v_min, val);
                    v_max = std::max(v_max, val);
                  }
                }
              }
              if (found) {
                dstp[x] = v_max - v_min;
                dstmp[x] = uint8_t(255U);
              }
            }
          }
        });

        if( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      convertScaleDepth(_dst, _dst, ddepth, autoscale);
      break;

    case REDUCE_ABSMAX: {
      if ( src.depth() == CV_8U || src.depth() == CV_16U ) { // fallback to REDUCE_MAX
        reduce_color_channels(src, _dst, cv::REDUCE_MAX);
      }
      else {
        std::vector<cv::Mat> channels;
        cv::split(src, channels);

        cv::Mat result = channels[0];
        cv::Mat max_abs = cv::abs(channels[0]);
        cv::Mat current_abs, mask;

        const int cn = src.channels();
        for (int i = 1; i < cn; ++i) {
          current_abs = cv::abs(channels[i]);
          cv::compare(current_abs, max_abs, mask, cv::CMP_GT);
          channels[i].copyTo(result, mask);
          current_abs.copyTo(max_abs, mask);
        }

        _dst.move(result);
      }
      convertScaleDepth(_dst, _dst, ddepth, autoscale);
      break;
    }

    case REDUCE_FIRST_NON_ZERO:
      if ( srcm.empty() ) {
        cv::Mat_<_Tp> dst = cv::Mat_<_Tp>::zeros(src.size());
        cv::Mat1b dstm = cv::Mat1b::zeros(src.size());

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols = src.cols, cn = src.channels()](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            _Tp * dstp = dst[y];
            uint8_t* dstmp = dstm[y];

            for (int x = 0; x < cols; ++x) {
              _Tp v = 0;
              for (int c = 0; c < cn; ++c) {
                if ((v = srcp[x * cn + c])) {
                  break;
                }
              }
              dstp[x] = v;
              dstmp[x] = v ? uint8_t(255U) : uint8_t(0);
            }
          }
        });

        if( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      else if ( srcm.channels() == 1 ) {
        cv::Mat_<_Tp> dst = cv::Mat_<_Tp>::zeros(src.size());
        cv::Mat1b dstm = cv::Mat1b::zeros(src.size());

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols = src.cols, cn = src.channels()](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            const uint8_t* mskp = srcm.ptr<const uint8_t>(y);
            _Tp * dstp = dst[y];
            uint8_t* dstmp = dstm[y];

            for (int x = 0; x < cols; ++x) {
              if ( mskp[x] ) {
                _Tp v = 0;
                for (int c = 0; c < cn; ++c) {
                  if ((v = srcp[x * cn + c])) {
                    break;
                  }
                }
                dstp[x] = v;
                dstmp[x] = v ? uint8_t(255U) : uint8_t(0) ;
              }
            }
          }
        });

        if( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      else {
        cv::Mat_<_Tp> dst(src.size());
        cv::Mat1b dstm (src.size());

        cv::parallel_for_(cv::Range(0, src.rows), [&, cols = src.cols, cn = src.channels()](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            const _Tp* srcp = src.ptr<const _Tp>(y);
            const uint8_t* mskp = srcm.ptr<const uint8_t>(y);
            _Tp * dstp = dst[y];
            uint8_t* dstmp = dstm[y];

            for (int x = 0; x < cols; ++x) {
              _Tp v = 0;
              for (int c = 0; c < cn; ++c) {
                if ( mskp[x * cn + c] ) {
                  if ((v = srcp[x * cn + c])) {
                    break;
                  }
                }
              }
              dstp[x] = v;
              dstmp[x] = v ? uint8_t(255U) : uint8_t(0) ;
            }
          }
        });

        if( _dst.needed() ) {
          _dst.move(dst);
        }
        if ( _dstm.needed() ) {
          _dstm.move(dstm);
        }
      }
      if ( _dst.needed() ) {
        convertScaleDepth(_dst, _dst, ddepth, autoscale);
      }
      break;

    default:
      CF_ERROR("Not supported reduce type %d requestd", rtype);
      return false;
  }

  return true;
}


static bool reduceChannels(cv::InputArray src, cv::OutputArray dst, cv::InputArray srcm, cv::OutputArray dstm, REDUCE_TYPES rtype, int ddepth, bool autoscale)
{
  switch (src.depth()) {
    case CV_8U: return _reduceChannels<uint8_t>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_8S: return _reduceChannels<int8_t>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_16U: return _reduceChannels<uint16_t>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_16S: return _reduceChannels<int16_t>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_32S: return _reduceChannels<int32_t>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_32F: return _reduceChannels<float>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
    case CV_64F: return _reduceChannels<double>(src, dst, srcm, dstm, rtype, ddepth, autoscale);
  }

  return false;
}

bool extract_channel(cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _srcm, cv::OutputArray _dstm,
    int channel, int ddepth, bool autoscale)
{
  if( ddepth < 0 ) {
    ddepth = _dst.fixedType() ? _dst.depth() : _src.depth();
  }

  if( channel == color_channel_dont_change || _src.channels() < 2 ) {

    if( _dst.needed() ) {
      if( _src.depth() == ddepth ) {
        _src.copyTo(_dst);
      }
      else {
        convertScaleDepth(_src, _dst, ddepth, autoscale);
      }
    }

    if( _dstm.needed() ) {
      if( _srcm.empty() || _srcm.channels() < 2 ) {
        _srcm.copyTo(_dstm);
      }
      else {
        reduce_color_channels(_srcm, _dstm, cv::REDUCE_MAX);
      }
    }

    return true;
  }

  const cv::Mat src = _src.getMat();
  const cv::Mat srcm = _srcm.getMat();

  cv::Mat dst, dstm;

  const auto extractChannelByIndex = [&](int coi) {
    if( _dst.needed() ) {
      if ( coi < 0 || coi >= src.channels() ) {
        CF_ERROR("Invalid channel index %d requested from image with %d channels ", coi, src.channels());
        return false;
      }
      cv::extractChannel(src, dst, coi);
      convertScaleDepth(dst, dst, ddepth, autoscale);
    }

    if( _dstm.needed() ) {
      if( srcm.empty() || srcm.channels() == 1 ) {
        srcm.copyTo(dstm);
      }
      else if ( coi < srcm.channels() ) {
        cv::extractChannel(srcm, dstm, coi);
      }
      else {
        CF_ERROR("Bad Input/Output mask channels combination: channel=%d srcm.channels()=%d", channel, srcm.channels());
        return false;
      }
    }

    return true;
  };

  const auto extractLumunanceChannel = [&](int colorSpace, int coi, double lumaScale, const std::string & colorSpaceName) {
    if( _dstm.needed() ) {
      if( srcm.empty() || srcm.channels() == 1 ) {
        srcm.copyTo(dstm);
      }
      else {
        CF_ERROR("Conversion to %s not supported for masks with %d channels", colorSpaceName.c_str(), srcm.channels());
        return false;
      }
    }

    if( _dst.needed() ) {
      if( src.channels() != 3 ) {
        CF_ERROR("Conversion to %s not supported for images with %d channels", colorSpaceName.c_str(), src.channels());
        return false;
      }

      cv::Mat s;

      static const std::set<int> special_cases = {
          cv::COLOR_BGR2Lab,
          cv::COLOR_BGR2Luv,
          cv::COLOR_BGR2HSV,
          cv::COLOR_BGR2HLS};

      const bool isSpecialCase = special_cases.find(colorSpace) != special_cases.end();

      if ( !isSpecialCase ) {
        s = src;
      }
      else {
        switch(src.depth()) {
          case CV_8U:
          case CV_32F: {
            s = src;
            break;
          }
          default : {
            double alpha, beta;
            getScaleOffset(src.depth(), CV_32F, &alpha, &beta);
            src.convertTo(s, CV_32F, alpha, beta);
            break;
          }
        }
      }

      cv::cvtColor(s, dst, colorSpace);
      if ( coi >= 0 ) {
        cv::extractChannel(dst, dst, coi);
      }

      convertScaleDepth(dst, dst, ddepth, autoscale, lumaScale);
    }

    return true;
  };

  switch (channel) {
    case color_channel_0:
    case color_channel_blue:
      if( !extractChannelByIndex(0) ) {
        return false;
      }
      break;

    case color_channel_1:
    case color_channel_green:
      if( !extractChannelByIndex(1) ) {
        return false;
      }
      break;

    case color_channel_2:
    case color_channel_red:
      if( !extractChannelByIndex(2) ) {
        return false;
      }
      break;

    case color_channel_3:
      if( !extractChannelByIndex(3) ) {
        return false;
      }
      break;

    case color_channel_gray:
      if( !extractLumunanceChannel(cv::COLOR_BGR2GRAY, -1, 1,  "Gray") ) {
        return false;
      }
      break;

    case color_channel_luminance_YCrCb:
      if( !extractLumunanceChannel(cv::COLOR_BGR2YCrCb, 0, 1,  "YCrCb") ) {
        return false;
      }
      break;

    case color_channel_luminance_lab: {
      if( !extractLumunanceChannel(cv::COLOR_BGR2Lab, 0, 0.01, "Lab") ) {
        return false;
      }
      break;

    case color_channel_luminance_luv:
      if( !extractLumunanceChannel(cv::COLOR_BGR2Luv, 0, 0.01, "Luv") ) {
        return false;
      }
      break;

    case color_channel_luminance_hsv:
      if( !extractLumunanceChannel(cv::COLOR_BGR2HSV, 2, 1, "HSV") ) {
        return false;
      }
      break;

    case color_channel_luminance_hls:
      if( !extractLumunanceChannel(cv::COLOR_BGR2HLS, 1, 1, "HLS") ) {
        return false;
      }
      break;

    case color_channel_min_inensity:
      if ( !reduceChannels(src, dst, srcm, dstm, REDUCE_MIN, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_max_intensity:
      if ( !reduceChannels(src, dst, srcm, dstm, REDUCE_MAX, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_avg_intensity:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_AVG, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_sum_intensity:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_SUM, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_sum2_intensity:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_SUM2, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_absmax:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_ABSMAX, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_first_nonzero:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_FIRST_NON_ZERO, ddepth, autoscale) ) {
        return false;
      }
      break;

    case color_channel_max_color:
      if( !reduceChannels(src, dst, srcm, dstm, REDUCE_MAX_COLOR, ddepth, autoscale) ) {
        return false;
      }
      break;

    default:
      CF_ERROR("Invalid color channel requested: %d", channel);
      return false;
    }
  }

  if( _dst.needed() ) {
    if( ddepth == dst.depth() ) {
      _dst.move(dst);
    }
    else {
      CF_DEBUG("APP BUG: unexpected convert!!!!");
      dst.convertTo(_dst, ddepth);
    }
  }

  if ( _dstm.needed() ) {
    _dstm.move(dstm);
  }

  return true;
}
