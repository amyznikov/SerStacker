/*
 * c_epipolar_matcher.cc
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#include "c_epipolar_matcher.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

#if HAVE_TBB
  #include <tbb/tbb.h>
  using tbb_range = tbb::blocked_range<int>;
#endif


using c_pixelblock = c_epipolar_matcher::c_pixelblock;

c_epipolar_matcher::c_epipolar_matcher()
{
}

c_epipolar_matcher::c_epipolar_matcher(const c_epipolar_matcher_options & opts) :
    options_(opts)
{

}

void c_epipolar_matcher::set_options(const c_epipolar_matcher_options & opts)
{
  options_ = opts;
}

const c_epipolar_matcher_options & c_epipolar_matcher::options() const
{
  return options_;
}

c_epipolar_matcher_options & c_epipolar_matcher::options()
{
  return options_;
}

bool c_epipolar_matcher::enabled() const
{
  return options_.enabled;
}

void c_epipolar_matcher::set_enabled(bool v)
{
  options_.enabled = v;
}

const cv::Mat & c_epipolar_matcher::current_median_hat() const
{
  return current_median_hat_;
}

const cv::Mat & c_epipolar_matcher::previous_median_hat() const
{
  return previous_median_hat_;
}

const cv::Mat1b & c_epipolar_matcher::current_median_hat_mask() const
{
  return current_median_hat_mask_;
}

const cv::Mat1b & c_epipolar_matcher::previous_median_hat_mask() const
{
  return previous_median_hat_mask_;
}

const cv::Mat2f & c_epipolar_matcher::matches() const
{
  return matches_;
}

const cv::Mat2i & c_epipolar_matcher::back_matches() const
{
  return back_matches_;
}

const cv::Mat1w & c_epipolar_matcher::costs() const
{
  return costs_;
}


bool c_epipolar_matcher::serialize(c_config_setting settings, bool save)
{

  SERIALIZE_OPTION(settings, save, options_, enabled);
  SERIALIZE_OPTION(settings, save, options_, median_hat_radius);
  SERIALIZE_OPTION(settings, save, options_, median_hat_threshold);
  SERIALIZE_OPTION(settings, save, options_, median_hat_close_radius);
  SERIALIZE_OPTION(settings, save, options_, max_disparity);

  return true;
}


bool c_epipolar_matcher::compute_block_array(cv::InputArray image, cv::InputArray mask,
    c_block_array * output_block_array,
    cv::Mat * output_median_hat,
    cv::Mat1b * output_median_hat_mask) const
{

  if( image.type() != CV_8UC3 ) {
    CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): CV_8UC3 image requited");
    return false;
  }

  if( !mask.empty() ) {
    if( mask.type() != CV_8UC1 ) {
      CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): mask type must be CV_8UC1");
      return false;
    }

    if( mask.size() != image.size() ) {
      CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): mask and image sizes must match");
      return false;

    }
  }

  if ( options_.median_hat_radius < 1 ) {
    CF_ERROR("ERROR: options_.median_hat_radius=%d", options_.median_hat_radius);
    return false;
  }


  cv::Mat median_hat_;
  cv::Mat1b median_hat_mask_;

  /// Create mask for pixels to be matched later

  cv::medianBlur(image, median_hat_, 2 * options_.median_hat_radius + 1);
  cv::absdiff(image, median_hat_, median_hat_);
  reduce_color_channels(median_hat_, median_hat_, cv::REDUCE_MAX);

  cv::compare(median_hat_, options_.median_hat_threshold, median_hat_mask_, cv::CMP_GE);

  if( options_.median_hat_close_radius > 0 ) {

    static thread_local cv::Mat SE;

    const int ksize =
        2 * options_.median_hat_close_radius + 1;

    if( SE.rows != ksize ) {
      SE = cv::getStructuringElement(cv::MORPH_ELLIPSE,
          cv::Size(ksize, ksize));
    }

    cv::morphologyEx(median_hat_mask_, median_hat_mask_,
        cv::MORPH_CLOSE,
        SE,
        cv::Point(-1, -1),
        1,
        cv::BORDER_REPLICATE);
  }

  if( !mask.empty() ) {
    cv::bitwise_and(median_hat_mask_, mask,
        median_hat_mask_);
  }

  if ( output_median_hat ) {
    * output_median_hat = median_hat_;
  }

  if ( output_median_hat_mask ) {
    * output_median_hat_mask = median_hat_mask_;
  }

  /// Extract descriptors for pixels to be matched later

  const cv::Mat3b src =
      image.getMat();

  output_block_array->create(src.size());

  const cv::Mat1b median_hat_values =
      median_hat_;

//#undef HAVE_TBB
#if HAVE_TBB
  constexpr int tbb_grain_size = 256;
  tbb::parallel_for(tbb_range(2, src.rows - 2, tbb_grain_size),
      [&](const tbb_range & range) {
        for ( int y = range.begin(); y < range.end(); ++y ) {
#else
        for ( int y = 2; y < src.rows - 2; ++y ) {
#endif

          c_pixelblock * descs =
              output_block_array->row(y);

          memset(descs, 0, sizeof(*descs) *
              output_block_array->cols());

          for ( int x = 2; x < src.cols - 2; ++x ) {
            //if ( median_hat_mask_[y][x] )
            {

              c_pixelblock & desc =
                  descs[x];

              static constexpr uint8_t window[5][5] = {
                  0, 1, 1, 1, 0,
                  1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1,
                  0, 1, 1, 1, 0,
              };

              int k = 0;

              for ( int yy = 0; yy < 5; ++yy ) {
                for ( int xx = 0; xx < 5; ++xx ) {
                  if ( window[yy][xx] ) {

                    desc.g[k++] = src[y + yy - 2][x + xx - 2][0];
                    desc.g[k++] = src[y + yy - 2][x + xx - 2][1];
                    desc.g[k++] = src[y + yy - 2][x + xx - 2][2];
                  }
                }
              }

              desc.g[k++] = median_hat_values[y][x];

              if ( k != sizeof(desc.g) ) {
                CF_ERROR("FATAL APP BUG: k=%d", k);
                exit (1);
              }

            }
          }

        }
#if HAVE_TBB
     });
#endif

  return true;
}


static inline uint16_t absdiff(const c_pixelblock & a, const c_pixelblock & b)
{
  uint16_t s = 0;

#if 0 // __AVX2__

  for( int i = 0; i < 2; ++i ) {

    union
    {
      __m256i m;
      uint16_t g[4];
    } u = {
        .m =
            _mm256_sad_epu8(_mm256_lddqu_si256((__m256i  const*) &a.g[i * 32]),
                _mm256_lddqu_si256((__m256i  const*) &b.g[i * 32]))
    };

    s += u.g[0];
  }

#elif 0 // __SSE__

    for( int i = 0; i < 8; ++i ) {

      union {
        __m64 m;
        uint16_t g[4];
      } u = {
          .m = _mm_sad_pu8(_m_from_int64(a.u64[i]), _m_from_int64(b.u64[i]))
      };

      s += u.g[0];
    }

#else

  for( int i = 0; i < 64; ++i ) {
    s += (uint16_t) (std::max(a.g[i], b.g[i]) - std::min(a.g[i], b.g[i]));
  }

#endif

  return s;
}

inline void c_epipolar_matcher::compute_search_range(const cv::Size & image_size, const cv::Point2d & E, int rx, int ry,
    int max_disparity, int * incx, int * incy, int * cxmax, int * cymax,  double * k)
{
  if ( rx >= E.x ) {
    if ( ry >= E.y ) {
      // corner 1
      if ( rx - E.x >= ry - E.y ) { // case H
        *incx = +1;
        *incy = 0;
        *k = (ry - E.y) / (rx - E.x);
        *cxmax = std::min(image_size.width - 2, rx + max_disparity);
      }
      else { // case V
        *incy = +1;
        *incx = 0;
        *k = (rx - E.x) / (ry - E.y);
        *cymax = std::min(image_size.height - 2, ry + max_disparity);
      }
    }
    else {
      // corner 0
      if ( rx - E.x >= E.y - ry ) { // case H
        *incx = +1;
        *incy = 0;
        *k = (ry - E.y) / (rx - E.x);
        *cxmax = std::min(image_size.width - 2, rx + max_disparity);
      }
      else { //case V
        *incx = 0;
        *incy = -1;
        *k = (rx - E.x) / (ry - E.y);
        *cymax = std::max(2, ry - max_disparity);
      }
    }
  }
  else {
    if ( ry > E.y ) { // corner 2
      if ( E.x - rx >= ry - E.y ) {
        *incx = -1;
        *incy = 0;
        *k = (ry - E.y) / (rx - E.x);
        *cxmax = std::max(2, rx - max_disparity);
      }
      else {
        *incx = 0;
        *incy = +1;
        *k = (rx - E.x) / (ry - E.y);
        *cymax = std::min(image_size.height - 2, ry + max_disparity);
      }
    }
    else { // corner 3
      if ( E.x - rx >= E.y - ry ) {
        *incx = -1;
        *incy = 0;
        *k = (ry - E.y) / (rx - E.x);
        *cxmax = std::max(2, rx - max_disparity);
      }
      else {
        *incx = 0;
        *incy = -1;
        *k = (rx - E.x) / (ry - E.y);
        *cymax = std::max(2, ry - max_disparity);
      }
    }
  }
}

bool c_epipolar_matcher::compute_matches(const c_block_array & current_image, const c_block_array & previous_image,
    const cv::Mat1b & mask, const cv::Point2d & E)
{

  INSTRUMENT_REGION("");


  const cv::Size size =
      previous_image.size();

  matches_.create(size);
  matches_.setTo(-1);

  back_matches_.create(size);
  back_matches_.setTo(-1);

  costs_.create(size);
  costs_.setTo(0);

  int max_disparity =
      options_.max_disparity;

  // loop for reference y
  for ( int ry = 2;  ry < size.height - 2; ++ry ) {
    // loop for reference x
    for ( int rx = 2;  rx < size.width - 2; ++rx ) {

      if ( !mask[ry][rx] ) {
        continue;
      }

      int best_cx = rx;
      int best_cy = ry;

      const c_pixelblock & referennce_descriptor =
          previous_image[ry][rx];

      uint16_t best_cost =
          absdiff(referennce_descriptor,
              current_image[ry][rx]);

      if ( best_cost > 0 ) {

        int incx, incy, cxmax, cymax;
        double k;

        compute_search_range(size, E, rx, ry, max_disparity,
            &incx, &incy, &cxmax, &cymax,
            &k);

        if ( incx ) { // H direction

          for( int cx = rx; cx != cxmax; cx += incx ) {

            const int cy = cvRound(E.y + k * (cx - E.x));
            if( cy < 2 || cy >= size.height - 2 ) {
              break;
            }

            if ( !mask[cy][cx] ) {
              continue;
            }

            const uint16_t current_cost =
                absdiff(referennce_descriptor,
                    current_image[cy][cx]);

            if( current_cost < best_cost ) {
              best_cx = cx;
              best_cy = cy;
              if( (best_cost = current_cost) == 0 ) {
                break;
              }
            }

          }

        }
        else { // V direction

          for( int cy = ry; cy != cymax; cy += incy ) {

            int cx = cvRound(E.x + k * (cy - E.y));
            if( cx < 2 || cx >= size.width - 2 ) {
              break;
            }

            if ( !mask[cy][cx] ) {
              continue;
            }

            const uint16_t current_cost =
                absdiff(referennce_descriptor,
                    current_image[cy][cx]);

            if( current_cost < best_cost ) {
              best_cx = cx;
              best_cy = cy;
              if( (best_cost = current_cost) == 0 ) {
                break;
              }
            }

          }

        }
      }

      if ( back_matches_[best_cy][best_cx][0] < 0 ) {
        matches_[ry][rx][0] = best_cx;
        matches_[ry][rx][1] = best_cy;
        back_matches_[best_cy][best_cx][0] = rx;
        back_matches_[best_cy][best_cx][1] = ry;
        costs_[ry][rx] = best_cost;
      }
      else {

        int brx = back_matches_[best_cy][best_cx][0];
        int bry = back_matches_[best_cy][best_cx][1];

        if ( best_cost < costs_[brx][brx] ) {

          matches_[bry][brx][0] = -1;
          matches_[bry][brx][1] = -1;
          // costs_[bry][brx] = 0;

          matches_[ry][rx][0] = best_cx;
          matches_[ry][rx][1] = best_cy;
          back_matches_[best_cy][best_cx][0] = rx;
          back_matches_[best_cy][best_cx][1] = ry;
          costs_[ry][rx] = best_cost;

        }
        else {
        }
      }
    }
  }

  return true;
}


cv::Mat1f matchesToEpipolarDisparity(const cv::Mat2f & matches, const cv::Point2d & E)
{
  const cv::Size size =
      matches.size();

  cv::Mat1f disparity(size, -1.f);

  static const auto hyp =
      [](double x, double y) -> double {
          return sqrt(x * x + y * y);
      };

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {
      if( matches[y][x][0] >= 0 && matches[y][x][1] >= 0 ) {

        disparity[y][x] =
            hyp(matches[y][x][0] - E.x, matches[y][x][1] - E.y) -
                hyp(x - E.x, y - E.y);

      }
    }
  }

  return disparity;
}
