/*
 * c_melp_stereo_matcher.cc
 *
 *  Created on: Jun 1, 2023
 *      Author: amyznikov
 */

#include "c_melp_stereo_matcher.h"
#include <core/debug.h>
#include <tbb/tbb.h>
#include <xmmintrin.h>
#include <immintrin.h>

using c_blockarray = c_melp_stereo_matcher::c_blockarray;
using c_blockdesc = c_melp_stereo_matcher::c_blockdesc;
using c_block_pyramid = c_melp_stereo_matcher::c_block_pyramid;

namespace {

void image_to_blockarray(const cv::Mat & image, c_blockarray & a)
{
  const cv::Size size = image.size();

  cv::Mat3f img;
  cv::copyMakeBorder(image, img, 1, 1, 1, 1, cv::BORDER_REPLICATE);

  a.create(size);

  for( int y = 0; y < size.height; ++y ) {

    c_blockdesc *descp = a[y];
    //const cv::Vec3f *imgp = img[y + 1];

    for( int x = 0; x < size.width; ++x ) {

      //const cv::Vec3f &v = imgp[x + 1];
      c_blockdesc &desc = descp[x];

      int k = 0;

      for( int yy = y - 1; yy <= y + 1; ++yy ) {
        for( int xx = x - 1; xx <= x + 1; ++xx ) {

          const cv::Vec3f &v = img[yy + 1][xx + 1];

          desc.g[k++] = cv::saturate_cast<uint8_t>(v[0]);
          desc.g[k++] = cv::saturate_cast<uint8_t>(v[1]);
          desc.g[k++] = cv::saturate_cast<uint8_t>(v[2]);
        }
      }

      while (k < sizeof(desc.g)) {
        desc.g[k++] = 0;
      }

    }
  }
}



inline uint16_t absdiff(const c_blockdesc & a, const c_blockdesc & b)
{
  uint16_t s;

#if 0 //  __AVX2__

  union {
    __m256i m; // 256 bit = 32 bytes
    uint16_t g[16]; // 2 * 16 = 32 bytes
  } u = {
      .m =
          _mm256_sad_epu8(_mm256_lddqu_si256((__m256i const*) a.g),
              _mm256_lddqu_si256((__m256i const*) &b.g))
  };

  s = u.g[0];

#elif 0 // __SSE__

  s = 0;
  for( int i = 0; i < 4; ++i ) {

    union {
      __m64 m; // 64 bit = 8 bytes
      uint16_t g[4]; // 2 * 4 = 8 bytes
    } u = {
        .m = _mm_sad_pu8(_m_from_int64(a.u64[i]), _m_from_int64(b.u64[i]))
    };

    s += u.g[0];
  }

#else

  s = 0;
  for( int i = 0; i < sizeof(a.g); ++i ) {
    s += (uint16_t) (std::max(a.g[i], b.g[i]) - std::min(a.g[i], b.g[i]));
  }

#endif

  return s;
}

static void search_matches(const c_blockarray & right, const c_blockarray & left, cv::Mat1w & M)
{
  typedef tbb::blocked_range<int> tbb_range;
  const int tbb_grain_size = 128;

  tbb::parallel_for(tbb_range(0, right.rows(), tbb_grain_size),
      [&](const tbb_range & range) {

        const int rwidth = right.cols();
        const int lwidth = left.cols();

        for( int y = range.begin(), ymax = range.end(); y < ymax; ++y ) {

          const c_blockdesc * lp = left[y];
          const c_blockdesc * rp = right[y];

          for( int x = 0; x < rwidth; ++x ) {

            uint16_t best_cost = UINT16_MAX;
            int best_xl = M[y][x];

            for( int xl = x; xl < lwidth - 1; ++xl ) {

              const uint16_t cost = absdiff(rp[x], lp[xl]);
              if( cost < best_cost ) {
                best_xl = xl;
                if ( !(best_cost = cost) ) {
                  break;
                }
              }
            }

            M[y][x] = best_xl;
          }
        }
      });

}

static void search_matches(const c_blockarray & right, const c_blockarray & left, const cv::Mat1w H[2], cv::Mat1w & M )
{
  typedef tbb::blocked_range<int> tbb_range;
  const int tbb_grain_size = 128;

  tbb::parallel_for(tbb_range(0, right.rows(), tbb_grain_size),
      [&](const tbb_range & range) {

        const int rwidth = right.cols();
        const int lwidth = left.cols();

        for( int y = range.begin(), ymax = range.end(); y < ymax; ++y ) {

          const c_blockdesc * lp = left[y];
          const c_blockdesc * rp = right[y];

          for( int x = 0; x < rwidth; ++x ) {

            uint16_t best_cost = UINT16_MAX;
            int best_xl = H[0][y][x];

            const int imax = H[0][y][x] == H[1][y][x] ? 1 : 2;

            for ( int i = 0; i < imax; ++i ) {

              const int xguess = H[i][y][x];

              for( int xl = std::max(x, xguess - 1); xl <= std::min(lwidth - 1, xguess + 1); ++xl ) {

                const uint16_t cost = absdiff(rp[x], lp[xl]);
                if( cost < best_cost ) {
                  best_xl = xl;
                  if ( !(best_cost = cost) ) {
                    break;
                  }
                }
              }
            }

            M[y][x] = best_xl;
          }
        }
      });
}

static void recurse(const c_block_pyramid::sptr & bp, const c_melp_pyramid::sptr & melp)
{
  image_to_blockarray(bp->image = melp->image, bp->a);
  if( melp->l ) {
    bp->g.reset(new c_block_pyramid());
    recurse(bp->g, melp->l);
  }
  if( melp->m ) {
    bp->m.reset(new c_block_pyramid());
    recurse(bp->m, melp->m);
  }
}

c_block_pyramid::sptr melp_to_block_pyramid(const c_melp_pyramid::sptr & melp)
{
  c_block_pyramid::sptr bp(new c_block_pyramid());
  recurse(bp, melp);
  return bp;
}



void compute_matches(const c_block_pyramid::sptr & lp, const c_block_pyramid::sptr & rp)
{
  if( !rp->g ) {

    rp->M.create(rp->a.size());

    for( int y = 0; y < rp->M.rows; ++y ) {
      for( int x = 0; x < rp->M.cols; ++x ) {
        rp->M[y][x] = x;
      }
    }

    search_matches(rp->a, lp->a, rp->M);
    lp->M = rp->M;
  }
  else {

    compute_matches(lp->g, rp->g);
    compute_matches(lp->m, rp->m);

    cv::pyrUp(rp->g->M, rp->MM[0], rp->a.size());
    cv::pyrUp(rp->m->M, rp->MM[1], rp->a.size());

    for( int i = 0; i < 2; ++i ) {
      cv::multiply(rp->MM[i], 2, rp->MM[i]);
      lp->MM[i] = rp->MM[i];
    }

    rp->M.create(lp->a.size());
    search_matches(lp->a, rp->a, rp->MM, rp->M);
    lp->M = rp->M;
  }
}

} // namespace


bool c_melp_stereo_matcher::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  cv::Mat3f left_img, right_img;

  if( left.type() != CV_8UC3 ) {
    CF_ERROR("Invalid left image type %d, must be CV_8UC3", left.type());
    return false;
  }


  left.getMat().convertTo(left_img, CV_32F);
  if ( !(lmelp_ = build_melp_pyramid(left_img, minimum_image_size_))) {
    CF_ERROR("build_melp_pyramid(left_img) fails");
    return false;
  }

  right.getMat().convertTo(right_img, CV_32F);
  if ( !(rmelp_ = build_melp_pyramid(right_img, minimum_image_size_))) {
    CF_ERROR("build_melp_pyramid(right_img) fails");
    return false;
  }

  if( !(lp_ = melp_to_block_pyramid(lmelp_)) ) {
    CF_ERROR("melp_to_block_pyramid(lmelp_) fails");
    return false;
  }

  if( !(rp_ = melp_to_block_pyramid(rmelp_)) ) {
    CF_ERROR("melp_to_block_pyramid(rmelp_) fails");
    return false;
  }

  compute_matches(lp_, rp_);

  if ( disparity.needed() ) {
    disparity.create(rp_->M.size(), CV_32FC1);

    cv::Mat1f D = disparity.getMatRef();

    for( int y = 0; y < rp_->M.rows; ++y ) {
      for( int x = 0; x < rp_->M.cols; ++x ) {
        D[y][x] = rp_->M[y][x] - x;
      }
    }
  }

  return true;
}

void c_melp_stereo_matcher::sad(int disp,
    const c_block_pyramid::sptr & lp,
    const c_block_pyramid::sptr & rp,
    cv::Mat1f & dists)
{
  const c_blockarray &la =
      lp->a;

  const c_blockarray &ra =
      rp->a;

  const int rows =
      rp->a.rows();

  const int cols =
      rp->a.cols();

  if ( dists.size() != rp->a.size() ) {
    dists.create(rp->a.size());
  }

  dists.setTo(0);

  for( int y = 0; y < rows; ++y ) {

    for( int x = 0; x < cols - disp; ++x ) {

      dists[y][x] =
          absdiff(ra[y][x],
              la[y][x + disp]);
    }
  }
}

