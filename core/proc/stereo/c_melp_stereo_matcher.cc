/*
 * c_melp_stereo_matcher.cc
 *
 *  Created on: Jun 1, 2023
 *      Author: amyznikov
 */

#include "c_melp_stereo_matcher.h"
#include <core/proc/array2d.h>
#include <core/debug.h>
#include <tbb/tbb.h>
#include <xmmintrin.h>
#include <immintrin.h>

namespace {

#pragma pack(push, 8)
struct c_blockdesc
{
  // 3x3 BGR pixels, 32 bytes = 256 bits
  union {
    uint8_t  g[32];
    uint64_t u64[4];
  };
};
#pragma pack(pop)

class c_blockarray :
    public c_array2d<c_blockdesc>
{
public :
  typedef c_blockarray this_class;
  typedef c_array2d<c_blockdesc> base;

  c_blockarray()
  {
  }

  c_blockarray(int rows, int cols)
  {
    create(rows, cols);
  }

  c_blockarray(const cv::Size & s)
  {
    create(s);
  }

  c_blockarray(const c_blockarray & rhs)
  {
    this_class :: operator = (rhs);
  }
};


struct c_block_pyramid
{
  typedef c_block_pyramid this_class;
  typedef std::shared_ptr<this_class> sptr;
  c_blockarray image;
  c_block_pyramid::sptr l, m;
};

void image_to_blockarray(const cv::Mat & image, c_blockarray & a)
{
  const cv::Size size = image.size();

  cv::Mat3f img;
  cv::copyMakeBorder(image, img, 1, 1, 1, 1, cv::BORDER_REPLICATE);

  a.create(size);

  for( int y = 0; y < size.height; ++y ) {

    c_blockdesc *descp = a[y];
    const cv::Vec3f *imgp = img[y + 1];

    for( int x = 0; x < size.width; ++x ) {

      const cv::Vec3f &v = imgp[x + 1];
      c_blockdesc &desc = descp[x];

      int k = 0;

      for( int yy = y - 1; yy <= y + 1; ++yy ) {
        for( int xx = x - 1; xx <= x + 1; ++xx ) {
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

#if  __AVX2__

  union {
    __m256i m; // 256 bit = 32 bytes
    uint16_t g[16]; // 2 * 16 = 32 bytes
  } u = {
      .m =
          _mm256_sad_epu8(_mm256_lddqu_si256((__m256i const*) a.g),
              _mm256_lddqu_si256((__m256i const*) &b.g))
  };

  s = u.g[0];

#elif __SSE__

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

static void search_matches(const c_blockarray & left, const c_blockarray & right, cv::Mat1w & M)
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

            for( int xl = x; xl <= std::min(lwidth-1, x + 1); ++xl ) {

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

static void search_matches(const c_blockarray & left, const c_blockarray & right, const cv::Mat1w H[2], cv::Mat1w & M )
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

              for( int xl = std::max(x, xguess-1); xl <= std::min(lwidth - 1, xguess + 1); ++xl ) {

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
  image_to_blockarray(melp->image, bp->image);
  if( melp->l ) {
    bp->l.reset(new c_block_pyramid());
    recurse(bp->l, melp->l);
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




c_block_pyramid::sptr build_block_pyramid(cv::InputArray image)
{
  static constexpr int minimum_image_size = 4;

  if( image.type() != CV_8UC3 ) {
    CF_ERROR("Invalid left image type %d, must be CV_8UC3", image.type());
    return nullptr;
  }

  cv::Mat3f img;
  image.getMat().convertTo(img, CV_32F);

  c_melp_pyramid::sptr melp =
      build_melp_pyramid(img, minimum_image_size);

  return melp_to_block_pyramid(melp);
}



void compute_matches(const c_block_pyramid::sptr & lp, const c_block_pyramid::sptr & rp, cv::Mat1w & M)
{
  if( !rp->l ) {

    M.create(rp->image.size());

    for( int y = 0; y < M.rows; ++y ) {
      for( int x = 0; x < M.cols; ++x ) {
        M[y][x] = x;
      }
    }

    // search_matches(lp->image, rp->image, M);
  }
  else {

    cv::Mat1w MM[2];

    compute_matches(lp->l, rp->l, MM[0]);
    compute_matches(lp->m, rp->m, MM[1]);


    for( int i = 0; i < 2; ++i ) {
      cv::pyrUp(MM[i], MM[i], lp->image.size());
      cv::multiply(MM[i], 2, MM[i]);
    }

    M.create(lp->image.size());
    search_matches(lp->image, rp->image, MM, M);
  }

}

} // namespace

c_melp_stereo_matcher::c_melp_stereo_matcher()
{
}


bool c_melp_stereo_matcher::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  c_block_pyramid::sptr lp, rp;

  if( !(lp = build_block_pyramid(left)) ) {
    CF_ERROR("build_block_pyramid(left) fails");
    return false;
  }

  if( !(rp = build_block_pyramid(right)) ) {
    CF_ERROR("build_block_pyramid(right) fails");
    return false;
  }

  cv::Mat1w M;

  compute_matches(lp, rp, M);

  disparity.create(M.size(), CV_32FC1);

  cv::Mat1f D = disparity.getMatRef();

  for( int y = 0; y < M.rows; ++y ) {
    for( int x = 0; x < M.cols; ++x ) {
      D[y][x] = M[y][x] - x;
    }
  }

  return true;
}
