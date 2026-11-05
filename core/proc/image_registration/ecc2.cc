/*
 * ecc2.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */
#include "ecc2.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
//#include <core/io/save_image.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<ECC_INTERPOLATION_METHOD>()
{
  static const c_enum_member members[] = {
      { ECC_INTER_LINEAR, "LINEAR", "" },
      { ECC_INTER_LINEAR_EXACT, "LINEAR_EXACT", "" },
      { ECC_INTER_AREA, "AREA", "" },
      { ECC_INTER_CUBIC, "CUBIC", "" },
      { ECC_INTER_LANCZOS4, "LANCZOS4", "" },
      { ECC_INTER_NEAREST, "NEAREST", "" },
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0)
      { ECC_INTER_NEAREST_EXACT, "NEAREST_EXACT", "" },
#endif
      { ECC_INTER_NEAREST }  // must be last
  };
  return members;
}

template<>
const c_enum_member * members_of<ECC_BORDER_MODE>()
{
  static const c_enum_member members[] = {
      { ECC_BORDER_REFLECT101, "BORDER_REFLECT101", },
      { ECC_BORDER_REFLECT, "BORDER_REFLECT", },
      { ECC_BORDER_REPLICATE, "BORDER_REPLICATE", },
      { ECC_BORDER_WRAP, "BORDER_WRAP", },
      { ECC_BORDER_CONSTANT, "BORDER_CONSTANT", },
      { ECC_BORDER_TRANSPARENT, "BORDER_TRANSPARENT", },
      { ECC_BORDER_ISOLATED, "BORDER_ISOLATED", },
      { ECC_BORDER_DEFAULT, }  // must be last
  };
  return members;
}

template<>
const c_enum_member * members_of<ECC_ALIGN_METHOD>()
{
  static const c_enum_member members[] = {
    {ECC_ALIGN_INVERSE_COMPOSITIONAL_LM, "INVERSE_COMPOSITIONAL", ""},
    {ECC_ALIGN_FORWARD_ADDITIVE_LM, "FORWARD_ADDITIVE", ""},
    {ECC_ALIGN_INVERSE_COMPOSITIONAL_LM},
  };
  return members;
}

template<>
const c_enum_member * members_of<ECCH_PYRAMID_TYPE>()
{
  static const c_enum_member members[] = {
    {ECCH_PYRAMID_PYRDOWN, "ECCH_PYRAMID_PYRDOWN", "Use cv::pyrDown() to downscale image"},
    {ECCH_PYRAMID_SCHARR, "ECCH_PYRAMID_SCHARR", "Use pyrDown on Scharr magnitudes"},
    {ECCH_PYRAMID_PYRDOWN},
  };
  return members;
}

static inline const cv::Mat1b & morphse33()
{
  static const cv::Mat1b SE(3, 3, 255);
  return SE;
}

static inline const cv::Mat1b & morphse55()
{
  static const cv::Mat1b SE(5, 5, 255);
  return SE;
}

// Ugly construction to avoid throw exceptions back to the calling code.
static inline bool ecc_solve(cv::InputArray src1, cv::InputArray src2, cv::OutputArray dst)
{
  bool fOk = false;
  try {
    if( !(fOk = cv::solve(src1, src2, dst, cv::DECOMP_CHOLESKY)) ) {
      fOk = cv::solve(src1, src2, dst, cv::DECOMP_SVD);
    }
  }
  catch(...)
  {}

  return fOk;
}

static double ecc_compute_rhs(const cv::Mat1f & reference_image, const cv::Mat1f & remapped_current_image,
    const cv::Mat1b & zmask,
    /*output */ cv::Mat1f & rhs,
    /*output */ double * K /*= nullptr*/)
{
  INSTRUMENT_REGION("");

  const cv::Size size = reference_image.size();
  rhs.create(size);

  const uint8_t * zmask_base = zmask.empty() ? nullptr : zmask.ptr();
  const size_t zmask_stride = zmask.empty() ? 0 : zmask.step;

  const uint8_t * ref_base = reference_image.ptr();
  const size_t ref_stride = reference_image.step;

  const uint8_t * cur_base = remapped_current_image.ptr();
  const size_t cur_stride = remapped_current_image.step;

  uint8_t * rhs_base = (uint8_t*) rhs.ptr();
  const size_t rhs_stride = rhs.step;

  alignas(std::hardware_destructive_interference_size)
  std::atomic<float> total_l2sqr(0.0f);

  alignas(std::hardware_destructive_interference_size)
  std::atomic<float> total_npix(0.0f);

  parallel_for(0, size.height, [=, &total_l2sqr, &total_npix](const auto & range) {
    float local_l2sqr = 0.0;
    float local_npix = 0.0;

    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

      const float * __restrict refp = (const float * )(ref_base + y * ref_stride);
      const float * __restrict curp = (const float * )(cur_base + y * cur_stride);
      float * __restrict rhsp = (float * )(rhs_base + y * rhs_stride);

      if ( !zmask_base ) {
        for ( int x = 0; x < size.width; ++x ) {
          const float val = refp[x] - curp[x];
          rhsp[x] = val;
          local_l2sqr += val * val;
        }
      }
      else {
        const uint8_t * __restrict zmaskp = (const uint8_t * )(zmask_base + y * zmask_stride);

        for ( int x = 0; x < size.width; ++x ) {
          if ( zmaskp[x] ) {
            rhsp[x] = 0.0f;
          }
          else {
            const float val = refp[x] - curp[x];
            rhsp[x] = val;
            local_l2sqr += val * val;
            local_npix += 1.0f;
          }
        }
      }
    }

    if ( local_l2sqr > 0.0f ) {
      float current = total_l2sqr.load(std::memory_order_relaxed);
      while (!total_l2sqr.compare_exchange_weak(current, current + local_l2sqr,
          std::memory_order_relaxed));
    }

    if ( local_npix > 0.0f ) {
      float current = total_npix.load(std::memory_order_relaxed);
      while (!total_npix.compare_exchange_weak(current, current + local_npix,
          std::memory_order_relaxed));
    }
  });

  const double l2sqr = total_l2sqr.load();
  const double scale = zmask_base ? size.area() / (double)total_npix.load() : 1.0;

  if( K ) {
    *K = scale;
  }

  return l2sqr * scale;
}

// zmask must be inverse of image mask (label bad pixels with 255)
static void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, cv::InputArray zmask = cv::noArray())
{
  INSTRUMENT_REGION("");

  // 4th order derivative vector (5x1)
  // 2nd order smoothing  vector (3x1)
  static const cv::Matx<float, 5, 1> d5( +1.f/12.f, -2.f/3.f, 0.f, +2.f/3.f, -1.f/12.f );
  static const cv::Matx<float, 3, 1> s3( 0.25f, 0.5f, 0.25f );

  parallel_invoke(
      [&]() {
        cv::sepFilter2D(src, gx, CV_32F, d5, s3, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !zmask.empty() ) {
          gx.setTo(0, zmask);
        }
      },
      [&]() {
        cv::sepFilter2D(src, gy, CV_32F, s3, d5, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !zmask.empty() ) {
          gy.setTo(0, zmask);
        }
      });
}


template<class _Tp>
static double _ecc_compute_correlation(cv::InputArray _src1, cv::InputArray _src2, cv::InputArray _mask)
{
  if( _src1.size() != _src2.size() || _src1.channels() != 1 || _src1.channels() != _src2.channels() ) {
    CF_ERROR("Bad input images: must be single-channel of the same size");
    return -1.0;
  }

  if( !_mask.empty() ) {
    if( _mask.size() != _src1.size() || _mask.type() != CV_8UC1 ) {
      CF_ERROR("Bad input mask");
      return -1.0;
    }
  }

  const cv::Mat src1 = _src1.getMat();
  const cv::Mat src2 = _src2.getMat();
  const cv::Mat mask = _mask.getMat();

  const cv::Size size = src1.size();

  const uint8_t * src1_base = src1.ptr();
  const size_t src1_stride  = src1.step;

  const uint8_t * src2_base = src2.ptr();
  const size_t src2_stride  = src2.step;

  const uint8_t * mask_base   = mask.empty() ? nullptr : mask.ptr();
  const size_t mask_stride    = mask.empty() ? 0 : mask.step;

  struct Accumulator {
    double sumX = 0;
    double sumY = 0;
    double sumXX = 0;
    double sumYY = 0;
    double sumXY = 0;
    double npix = 0;
  } acc;

  std::mutex mtx;

  parallel_for(0, size.height, [=, &acc, &mtx](const auto & range) {

    using _Tcomp = std::conditional_t<std::is_floating_point_v<_Tp>, _Tp, float>;

    _Tcomp sumX = 0;
    _Tcomp sumY = 0;
    _Tcomp sumXX = 0;
    _Tcomp sumYY = 0;
    _Tcomp sumXY = 0;
    _Tcomp npix = 0;

    for ( int y = rbegin(range); y < rend(range); ++y ) {
      const _Tp * __restrict srcp1 = (const _Tp * )(src1_base + y * src1_stride);
      const _Tp * __restrict srcp2 = (const _Tp * )(src2_base + y * src2_stride);

      if ( !mask_base ) {
        for ( int x = 0; x < size.width; ++x ) {
          const _Tcomp x_val = _Tcomp(srcp1[x]);
          const _Tcomp y_val = _Tcomp(srcp2[x]);
          sumX += x_val;
          sumY += y_val;
          sumXX += x_val * x_val;
          sumYY += y_val * y_val;
          sumXY += x_val * y_val;
        }
      }
      else {
        const uint8_t * __restrict pm = mask_base ? (const uint8_t * )(mask_base + y * mask_stride) : nullptr;
        for ( int x = 0; x < size.width; ++x ) {
          if ( pm[x] ) {
            const _Tcomp x_val = _Tcomp(srcp1[x]);
            const _Tcomp y_val = _Tcomp(srcp2[x]);
            sumX += x_val;
            sumY += y_val;
            sumXX += x_val * x_val;
            sumYY += y_val * y_val;
            sumXY += x_val * y_val;
            npix += 1.0;
          }
        }
      }
    }

    std::lock_guard<std::mutex> lock(mtx);
    acc.sumX  += sumX;
    acc.sumY  += sumY;
    acc.sumXX += sumXX;
    acc.sumYY += sumYY;
    acc.sumXY += sumXY;
    acc.npix  += npix;
  });


  const double npx = mask.empty() ?  size.area() : acc.npix;
  if ( npx < 2 ) {
    return -1.0;
  }

  const double meanX = acc.sumX / npx;
  const double meanY = acc.sumY / npx;
  const double varX = acc.sumXX / npx - meanX * meanX;
  const double varY = acc.sumYY / npx - meanY * meanY;
  const double covar = acc.sumXY / npx - meanX * meanY;
  constexpr double safe_min = std::numeric_limits<double>::min();
  const double total_correlation = (varX > safe_min && varY > safe_min) ?
      covar / (std::sqrt(varX) * std::sqrt(varY)) : 0.0;

  return total_correlation;
}

double ecc_compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask)
{
  INSTRUMENT_REGION("");
  if( src1.type() != src2.type() ) {
    CF_ERROR("src1 and src2 types must be equal");
    return -1.0;
  }

  CV_DISPATCH(src1.depth(), _ecc_compute_correlation, src1, src2, mask);
  CF_ERROR("Not supported image depth=%d", src1.depth());
  return -1.0;
}

double ecc_compute_correlation(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const cv::Mat2f & rmap)
{
  INSTRUMENT_REGION("");
  if( current_image.type() != reference_image.type() ) {
    CF_ERROR("src1 and src2 types must be equal");
    return -1.0;
  }

  if( current_image.channels() != 1 ) {
    CF_ERROR("mutlt-channel input images are not supported");
    return -1.0;
  }

  cv::Mat remapped_image, remapped_mask;

  cv::remap(current_image, remapped_image,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  if ( !current_mask.empty() ) {
    cv::remap(current_mask, remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));
  }
  else {
    cv::remap(cv::Mat1b(current_image.size(), (uint8_t) 255), remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));
  }

  if ( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  return ecc_compute_correlation(reference_image, remapped_image, remapped_mask);
}

/*
 * Pyramid down to specific level
 */
static bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);
  for( int l = 1; l < level; ++l ) {
    cv::pyrDown(dst, dst, cv::Size(), border_mode);
  }

  return true;
}

/*
 * Pyramid up to specific size
 */
static bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {

      const cv::Size nextSize((sizes.back().width + 1) / 2,
          (sizes.back().height + 1) / 2);

      if( nextSize == inputSize ) {
        break;
      }

      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }

      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}

/**
 * Compute Hessian matrix for ECC image alignment
 */
static void ecc_compute_hessian_matrix(const std::vector<cv::Mat1f> & J, cv::Mat1f & H/*, int nparams*/)
{
  INSTRUMENT_REGION("");

  const int M = J.size();
  H.create(M, M);

  struct index_t { int i, j; } index[(M * (M + 1)) / 2];
  int nidx = 0;

  for( int i = 0; i < M; ++i ) {
    for( int j = 0; j <= i; ++j ) {
      index[nidx++] = index_t{i, j};
    }
  }

  parallel_loop(0, nidx, [&](int p) {
    const int i = index[p].i, j = index[p].j;
    H(i,j) = J[i].dot(J[j]);
  });

  for( int i = 0; i < M; ++i ) {
    for( int j = i + 1; j < M; ++j ) {
      H[i][j] = H[j][i];
    }
  }

}

/*
 * Project error image to jacobian for ECC image alignment
 * */
void ecc_project_error_image(const std::vector<cv::Mat1f> & J, const cv::Mat & rhs, cv::Mat1f & v)
{
  INSTRUMENT_REGION("");

  const int M = J.size();
  v.create(M, 1);
  parallel_for(0, M, [&](const auto & range) {
    for ( int i = rbegin(range), n = rend(range); i < n; ++i ) {
      v[i][0] = J[i].dot(rhs);
    }
  });
}

// Search match for stable central reference core
// Return relative shift. For maxShift = 3, the point (3,3) becomes (0,0).
static cv::Vec2f ecc_match_template_zncc(const cv::Mat1f & reference_image, const cv::Mat1f & current_image,
    int maxShift)
{
  const cv::Mat templateROI =
      reference_image(cv::Rect(maxShift, maxShift,
          reference_image.cols - 2 * maxShift,
          reference_image.rows - 2 * maxShift));

  cv::Mat matchResult;
  cv::Point maxLoc;

  cv::matchTemplate(current_image, templateROI, matchResult, cv::TM_CCOEFF_NORMED);
  cv::minMaxLoc(matchResult, nullptr, nullptr, nullptr, &maxLoc);

  return cv::Vec2f(maxLoc.x - maxShift, maxLoc.y- maxShift);
}

template<class _Tp>
static bool _ecc_convert_input_image(cv::InputArray _src, cv::InputArray _srcmask,
    cv::OutputArray _dst, cv::OutputArray _dstmask)
{
  if( !_srcmask.empty() ) {
    if ( _srcmask.size() != _src.size() || _srcmask.channels() != 1 ) {
      CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be single-channel %dx%d",
          _srcmask.cols(), _srcmask.rows(), _srcmask.channels(), _srcmask.depth(),
          _src.cols(), _src.rows());
      return false;
    }
  }

  const cv::Size size = _src.size();

  cv::Mat src, mask;
  cv::Scalar mv, sv;

  if ( _src.channels() == 1 ) {
    src = _src.getMat();
  }
  else {
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
  }

  if (!_srcmask.empty()) {
    if ( _srcmask.depth() == CV_8U ) {
      mask = _srcmask.getMat();
    }
    else {
      cv::compare(_srcmask, 0, mask, cv::CMP_GT);
    }
  }

  cv::meanStdDev(src, mv, sv, mask);
  const float m = mv[0];
  const float s = 1 / sv[0];
  const float alpha = s;
  const float beta = -m * s;

  if ( mask.empty() ) {
    src.convertTo(_dst, CV_32F, alpha, beta);
  }
  else {
    _dst.create(size, CV_32FC1);
    cv::Mat1f dst = _dst.getMatRef();

    const uint8_t * src_base = src.ptr();
    const size_t src_stride = src.step;

    const uint8_t * mask_base = mask.ptr();
    const size_t mask_stride = mask.step;

    uint8_t * dst_base = dst.ptr();
    size_t dst_stride = dst.step;

    parallel_for(0, size.height, [=](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y ) {
        const _Tp * srcp = (const _Tp * )(src_base + y * src_stride);
        const uint8_t * mskp = (const uint8_t * )(mask_base + y * mask_stride);
        float * __restrict dstp = (float * )(dst_base + y * dst_stride);
        for ( int x = 0; x < size.width; ++x ) {
          dstp[x] = mskp[x] ? srcp[x] * alpha + beta : 0.f;
        }
      }
    });
  }

  _dstmask.assign(mask);

  return true;
}

static bool ecc_convert_input_image(cv::InputArray _src, cv::InputArray _srcmask,
    cv::OutputArray _dst, cv::OutputArray _dstmask,
    bool scharr_gradient)
{
  if ( !scharr_gradient ) {
    CV_DISPATCH(_src.depth(), _ecc_convert_input_image, _src, _srcmask, _dst, _dstmask);
    CF_ERROR("Not supported image depth=%d", _src.depth());
    return false;
  }

  cv::Mat1f g;
  cv::Mat1b m;

  if( true ) {
    cv::Mat1f gx, gy;
    parallel_invoke(
        [&] { cv::Scharr(_src, gx, CV_32F, 1, 0, 1.0, 0.0, cv::BORDER_REPLICATE); },
        [&] { cv::Scharr(_src, gy, CV_32F, 0, 1, 1.0, 0.0, cv::BORDER_REPLICATE); }
    );
    cv::magnitude(gx, gy, g);
  }

  if ( !_srcmask.empty() ) {
    cv::erode(_srcmask, m, morphse33());
  }

  CV_DISPATCH(g.depth(), _ecc_convert_input_image, g, m, _dst, _dstmask);
  CF_ERROR("Not supported image depth=%d", _src.depth());
  return false;

}

void ecc_normalize(cv::InputArray _src, cv::InputArray _src_mask, cv::OutputArray _dst, int lvl, double /*eps*/)
{
  const cv::Mat src = _src.getMat();
  const cv::Size src_size = _src.size();

  cv::Mat m;
  ecc_downscale(src, m, lvl, cv::BORDER_REPLICATE);
  ecc_upscale(m, src_size);
  cv::subtract(src, m, _dst, cv::noArray(), CV_32F);
  if ( !_src_mask.empty() ) {
    _dst.getMatRef().setTo(0, ~_src_mask.getMat());
  }
}


/* Remap to Flow
 * */
void ecc_remap_to_optflow(const cv::Mat2f & rmap, cv::Mat2f & flow)
{
  if( &flow == &rmap ) {

    parallel_for(0, flow.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        float * __restrict fp = (float * )flow[y];
        for( int x = 0; x < flow.cols; ++x, fp += 2 ) {
          fp[0] -= x;
          fp[1] -= y;
        }
      }
    });
  }
  else if( flow.data != rmap.data ) {

    flow.create(rmap.size());

    parallel_for(0, rmap.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const float * mp = (const float * )rmap[y];
        float * __restrict fp = (float * )flow[y];
        for( int x = 0; x < rmap.cols; ++x, mp += 2, fp += 2 ) {
          fp[0] = mp[0] - x;
          fp[1] = mp[1] - y;
        }
      }
    });
  }
  else {

    cv::Mat2f tmp(rmap.size());

    parallel_for(0, rmap.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const float * mp = (const float * )rmap[y];
        float * __restrict fp = (float * )tmp[y];
        for( int x = 0; x < rmap.cols; ++x, mp += 2, fp += 2 ) {
          fp[0] = mp[0] - x;
          fp[1] = mp[1] - y;
        }
      }
    });

    flow = std::move(tmp);
  }
}

/* Flow to Remap
 * */
void ecc_flow_to_remap(const cv::Mat2f & flow, cv::Mat2f & rmap)
{
  if( &flow == &rmap ) {

    parallel_for(0, rmap.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        float * mp = (float * )rmap[y];
        for( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
          mp[0] += x;
          mp[1] += y;
        }
      }
    });

  }
  else if( flow.data != rmap.data ) {

    rmap.create(flow.size());

    parallel_for(0, rmap.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const float * fp = (const float * )flow[y];
        float * mp = (float * )rmap[y];
        for( int x = 0; x < rmap.cols; ++x, mp += 2, fp += 2 ) {
          mp[0] = fp[0] + x;
          mp[1] = fp[1] + y;
        }
      }
    });

  }
  else {

    cv::Mat2f tmp(flow.size());

    parallel_for(0, tmp.rows, [&](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const float * fp = (const float * )flow[y];
        float * mp = (float * )tmp[y];
        for( int x = 0; x < tmp.cols; ++x, mp += 2, fp += 2 ) {
          mp[0] = fp[0] + x;
          mp[1] = fp[1] + y;
        }
      }
    });

    rmap = std::move(tmp);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_align::c_ecc_align(c_image_transform * transform) :
    _transform(transform)
{
}

void c_ecc_align::set_image_transform(c_image_transform * image_transform)
{
  _transform = image_transform;
}

c_image_transform * c_ecc_align::image_transform() const
{
  return _transform;
}

void c_ecc_align::set_max_iterations(int v)
{
  this->_max_iterations = v;
}

int c_ecc_align::max_iterations() const
{
  return this->_max_iterations;
}

void c_ecc_align::set_xtol(double v)
{
  this->_xtol = v;
}

double c_ecc_align::xtol() const
{
  return this->_xtol;
}

void c_ecc_align::set_ftol(double v)
{
  this->_ftol = v;
}

double c_ecc_align::ftol() const
{
  return this->_ftol;
}

void c_ecc_align::set_gtol(double v)
{
  this->_gtol = v;
}

double c_ecc_align::gtol() const
{
  return this->_gtol;
}

void c_ecc_align::set_interpolation(enum ECC_INTERPOLATION_METHOD  v)
{
  _interpolation = v;
}

enum ECC_INTERPOLATION_METHOD c_ecc_align::interpolation() const
{
  return _interpolation;
}

void c_ecc_align::set_update_step_scale(double v)
{
  _update_step_scale = v;
}

double c_ecc_align::update_step_scale() const
{
  return _update_step_scale;
}

void c_ecc_align::copy_parameters(const this_class & rhs)
{
  _update_step_scale = rhs._update_step_scale;
  _xtol = rhs._xtol;
  _ftol = rhs._ftol;
  _gtol = rhs._gtol;
  _max_iterations = rhs._max_iterations;
  _interpolation = rhs._interpolation;
}


bool c_ecc_align::failed() const
{
  return this->_failed;
}

int c_ecc_align::num_iterations() const
{
  return this->_num_iterations;
}

double c_ecc_align::eps() const
{
  return _eps;
}


bool c_ecc_align::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if( reference_image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
        reference_image.cols(), reference_image.rows(),
        reference_image.depth(),
        reference_image.channels());
    return false;
  }

  reference_image.copyTo(_reference_image);

  if( reference_mask.empty() ) {
    _reference_mask.release();
  }
  else {
    if( reference_mask.size() != reference_image.size() || reference_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
          reference_mask.cols(), reference_mask.rows(), reference_mask.channels(), reference_mask.depth(),
          reference_image.cols(), reference_image.rows());

      return false;
    }

    reference_mask.copyTo(_reference_mask);

    //cv::erode(reference_mask, _reference_mask, morphse33());
  }

  return true;
}

bool c_ecc_align::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");

  if( current_image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
        current_image.cols(), current_image.rows(),
        current_image.depth(),
        current_image.channels());
    return false;
  }

  current_image.copyTo(_current_image);

  if( current_mask.empty() ) {
    _current_mask.release();
  }
  else {
    if( current_mask.size() != current_image.size() || current_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
          current_mask.cols(), current_mask.rows(), current_mask.channels(), current_mask.depth(),
          current_image.cols(), current_image.rows());

      return false;
    }

    current_mask.copyTo(_current_mask);
  }

  return true;
}

void c_ecc_align::release_cached_memory()
{
  _current_image.release();
  _current_mask.release();
  _remapped_current_image.release();
  _remapped_current_zmask.release();
  _rmap.release();
}

const cv::Mat1f & c_ecc_align::reference_image() const
{
  return _reference_image;
}

const cv::Mat1b & c_ecc_align::reference_mask() const
{
  return _reference_mask;
}

const cv::Mat1f & c_ecc_align::current_image() const
{
  return _current_image;
}

const cv::Mat1b & c_ecc_align::current_mask() const
{
  return _current_mask;
}

const cv::Mat2f & c_ecc_align::current_remap() const
{
  return _rmap;
}

const cv::Mat1f & c_ecc_align::remapped_current_image() const
{
  return _remapped_current_image;
}

const cv::Mat1b & c_ecc_align::remapped_current_zmask() const
{
  return _remapped_current_zmask;
}

bool c_ecc_align::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");
  if ( !set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("c_ecc_align: set_reference_image() fails");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecc_align: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecc_align::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");
  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecc_align: reference image was not set");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecc_align: set_current_image() fails");
    return false;
  }

  return align();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const cv::Mat1f c_ecch::_empty_image;
const cv::Mat1b c_ecch::_empty_mask;
const cv::Mat2f c_ecch::_empty_remap;

c_ecch::c_ecch( c_image_transform * image_transform) :
    _image_transform(image_transform)
{
}

c_ecch::c_ecch(ECC_ALIGN_METHOD method)
{
  _opts.method = method;
}

c_ecch::c_ecch(const c_ecch_options& opts) :
    _opts(opts)
{
}

c_ecch::c_ecch(c_image_transform * image_transform, const c_ecch_options& opts) :
    _image_transform(image_transform),
    _opts(opts)
{
}

c_ecch::c_ecch(c_image_transform * image_transform, ECC_ALIGN_METHOD method) :
    _image_transform(image_transform)
{
  _opts.method = method;
}

void c_ecch::set_image_transform(c_image_transform * image_transform)
{
  if ( _image_transform != image_transform ) {
    _image_transform = image_transform;
    for ( const auto & m : _pyramid ) {
      m->set_image_transform(image_transform);
    }
  }
}

c_image_transform * c_ecch::image_transform() const
{
  return _image_transform;
}

void c_ecch::set_method(ECC_ALIGN_METHOD v)
{
  if ( _opts.method != v ) {
    _opts.method = v;
    _pyramid.clear();
  }
}

ECC_ALIGN_METHOD c_ecch::method() const
{
  return _opts.method;
}

void c_ecch::set_maxlevel(int v)
{
  _opts.maxlevel = v;
  _pyramid.clear();
}

int c_ecch::maxlevel() const
{
  return _opts.maxlevel;
}

void c_ecch::set_minimum_image_size(int v)
{
  _opts.minimum_image_size = v;
  _pyramid.clear();
}

int c_ecch::minimum_image_size() const
{
  return _opts.minimum_image_size;
}

void c_ecch::set_max_iterations(int v)
{
  _opts.max_iterations = v;
  for( const auto & m : _pyramid ) {
    m->set_max_iterations(v);
  }
}

int c_ecch::max_iterations() const
{
  return _opts.max_iterations;
}


void c_ecch::set_xtol(double v)
{
  _opts.xtol = v;
  for( const auto & m : _pyramid ) {
    m->set_xtol(v);
  }
}

double c_ecch::xtol() const
{
  return _opts.xtol;
}

void c_ecch::set_ftol(double v)
{
  _opts.ftol = v;
  for( const auto & m : _pyramid ) {
    m->set_ftol(v);
  }
}

double c_ecch::ftol() const
{
  return _opts.ftol;
}

void c_ecch::set_gtol(double v)
{
  _opts.gtol = v;
  for( const auto & m : _pyramid ) {
    m->set_gtol(v);
  }
}

double c_ecch::gtol() const
{
  return _opts.gtol;
}

double c_ecch::eps() const
{
  return _pyramid.empty() ? -1 : _pyramid.front()->eps();
}

int c_ecch::num_iterations() const
{
  return _total_iterations;
}

void c_ecch::set_interpolation(enum ECC_INTERPOLATION_METHOD v)
{
  _opts.interpolation = v;
  for( const auto & m : _pyramid ) {
    m->set_interpolation(v);
  }
}

enum ECC_INTERPOLATION_METHOD c_ecch::interpolation() const
{
  return _opts.interpolation;
}

void c_ecch::set_input_smooth_sigma(double v)
{
  _opts.input_smooth_sigma = v;
}

double c_ecch::input_smooth_sigma() const
{
  return _opts.input_smooth_sigma;
}

void c_ecch::set_reference_smooth_sigma(double v)
{
  _opts.reference_smooth_sigma = v;
}

double c_ecch::reference_smooth_sigma() const
{
  return _opts.reference_smooth_sigma;
}

void c_ecch::set_update_step_scale(double v)
{
  _opts.update_step_scale = v;
}

double c_ecch::update_step_scale() const
{
  return _opts.update_step_scale;
}

void c_ecch::copy_parameters(const this_class & rhs)
{
  _opts = rhs._opts;
}

const cv::Mat1f & c_ecch::reference_image() const
{
  return _pyramid.empty() ? _empty_image : _pyramid.front()->reference_image();
}

const cv::Mat1b & c_ecch::reference_mask() const
{
  return _pyramid.empty() ? _empty_mask : _pyramid.front()->reference_mask();
}

const cv::Mat1f & c_ecch::current_image() const
{
  return _pyramid.empty() ? _empty_image : _pyramid.front()->current_image();
}

const cv::Mat1b & c_ecch::current_mask() const
{
  return _pyramid.empty() ? _empty_mask : _pyramid.front()->current_mask();
}

const cv::Mat2f & c_ecch::current_remap() const
{
  return _pyramid.empty() ? _empty_remap : _pyramid.front()->current_remap();
}

const cv::Mat1f & c_ecch::remapped_current_image() const
{
  return _pyramid.empty() ? _empty_image : _pyramid.front()->remapped_current_image();
}

const cv::Mat1b & c_ecch::remapped_current_zmask() const
{
  return _pyramid.empty() ? _empty_mask : _pyramid.front()->remapped_current_zmask();
}

bool c_ecch::create_remap(cv::Mat2f & rmap) const
{
  if( _image_transform ) {
    const cv::Mat & rimage = reference_image();
    if( !rimage.size().empty() ) {
      return _image_transform->create_remap(rimage.size(), rmap);
    }
  }
  return false;
}

cv::Mat2f c_ecch::create_remap() const
{
  cv::Mat2f rmap;
  if ( _image_transform ) {
    const cv::Mat & rimage = reference_image();
    if ( !rimage.size().empty() ) {
      _image_transform->create_remap(rimage.size(), rmap);
    }
  }
  return rmap;
}

c_ecc_align::uptr c_ecch::create_ecc_align(enum ECC_ALIGN_METHOD method, double xtol) const
{
  c_ecc_align::uptr ecc;

  switch (method) {
    case ECC_ALIGN_INVERSE_COMPOSITIONAL_LM:
      ecc.reset(new c_ecclm_inverse_compositional());
      break;
    case ECC_ALIGN_FORWARD_ADDITIVE_LM:
    default:
      ecc.reset(new c_ecclm_forward_additive());
      break;
  }

  ecc->set_image_transform(_image_transform);
  ecc->set_interpolation(_opts.interpolation);
  ecc->set_max_iterations(_opts.max_iterations);
  ecc->set_update_step_scale(_opts.update_step_scale);
  ecc->set_ftol(_opts.ftol);
  ecc->set_gtol(_opts.gtol);
  ecc->set_xtol(xtol);

  return ecc;
}

inline void c_ecch::downscale_image(cv::Mat & image, cv::Mat & mask,
    const cv::Size & nextSize) const
{
  if ( !image.empty() ) {
    cv::pyrDown(image, image, nextSize);
  }

  if( !mask.empty() ) {
    cv::resize(mask, mask, nextSize, 0, 0, cv::INTER_NEAREST);
  }
}

bool c_ecch::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  cv::Mat1f image;
  cv::Mat1b mask;

  const bool scharr_gradient =
      _opts.pyramid_type == ECCH_PYRAMID_SCHARR;

  _total_iterations = -1;

  if( !ecc_convert_input_image(reference_image, reference_mask, image, mask, scharr_gradient) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( _opts.reference_smooth_sigma > 0 ) {
    if( _opts.reference_smooth_sigma != sigmaRef || Gref.empty() ) {
      const int ksize = std::max(3, 2 * ((int) (3 * _opts.reference_smooth_sigma)) + 1);
      Gref = cv::getGaussianKernel(ksize, _opts.reference_smooth_sigma);
      sigmaRef = _opts.reference_smooth_sigma;
    }
    cv::Mat1f tmp;
    cv::sepFilter2D(image, tmp, -1, Gref, Gref, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    image = std::move(tmp);
  }


  // Estimate how many pyramid levels is required

  const int min_image_size = std::max(4, this->_opts.minimum_image_size);
  std::vector<cv::Size> sizes;

  sizes.emplace_back(image.size());
  while (true) {
    if( _opts.maxlevel >= 0 && sizes.size() >= std::max(0, _opts.maxlevel) ) {
      break;
    }

    const cv::Size next_size = compute_next_pyramid_layer_size(sizes.back());
    if( next_size.width < min_image_size || next_size.height < min_image_size ) {
      break;
    }

    sizes.emplace_back(next_size);
  }

  static const auto is_direct_method = [](enum ECC_ALIGN_METHOD method) {
    return method == ECC_ALIGN_FORWARD_ADDITIVE_LM;
  };

  const int required_lvls = sizes.size();

  if( _pyramid.size() != required_lvls ) {
    _pyramid.clear();

    double xtol = _opts.xtol;
    for( int lvl = 0; lvl < required_lvls; ++lvl ) {
      ECC_ALIGN_METHOD method = _opts.method;
      if ( _opts.force_direct_methods_for_coarse_levels && !is_direct_method(_opts.method) ) {
        const cv::Size & size = sizes[lvl];
        if ( std::min(size.width, size.height) < 256 ) {
          method = ECC_ALIGN_FORWARD_ADDITIVE_LM;
        }
      }
      _pyramid.emplace_back(create_ecc_align(method, xtol / (lvl + 1)));
    }
  }

  if( !_pyramid[0]->set_reference_image(image, mask) ) {
    CF_ERROR("_pyramid[0]->set_reference_image() fails");
    return false;
  }

  for( int lvl = 1; lvl < required_lvls; ++lvl ) {
    const cv::Size prev_size = image.size();
    const cv::Size next_size = compute_next_pyramid_layer_size(prev_size);
    downscale_image(image, mask, next_size);
    if( !_pyramid[lvl]->set_reference_image(image, mask) ) {
      CF_ERROR("_pyramid[lvl=%d]->set_reference_image() fails", lvl);
      return false;
    }
  }

  return true;
}


bool c_ecch::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");

  if( _pyramid.empty() ) {
    CF_ERROR("Reference image must be set first");
    return false;
  }

  cv::Mat1f image;
  cv::Mat1b mask;

  const bool scharr_gradient =
      _opts.pyramid_type == ECCH_PYRAMID_SCHARR;

  _total_iterations = -1;

  if( !ecc_convert_input_image(current_image, current_mask, image, mask, scharr_gradient) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( _opts.input_smooth_sigma > 0 ) {
    if( _opts.input_smooth_sigma != sigmaCur || Gcur.empty() ) {
      const int ksize = std::max(3, 2 * ((int) (3 * _opts.input_smooth_sigma)) + 1);
      Gcur = cv::getGaussianKernel(ksize, _opts.input_smooth_sigma);
      sigmaCur = _opts.input_smooth_sigma;
    }
    cv::Mat1f tmp;
    cv::sepFilter2D(image, tmp, -1, Gcur, Gcur, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    image = std::move(tmp);
  }

  const int lvls = _pyramid.size();

  int lvl = 0;

  for( ; lvl < lvls; ++lvl ) {

    if ( !_pyramid[lvl]->set_current_image(image, mask) ) {
      CF_ERROR("L[%d] pyramid_[lvl]->set_current_image() fails");
      return false;
    }

    if( lvl < lvls - 1 ) {
      const cv::Size prev_size = image.size();
      const cv::Size next_size = compute_next_pyramid_layer_size(prev_size);
      if( next_size.width < 4 || next_size.height < 4 ) {
        break;
      }

      downscale_image(image, mask, next_size);
    }
  }

  for( ; lvl < lvls; ++lvl ) {
    _pyramid[lvl]->release_cached_memory();
  }

  return true;
}

bool c_ecch::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecch: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecch::align()
{
  INSTRUMENT_REGION("");
  if( _pyramid.empty() ) {
    CF_ERROR("c_ecch: no reference image was set");
    return false;
  }

  if( _pyramid.front()->current_image().empty() ) {
    CF_ERROR("c_ecch: no current_image image was set");
    return false;
  }

  const int lvls = _pyramid.size();

  int lvl = lvls - 1;
  while (lvl > 0 && _pyramid[lvl]->current_image().empty()) {
    --lvl;
  }

  if ( lvl > 0 ) {
    const cv::Size size0 = _pyramid[0]->reference_image().size();
    const cv::Size size1 = _pyramid[lvl]->reference_image().size();
    _image_transform->scale_transfrom((double) size1.width / (double) size0.width);
  }

  _total_iterations = 0;

  for( ; lvl >= 0; --lvl ) {
    const cv::Size curSize = _pyramid[lvl]->current_image().size();
    const cv::Size refSize = _pyramid[lvl]->reference_image().size();
    const int minWidth = std::min(curSize.width, refSize.width);
    const int minHeight = std::min(curSize.height, refSize.height);
    const int minSize = std::min(minWidth, minHeight);

    bool fOK = false;

    if ( (lvl == lvls - 1) && (minSize <= 32) ) {
      const cv::Mat1f & rimg = _pyramid[lvl]->reference_image();
      const cv::Mat1f & cimg = _pyramid[lvl]->current_image();
      const cv::Vec2f T = ecc_match_template_zncc(rimg, cimg, 2);
      _pyramid[lvl]->image_transform()->set_translation(T);
      //  CF_DEBUG("[lvl=%d %dx%d] Small image: dx=%g dy=%g", lvl, rimg.cols, rimg.rows, T(0), T(1));
      fOK = true;
    }
    else {
      if( !(fOK = _pyramid[lvl]->align()) ) {
        CF_ERROR("pyramid[lvl=%d]->align() fails", lvl);
      }
      //  const cv::Mat1f & rimg = _pyramid[lvl]->reference_image();
      //  const int iterations = _pyramid[lvl]->num_iterations();
      //  const double eps = _pyramid[lvl]->eps();
      //  const double xtol = _pyramid[lvl]->xtol();
      //  const cv::Vec2f T = _pyramid[lvl]->image_transform()->translation();
      //  CF_DEBUG("[lvl=%d %dx%d] %d iterations eps=%g/%g Tx=%g Ty=%g", lvl, rimg.cols, rimg.rows,
      //    iterations, eps, maxeps, T[0], T[1]);
    }

    if( fOK && lvl > 0 ) {
      const cv::Size size0 = _pyramid[lvl]->reference_image().size();
      const cv::Size size1 = _pyramid[lvl - 1]->reference_image().size();
      c_image_transform * nextTransform = _pyramid[lvl - 1]->image_transform();
      nextTransform->scale_transfrom((double) size1.width / (double) size0.width);
    }

    _total_iterations += _pyramid[lvl]->num_iterations();
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
c_ecclm_forward_additive::c_ecclm_forward_additive(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm_forward_additive::release_cached_memory()
{
  INSTRUMENT_REGION("");
  _current_zmask.release();
  _reference_zmask.release();
  _remapped_gx.release();
  _remapped_gy.release();
  _rhs.release();
  J.clear(), J.shrink_to_fit();
  base::release_cached_memory();
}

bool c_ecclm_forward_additive::set_reference_image(cv::InputArray reference_image,
    cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if ( !base::set_reference_image(reference_image, reference_mask) ) {
    return false;
  }

  if ( !_reference_mask.empty() ) {
    cv::bitwise_not(_reference_mask, _reference_zmask);
  }
  else {
    _reference_zmask.release();
  }

  return true;
}

bool c_ecclm_forward_additive::set_current_image(cv::InputArray current_image,
    cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");

  if( !base::set_current_image(current_image, current_mask) ) {
    return false;
  }

  if( !_current_mask.empty() ) {
    cv::bitwise_not(_current_mask, _current_zmask);
    cv::dilate(_current_zmask, _current_zmask, morphse33(),
        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  }
  else {
    _current_zmask.create(_current_image.size());
    _current_zmask.setTo(0);
  }

  return true;
}

bool c_ecclm_forward_additive::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecclm_forward_additive::align_to_reference(cv::InputArray current_image,
    cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

double c_ecclm_forward_additive::compute_rhs(const cv::Mat1f & params, double * K /*= nullptr*/)
{
  const cv::Size size(_reference_image.size());

  if( !_transform->create_remap(params, size, _rmap) ) {
    CF_ERROR("_transform->create_remap() fails");
    return -1;
  }

  cv::remap(_current_image, _remapped_current_image,
      _rmap, cv::noArray(),
      _interpolation,
      cv::BORDER_CONSTANT);

  cv::remap(_current_zmask, _remapped_current_zmask,
      _rmap, cv::noArray(),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT,
      cv::Scalar(255));

  if( !_reference_zmask.empty() ) {
    cv::bitwise_or(_reference_zmask, _remapped_current_zmask,
        _remapped_current_zmask);
  }

  return ecc_compute_rhs(_reference_image, _remapped_current_image,
      _remapped_current_zmask, _rhs, K);
}

bool c_ecclm_forward_additive::align()
{
  INSTRUMENT_REGION("");
  _failed = true;

  if ( !_transform ) {
    CF_ERROR("image transform is null");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("_reference_image is empty");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("_current_image is empty");
    return false;
  }

//  if ( !_reference_mask.empty() ) {
//    cv::bitwise_not(_reference_mask, _reference_zmask);
//  }
//  else {
//    _reference_zmask.release();
//  }
//
//  if ( !_current_mask.empty() ) {
//    cv::bitwise_not(_current_mask, _current_zmask);
//    cv::dilate(_current_zmask, _current_zmask, morphse33(),
//        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  }
//  else {
//    _current_zmask.create(_current_image.size());
//    _current_zmask.setTo(0);
//  }

  cv::Mat1f H, Hp, v, deltap, temp_d;
  cv::Mat1f params, newparams;

  params = _transform->parameters();

  const int M = params.rows;
  const double xtol = _xtol;
  const double ftol = _ftol;
  const double gtol = _gtol;
  const int max_iterations = _max_iterations;
  constexpr double safe_min = double(std::numeric_limits<float>::min());

  double olderr = 0, newerr = 0, delta_error = 0;
  double lambda = 0.1;
  double eps = 0;
  double K = 1.0;
  int iterations = 0;
  bool converged = false;

  /**
   * PreCompute
   * */
  _eps = 0;

  if( !((olderr = compute_rhs(params)) >= 0) ) {
    CF_ERROR("Invalid initial error value : %g", olderr);
    _eps = FLT_MAX;
    return false;
  }

  if( olderr <= safe_min ) {

    const int total_pixels = _reference_image.size().area();
    const int valid_pixels = _remapped_current_zmask.empty() ? total_pixels :
        total_pixels - cv::countNonZero(_remapped_current_zmask);

    if( valid_pixels <= M ) {
      CF_ERROR("insufficient overlap mask (%d pixels for %d params)", valid_pixels, M);
      _eps = FLT_MAX;
      return false;
    }

    _failed = false;
    return true;
  }

  v.create(M, 1);
  H.create(M, M);
  J.resize(M);

  while (iterations < max_iterations) {

    /*
     * Solve normal equations for given Jacobian and lambda
     * */

    ecc_differentiate(_remapped_current_image, _remapped_gx, _remapped_gy, _remapped_current_zmask);
    _transform->create_steepest_descent_images(params, _remapped_gx, _remapped_gy, J.data());
    ecc_project_error_image(J, _rhs, v);
    ecc_compute_hessian_matrix(J, H);
    H.copyTo(Hp);

    if( gtol > safe_min ) {
      double g_max_rel = 0;
      const double norm_e = std::sqrt(olderr);
      for( int i = 0; i < M; ++i ) {
        const double diag_H = Hp(i, i);
        if( diag_H > safe_min ) {
          const double rel_g = std::abs(v(i, 0)) / (norm_e * std::sqrt(diag_H));
          g_max_rel = std::max(g_max_rel, rel_g);
        }
      }
      if( g_max_rel <= gtol ) {
        converged = true;
        _failed = false;
        break;
      }
    }

    bool step_accepted = false;

    while (iterations++ < max_iterations) {

      /* Increase diagonal elements by lambda */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to compute parameters update
       *  deltap = H.inv() * v
       *  */
      if( !ecc_solve(H, v, deltap) ) {
        if( (lambda *= 5) > 1e6 ) {
          break;
        }
        continue;
      }

      /*
       * Compute function for newparams
       * */
      cv::scaleAdd(deltap, _update_step_scale, params, newparams);

      /* Check for increment in parameters  */
      if( (eps = _transform->eps(deltap, _reference_image.size())) <= xtol ) {
        step_accepted = converged = true;
        break;
      }

      /* Compute error function for new parameters */
      newerr = compute_rhs(newparams, &K);
      delta_error = olderr - newerr;
      if( delta_error < 0 ) {
        if ( lambda > 1e6 ) {
          break; // no convergence
        }
        lambda *= 10.0f;
        continue;
      }

      /*
       * Accept new params and compute update to lambda
       * */
      step_accepted = true;
      if (delta_error <= olderr * ftol ) {
        converged = true;
        break;
      }

      cv::gemm(Hp, deltap, -1, v, 2, temp_d);
      const double dS = deltap.dot(temp_d) * K;
      const double rho = (olderr - newerr) / (std::abs(dS) > 0 ? dS : 1);

      if( rho > 0.25 && lambda > 1e-6 ) {
        /* Decrease lambda ==> Gauss-Newton method */
        lambda = std::max(1e-6, lambda * 0.2 );
      }
      else if ( rho < 0.01 ) {
        lambda *= 10; /** Try increase lambda ==> gradient descend */
      }

      break;
    }

    if ( !step_accepted ) {
      /* No further improvement is possible */
      break;
    }

    /* Accept new parameters */
    _eps = eps;
    _failed = false;
    _transform->set_parameters(newparams);
    if (converged) {
      break;
    }

    params = _transform->parameters();
    olderr = newerr;
  }

  _num_iterations = iterations;

  //  CF_DEBUG("[%dx%d] iterations=%d eps=%g/%g delta_error=%g/%g lambda=%g update_step_scale=%g converged=%d",
  //      _reference_image.cols, _reference_image.rows, iterations,
  //      _eps, _xtol,
  //      delta_error, olderr * ftol,
  //      lambda, _update_step_scale,
  //      converged);

  return !_failed;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecclm_inverse_compositional::c_ecclm_inverse_compositional(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm_inverse_compositional::release_cached_memory()
{
  INSTRUMENT_REGION("");
  _current_zmask.release();
  _reference_zmask.release();
  gx.release();
  gy.release();
  _rhs.release();
  J.clear(), J.shrink_to_fit();
  base::release_cached_memory();
}

void c_ecclm_inverse_compositional::set_image_transform(c_image_transform * image_transform)
{
  INSTRUMENT_REGION("");
  if( image_transform != this->_transform ) {
    J.clear();
    _reference_image_changed = true;
    base::set_image_transform(image_transform);
  }
}

bool c_ecclm_inverse_compositional::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  _reference_image_changed = true;

  if ( !base::set_reference_image(reference_image, reference_mask) ) {
    return false;
  }

  if ( _reference_mask.empty() ) {
    _reference_zmask.release();
  }
  else {
    cv::bitwise_not(_reference_mask, _reference_zmask);
    cv::dilate(_reference_zmask, _reference_zmask, morphse33(),
        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  }

  return true;
}

bool c_ecclm_inverse_compositional::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");

  if ( !base::set_current_image(current_image, current_mask) ) {
    return false;
  }

  if ( _current_mask.empty() ) {
    _current_zmask.create(_current_image.size());
    _current_zmask.setTo(0);
  }
  else {
    cv::bitwise_not(_current_mask, _current_zmask);
    cv::dilate(_current_zmask, _current_zmask, morphse33(),
        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  }

  return true;
}

bool c_ecclm_inverse_compositional::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecclm_inverse_compositional::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

double c_ecclm_inverse_compositional::compute_rhs(const cv::Mat1f & params, double * K /*= nullptr*/)
{
  INSTRUMENT_REGION("");
  const cv::Size size(_reference_image.size());

  if( !_transform->create_remap(params, size, _rmap) ) {
    CF_ERROR("_transform->create_remap() fails");
    return -1;
  }

  cv::remap(_current_image, _remapped_current_image,
      _rmap, cv::noArray(),
      _interpolation,
      cv::BORDER_CONSTANT);

  cv::remap(_current_zmask, _remapped_current_zmask,
      _rmap, cv::noArray(),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT,
      cv::Scalar(255));

  if( !_reference_zmask.empty() ) {
    cv::bitwise_or(_reference_zmask, _remapped_current_zmask,
        _remapped_current_zmask);
  }

  return ecc_compute_rhs(_remapped_current_image, _reference_image,
      _remapped_current_zmask, _rhs, K);
}

bool c_ecclm_inverse_compositional::align()
{
  INSTRUMENT_REGION("");

  _failed = true;

  if ( !_transform ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform is null");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: reference_image is empty");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: current_image is empty");
    return false;
  }

//  if ( _reference_mask.empty() ) {
//    _reference_zmask.release();
//  }
//  else {
//    cv::bitwise_not(_reference_mask, _reference_zmask);
//    cv::dilate(_reference_zmask, _reference_zmask, morphse33(),
//        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  }
//
//  if ( _current_mask.empty() ) {
//    _current_zmask.create(_current_image.size());
//    _current_zmask.setTo(0);
//  }
//  else {
//    cv::bitwise_not(_current_mask, _current_zmask);
//    cv::dilate(_current_zmask, _current_zmask, morphse33(),
//        cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  }

  cv::Mat1f params, newparams, deltap;
  cv::Mat1f H, temp_d, v;

  params = _transform->parameters();
  const int M = params.rows;
  const double xtol = _xtol;
  const double ftol = _ftol;
  const double gtol = _gtol;
  const int max_iterations = _max_iterations;
  constexpr double safe_min = double(std::numeric_limits<float>::min());

  double olderr = 0, newerr = 0, delta_error = 0;
  double lambda = 0.1;
  double eps = 0;
  double K = 1.0;
  int iterations = 0;
  bool converged = false;

  _eps = 0;

  /**
   * PreCompute
   * */

  if( !((olderr = compute_rhs(params)) >= 0) ) {
    CF_ERROR("Invalid initial error value : %g", olderr);
    _eps = FLT_MAX;
    return false;
  }

  if( olderr <= safe_min ) {

    const int total_pixels = _reference_image.size().area();
    const int valid_pixels = _remapped_current_zmask.empty() ? total_pixels :
        total_pixels - cv::countNonZero(_remapped_current_zmask);

    if( valid_pixels <= M ) {
      CF_ERROR("insufficient overlap mask (%d pixels for %d params)", valid_pixels, M);
      _eps = FLT_MAX;
      return false;
    }

    _failed = false;
    return true;
  }

  if( J.size() != M || _reference_image_changed ) {
    J.resize(M);
    ecc_differentiate(_reference_image, gx, gy, _reference_zmask);
    _transform->create_steepest_descent_images(gx, gy, J.data());
    ecc_compute_hessian_matrix(J, Hp);
    _reference_image_changed = false;
  }

  while (iterations < max_iterations) {

    /*
     * Solve normal equations for given Jacobian and lambda
     * */

    ecc_project_error_image(J, _rhs, v);
    Hp.copyTo(H);

    if( gtol > safe_min ) {
      double g_max_rel = 0;

      const double norm_e = std::sqrt(olderr);
      for( int i = 0; i < M; ++i ) {
        const double diag_H = Hp(i, i);
        if( diag_H > safe_min ) {
          const double rel_g = std::abs(v(i, 0)) / (norm_e * std::sqrt(diag_H));
          g_max_rel = std::max(g_max_rel, rel_g);
        }
      }
      if( g_max_rel <= gtol ) {
        converged = true;
        _failed = false;
        break;
      }
    }

    bool step_accepted = false;

    while (iterations++ < max_iterations) {

      /* Increase diagonal elements by lambda */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to compute parameters update
       *  deltap = H.inv() * v
       *  */
      if( !ecc_solve(H, v, deltap) ) {
        if( (lambda *= 5) > 1e6 ) {
          break;
        }
        continue;
      }

      /*
       * Compute function for newparams
       * */
      newparams = _transform->invert_and_compose(params, _update_step_scale * deltap);

      /* Check for increment in parameters  */
      if( (eps = _transform->eps(deltap, _reference_image.size())) <= xtol ) {
        step_accepted = converged = true;
        break;
      }

      /* Compute error function for new parameters */
      newerr = compute_rhs(newparams, &K);
      delta_error = olderr - newerr;
      if( delta_error < 0 ) {
        if ( lambda > 1e6 ) {
          break; // no convergence
        }
        lambda *= 10.0f;
        continue;
      }

      /*
       * Accept new params and compute update to lambda
       * */
      step_accepted = true;
      if (delta_error <= olderr * ftol ) {
        converged = true;
        break;
      }

      cv::gemm(Hp, deltap, -1, v, 2, temp_d);
      const double dS = deltap.dot(temp_d) * K;
      const double rho = (olderr - newerr) / (std::abs(dS) > 0 ? dS : 1);

      if( rho > 0.25 && lambda > 1e-6 ) {
        /* Decrease lambda ==> Gauss-Newton method */
        lambda = std::max(1e-6, lambda * 0.2 );
      }
      else if ( rho < 0.01 ) {
        lambda *= 10; /** Try increase lambda ==> gradient descend */
      }

      break;
    }

    if ( !step_accepted ) {
      /* No further improvement is possible */
      break;
    }

    /* Accept new parameters */
    _eps = eps;
    _failed = false;
    _transform->set_parameters(newparams);
    if (converged) {
      break;
    }

    params = _transform->parameters();
    olderr = newerr;
  }

  _num_iterations = iterations;

//  CF_DEBUG("[%dx%d] iterations=%d eps=%g/%g delta_error=%g/%g lambda=%g update_step_scale=%g converged=%d",
//      _reference_image.cols, _reference_image.rows, iterations,
//      _eps, _xtol,
//      delta_error, olderr * ftol,
//      lambda, _update_step_scale,
//      converged);

  return !_failed;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member* members_of<ECCFlowDownscaleMethod>()
{
  static const c_enum_member members[] = {
      { ECCFlowDownscaleRecursiveResize, "RecursiveResize", "Recursive cv::resize() with scale factor" },
      { ECCFlowDownscaleFullResize, "FullResize", "Direct cv::resize() from full to target resolution" },
      { ECCFlowDownscalePyramid, "Pyramid", "Recursive resize using cv::pyrDown()" },
      { ECCFlowDownscaleRecursiveResize },
  };

  return members;
}

void c_eccflow::set_support_scale(int v)
{
  _opts.support_scale = v;
}

int c_eccflow::support_scale() const
{
  return _opts.support_scale;
}


void c_eccflow::set_max_iterations(int v)
{
  _opts.max_iterations = v;
}

int c_eccflow::max_iterations() const
{
  return _opts.max_iterations;
}

void c_eccflow::set_update_multiplier(double v)
{
  _opts.update_multiplier = v;
}

double c_eccflow::update_multiplier() const
{
  return _opts.update_multiplier;
}

void c_eccflow::set_input_smooth_sigma(double v)
{
  _opts.input_smooth_sigma = v;
}

double c_eccflow::input_smooth_sigma() const
{
  return _opts.input_smooth_sigma;
}


void c_eccflow::set_reference_smooth_sigma(double v)
{
  _opts.reference_smooth_sigma = v;
}

double c_eccflow::reference_smooth_sigma() const
{
  return _opts.reference_smooth_sigma;
}

void c_eccflow::set_downscale_method(ECCFlowDownscaleMethod v)
{
  _opts.downscale = v;
}

ECCFlowDownscaleMethod c_eccflow::downscale_method() const
{
  return _opts.downscale;
}

void c_eccflow::set_scale_factor(double v)
{
  _opts.scale_factor = v;
}

double c_eccflow::scale_factor() const
{
  return _opts.scale_factor;
}

void c_eccflow::set_min_image_size(int v)
{
  _opts.min_image_size = v;
}

int c_eccflow::min_image_size() const
{
  return _opts.min_image_size;
}

void c_eccflow::set_max_pyramid_level(int v)
{
  _opts.max_pyramid_level = v;
}

int c_eccflow::max_pyramid_level() const
{
  return _opts.max_pyramid_level;
}

void c_eccflow::set_noise_level(double v)
{
  _opts.noise_level = v;
}

double c_eccflow::noise_level() const
{
  return _opts.noise_level;
}

void c_eccflow::copy_parameters(const this_class & rhs)
{
  _opts = rhs._opts;
}

const cv::Mat2f& c_eccflow::current_uv() const
{
  return uv;
}

const std::vector<c_eccflow::pyramid_entry>& c_eccflow::current_pyramid() const
{
  return _pyramid;
}

bool c_eccflow::convert_input_images(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask) const
{
  INSTRUMENT_REGION("");

  src.getMat().convertTo(dst, dst.depth());

  if ( src_mask.empty() /*|| cv::countNonZero(src_mask) == src_mask.size().area() */) {
    dst_mask.release();
  }
  else {
    src_mask.getMat().copyTo(dst_mask);
  }

  return true;
}

bool c_eccflow::compute_uv(pyramid_entry & e, const cv::Mat2f & rmap, cv::Mat2f & uv) const
{
  INSTRUMENT_REGION("");

  cv::Mat1b M;
  cv::Mat1f It;
  cv::Mat2f Itxy;

  if( true ) {
    // INSTRUMENT_REGION("compute_uv_1_remap");

    cv::remap(e.current_image, W,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);

    if( e.current_mask.empty() ) {
      M.release();
    }
    else {
      cv::remap(e.current_mask, M,
          rmap, cv::noArray(),
          cv::INTER_NEAREST,
          cv::BORDER_CONSTANT,
          cv::Scalar::all(0));
    }
  }

  const cv::Mat1f & I1 = W;
  const cv::Mat1f & I2 = e.reference_image;

  /*
   * It = I2 - I1
   * Itx = It * Ix
   * Ity = It * Iy
   */

  if ( true ) {
    // INSTRUMENT_REGION("compute_Itxy");

    if ( !e.reference_mask.empty() ) {
      if ( M.empty() ) {
        M = e.reference_mask;
      }
      else {
        cv::bitwise_and(e.reference_mask, M, M);
      }
    }

    const cv::Size size = I2.size();

    Itxy.create(size);

    const uint8_t * M_base = M.empty() ? nullptr : M.ptr();
    const size_t M_stride = M.empty() ? 0 : M.step;

    const uint8_t * I1_base = I1.ptr();
    const size_t I1_stride = I1.step;

    const uint8_t * I2_base = I2.ptr();
    const size_t I2_stride = I2.step;

    const uint8_t * Ix_base = e.Ix.ptr();
    const size_t Ix_stride = e.Ix.step;

    const uint8_t * Iy_base = e.Iy.ptr();
    const size_t Iy_stride = e.Iy.step;

    uint8_t * Itxy_base = Itxy.ptr();
    const size_t Itxy_stride = Itxy.step;

    parallel_for(0, size.height, [=](const auto & range) {
      for ( int y = rbegin(range); y < rend(range); ++y ) {
        const float * I1p = (const float * )(I1_base + y * I1_stride);
        const float * I2p = (const float * )(I2_base + y * I2_stride);
        const float * Ixp = (const float * )(Ix_base + y * Ix_stride);
        const float * Iyp = (const float * )(Iy_base + y * Iy_stride);

        float * __restrict Itxyp = (float * )(Itxy_base + y * Itxy_stride);

        if ( !M_base ) {
          for ( int x = 0; x < size.width; ++x, Itxyp += 2 ) {
            const float It = I2p[x] - I1p[x];
            Itxyp[0] = It * Ixp[x];
            Itxyp[1] = It * Iyp[x];
          }
        }
        else {
          const uint8_t * Mp = (const uint8_t * )(M_base + y * M_stride);
          for ( int x = 0; x < size.width; ++x, ++Mp,  Itxyp += 2 ) {
            if ( !*Mp ) {
              Itxyp[0] = Itxyp[1] = 0;
            }
            else {
              const float It = I2p[x] - I1p[x];
              Itxyp[0] = It * Ixp[x];
              Itxyp[1] = It * Iyp[x];
            }
          }
        }
      }
    });
  }

  if ( true ) {
    // INSTRUMENT_REGION("avgdown_Itxy");
    avgdown(Itxy, Itxy);
  }

  //  a00 = Ixx;
  //  a01 = Ixy;
  //  a10 = Ixy;
  //  a11 = Iyy;
  //  b0  = 2 * Itx;
  //  b1  = 2 * Ity;
  //  D = a00 * a11 - a10 * a01
  //  u = 1/D * (a11 * b0 - a01 * b1);
  //  v = 1/D * (a00 * b1 - a10 * b0);

  uv.create(e.D.size());

  if ( true ) {
    // INSTRUMENT_REGION("main_loop");

    const int rows = uv.rows;
    const int cols = uv.cols;

    const uint8_t * D_base = e.D.ptr();
    const size_t D_stride = e.D.step;

    const uint8_t * Itxy_base = Itxy.ptr();
    const size_t Itxy_stride = Itxy.step;

    const uint8_t * uv_base = uv.ptr();
    const size_t uv_stride = uv.step;

    parallel_for(0, rows, [=](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const float * D = (const float * )(D_base + y * D_stride);
        const float * Itxy = (float * )(Itxy_base + y * Itxy_stride);
        float * __restrict uv = (float * )(uv_base + y * uv_stride);
        for ( int x = 0; x < cols; ++x, D += 4, Itxy += 2, uv += 2 ) {
          const float & a00 = D[0];
          const float & a01 = D[1];
          const float & a10 = D[1];
          const float & a11 = D[2];
          const float & det = D[3];
          const float & b0  = Itxy[0];
          const float & b1  = Itxy[1];
          uv[0] = det * (a11 * b0 - a01 * b1);
          uv[1] = det * (a00 * b1 - a10 * b0);
        }
      }
    });
  }

  if ( true ) {
    // INSTRUMENT_REGION("resize(uv)");
    //avgup(uv, I1.size());
    cv::resize(uv, uv, I1.size(), 0, 0, cv::INTER_CUBIC);
  }

  return true;
}

void c_eccflow::avgdown(cv::InputArray src, cv::Mat & dst) const
{
  INSTRUMENT_REGION("");

  cv::Size size = src.size();

  for ( int i = 0; i < _opts.support_scale; ++i ) {
    size.width = (size.width + 1) / 2;
    size.height = (size.height + 1) / 2;
  }

  cv::resize(src, dst, size, 0, 0, cv::INTER_AREA);

  static const cv::Mat G =
      cv::getGaussianKernel(3, 0, CV_32F);

  cv::sepFilter2D(dst, dst, -1, G, G, cv::Point(-1,-1), 0,
      cv::BORDER_REPLICATE);
}

void c_eccflow::avgup(cv::Mat & image, const cv::Size & dstSize) const
{
  ecc_upscale(image, dstSize);
}

void c_eccflow::avgp(cv::InputArray src1, cv::InputArray src2, cv::Mat & dst) const
{
  cv::multiply(src1, src2, dst);
  avgdown(dst, dst);
}

void c_eccflow::downscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{
  INSTRUMENT_REGION("");

  switch (_opts.downscale) {
    case ECCFlowDownscalePyramid:
      cv::pyrDown(src, dst, dst_size);
      break;
    default:
      cv::resize(src, dst, dst_size, 0, 0, cv::INTER_AREA);
      break;
  }

  if( dst_mask.needed() ) {
    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {
      cv::resize(src_mask, dst_mask, dst.size(), 0, 0, cv::INTER_NEAREST);
      //cv::resize(src_mask, dst_mask, dst.size(), 0, 0, cv::INTER_AREA);
      //cv::compare(dst_mask.getMat(), cv::Scalar::all(250), dst_mask, cv::CMP_GE);
    }
  }
}

void c_eccflow::upscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{
  INSTRUMENT_REGION("");

  cv::resize(src, dst, dst_size, 0, 0, cv::INTER_CUBIC);

  if( dst_mask.needed() ) {
    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {
      cv::resize(src_mask, dst_mask, dst.size(), 0, 0, cv::INTER_LINEAR);
      cv::compare(dst_mask.getMat(), cv::Scalar::all(254), dst_mask, cv::CMP_GE);
    }
  }
}

const cv::Mat1f & c_eccflow::reference_image() const
{
  static const cv::Mat1f empty_stub;
  return _pyramid.empty() ?  empty_stub : _pyramid.front().reference_image;
}

const cv::Mat1b & c_eccflow::reference_mask() const
{
  static const cv::Mat1b empty_stub;
  return _pyramid.empty() ? empty_stub : _pyramid.front().reference_mask;
}

bool c_eccflow::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if ( reference_image.channels() != 1 ) {
    CF_ERROR("Single channel input image is expected on input. reference_image.channels=%d",
        reference_image.channels());
    return false;
  }

  if ( !reference_mask.empty() ) {

    if ( reference_mask.size() != reference_image.size() ) {
      CF_ERROR("Invalid reference mask size: %dx%d. Must be is %dx%d",
          reference_mask.cols(), reference_mask.rows(),
          reference_image.cols(), reference_image.rows());

      return false;
    }

    if ( reference_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid reference mask type: %d. Must be CV_8UC1",
          reference_mask.type());
      return false;
    }
  }

  const double noise_level =
      _opts.noise_level >= 0 ? _opts.noise_level : 1e-3;

  cv::Mat1f Ixx;
  cv::Mat1f Ixy;
  cv::Mat1f Iyy;

  _pyramid.clear();
  _pyramid.reserve(32);

  const int min_image_size = std::max(4, _opts.min_image_size);
  const cv::Size image_size = reference_image.size();
  const bool big_aspect_ratio =
      std::max(image_size.width, image_size.height) / std::min(image_size.width, image_size.height) >= 2;

  for( int current_level = 0; ; ++current_level ) {

    if ( current_level == 0 ) {

      _pyramid.emplace_back();

      convert_input_images(reference_image, reference_mask,
          _pyramid.back().reference_image,
          _pyramid.back().reference_mask);
    }
    else {

      const cv::Size previous_size = _pyramid.back().reference_image.size();

      if( _opts.downscale == ECCFlowDownscaleRecursiveResize ) {

        const cv::Size next_size(std::max(_opts.min_image_size, (int) ((previous_size.width + 1) * _opts.scale_factor)),
            std::max(_opts.min_image_size, (int) ((previous_size.height + 1) * _opts.scale_factor)));

        if( previous_size == next_size || std::max(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        _pyramid.emplace_back();

        if( big_aspect_ratio && std::min(next_size.width, next_size.height) <= min_image_size + 1 ) {
          downscale(_pyramid.front().reference_image, _pyramid.front().reference_mask,
              _pyramid.back().reference_image, _pyramid.back().reference_mask,
              next_size);
        }
        else {
          downscale(_pyramid[current_level - 1].reference_image, _pyramid[current_level - 1].reference_mask,
              _pyramid.back().reference_image, _pyramid.back().reference_mask,
              next_size);
        }

      }
      else if( _opts.downscale == ECCFlowDownscaleFullResize ) {

        const cv::Size next_size(std::max(_opts.min_image_size, (int) ((previous_size.width + 1) * _opts.scale_factor)),
            std::max(_opts.min_image_size, (int) ((previous_size.height + 1) * _opts.scale_factor)));

        if( previous_size == next_size || std::max(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        _pyramid.emplace_back();

        downscale(_pyramid.front().reference_image, _pyramid.front().reference_mask,
            _pyramid.back().reference_image, _pyramid.back().reference_mask,
            next_size);

      }
      else {  // DownscalePyramid

        const cv::Size next_size(std::max(_opts.min_image_size, (previous_size.width + 1) / 2),
            std::max(_opts.min_image_size, (previous_size.height + 1) / 2));

        if( previous_size == next_size || std::min(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        _pyramid.emplace_back();

        downscale(_pyramid[current_level - 1].reference_image, _pyramid[current_level - 1].reference_mask,
            _pyramid.back().reference_image, _pyramid.back().reference_mask,
            next_size);

      }
    }

    pyramid_entry & current_scale = _pyramid.back();

    ecc_differentiate(current_scale.reference_image,
        current_scale.Ix, current_scale.Iy);

    if( true ) {
      // INSTRUMENT_REGION("avgp2");
      parallel_invoke(
        [this, &current_scale, &Ixx]() {
          avgp(current_scale.Ix, current_scale.Ix, Ixx);
        },
        [this, &current_scale, &Ixy]() {
          avgp(current_scale.Ix, current_scale.Iy, Ixy);
        },
        [this, &current_scale, &Iyy]() {
          avgp(current_scale.Iy, current_scale.Iy, Iyy);
        }
     );
    }


    if (true) {
      // INSTRUMENT_REGION("compute_d");

      // FIXME: this regularization term estimation looks crazy
      const float RegularizationTerm = noise_level > 0 ? pow(1e-5 * noise_level / (1 << current_level), 4) : 0;
      const float update_mult_f = float(_opts.update_multiplier);

      const cv::Size size = Ixx.size();
      current_scale.D.create(size);

      const uint8_t * Ixx_base = Ixx.ptr();
      const size_t Ixx_stride = Ixx.step;

      const uint8_t * Ixy_base = Ixy.ptr();
      const size_t Ixy_stride = Ixy.step;

      const uint8_t * Iyy_base = Iyy.ptr();
      const size_t Iyy_stride = Iyy.step;

      uint8_t * D_base = (uint8_t*) current_scale.D.ptr();
      const size_t D_stride = current_scale.D.step;

      parallel_for(0, size.height, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          const float * pIxx = (const float * )(Ixx_base + y * Ixx_stride);
          const float * pIxy = (const float * )(Ixy_base + y * Ixy_stride);
          const float * pIyy = (const float * )(Iyy_base + y * Iyy_stride);
          cv::Vec4f * __restrict pD = (cv::Vec4f * )(D_base + y * D_stride);
          for ( int x = 0; x < size.width; ++x, ++pD ) {
            const float a00 = pIxx[x];
            const float a01 = pIxy[x];
            const float a11 = pIyy[x];

            // SLAE determinant: det = abs(a00 * a11 - a01 * a01)
            const float det = std::abs(a00 * a11 - a01 * a01);
            const float idet = update_mult_f / (det + RegularizationTerm);
            *pD = cv::Vec4f(a00, a01, a11, idet);
          }
        }
      });
    }

    if ( _opts.max_pyramid_level >= 0 && current_level >= _opts.max_pyramid_level ) {
      break;
    }

  }

  return true;
}

bool c_eccflow::setup_input_image(cv::InputArray input_image, cv::InputArray input_mask)
{
  INSTRUMENT_REGION("");

  if( _pyramid.empty() ) {
    CF_ERROR("Reference pyramid is empty: set_reference_image() must be called first");
    return false;
  }

  if ( input_image.channels() != 1 ) {
    CF_ERROR("Single channel input image is expected. input_image.channels=%d",
        input_image.channels());
    return false;
  }

  if( !input_mask.empty() ) {

    if( input_mask.size() != input_image.size() ) {
      CF_ERROR("Invalid input mask size: %dx%d. Must be is %dx%d",
          input_mask.cols(), input_mask.rows(),
          input_image.cols(), input_image.rows());

      return false;
    }

    if( input_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid input mask type: %d. Must be CV_8UC1",
          input_mask.type());
      return false;
    }
  }

  const cv::Size image_size = input_image.size();

  const bool big_aspect_ratio =
      std::max(image_size.width, image_size.height) /
        std::min(image_size.width, image_size.height) >= 2;

  const int num_levels = (int)(_pyramid.size());
  for( int current_level = 0; current_level < num_levels; ++current_level ) {

    pyramid_entry & current_scale = _pyramid[current_level];

    if ( current_level == 0 ) {
      convert_input_images(input_image, input_mask,
          current_scale.current_image,
          current_scale.current_mask);
    }
    else if( _opts.downscale == ECCFlowDownscaleFullResize ) {

      const pyramid_entry & base_scale = _pyramid.front();

      downscale(base_scale.current_image, base_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
    }
    else if( _opts.downscale == ECCFlowDownscaleRecursiveResize ) {

      const cv::Size next_size = current_scale.reference_image.size();
      const int min_image_size = std::max(4, _opts.min_image_size);

        if( big_aspect_ratio && std::min(next_size.width, next_size.height) <= min_image_size + 1 ) {

          const pyramid_entry & base_scale = _pyramid.front();

          downscale(base_scale.current_image, base_scale.current_mask,
              current_scale.current_image, current_scale.current_mask,
              current_scale.reference_image.size());
        }
        else {

          const pyramid_entry & previous_scale = _pyramid[current_level - 1];

          downscale(previous_scale.current_image, previous_scale.current_mask,
              current_scale.current_image, current_scale.current_mask,
              current_scale.reference_image.size());

        }
    }
    else {

      const pyramid_entry & previous_scale = _pyramid[current_level - 1];

      downscale(previous_scale.current_image, previous_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
    }

  }

  return true;
}



// Must be called after set_reference_image()
bool c_eccflow::compute_uv(cv::InputArray input_image, const cv::Mat2f & rmap, cv::InputArray input_mask)
{
  INSTRUMENT_REGION("");

  cv::Mat2f cuv, crmap;

  if ( !setup_input_image(input_image, input_mask) ) {
    CF_ERROR("setup_input_image() fails");
    return false;
  }

  const int num_levels = (int) (_pyramid.size());

  if( rmap.empty() ) {
    uv.create(_pyramid.back().reference_image.size());
    uv.setTo(cv::Scalar::all(0));
  }
  else if( rmap.size() == _pyramid.front().reference_image.size() ) {

    const pyramid_entry & first_scale = _pyramid.front();
    const cv::Size first_size = first_scale.reference_image.size();

    const pyramid_entry & last_scale = _pyramid.back();
    const cv::Size last_size = last_scale.reference_image.size();

    const cv::Scalar size_ratio((double) last_size.width / (double) first_size.width,
        (double) last_size.height / (double) first_size.height);

    ecc_remap_to_optflow(rmap, uv);
    cv::resize(uv, uv, last_size, 0, 0, cv::INTER_CUBIC); // / cv::INTER_AREA/*cv::INTER_CUBIC*/);
    cv::multiply(uv, size_ratio, uv);
  }
  else {
    CF_ERROR("Invalid args to c_eccflow::compute(): reference image and rmap sizes not match");
    return false;
  }

  /////////////////////////////////////

  for( int i = num_levels - 1; i >= 0; --i ) {

    pyramid_entry & current_scale = _pyramid[i];

    if( i < num_levels - 1 ) {

      const pyramid_entry & prev_scale = _pyramid[i + 1];

      const cv::Size current_size = current_scale.current_image.size();
      const cv::Size prev_size = prev_scale.current_image.size();

      const cv::Scalar size_ratio((double) current_size.width / (double) prev_size.width,
          (double) current_size.height / (double) prev_size.height);

      cv::multiply(uv, size_ratio, uv);
      upscale(uv, cv::noArray(), uv, cv::noArray(), current_size);
    }

    for( int j = 0; j < _opts.max_iterations; ++j ) {
      ecc_flow_to_remap(uv, crmap);
      compute_uv(current_scale, crmap, cuv);
      cv::add(uv, cuv, uv);
    }
  }

  return true;
}

// Sets reference image and calls compute_uv()
bool c_eccflow::compute_uv(cv::InputArray input_image, cv::InputArray reference_image, const cv::Mat2f & rmap,
    cv::InputArray input_mask, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");
  set_reference_image(reference_image, reference_mask);
  return compute_uv(input_image, rmap, input_mask);
}


bool c_eccflow::compute(cv::InputArray input_image, cv::InputArray reference_image, cv::Mat2f & rmap,
    cv::InputArray input_mask, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  set_reference_image(reference_image, reference_mask);
  return compute(input_image, rmap, input_mask);
}

bool c_eccflow::compute(cv::InputArray input_image, cv::Mat2f & rmap, cv::InputArray input_mask)
{
  INSTRUMENT_REGION("");

  if ( compute_uv(input_image, rmap, input_mask) ) {
    ecc_flow_to_remap(uv, rmap);
    return true;
  }
  return false;
}

bool serialize_ecch_options(c_config_setting section, bool save, c_ecch_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, xtol);
  SERIALIZE_OPTION(section, save, opts, ftol);
  SERIALIZE_OPTION(section, save, opts, gtol);
  SERIALIZE_OPTION(section, save, opts, reference_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, input_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, update_step_scale);
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, pyramid_type);
  SERIALIZE_OPTION(section, save, opts, interpolation);
  SERIALIZE_OPTION(section, save, opts, max_iterations);
  SERIALIZE_OPTION(section, save, opts, minimum_image_size);
  SERIALIZE_OPTION(section, save, opts, maxlevel);
  SERIALIZE_OPTION(section, save, opts, force_direct_methods_for_coarse_levels);

  return true;
}

bool serialize_eccflow_options(c_config_setting section, bool save, c_eccflow_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, downscale);
  SERIALIZE_OPTION(section, save, opts, min_image_size);
  SERIALIZE_OPTION(section, save, opts, max_pyramid_level);
  SERIALIZE_OPTION(section, save, opts, noise_level);
  SERIALIZE_OPTION(section, save, opts, support_scale);
  SERIALIZE_OPTION(section, save, opts, max_iterations);
  SERIALIZE_OPTION(section, save, opts, input_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, reference_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, update_multiplier);
  SERIALIZE_OPTION(section, save, opts, scale_factor);
  return true;
}
