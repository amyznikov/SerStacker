/*
 * ecc2.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */
#include "ecc2.h"
#include <core/proc/run-loop.h>
#include <core/settings/opencv_settings.h>
//#include <core/ssprintf.h>
//#include <core/io/save_image.h>
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

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
    {ECC_ALIGN_FORWARD_ADDITIVE, "FORWARD_ADDITIVE", },
    {ECC_ALIGN_LM, "LM", },
    {ECC_ALIGN_INVERSE_COMPOSITIONAL, "INVERSE_COMPOSITIONAL", "Requires invertible image transform"},
    {ECC_ALIGN_INVERSE_COMPOSITIONAL_LM, "INVERSE_COMPOSITIONAL_LM", },
    {ECC_ALIGN_LM},
  };
  return members;
}

double compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask)
{
  const cv::Size size = src1.size();
  const int cn = src1.channels();

  cv::Mat img1, img2;
//  cv::Mat img1(size, CV_MAKETYPE(CV_32F, cn), cv::Scalar::all(0));
//  cv::Mat img2(size, CV_MAKETYPE(CV_32F, cn), cv::Scalar::all(0));

  cv::Scalar m1, m2, s1, s2;

  const int npix = mask.empty() ? size.area() : cv::countNonZero(mask);

  cv::meanStdDev(src1, m1, s1, mask);
  cv::meanStdDev(src2, m2, s2, mask);

  cv::subtract(src1, m1, img1, mask, CV_32F);
  cv::subtract(src2, m2, img2, mask, CV_32F);

  if ( !mask.empty() ) {
    img1.setTo(0, ~mask.getMat());
    img2.setTo(0, ~mask.getMat());
  }

  const double covar = img1.dot(img2) / (npix);
  double std1 = 0, std2 = 0;

  for( int i = 0; i < cn; ++i ) {
    std1 += s1[i];
    std2 += s2[i];
  }

  // CF_DEBUG("m1=%g m2=%g s1=%g s2=%g npix=%d covar=%g", m1[0], m2[0], s1[0], s2[0], npix, covar);

  return covar * cn / (std1 * std2);
}


double compute_correlation(cv::InputArray current_image, cv::InputArray current_mask, cv::InputArray reference_image, cv::InputArray reference_mask,  const cv::Mat2f & rmap)
{
  cv::Mat remapped_image, remapped_mask;

  cv::remap(current_image, remapped_image,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  if ( current_mask.empty() ) {

    cv::remap(cv::Mat1b(current_image.size(), (uint8_t) 255), remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

  }
  else {

    cv::remap(current_mask, remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
  }

  cv::compare(remapped_mask, 254, remapped_mask,
      cv::CMP_GE);

  if ( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  return compute_correlation(reference_image, remapped_image, remapped_mask);
}


namespace {

static void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, cv::InputArray mask = cv::noArray() )
{
  INSTRUMENT_REGION("");

  // 4th order derivative vector (5x1)
  // 2nd order smoothing  vector (3x1)
  static const cv::Matx<float, 5, 1> d5( +1.f/12.f, -2.f/3.f, 0.f, +2.f/3.f, -1.f/12.f );
  static const cv::Matx<float, 3, 1> s3( 0.25f, 0.5f, 0.25f );

  cv::Mat1b m;
  if( !mask.empty() ) {
    cv::bitwise_not(mask, m);
  }

  parallel_invoke(
      [&]() {
        cv::sepFilter2D(src, gx, CV_32F, d5, s3, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !m.empty() ) {
          gx.setTo(0, m);
        }
      },
      [&]() {
        cv::sepFilter2D(src, gy, CV_32F, s3, d5, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !m.empty() ) {
          gy.setTo(0, m);
        }
      });
}


static inline void ecc_remap(cv::InputArray _src, cv::OutputArray _dst, const cv::Mat2f & rmap, cv::BorderTypes borderType = cv::BORDER_REPLICATE)
{
  INSTRUMENT_REGION("");
  cv::remap(_src.getMat(), _dst, rmap, cv::noArray(), cv::INTER_LINEAR, borderType);
}

static bool ecc_remap(const c_image_transform * image_transform,
    const cv::Mat1f & params,
    const cv::Size & size,
    cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    cv::BorderTypes borderType = cv::BORDER_REPLICATE)
{
  INSTRUMENT_REGION("");

  cv::Mat2f rmap;

  if( !image_transform->create_remap(params, size, rmap) ) {
    CF_ERROR("image_transform->create_remap() fails");
    return false;
  }

  ecc_remap(src, dst, rmap, borderType);

  if( !src_mask.empty() ) {

    cv::remap(src_mask.getMat(), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }
  else {

    cv::remap(cv::Mat1b(src.size(), (uint8_t) (255)), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }

  cv::compare(dst_mask.getMat(), 250, dst_mask,
      cv::CMP_GE);

  return true;
}

/*
 * Pyramid down to specific level
 */
bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
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
bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
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

/*
 * Create identity remap
 */
void ecc_create_identity_remap(cv::Mat2f & rmap, const cv::Size & size)
{
  rmap.create(size);

  parallel_for(0, rmap.rows, [&rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float * )rmap[y];
      for ( int x = 0; x < rmap.cols; ++x, mp += 2) {
        mp[0] = x;
        mp[1] = y;
      }
    }
  });
}

/**
 * Compute Hessian matrix for ECC image alignment
 */
void ecc_compute_hessian_matrix(const std::vector<cv::Mat1f> & J, cv::Mat1f & H/*, int nparams*/)
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


} // namespace



bool ecc_convert_input_image(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask)
{
  INSTRUMENT_REGION("");

  if( !src_mask.empty() && (src_mask.size() != src.size() || src_mask.type() != CV_8UC1) ) {

    CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
        src_mask.cols(), src_mask.rows(), src_mask.channels(), src_mask.depth(),
        src.cols(), src.rows());

    return false;
  }


  if( src.channels() == 1 ) {
    src.getMat().convertTo(dst, dst.depth());
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(src, tmp, cv::COLOR_BGR2GRAY);
    if( tmp.depth() == dst.depth() ) {
      dst = std::move(tmp);
    }
    else {
      tmp.convertTo(dst, dst.depth());
    }
  }

  if( src_mask.empty() ) {
    dst_mask.release();
  }
  else {
    src_mask.copyTo(dst_mask);
  }

  return true;
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

void c_ecc_align::set_max_eps(double v)
{
  this->_max_eps = v;
}

double c_ecc_align::max_eps() const
{
  return this->_max_eps;
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
  _interpolation = rhs._interpolation;
  _num_iterations  = rhs._num_iterations;
  _max_iterations = rhs._max_iterations;
  _update_step_scale = rhs._update_step_scale;
  _max_eps = rhs._max_eps;
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
  if( !reference_mask.empty()
      && (reference_mask.size() != reference_image.size() || reference_mask.type() != CV_8UC1) ) {

    CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
        reference_mask.cols(), reference_mask.rows(),  reference_mask.channels(), reference_mask.depth(),
        reference_image.cols(), reference_image.rows());

    return false;
  }

  if( reference_image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
        reference_image.cols(), reference_image.rows(),
        reference_image.depth(),
        reference_image.channels());
    return false;
  }

  reference_image.copyTo(_reference_image);
  reference_mask.copyTo(_reference_mask);

  return true;
}

bool c_ecc_align::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if( !current_mask.empty() && (current_mask.size() != current_image.size() || current_mask.type() != CV_8UC1) ) {

     CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
         current_mask.cols(), current_mask.rows(), current_mask.channels(), current_mask.depth(),
         current_image.cols(), current_image.rows());

     return false;
   }

   if( current_image.type() != CV_32FC1 ) {
     CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
         current_image.cols(), current_image.rows(), current_image.depth(), current_image.channels());
     return false;
   }

   current_image.copyTo(_current_image);
   current_mask.copyTo(_current_mask);

   return true;
}

void c_ecc_align::release_current_image()
{
  _current_image.release();
  _current_mask.release();
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

void c_ecch::set_epsx(double v)
{
  _opts.epsx = v;
  for( const auto & m : _pyramid ) {
    m->set_max_eps(v);
    v *= 2;
  }
}

double c_ecch::epsx() const
{
  return _opts.epsx;
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

void c_ecch::set_max_eps(double v)
{
  set_epsx(v);
}

double c_ecch::max_eps() const
{
  return _opts.epsx;
}

//void c_ecch::set_min_rho(double v)
//{
//  _opts.min_rho = v;
//}
//
//double c_ecch::min_rho() const
//{
//  return _opts.min_rho;
//}

double c_ecch::eps() const
{
  return _pyramid.empty() ? -1 : _pyramid.front()->eps();
}

//double c_ecch::rho() const
//{
//  return _pyramid.empty() ? -1 : _pyramid.front()->();
//}

int c_ecch::num_iterations() const
{
  return _num_iterations;//  pyramid_.empty() ? -1 : pyramid_.front()->num_iterations();
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
  if( _pyramid.empty() ) {
    static const cv::Mat1f empty_image;
    return empty_image;
  }
  return _pyramid.front()->reference_image();
}

const cv::Mat1b & c_ecch::reference_mask() const
{
  if( _pyramid.empty() ) {
    static const cv::Mat1b empty_image;
    return empty_image;
  }
  return _pyramid.front()->reference_mask();
}

const cv::Mat1f & c_ecch::current_image() const
{
  if( _pyramid.empty() ) {
    static const cv::Mat1f empty_image;
    return empty_image;
  }
  return _pyramid.front()->current_image();
}

const cv::Mat1b & c_ecch::current_mask() const
{
  if( _pyramid.empty() ) {
    static const cv::Mat1b empty_image;
    return empty_image;
  }
  return _pyramid.front()->current_mask();
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

c_ecc_align::uptr c_ecch::create_ecc_align(double epsx) const
{
  c_ecc_align::uptr ecc;

  switch (_opts.method) {
    case ECC_ALIGN_FORWARD_ADDITIVE:
      ecc.reset(new c_ecc_forward_additive());
      break;
    case ECC_ALIGN_INVERSE_COMPOSITIONAL:
      ecc.reset(new c_ecc_inverse_compositional());
      break;
    case ECC_ALIGN_INVERSE_COMPOSITIONAL_LM:
      ecc.reset(new c_ecclm_inverse_compositional());
      break;
    default:
      ecc.reset(new c_ecclm());
      break;
  }

  ecc->set_image_transform(_image_transform);
  ecc->set_interpolation(_opts.interpolation);
  ecc->set_max_iterations(_opts.max_iterations);
  ecc->set_update_step_scale(_opts.update_step_scale);
  ecc->set_max_eps(epsx);

  return ecc;
}

inline void c_ecch::downscale_image(cv::Mat & image, cv::Mat & mask,
    const cv::Size & nextSize)
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
  INSTRUMENT_REGION("c_ecch");

  cv::Mat1f image;
  cv::Mat1b mask;

  _num_iterations = -1;

  if( !ecc_convert_input_image(reference_image, reference_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( _opts.reference_smooth_sigma > 0 ) {
    INSTRUMENT_REGION("sepFilter2D");
    if( _opts.reference_smooth_sigma != sigmaRef || Gref.empty() ) {
      const int ksize = std::max(3, 2 * ((int) (3 * _opts.reference_smooth_sigma)) + 1);
      Gref = cv::getGaussianKernel(ksize, _opts.reference_smooth_sigma);
      sigmaRef = _opts.reference_smooth_sigma;
    }
    cv::sepFilter2D(image, image, -1, Gref, Gref, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }


  // Estimate how many pyramid levels is required
  const int min_image_size = std::max(4, this->_opts.minimum_image_size);

  cv::Size test_size = image.size();
  int required_lvls = 1;
  while (true) {
    if( _opts.maxlevel >= 0 && required_lvls >= std::max(0, _opts.maxlevel) ) {
      break;
    }

    const cv::Size next_size = compute_next_pyramid_layer_size(test_size);
    if( next_size.width < min_image_size || next_size.height < min_image_size ) {
      break;
    }

    test_size = next_size;
    ++required_lvls;
  }

  if( _pyramid.size() != required_lvls ) {
    _pyramid.clear();
    double epsx = _opts.epsx;
    for( int i = 0; i < required_lvls; ++i ) {
      _pyramid.emplace_back(create_ecc_align(epsx));
      epsx *= 2; // scale stop criteria for coarse levels
    }
  }

  if ( true ) {
    INSTRUMENT_REGION("build_pyramid");

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
  }

  return true;
}


bool c_ecch::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("c_ecch");

  if( _pyramid.empty() ) {
    CF_ERROR("Reference image must be set first");
    return false;
  }

  cv::Mat1f image;
  cv::Mat1b mask;

  _num_iterations = -1;

  if( !ecc_convert_input_image(current_image, current_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( _opts.input_smooth_sigma > 0 ) {
    INSTRUMENT_REGION("sepFilter2D");
    if( _opts.input_smooth_sigma != sigmaCur || Gcur.empty() ) {
      const int ksize = std::max(3, 2 * ((int) (3 * _opts.input_smooth_sigma)) + 1);
      Gcur = cv::getGaussianKernel(ksize, _opts.input_smooth_sigma);
      sigmaCur = _opts.input_smooth_sigma;
    }
    cv::sepFilter2D(image, image, -1, Gcur, Gcur, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }

  const int lvls = _pyramid.size();

  int lvl = 0;

  if ( true ) {
    INSTRUMENT_REGION("build_pyramid");
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
      _pyramid[lvl]->release_current_image();
    }
  }

  return true;
}

bool c_ecch::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("c_ecch");
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecch: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecch::align()
{
  INSTRUMENT_REGION("c_ecch");
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

  _num_iterations = 0;

  for( ; lvl >= 0; --lvl ) {

    if( !_pyramid[lvl]->align() ) {
      CF_ERROR("pyramid_[lvl=%d]->align() fails", lvl);
    }
    else if( lvl > 0 ) {
      const cv::Size size0 = _pyramid[lvl]->reference_image().size();
      const cv::Size size1 = _pyramid[lvl - 1]->reference_image().size();
      _image_transform->scale_transfrom((double) size1.width / (double) size0.width);
    }

    _num_iterations += _pyramid[lvl]->num_iterations();
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////


c_ecc_forward_additive::c_ecc_forward_additive(c_image_transform * image_transform) :
    base(image_transform)
{
}

bool c_ecc_forward_additive::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  //jac.clear();

  if( !base::set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("base::set_reference_image() fails");
    return false;
  }

  if ( !_reference_mask.empty() ) {

    if ( cv::countNonZero(_reference_mask) == _reference_mask.size().area() ) {
      _reference_mask.release();
    }
    else { // may need to protect some border near mask edges because of differentiation

      cv::erode(_reference_mask, _reference_mask, cv::Mat1b(5, 5, 255),
          cv::Point(-1, -1), 1,
          cv::BORDER_REPLICATE);

    }
  }

  return true;
}


bool c_ecc_forward_additive::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if( !base::set_current_image(current_image, current_mask) ) {
    CF_ERROR("base::set_current_image() fails");
    return false;
  }

  if( _current_mask.empty() ) {
    _current_mask.create(_current_image.size());
    _current_mask.setTo(255);
  }
  else if( cv::countNonZero(_current_mask) != _current_mask.size().area() ) {
    // may need to protect some border near mask edges because of differentiation

    cv::erode(_current_mask, _current_mask, cv::Mat1b(5, 5, 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }

  return true;
}

bool c_ecc_forward_additive::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecc_forward_additive::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

bool c_ecc_forward_additive::align()
{
  INSTRUMENT_REGION("");

  _failed = true;

  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecc_forward_additive: reference image was not set");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("c_ecc_forward_additive: current image was not set");
    return false;
  }

  if( !_transform ) {
    CF_ERROR("c_ecc_forward_additive: image transform was not set");
    return false;
  }

  _failed = false;
  _num_iterations = -1;
  //rho_ = -1;

  if( _max_eps <= 0 ) {
    _max_eps = 1e-3;
  }

  if( (_nparams = _transform->parameters().rows) < 1 ) {
    CF_FATAL("image_transform_->image_transform_->parameters().rows return %d", _nparams);
    _failed = true;
    return false;
  }

  if ( jac.size() != _nparams ) {
    jac.resize(_nparams);
  }

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio;
  cv::Mat2f current_remap;

  _num_iterations = 0;
  while (_num_iterations++ < _max_iterations) {

    // Warp g, gx and gy with W(x; p) to compute warped input image g(W(x; p)) and it's gradients

    if( !_transform->create_remap(_reference_image.size(), current_remap) ) {
      CF_ERROR("[i %d] create_current_remap() fails", _num_iterations);
      _failed = true;
      break;
    }

    tbb::parallel_invoke(
        [this, &current_remap]() {
          cv::remap(_current_image, gw, current_remap, cv::noArray(), _interpolation, cv::BORDER_REPLICATE);
          ecc_differentiate(gw, gxw, gyw);
        },
        [this, &current_remap]() {
          cv::remap(_current_mask, wmask, current_remap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
          cv::compare(wmask, 255, wmask, cv::CMP_GE);
          if( !_reference_mask.empty() ) {
            bitwise_and(wmask, _reference_mask, wmask);
          }
          cv::bitwise_not(wmask, iwmask);
        });


    gxw.setTo(0, iwmask);
    gyw.setTo(0, iwmask);

    // compute stdev ratio stdev(g)/stdev(f) and mean values
    cv::meanStdDev(_reference_image, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = gStd[0] / fStd[0];

    // create steepest descent images
    _transform->create_steepest_descent_images(gxw, gyw, jac.data());
    ecc_compute_hessian_matrix(jac, H);
    if( !cv::invert(H, H, cv::DECOMP_CHOLESKY) ) {
      CF_ERROR("[i %d] cv::invert(H) fails", _num_iterations);
      _failed = true;
      break;
    }

    // calculate Hessian and its inverse


    // compute update parameters
    // e = -(gwzm - stdev_ratio * fzm);
    cv::scaleAdd(_reference_image, -stdev_ratio, gw, rhs);
    cv::subtract(rhs, gMean - stdev_ratio * fMean, rhs);
    rhs.setTo(0, iwmask);

    // compute projected error
    ecc_project_error_image(jac, rhs, ep);

    // compute update parameters
    dp = -_update_step_scale * (H * ep);

    // update warping matrix
    _transform->set_parameters(_transform->parameters() + dp);

    //eps_ = cv::norm(dp, cv::NORM_INF);
    _eps = _transform->eps(dp, _reference_image.size());
    if( _eps < _max_eps ) {
      break;
    }
  }

//  CF_DEBUG("RET: num_iterations=%d eps_=%g ", num_iterations_, eps_);

  return !_failed; //  && rho_ > 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////


c_ecclm::c_ecclm(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm::set_image_transform(c_image_transform * image_transform)
{
  if ( image_transform != this->_transform ) {
    J.clear();
    base::set_image_transform(image_transform);
  }
}

bool c_ecclm::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  J.clear();

  if( !base::set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("base::set_reference_image() fails");
    return false;
  }

  if ( !_reference_mask.empty() ) {

    cv::erode(_reference_mask, _reference_mask,
        cv::Mat1b(5, 5, 255));
  }

  if ( !_transform ) {
    // CF_DEBUG("Still wait for image transform");
    return true;
  }


  return true;
}

bool c_ecclm::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !base::set_current_image(current_image, current_mask) ) {
    CF_ERROR("base::set_current_image() fails");
    return false;
  }

   return true;
}

bool c_ecclm::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  if ( !set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("set_reference_image() fails");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecclm::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}

double c_ecclm::compute_remap(const cv::Mat1f & params,
    cv::Mat1f & remapped_image, cv::Mat1b & remapped_mask, cv::Mat1f & rhs)
{
  const int M = params.rows;
  const cv::Size size(_reference_image.size());

  ecc_remap(_transform, params, size,
      _current_image, _current_mask,
      remapped_image, remapped_mask,
      cv::BORDER_REPLICATE);

  if( remapped_mask.empty() ) {
    remapped_mask = _reference_mask;
  }
  else if( !_reference_mask.empty() ) {
    cv::bitwise_and(_reference_mask, remapped_mask,
        remapped_mask);
  }

  cv::subtract(remapped_image, _reference_image, rhs);
  if ( !remapped_mask.empty() ) {
    rhs.setTo(0, ~remapped_mask);
  }

  const double nrms =
      remapped_mask.empty() ? size.area() :
          cv::countNonZero(remapped_mask);

  return nrms;

}

double c_ecclm::compute_rhs(const cv::Mat1f & params)
{
  compute_remap(params, remapped_image, remapped_mask, rhs);
  return (rms = cv::norm(rhs, cv::NORM_L2SQR));
}

double c_ecclm::compute_jac(const cv::Mat1f & params, bool recompute_remap,
    cv::Mat1f & H, cv::Mat1f & v)
{

  if( recompute_remap ) {
    compute_remap(params, remapped_image, remapped_mask, rhs);
    rms = cv::norm(rhs, cv::NORM_L2SQR);
  }

  const int M = params.rows;
  if( J.size() != M ) {
    J.resize(M);
  }

  ecc_differentiate(remapped_image, _gx, _gy, remapped_mask);

  _transform->create_steepest_descent_images(params, _gx, _gy, J.data());

  v.create(M, 1);

  parallel_for(0, M, [&](const auto & range) {
    for ( int i = rbegin(range), ni = rend(range); i < ni; ++i ) {
      v[i][0] = J[i].dot(rhs);
    }
  });

  H.create(M, M);

  parallel_for(0, M, [&](const auto & range) {
    for ( int i = rbegin(range), ni = rend(range); i < ni; ++i ) {
      for( int j = 0; j <= i; ++j ) {
        H[i][j] = J[i].dot(J[j]);
      }
    }
  });

  for( int i = 0; i < M; ++i ) {
    for( int j = i + 1; j < M; ++j ) {
      H[i][j] = H[j][i];
    }
  }

  return rms;
}


bool c_ecclm::align()
{
  if ( !_transform ) {
    CF_ERROR("c_ecclm: image_transform_ is null");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecclm: reference_image_ is empty");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("c_ecclm: current_image_ is empty");
    return false;
  }

  cv::Mat1f H, Hp, v, deltap, temp_d;
  cv::Mat1f params, newparams;

  params = _transform->parameters();

  const int M = params.rows;
  constexpr double eps = std::numeric_limits<double>::epsilon();

  const double epsx = _max_eps;
  const double epse = _max_epse;
  const int max_iterations = _max_iterations;

  double lambda = 0.1;
  int iteration = 0;
  bool converged = false;
  bool recompute_remap = true;

  J.clear();

  while (iteration < max_iterations) {

    const double err = compute_jac(params, recompute_remap, H, v);
    if ( err < 1 ) {
      converged = true;
      break;
    }

    H.copyTo(Hp);

    /*
     * Solve normal equation for given Jacobian and lambda
     * */
    while (iteration++ < max_iterations) {

      recompute_remap = true;

      /* Increase diagonal elements by lambda */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to compute parameters update
       *  deltap = H.inv() * v
       *  */
      cv::solve(H, v, deltap, cv::DECOMP_CHOLESKY);
      cv::scaleAdd(deltap, -_update_step_scale, params, newparams);

      /* Check for increment in parameters  */
      if( (_eps = _transform->eps(deltap, _reference_image.size())) <= epsx ) {
        _transform->set_parameters(newparams);
        params = _transform->parameters();
        converged = true;
        break;
      }

      /* Compute error function for new parameters */
      const double newerr = compute_rhs(newparams);
      if( newerr > err ) {
        if ( lambda > 1e6 ) {
          break; // no convergence
        }
        lambda *= 10.0f;
        continue;
      }

      /* Accept new parameters */
      _transform->set_parameters(newparams);
      params = _transform->parameters();
      recompute_remap = false;

      /* Check Function Tolerance */
      const double diff = err - newerr;
      if (diff < err * epse ) {
        converged = true;
        break;
      }

      /*
       * Compute step quality (rho) and update to lambda
       * Predicted improvement dS
       * rho = (actual improvement) / (predicted improvement)
       * */
      cv::gemm(Hp, deltap, -1, v, 2, temp_d);
      const double dS = deltap.dot(temp_d);
      //const double rho = (err - newerr) / (std::abs(dS) > eps ? dS : 1);
      const double rho = std::abs(dS) > 1e-9f ? diff / std::abs(dS)  : diff;
      if (rho > 0.25 ) { /* Good step, decrease lambda ==> Gauss-Newton */
        lambda = std::max(1e-8, 0.2 * lambda);
      }
      else if (rho < 0.1) { /* The Taylor model looks poor ==> gradient descend*/
        lambda = (lambda < 1.0) ? 1.0 : lambda * 10.0;
      }
      else {
      }

      break;
    }

    if (converged) {
      break;
    }
  }

  _num_iterations = iteration;
  return converged;
}



///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_inverse_compositional::c_ecc_inverse_compositional(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecc_inverse_compositional::set_image_transform(c_image_transform * image_transform)
{
  jac.clear();
  gx.release();
  gy.release();
  return base::set_image_transform(image_transform);
}

bool c_ecc_inverse_compositional::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  jac.clear();
  gx.release();
  gy.release();
  return base::set_reference_image(reference_image, reference_mask);
}

bool c_ecc_inverse_compositional::set_current_image(cv::InputArray current_image, cv::InputArray current_mask )
{
  return base::set_current_image(current_image, current_mask);
}

bool c_ecc_inverse_compositional::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask )
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecc_inverse_compositional::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

bool c_ecc_inverse_compositional::align()
{
  _failed = true;

  if ( !_transform ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is null");
    return false;
  }

  if ( !_transform->invertible() ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is not invertible");
    return false;
  }


  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: reference_image_ is empty");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: current_image_ is empty");
    return false;
  }

  cv::Mat1f params, newparams, deltap;
  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs;
  cv::Mat1f v;

  double rmsold, rmsnew;

  params = _transform->parameters();
  const int M = params.rows;
  const double RMA = _reference_mask.empty() ? _reference_image.size().area() : cv::countNonZero(_reference_mask);

  /**
   * PreCompute
   * */
  if( jac.size() != M || gx.empty() || gy.empty() ) {
    jac.resize(M);
    ecc_differentiate(_reference_image, gx, gy, _reference_mask);
    _transform->create_steepest_descent_images(gx, gy, jac.data());
    ecc_compute_hessian_matrix(jac, H);
  }

  _num_iterations = 0;
  _eps = FLT_MAX;
  _failed = false;


  rmsold = FLT_MAX;

  const double lambda = _update_step_scale;

  while ( _num_iterations++ < _max_iterations ) {

    params = _transform->parameters();

    ecc_remap(_transform, params, reference_image().size(),
        _current_image, _current_mask, remapped_image, remapped_mask);

    cv::subtract(remapped_image, _reference_image, rhs);
    if ( !remapped_mask.empty() ) {
      rhs.setTo(0, ~remapped_mask);
    }

    const double CMA =
        remapped_mask.empty() ? rhs.size().area() :
            cv::countNonZero(remapped_mask);

    tbb::parallel_invoke(
        [&]() {
          rmsnew = cv::norm(rhs, cv::NORM_L2SQR) * (RMA * RMA) / (CMA * CMA);
        },
        [&]() {
          ecc_project_error_image(jac, rhs, v);
          cv::solve(H, v * (RMA / CMA), deltap, cv::DECOMP_CHOLESKY);
        });

    if ( rmsnew >= rmsold ) {
      break;
    }

    newparams = _transform->invert_and_compose(params, lambda * deltap);
    rmsold = rmsnew;
    _transform->set_parameters(newparams);

    if ( (_eps = _transform->eps(deltap, _reference_image.size())) < _max_eps ) {
      break;
    }
  }

  return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecclm_inverse_compositional::c_ecclm_inverse_compositional(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm_inverse_compositional::set_image_transform(c_image_transform * image_transform)
{
  return base::set_image_transform(image_transform);
}

bool c_ecclm_inverse_compositional::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");
  _reference_image_changed = true;
  return base::set_reference_image(reference_image, reference_mask);
}

bool c_ecclm_inverse_compositional::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  INSTRUMENT_REGION("");
  return base::set_current_image(current_image, current_mask);
}

void c_ecclm_inverse_compositional::release_current_image()
{
  _remapped_image.release();
  _inv_remapped_mask.release();
  _inv_current_mask.release();
  _rmap.release();
  _rhs.release();
  base::release_current_image();
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

void c_ecclm_inverse_compositional::ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy,
    cv::InputArray inv_mask)
{
  INSTRUMENT_REGION("");

  // 4th order derivative vector (5x1)
  // 2nd order smoothing  vector (3x1)
  static const cv::Matx<float, 5, 1> d5( +1.f/12.f, -2.f/3.f, 0.f, +2.f/3.f, -1.f/12.f );
  static const cv::Matx<float, 3, 1> s3( 0.25f, 0.5f, 0.25f );

  const cv::Mat1b m = inv_mask.getMat();

  parallel_invoke(
      [&]() {
        cv::sepFilter2D(src, gx, CV_32F, d5, s3, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !m.empty() ) {
          gx.setTo(0, m);
        }
      },
      [&]() {
        cv::sepFilter2D(src, gy, CV_32F, s3, d5, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        if ( !m.empty() ) {
          gy.setTo(0, m);
        }
      });
}

bool c_ecclm_inverse_compositional::ecc_remap(const c_image_transform * image_transform,
    const cv::Mat1f & params, const cv::Size & size,
    cv::InputArray _src, cv::InputArray _inv_src_mask,
    cv::OutputArray _dst, cv::OutputArray _inv_dst_mask)
{
  INSTRUMENT_REGION("");

  if( !image_transform->create_remap(params, size, _rmap) ) {
    CF_ERROR("image_transform->create_remap() fails");
    return false;
  }

  cv::remap(_src, _dst, _rmap, cv::noArray(), cv::INTER_LINEAR,
      cv::BORDER_REPLICATE);

  cv::remap(_inv_src_mask, _inv_dst_mask,
      _rmap, cv::noArray(),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT,
      cv::Scalar(255));

#if 0 // TODO: Check carefully if this call can be really avoided not producing artifacts
static const cv::Mat1b SE(3,3, 255);
cv::dilate(_inv_dst_mask, _inv_dst_mask, SE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
#endif

  return true;
}

double c_ecclm_inverse_compositional::compute_rhs(const cv::Mat1f & params)
{
  INSTRUMENT_REGION("");

  const int M = params.rows;
  const cv::Size size(_reference_image.size());

  ecc_remap(_transform, params, size,
      _current_image, _inv_current_mask,
      _remapped_image, _inv_remapped_mask);

  if( !_inv_reference_mask.empty() ) {
    cv::bitwise_or(_inv_reference_mask, _inv_remapped_mask,
        _inv_remapped_mask);
  }

  cv::subtract(_remapped_image, _reference_image, _rhs);
  const int bad_pixels = cv::countNonZero(_inv_remapped_mask);
  CMA = size.area() - bad_pixels;

  // Zero fill defects in the differences
  _rhs.setTo(0, _inv_remapped_mask);
  return (_last_rms = cv::norm(_rhs, cv::NORM_L2SQR) * (RMA * RMA) / (CMA * CMA));
}

void c_ecclm_inverse_compositional::compute_v(const cv::Mat1f & params, cv::Mat1f & v)
{
  INSTRUMENT_REGION("");
  ecc_project_error_image(jac, _rhs, v);
  v *= (RMA / CMA);
}

bool c_ecclm_inverse_compositional::align()
{
  INSTRUMENT_REGION("(ecclm)");

  _failed = true;

  if ( !_transform ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is null");
    return false;
  }

  if ( !_transform->invertible() ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is not invertible");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: reference_image_ is empty");
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: current_image_ is empty");
    return false;
  }

  if( !_reference_mask.empty() ) {
    cv::bitwise_not(_reference_mask, _inv_reference_mask);
  }
  else {
    _inv_reference_mask.release();
  }

  if( !_current_mask.empty() ) {
    cv::bitwise_not(_current_mask, _inv_current_mask);
  }
  else if (_inv_current_mask.size() != _current_image.size() ) {
    _inv_current_mask.create(_current_image.size());
    _inv_current_mask.setTo(0);
  }

  cv::Mat1f params, newparams, deltap;
  cv::Mat1f H, temp_d, v;

  double err, newerr;

  params = _transform->parameters();
  const int M = params.rows;

  constexpr double eps = std::numeric_limits<double>::epsilon();
  double lambda = 0.001;
  double dp = 0;
  bool recompute_remap = true;

  RMA = _reference_mask.empty() ? _reference_image.size().area() : cv::countNonZero(_reference_mask);

  /**
   * PreCompute
   * */
  if( jac.size() != M || _reference_image_changed ) {
    jac.resize(M);
    ecc_differentiate(_reference_image, gx, gy, _inv_reference_mask);
    _transform->create_steepest_descent_images(gx, gy, jac.data());
    ecc_compute_hessian_matrix(jac, Hp);
    _reference_image_changed = false;
  }

  _num_iterations = 0;
  _eps = FLT_MAX;
  _failed = false;


  while (_num_iterations < _max_iterations) {

    if( recompute_remap ) {
      compute_rhs(params);
    }

    compute_v(params, v);
    Hp.copyTo(H);
    err = _last_rms;

    /*
     * Solve normal equation for given Jacobian and lambda
     * */
    do {

      ++_num_iterations;
      recompute_remap = true;

      /*
       * Increase diagonal elements by lambda
       * */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to define delta and define new value of params */
      cv::solve(H, v, deltap, cv::DECOMP_CHOLESKY);
      newparams = _transform->invert_and_compose(params, _update_step_scale * deltap);

      /* Compute function for newparams */
      newerr = compute_rhs(newparams);

      /* Check for increments in parameters  */
      if( (dp = _transform->eps(deltap, _reference_image.size())) < _max_eps ) {
        // CF_DEBUG("BREAK by eps= %g / %g ", dp, max_eps);
        break;
      }

      /*
       * Compute update to lambda
       * */

      cv::gemm(Hp, deltap, -1, v, 2, temp_d);

      const double dS = deltap.dot(temp_d);
      const double rho = (err - newerr) / (std::abs(dS) > eps ? dS : 1);

      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-6 ) {
          lambda = std::max(1e-6, lambda / 5);
        }
      }
      else if( rho > 0.1 ) {
      }
      else if( lambda < 1 ) {       /** Try increase lambda ==> gradient descend */
        lambda = 1;
      }
      else {
        lambda *= 10;
      }

      if ( newerr < err ) {
        // CF_DEBUG("  ACCEPT");
        break;
      }

    } while (_num_iterations < _max_iterations);

    if( newerr < err ) {
      /*
       * Accept new params if were not yet accepted
       * */
      err = newerr;
      recompute_remap = false;
      _transform->set_parameters(newparams);
      params = _transform->parameters();
    }

    if( dp < _max_eps ) {
      // CF_DEBUG("BREAK2 by dp");
      break;
    }
  }

  _eps = dp;
  dp = cv::norm(deltap, cv::NORM_INF);
  return true;
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
          cv::BORDER_CONSTANT);
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
  SERIALIZE_OPTION(section, save, opts, epsx);
  // SERIALIZE_OPTION(section, save, opts, min_rho);
  SERIALIZE_OPTION(section, save, opts, reference_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, input_smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, update_step_scale);
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, interpolation);
  SERIALIZE_OPTION(section, save, opts, max_iterations);
  SERIALIZE_OPTION(section, save, opts, minimum_image_size);
  SERIALIZE_OPTION(section, save, opts, maxlevel);
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
