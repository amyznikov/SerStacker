/*
 * histogram.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */
#include "histogram.h"
#include <core/debug.h>


void c_histogram_builder::set_channels(int cn)
{
  channels_ = cn;
}

int c_histogram_builder::channels() const
{
  return channels_;
}

void c_histogram_builder::set_input_range(double minval, double maxval)
{
  minval_ = minval;
  maxval_ = maxval;
}

void c_histogram_builder::get_input_range(double *minval, double *maxval) const
{
  *minval = minval_;
  *maxval = maxval_;
}

void c_histogram_builder::set_bins(int nbins)
{
  bins_ = nbins;
}

int c_histogram_builder::bins() const
{
  return bins_;
}

void c_histogram_builder::set_minval(double v)
{
  minval_ = v;
}

double c_histogram_builder::minval() const
{
  return minval_;
}

void c_histogram_builder::set_maxval(double v)
{
  maxval_ = v;
}

double c_histogram_builder::maxval() const
{
  return maxval_;
}

void c_histogram_builder::set_cumulative(bool v)
{
  cumulative_ = v;
}

bool c_histogram_builder::cumulative() const
{
  return cumulative_;
}

void c_histogram_builder::set_scaled(bool v)
{
  scaled_ = v;
}

bool c_histogram_builder::scaled() const
{
  return scaled_;
}

void c_histogram_builder::set_logscale(bool v)
{
  logscale_ = v;
}

bool c_histogram_builder::logscale() const
{
  return logscale_;
}

void c_histogram_builder::reset()
{
  H_.release();
}

void c_histogram_builder::initialize()
{
  if ( bins_ > 10000 ) {
    bins_ = 10000;
  }

  if ( channels_ < 1 ) {
    channels_ = 1;
  }

  if ( maxval_ <= minval_ ) {
    minval_ = 0;
    maxval_ = 1.0;
  }

  scale_ = bins_ / (maxval_ - minval_);

  H_.create(bins_, channels_);
  H_.setTo(0);

}

void c_histogram_builder::add_pixel(const cv::Scalar & s)
{
  if( H_.empty() ) {
    initialize();
  }

  for( int c = 0; c < channels_; ++c ) {
    const int b =
        std::max(0, std::min(bins_ - 1,
            (int) ((s[c] - minval_) * scale_)));

    H_[b][c] += 1;
  }

}

void c_histogram_builder::compute(cv::OutputArray outputH)
{
  cv::Mat1f H;
  H_.copyTo(H);

  if( !H.empty() ) {

    if( scaled_ ) {
      cv::Mat1f sums;
      cv::reduce(H, sums, 0, cv::REDUCE_SUM, sums.depth());
      for( int y = 0; y < H.rows; ++y ) {
        for( int x = 0; x < H.cols; ++x ) {
          H[y][x] /= sums[0][x];
        }
      }
    }

    if( cumulative_ ) {
      accumulate_histogram(H, H);
    }

    if( logscale_ ) {
      double min = 0, max = 1;

      cv::log(H + 1, H);

      cv::minMaxLoc(H, &min, &max);
      if( max > 0 ) {
        cv::multiply(H, 1 / max, H);
      }
    }
  }

  if( outputH.fixedType() && outputH.depth() != H.depth() ) {
    H.convertTo(outputH, outputH.depth());
  }
  else {
    outputH.move(H);
  }
}

template<class T, int cn>
static bool update_histogram__(cv::InputArray _src, c_histogram_builder & builder,
    cv::InputArray _mask)
{
  const double minval =
      builder.minval();

  const double maxval =
      builder.maxval();

  const int bn =
      builder.bins();

  cv::Scalar s;

  const cv::Mat_<cv::Vec<T, cn>> src =
      _src.getMat();

  const cv::Vec<double, cn> mv =
      cv::Vec<float, cn>::all(minval);

  const double scale =
      bn / (maxval - minval);

  if( _mask.empty() ) {

    for( int y = 0; y < src.rows; ++y ) {
      for( int x = 0; x < src.cols; ++x ) {
        for( int c = 0; c < cn; ++c ) {
          s[c] = src[y][x][c];
        }
        builder.add_pixel(s);
      }
    }
  }
  else {

    const cv::Mat1b mask = _mask.getMat();

    for( int y = 0; y < src.rows; ++y ) {
      for( int x = 0; x < src.cols; ++x ) {
        if( mask[y][x] ) {
          for( int c = 0; c < cn; ++c ) {
            s[c] = src[y][x][c];
          }
          builder.add_pixel(s);
        }
      }
    }
  }

  return true;
}


static bool update_histogram_(cv::InputArray src, c_histogram_builder & builder, cv::InputArray mask)
{
  switch ( src.type() ) {

  case CV_8UC1 : return update_histogram__<uint8_t, 1>(src, builder, mask);
  case CV_8UC2 : return update_histogram__<uint8_t, 2>(src, builder, mask);
  case CV_8UC3 : return update_histogram__<uint8_t, 3>(src, builder, mask);
  case CV_8UC4 : return update_histogram__<uint8_t, 4>(src, builder, mask);

  case CV_8SC1 : return update_histogram__<int8_t, 1>(src, builder, mask);
  case CV_8SC2 : return update_histogram__<int8_t, 2>(src, builder, mask);
  case CV_8SC3 : return update_histogram__<int8_t, 3>(src, builder, mask);
  case CV_8SC4 : return update_histogram__<int8_t, 4>(src, builder, mask);

  case CV_16UC1 : return update_histogram__<uint16_t, 1>(src, builder, mask);
  case CV_16UC2 : return update_histogram__<uint16_t, 2>(src, builder, mask);
  case CV_16UC3 : return update_histogram__<uint16_t, 3>(src, builder, mask);
  case CV_16UC4 : return update_histogram__<uint16_t, 4>(src, builder, mask);

  case CV_16SC1 : return update_histogram__<int16_t, 1>(src, builder, mask);
  case CV_16SC2 : return update_histogram__<int16_t, 2>(src, builder, mask);
  case CV_16SC3 : return update_histogram__<int16_t, 3>(src, builder, mask);
  case CV_16SC4 : return update_histogram__<int16_t, 4>(src, builder, mask);

  case CV_32SC1 : return update_histogram__<int32_t, 1>(src, builder, mask);
  case CV_32SC2 : return update_histogram__<int32_t, 2>(src, builder, mask);
  case CV_32SC3 : return update_histogram__<int32_t, 3>(src, builder, mask);
  case CV_32SC4 : return update_histogram__<int32_t, 4>(src, builder, mask);

  case CV_32FC1 : return update_histogram__<float, 1>(src, builder, mask);
  case CV_32FC2 : return update_histogram__<float, 2>(src, builder, mask);
  case CV_32FC3 : return update_histogram__<float, 3>(src, builder, mask);
  case CV_32FC4 : return update_histogram__<float, 4>(src, builder, mask);

  case CV_64FC1 : return update_histogram__<double, 1>(src, builder, mask);
  case CV_64FC2 : return update_histogram__<double, 2>(src, builder, mask);
  case CV_64FC3 : return update_histogram__<double, 3>(src, builder, mask);
  case CV_64FC4 : return update_histogram__<double, 4>(src, builder, mask);

  default :
    CF_FATAL("Invalid argument: Unsupported src image type : %d", src.type());
    break;
  }

  return false;
}


template<class T, int cn>
static bool create_histogram__(cv::InputArray _src, cv::Mat1f & H,
    double minval, double maxval, int bn, cv::InputArray _mask)
{
  c_histogram_builder builder;

  builder.set_channels(cn);
  builder.set_bins(bn);
  builder.set_input_range(minval, maxval);

  update_histogram__<T, cn>(_src, builder, _mask);

//
//  const cv::Mat_<cv::Vec<T, cn>> src =
//      _src.getMat();
//
//  const cv::Vec<double, cn> mv =
//      cv::Vec<float, cn>::all(minval);
//
//  const double scale =
//      bn / (maxval - minval);
//
//  //  H.create(bn, cn);
//  //  H.setTo(0);
//
//  if( _mask.empty() ) {
//
//    for( int y = 0; y < src.rows; ++y ) {
//      for( int x = 0; x < src.cols; ++x ) {
//        for( int c = 0; c < cn; ++c ) {
//          s[c] = src[y][x][c];
//        }
//        builder.add_pixel(s);
//      }
//    }
//  }
//  else {
//
//    const cv::Mat1b mask = _mask.getMat();
//
//    for( int y = 0; y < src.rows; ++y ) {
//      for( int x = 0; x < src.cols; ++x ) {
//        if( mask[y][x] ) {
//          for( int c = 0; c < cn; ++c ) {
//            s[c] = src[y][x][c];
//          }
//          builder.add_pixel(s);
//        }
//      }
//    }
//  }

  builder.compute(H);

  return true;
}

static bool create_histogram_(cv::InputArray src, cv::Mat1f & dst,
    double minval, double maxval, int bn, cv::InputArray mask)
{
  switch ( src.type() ) {

  case CV_8UC1 : return create_histogram__<uint8_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_8UC2 : return create_histogram__<uint8_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_8UC3 : return create_histogram__<uint8_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_8UC4 : return create_histogram__<uint8_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_8SC1 : return create_histogram__<int8_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_8SC2 : return create_histogram__<int8_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_8SC3 : return create_histogram__<int8_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_8SC4 : return create_histogram__<int8_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_16UC1 : return create_histogram__<uint16_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_16UC2 : return create_histogram__<uint16_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_16UC3 : return create_histogram__<uint16_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_16UC4 : return create_histogram__<uint16_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_16SC1 : return create_histogram__<int16_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_16SC2 : return create_histogram__<int16_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_16SC3 : return create_histogram__<int16_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_16SC4 : return create_histogram__<int16_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_32SC1 : return create_histogram__<int32_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_32SC2 : return create_histogram__<int32_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_32SC3 : return create_histogram__<int32_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_32SC4 : return create_histogram__<int32_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_32FC1 : return create_histogram__<float, 1>(src, dst, minval, maxval, bn, mask);
  case CV_32FC2 : return create_histogram__<float, 2>(src, dst, minval, maxval, bn, mask);
  case CV_32FC3 : return create_histogram__<float, 3>(src, dst, minval, maxval, bn, mask);
  case CV_32FC4 : return create_histogram__<float, 4>(src, dst, minval, maxval, bn, mask);

  case CV_64FC1 : return create_histogram__<double, 1>(src, dst, minval, maxval, bn, mask);
  case CV_64FC2 : return create_histogram__<double, 2>(src, dst, minval, maxval, bn, mask);
  case CV_64FC3 : return create_histogram__<double, 3>(src, dst, minval, maxval, bn, mask);
  case CV_64FC4 : return create_histogram__<double, 4>(src, dst, minval, maxval, bn, mask);

  default :
    CF_FATAL("Invalid argument: Unsupported src image type : %d", src.type());
    break;
  }

  return false;
}



/// @brief  convert conventional image histogram H into cumulative
///         by accumulating the bin values along rows
bool accumulate_histogram(cv::InputArray H, cv::OutputArray cumulative)
{
  cv::Mat1f src;
  cv::Mat1f dst;

  double sums[H.cols()];

  if ( H.channels() != 1 ) {
    CF_FATAL("Invalid argument: Input histogram must be single-channel");
    return false;
  }

  if ( cumulative.fixedType() && cumulative.channels() != 1 ) {
    CF_FATAL("Invalid argument: output cumulative histogram must be single-channel");
    return false;
  }

  if ( H.depth() == src.depth() ) {
    src = H.getMat();
  }
  else {
    H.getMat().convertTo(src, src.depth());
  }


  memset(sums, 0, sizeof(sums));

  dst.create(src.size());

  for ( int y = 0; y < src.rows; ++y ) {
    for ( int x = 0; x < src.cols; ++x ) {
      dst[y][x] = (sums[x] += src[y][x]);
    }
  }

  if ( !cumulative.fixedType() || cumulative.depth() == dst.depth() ) {
    cumulative.move(dst);
  }
  else {
    dst.convertTo(cumulative, cumulative.depth());
  }

  return true;
}

void scale_histogram(cv::Mat1f & H)
{
  cv::Mat1f sums;
  cv::reduce(H, sums, 0, cv::REDUCE_SUM, sums.depth());
  for( int y = 0; y < H.rows; ++y ) {
    for( int x = 0; x < H.cols; ++x ) {
      H[y][x] /= sums[0][x];
    }
  }
}

/// @brief  scale conventional image histogram H by it's total sum
void scale_histogram(const cv::Mat1f & Hsrc, cv::Mat1f & Hdst)
{
  if( &Hsrc == &Hdst ) {
    scale_histogram (Hdst);
  }
  else {

    cv::Mat1f H;
    cv::Mat1f sums;

    cv::reduce(Hsrc, sums, 0, cv::REDUCE_SUM, sums.depth());

    H.create(Hsrc.size());

    for( int y = 0; y < H.rows; ++y ) {
      for( int x = 0; x < H.cols; ++x ) {
        H[y][x] = Hsrc[y][x] / sums[0][x];
      }
    }

    Hdst = std::move(H);
  }
}

bool create_histogram(cv::InputArrayOfArrays images, cv::InputArrayOfArrays masks,
    cv::OutputArray dst,
    double * minval,
    double * maxval,
    int nbins,
    bool cumulative,
    bool scaled)
{
  CF_DEBUG("Enter");

  static const auto is_vector_kind =
      [](cv::InputArrayOfArrays a) -> bool {
        switch (a.kind()) {
          case cv::_InputArray::STD_VECTOR_VECTOR:
          case cv::_InputArray::STD_VECTOR_MAT:
          case cv::_InputArray::STD_VECTOR_UMAT:
          case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
          case cv::_InputArray::STD_ARRAY_MAT:
          return true;
        }
        return false;
      };

  static const auto get_items_count =
      [](cv::InputArrayOfArrays a) -> int {
        return a.empty() ? 0 : is_vector_kind(a) ? a.total() : 1;
      };

  static const auto get_item_depth =
      [](cv::InputArrayOfArrays a, int index) -> int {
        return a.empty() ? -1 : is_vector_kind(a) ? a.depth(index) : a.depth(-1);
      };

  static const auto get_item_channels =
      [](cv::InputArrayOfArrays a, int index) -> int {
        return a.empty() ? 0 : is_vector_kind(a) ? a.channels(index) : a.channels(-1);
      };

  static const auto get_item =
      [](cv::InputArrayOfArrays a, int index) -> cv::Mat {
        return a.empty() ? cv::Mat() : is_vector_kind(a) ? a.getMat(index) : a.getMat(-1);
      };

  const int num_images =
      get_items_count(images);

  if( num_images < 1 ) {
    dst.release();
    return false;
  }

  int max_depth = get_item_depth(images, 0);
  int max_cn = get_item_channels(images, 0);
  for( int i = 1; i < num_images; ++i ) {
    int depth = get_item_depth(images, i);
    if( depth > max_depth ) {
      max_depth = depth;
    }
    int cn = get_item_channels(images, i);
    if( cn > max_cn ) {
      max_cn = cn;
    }
  }

  switch (max_depth) {
    //
    case CV_8U:
      if( *minval >= *maxval ) {
        *minval = 0;
        *maxval = UINT8_MAX;
      }
      if( *minval < 0 ) {
        *minval = 0;
      }
      if( *maxval > UINT8_MAX ) {
        *maxval = UINT8_MAX;
      }
      if( nbins < 1 ) {
        nbins = *maxval - *minval + 1;
      }
      break;

    case CV_8S:
      if( *minval >= *maxval ) {
        *minval = INT8_MIN;
        *maxval = INT8_MAX;
      }
      if( *minval < INT8_MIN ) {
        *minval = INT8_MIN;
      }
      if( *maxval > INT8_MAX ) {
        *maxval = INT8_MAX;
      }
      if( nbins < 1 ) {
        nbins = *maxval - *minval + 1;
      }
      break;

      //
    case CV_16U:
      if( *minval >= *maxval ) {
        *minval = 0;
        *maxval = UINT16_MAX;
      }
      if( *minval < 0 ) {
        *minval = 0;
      }
      if( *maxval > UINT16_MAX ) {
        *maxval = UINT16_MAX;
      }
      if( nbins < 1 ) {
        nbins = *maxval - *minval + 1;
      }
      break;

    case CV_16S:
      if( *minval >= *maxval ) {
        *minval = INT16_MIN;
        *maxval = INT16_MAX;
      }
      if( *minval < INT16_MIN ) {
        *minval = INT16_MIN;
      }
      if( *maxval > INT16_MAX ) {
        *maxval = INT16_MAX;
      }
      if( nbins < 1 ) {
        nbins = *maxval - *minval + 1;
      }
      break;

      //
    case CV_32S:
      if( *minval >= *maxval ) {
        *minval = INT32_MIN;
        *maxval = INT32_MAX;
      }
      if( *minval < INT32_MIN ) {
        *minval = INT32_MIN;
      }
      if( *maxval > INT32_MAX ) {
        *maxval = INT32_MAX;
      }
      if( nbins < 1 ) {
        nbins = *maxval - *minval + 1;
      }
      break;

      //
    case CV_32F:
      case CV_64F:
      if( *minval >= *maxval ) {

        cv::minMaxLoc(get_item(images, 0), minval, maxval);
        for( int i = 0; i < num_images; ++i ) {

          double minv = *minval;
          double maxv = *maxval;

          cv::minMaxLoc(get_item(images, i), &minv, &maxv);
          if( minv < *minval ) {
            *minval = minv;
          }
          if( maxv < *maxval ) {
            *maxval = maxv;
          }
        }
      }
      if( nbins < 1 ) {
        nbins = (int) std::min(10000., ((*maxval - *minval) / FLT_MIN) + 1);
      }
      break;

      //
    default:
      CF_FATAL("Invalid argument: Unsupported image depth %d encountered", max_depth);
      return false;
  }

  if ( nbins > 10000 ) {
    nbins = 10000;
  }

  CF_DEBUG("Compute");

  cv::Mat1f H;
  c_histogram_builder builder;

  builder.set_channels(max_cn);
  builder.set_bins(nbins);
  builder.set_input_range(*minval, *maxval);

  for ( int i = 0; i < num_images; ++i ) {
    update_histogram_(get_item(images, i), builder, get_item(masks, i));
  }

  builder.compute(H);

  if ( scaled ) {
    scale_histogram(H);
  }

  if ( cumulative ) {
    accumulate_histogram(H, H);
  }

  if ( dst.fixedType() && dst.depth() != H.depth() ) {
    H.convertTo(dst, dst.depth());
  }
  else {
    dst.move(H);
  }

  CF_DEBUG("leave");
  return true;
}


//
//template<class T, int cn>
//static bool create_histogram__(cv::InputArray _src, cv::Mat1f & H,
//    double minval, double maxval, int bn, cv::InputArray _mask)
//{
//  // FIXME: implement with TBB
//
//  const cv::Mat_<cv::Vec<T, cn>> src =
//      _src.getMat();
//
//  const cv::Vec<double, cn> mv =
//      cv::Vec<float, cn>::all(minval);
//
//  const double scale =
//      bn / (maxval - minval);
//
//  H.create(bn, cn);
//  H.setTo(0);
//
//  if ( _mask.empty() ) {
//
//    for ( int y = 0; y < src.rows; ++y ) {
//      for ( int x = 0; x < src.cols; ++x ) {
//
//        for( int c = 0; c < cn; ++c ) {
//
//          const int b = std::max(0,  std::min(bn-1,
//                  (int)( (src[y][x][c] - mv[c]) * scale )));
//
//          H[b][c] += 1;
//        }
//
//      }
//    }
//  }
//  else {
//
//    const cv::Mat1b mask = _mask.getMat();
//
//    for ( int y = 0; y < src.rows; ++y ) {
//      for ( int x = 0; x < src.cols; ++x ) {
//        if ( mask[y][x] ) {
//
//          for( int c = 0; c < cn; ++c ) {
//
//            const int b = std::max(0,  std::min(bn-1,
//                    (int)( (src[y][x][c] - mv[c]) * scale )));
//
//            H[b][c] += 1;
//          }
//
//
//        }
//      }
//    }
//  }
//
//  return true;
//}


