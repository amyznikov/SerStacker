/*
 * seed_fill_segmentation.cc
 *
 *  Created on: Oct 31, 2020
 *      Author: amyznikov
 *
 *
 *  C++/OpenCV implementation of Connected Component Labeling algorithm from paper by M. Emre Celebi,
 *    "A Simple and Efficient Algorithm for Connected Component Labeling in Color Images".
 *
 *    https://ui.adsabs.harvard.edu/abs/2012SPIE.8295E..1HC/abstract
 */


#include "seed_fill_segmentation.h"
#include <stack>

namespace {

struct c_segment
{
  int y, xl, xr, dy;

  c_segment(int _y, int _xl, int _xr, int _dy)
      : y(_y), xl(_xl), xr(_xr), dy(_dy)
  {
  }
};

typedef std::stack<c_segment>
  c_segment_stack;



static inline double square(double x)
{
  return x * x;
}

template<class PixType>
static double color_diff(const PixType c1, const PixType c2 )
{
  return abs(c1 - c2);
}

template<class PixType, int cn>
static double color_diff(const cv::Vec<PixType, cn> & c1, const cv::Vec<PixType, cn> & c2)
{
  double s = square((double) (c1[0]) - (double) (c2[0]));
  for ( int i = 1; i < cn; ++i ) {
    s += square((double) (c1[i]) - (double) (c2[i]));
  }
  return sqrt(s);
}


static inline bool push(c_segment_stack & stack, int y, int xl, int xr, int dy, const cv::Size & image_size)
{
  if ( y + dy >= 0 && y + dy < image_size.height ) {
    stack.emplace(y, xl, xr, dy);
    return true;
  }
  return false;
}

static inline bool pop(c_segment_stack & stack, int * y, int * xl, int * xr, int * dy)
{
  if ( stack.empty() ) {
    return false;
  }

  const c_segment & e = stack.top();

  *dy = e.dy;
  *y = e.y + e.dy;
  *xl = e.xl;
  *xr = e.xr;

  stack.pop();

  return true;
}



// Just as in paper
template<class PixType>
static void fillcolorcc(int x, int y, int label, const cv::Mat_<PixType> & P, cv::Mat1i & L, double T, c_segment_stack & stack)
{
  const PixType & seed_val = P[y][x];

  push(stack, y, x, x, 1, P.size());
  push(stack, y + 1, x, x, -1, P.size());

  while ( !stack.empty() ) {

    // Pop segment off stack and fill a neighboring scan line;
    int x1, x2, dy, start;

    pop(stack, &y, &x1, &x2, &dy);

    // Segment of scan line y − dy for x1 ≤ x ≤ x2 was previously filled, now explore adjacent pixels in scan line y;
    for ( x = x1; x >= 0 && !L[y][x] && color_diff(P[y][x], seed_val) <= T; --x ) {
      L[y][x] = label;
    }

    if ( x >= x1 ) {
      goto skip;
    }

    start = x + 1;
    if ( start < x1 ) {
      push(stack, y, start, x1 - 1, -dy, P.size());  // Leak on left?;
    }

    x = x1 + 1;

    do {

      for (; x < P.cols && !L[y][x] && color_diff(P[y][x], seed_val) <= T; ++x ) {
        L[y][x] = label;
      }

      push(stack, y, start, x - 1, dy, P.size());

      if ( x > x2 + 1 ) {
        push(stack, y, x2 + 1, x - 1, -dy, P.size());  // Leak on right?;
      }

skip:
      ++x;
      while ( x <= x2 && (L[y][x] || color_diff(P[y][x], seed_val) > T) ) {
        ++x;
      }

      start = x;

    } while ( x <= x2 );
  }
}


template<class T>
static int seedfill_segmentation_(cv::InputArray src,
    cv::Mat1i & output_labels,
    double threshold)
{
  const cv::Mat_<T> image = src.getMat();


  c_segment_stack stack;
  cv::Mat1i labels;
  int max_label = 0;

  if ( image.data == output_labels.data ) {
    labels.create(image.size());
    labels.setTo(0);
  }
  else {
    output_labels.create(image.size());
    output_labels.setTo(0);
    labels = output_labels;
  }


  for ( int y = 0; y < image.rows; ++y ) {
    for ( int x = 0; x < image.cols; ++x ) {
      if ( !labels[y][x] ) {
        fillcolorcc<T>(x, y, ++max_label, image, labels, threshold, stack);
      }
    }
  }

  if ( labels.data != output_labels.data ) {
    output_labels = std::move(labels);
  }

  return max_label;


}


}  // unnamed namespace



int seed_fill_segmentation(cv::InputArray src, cv::Mat1i & output_labels, double threshold)
{
  switch ( src.type() ) {

  case CV_8UC1 :
    return seedfill_segmentation_<uint8_t>(src, output_labels, threshold);
  case CV_8UC2 :
    return seedfill_segmentation_<cv::Vec<uint8_t, 3>>(src, output_labels, threshold);
  case CV_8UC3 :
    return seedfill_segmentation_<cv::Vec<uint8_t, 3>>(src, output_labels, threshold);
  case CV_8UC4 :
    return seedfill_segmentation_<cv::Vec<uint8_t, 4>>(src, output_labels, threshold);


  case CV_8SC1 :
    return seedfill_segmentation_<int8_t>(src, output_labels, threshold);
  case CV_8SC2 :
    return seedfill_segmentation_<cv::Vec<int8_t, 2>>(src, output_labels, threshold);
  case CV_8SC3 :
    return seedfill_segmentation_<cv::Vec<int8_t, 3>>(src, output_labels, threshold);
  case CV_8SC4 :
    return seedfill_segmentation_<cv::Vec<int8_t, 4>>(src, output_labels, threshold);


  case  CV_16UC1:
    return seedfill_segmentation_<uint16_t>(src, output_labels, threshold);
  case  CV_16UC2:
    return seedfill_segmentation_<cv::Vec<uint16_t, 2>>(src, output_labels, threshold);
  case  CV_16UC3:
    return seedfill_segmentation_<cv::Vec<uint16_t, 3>>(src, output_labels, threshold);
  case  CV_16UC4:
    return seedfill_segmentation_<cv::Vec<uint16_t, 4>>(src, output_labels, threshold);


  case  CV_16SC1:
    return seedfill_segmentation_<int16_t>(src, output_labels, threshold);
  case  CV_16SC2:
    return seedfill_segmentation_<cv::Vec<int16_t, 2>>(src, output_labels, threshold);
  case  CV_16SC3:
    return seedfill_segmentation_<cv::Vec<int16_t, 3>>(src, output_labels, threshold);
  case  CV_16SC4:
    return seedfill_segmentation_<cv::Vec<int16_t, 4>>(src, output_labels, threshold);


  case  CV_32SC1:
    return seedfill_segmentation_<int32_t>(src, output_labels, threshold);
  case  CV_32SC2:
    return seedfill_segmentation_<cv::Vec<int32_t, 2>>(src, output_labels, threshold);
  case  CV_32SC3:
    return seedfill_segmentation_<cv::Vec<int32_t, 3>>(src, output_labels, threshold);
  case  CV_32SC4:
    return seedfill_segmentation_<cv::Vec<int32_t, 4>>(src, output_labels, threshold);


  case  CV_32FC1:
    return seedfill_segmentation_<float>(src, output_labels, threshold);
  case  CV_32FC2:
    return seedfill_segmentation_<cv::Vec<float, 2>>(src, output_labels, threshold);
  case  CV_32FC3:
    return seedfill_segmentation_<cv::Vec<float, 3>>(src, output_labels, threshold);
  case  CV_32FC4:
    return seedfill_segmentation_<cv::Vec<float, 4>>(src, output_labels, threshold);


  case  CV_64FC1:
    return seedfill_segmentation_<double>(src, output_labels, threshold);
  case  CV_64FC2:
    return seedfill_segmentation_<cv::Vec<double, 2>>(src, output_labels, threshold);
  case  CV_64FC3:
    return seedfill_segmentation_<cv::Vec<double, 3>>(src, output_labels, threshold);
  case  CV_64FC4:
    return seedfill_segmentation_<cv::Vec<double, 4>>(src, output_labels, threshold);
  }


  return 0;
}
