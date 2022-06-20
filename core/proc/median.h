/*
 * median.h
 *
 *  Created on: Nov 11, 2016
 *      Author: amyznikov
 */
#pragma once
#ifndef __optimg_median_h__
#define __optimg_median_h__

#include <opencv2/opencv.hpp>


/* compute median of image pixels
 * using the C.A.R.Hoare Quick Median
 * http://www.i-programmer.info/babbages-bag/505-quick-median.html
 * */
double median(const cv::Mat & image, cv::InputArray _mask = cv::noArray());


/*
 * https://petapixel.com/2013/05/29/a-look-at-reducing-noise-in-photographs-using-median-blending
 * */
void median_sort(std::vector<cv::Mat> & src);
void median_stack(std::vector<cv::Mat> & src, cv::Mat & dst);






/* C.A.R. Hoare, the Quick Median
 *  http://www.i-programmer.info/babbages-bag/505-quick-median.html
 */

template<class T>
void median_split_(T a[], T x, size_t & i, size_t & j)
{
  // do the left and right scan until the pointers cross
  do {

    // scan from the left
    while ( a[i] < x ) {
      ++i;
    }

    // then scan from the right
    while ( x < a[j] ) {
      --j;
    }

    // now swap values if they are in the wrong part:
    if ( i <= j ) {
      T t = a[i];
      a[i] = a[j];
      a[j] = t;
      ++i;
      --j;
    }
    // and continue the scan until the pointers cross:
  } while ( i <= j );
}


template<class T>
T median_(T a[], size_t n)
{
  size_t L = 0;
  size_t R = n - 1;
  size_t k = n / 2;

  size_t i, j;

  while ( L < R ) {

    T x = a[k];

    i = L;
    j = R;

    median_split_(a, x, i, j);

    if ( j < k ) {
      L = i;
    }

    if ( k < i ) {
      R = j;
    }
  }

  return a[k];
}

template<class T>
T median(std::vector<T> & v)
{
  return median_(&*v.begin(), v.size());
}


#endif /* __optimg_median_h__*/
