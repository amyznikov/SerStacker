/*
 * geo-reconstruction.cc
 *
 *  Created on: Jul 22, 2019
 *      Author: amyznikov
 */

#include "geo-reconstruction.h"
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
# include <atomic>
#endif

// prevent conflicts with msvs min() and max() macroses
#ifdef _MSC_VER
# undef min
# undef max
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
static bool equal(const cv::Mat_<T> & src1, const cv::Mat_<T> & src2)
{

#if HAVE_TBB
  // tbb::atomic<bool> difference_found = false;
  std::atomic<bool> difference_found(false);

  tbb::parallel_for(0,  src1.rows, [&src1, &src2, &difference_found] (int y) {
#else
  bool difference_found = false;

  for ( int y = 0; y < src1.rows; ++y ) {
#endif
    if ( !difference_found ) {
      const T * s1 = src1[y];
      const T * s2 = src2[y];
      for ( int x = 0, n = src1.cols * src1.channels(); x < n; ++x ) {
        if ( s1[x] != s2[x] ) {
          difference_found = true;
          break;
        }
      }
    }
#if !HAVE_TBB
    else {
      break;
    }
#endif
  }
#if HAVE_TBB
  );
#endif

  return !difference_found;
}

static bool equal(const cv::Mat & src1, const cv::Mat & src2)
{
  switch ( src1.depth() ) {
  case CV_8U :
    return equal((cv::Mat_<uint8_t> &) src1, (cv::Mat_<uint8_t> &) src2);
  case CV_8S :
    return equal((cv::Mat_<int8_t> &) src1, (cv::Mat_<int8_t> &) src2);
  case CV_16U :
    return equal((cv::Mat_<uint16_t> &) src1, (cv::Mat_<uint16_t> &) src2);
  case CV_16S :
    return equal((cv::Mat_<int16_t> &) src1, (cv::Mat_<int16_t> &) src2);
  case CV_32S :
    return equal((cv::Mat_<int32_t> &) src1, (cv::Mat_<int32_t> &) src2);
  case CV_32F :
    return equal((cv::Mat_<float> &) src1, (cv::Mat_<float> &) src2);
  case CV_64F :
    return equal((cv::Mat_<double> &) src1, (cv::Mat_<double> &) src2);
  default :
    break;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int geo_dilate(cv::InputArray _marker_image, cv::InputArray _reference_image, cv::Mat &  dst,
    cv::InputArray SE, const cv::Point & anchor, int max_iterations, int borderType, const cv::Scalar & borderValue)
{
  cv::Mat marker_image, reference_image, dilated_image;

  int iterations = 0;

  cv::min(_marker_image.getMat(), (reference_image = _reference_image.getMat()), marker_image);

  if ( max_iterations < 0 ) {
    max_iterations = INT32_MAX;
  }

  while ( ++iterations < max_iterations ) {

    cv::dilate(marker_image, dilated_image, SE, anchor, 1, borderType, borderValue);
    cv::min(dilated_image, reference_image, dilated_image);

    if ( equal(marker_image, dilated_image) ) {
      break;
    }

    // swap buffers
    cv::Mat tmp = marker_image;
    marker_image = dilated_image;
    dilated_image = tmp;
  }

  dst = marker_image;

  return iterations;
}



int geo_dilate(cv::InputArray marker_image, cv::InputArray reference_image, cv::Mat & dst,
    int connectivity, int max_iterations, int borderType, const cv::Scalar& borderValue)
{
  static thread_local int previous_connectivity = 0;
  static thread_local cv::Mat1b SE;

  if ( connectivity != previous_connectivity || SE.empty() ) {
    switch ( (previous_connectivity = connectivity) ) {
    case 4 :
      SE = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
      break;
    case 8 :
      default :
      SE = cv::Mat1b(3, 3, 255);
      break;
    }
  }

  return geo_dilate(marker_image, reference_image, dst, SE, cv::Point(-1, -1),
      max_iterations, borderType, borderValue);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int geo_erode(cv::InputArray _marker_image, cv::InputArray _reference_image, cv::Mat & dst,
    cv::InputArray SE, const cv::Point & anchor, int max_iterations,
    int borderType, const cv::Scalar& borderValue)
{
  cv::Mat marker_image, reference_image, eroded_image;

  int iterations = 0;

  reference_image = _reference_image.getMat();
  _marker_image.getMat().copyTo(marker_image);

  if ( max_iterations < 0 ) {
    max_iterations = INT32_MAX;
  }

  while ( ++iterations < max_iterations ) {

    cv::erode(marker_image, eroded_image, SE, anchor, 1, borderType, borderValue);
    cv::max(eroded_image, reference_image, eroded_image);

    if ( equal(marker_image, eroded_image) ) {
      break;
    }

    // swap buffers
    cv::Mat tmp = marker_image;
    marker_image = eroded_image;
    eroded_image = tmp;
  }

  dst = marker_image;

  return iterations;
}

int geo_erode(cv::InputArray marker_image, cv::InputArray reference_image, cv::Mat & dst,
    int connectivity, int max_iterations, int borderType, const cv::Scalar& borderValue)
{
  static thread_local int previous_connectivity = 0;
  static thread_local cv::Mat1b SE;

  if ( connectivity != previous_connectivity || SE.empty() ) {
    switch ( (previous_connectivity = connectivity) ) {
    case 4 :
      SE = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
      break;
    case 8 :
      default :
      SE = cv::Mat1b(3, 3, 255);
      break;
    }
  }

  return geo_erode(marker_image, reference_image, dst, SE, cv::Point(-1, -1),
      max_iterations, borderType, borderValue);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Geodesic reconstruction stuff


/**
 * Update result image using pixels in the upper left neighborhood
 */
template<class T>
static void forward_dilate(const cv::Mat_<T> & mask, cv::Mat_<T> & res, int connectivity)
{
  switch ( connectivity ) {

  case 1 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

//        if ( x > 0 && res[y][x - 1] > maxValue ) {
//          maxValue = res[y][x - 1];
//        }

        if ( y > 0 && res[y - 1][x] > maxValue ) {
          maxValue = res[y - 1][x];
        }

        // update value of current pixel
        if ( (maxValue = std::min(maxValue, mask[y][x])) > currentValue ) {
          res[y][x] = maxValue;
        }
      }
    }  // );
    break;

  case 2 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( x > 0 && res[y][x - 1] > maxValue ) {
          maxValue = res[y][x - 1];
        }

//        if ( y > 0 && res[y - 1][x] > maxValue ) {
//          maxValue = res[y - 1][x];
//        }

        // update value of current pixel
        if ( (maxValue = std::min(maxValue, mask[y][x])) > currentValue ) {
          res[y][x] = maxValue;
        }
      }
    }  // );
    break;

  case 4 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( x > 0 && res[y][x - 1] > maxValue ) {
          maxValue = res[y][x - 1];
        }

        if ( y > 0 && res[y - 1][x] > maxValue ) {
          maxValue = res[y - 1][x];
        }

        // update value of current pixel
        if ( (maxValue = std::min(maxValue, mask[y][x])) > currentValue ) {
          res[y][x] = maxValue;
        }
      }
    }  // );
    break;

  case 8 :
    // Process all lines
//      tbb::parallel_for(0, res.rows,
//          [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {

      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( y > 0 ) {
          // process the 3 values on the line above current pixel
          if ( x > 0 && res[y - 1][x - 1] > maxValue ) {
            maxValue = res[y - 1][x - 1];
          }

          if ( res[y - 1][x] > maxValue ) {
            maxValue = res[y - 1][x];
          }

          if ( x < res.cols - 1 && res[y - 1][x + 1] > maxValue ) {
            maxValue = res[y - 1][x + 1];
          }
        }

        if ( x > 0 && res[y][x - 1] > maxValue ) {
          maxValue = res[y][x - 1];
        }

        // update value of current pixel
        if ( (maxValue = std::min(maxValue, mask[y][x])) > currentValue ) {
          res[y][x] = maxValue;
        }
      }
    } // );
    break;
  }
}



/**
 * Update result image using pixels in the upper left neighborhood
 */
template<class T>
static void forward_erode(const cv::Mat_<T> & mask, cv::Mat_<T> & res, int connectivity)
{
  switch ( connectivity ) {

  case 1 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

//        if ( x > 0 && res[y][x - 1] < minValue ) {
//          minValue = res[y][x - 1];
//        }

        if ( y > 0 && res[y - 1][x] < minValue ) {
          minValue = res[y - 1][x];
        }

        // update value of current pixel
        if ( (minValue = std::max(minValue, mask[y][x])) < currentValue ) {
          res[y][x] = minValue;
        }
      }
    }  // );
    break;

  case 2 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( x > 0 && res[y][x - 1] < minValue ) {
          minValue = res[y][x - 1];
        }

//        if ( y > 0 && res[y - 1][x] < minValue ) {
//          minValue = res[y - 1][x];
//        }

        // update value of current pixel
        if ( (minValue = std::max(minValue, mask[y][x])) < currentValue ) {
          res[y][x] = minValue;
        }
      }
    }  // );
    break;

  case 4 :
    // Process all lines
    //  tbb::parallel_for(0, res.rows,
    //      [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {
      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( x > 0 && res[y][x - 1] < minValue ) {
          minValue = res[y][x - 1];
        }

        if ( y > 0 && res[y - 1][x] < minValue ) {
          minValue = res[y - 1][x];
        }

        // update value of current pixel
        if ( (minValue = std::max(minValue, mask[y][x])) < currentValue ) {
          res[y][x] = minValue;
        }
      }
    }  // );
    break;

  case 8 :
    // Process all lines
//      tbb::parallel_for(0, res.rows,
//          [&](int y) {
    for ( int y = 0; y < res.rows; ++y ) {

      // Process pixels in the middle of the line
      for ( int x = 0; x < res.cols; ++x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( y > 0 ) {
          // process the 3 values on the line above current pixel
          if ( x > 0 && res[y - 1][x - 1] < minValue ) {
            minValue = res[y - 1][x - 1];
          }

          if ( res[y - 1][x] < minValue ) {
            minValue = res[y - 1][x];
          }

          if ( x < res.cols - 1 && res[y - 1][x + 1] < minValue ) {
            minValue = res[y - 1][x + 1];
          }
        }

        if ( x > 0 && res[y][x - 1] < minValue ) {
          minValue = res[y][x - 1];
        }

        // update value of current pixel
        if ( (minValue = std::max(minValue, mask[y][x])) < currentValue ) {
          res[y][x] = minValue;
        }
      }
    } // );
    break;
  }
}





/**
 * Adds the current position to the queue if and only if the value is greater than the value of the mask.
 */
template<class T>
static void update_queue_dilate(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, const cv::Mat_<T> & res,
    int x, int y, T value)
{
  // update current value only if value is strictly greater
  if ( res[y][x] < std::min(value, mask[y][x]) ) {
    queue.emplace_back(x, y);
  }
}

/**
 * Adds the current position to the queue if and only if the value is less than the value of the mask.
 */
template<class T>
static void update_queue_erode(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, const cv::Mat_<T> & res,
    int x, int y, T value)
{
  // update current value only if value is strictly less
  if ( res[y][x] > std::max(value, mask[y][x]) ) {
    queue.emplace_back(x, y);
  }
}



/**
 * Update result image using pixels in the lower-right neighborhood,
 * using the 4-adjacency.
 */
template<class T>
static void backward_dilate(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, cv::Mat_<T> & res, int connectivity)
{
  switch ( connectivity ) {
  case 1 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

//        if ( x < res.cols - 1 && res[y][x + 1] > maxValue ) {
//          maxValue = res[y][x + 1];
//        }

        if ( y < res.rows - 1 && res[y + 1][x] > maxValue ) {
          maxValue = res[y + 1][x];
        }

        // combine with mask
        // check if update is required
        if ( (maxValue = std::min(maxValue, mask[y][x])) <= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = maxValue;

        // eventually add lower-right neighbors to queue
//        if ( x < res.cols - 1 ) {
//          update_queue_dilate(queue, mask, res, x + 1, y, maxValue);
//        }

        if ( y < res.rows - 1 ) {
          update_queue_dilate(queue, mask, res, x, y + 1, maxValue);
        }
      }
    }
    break;

  case 2 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( x < res.cols - 1 && res[y][x + 1] > maxValue ) {
          maxValue = res[y][x + 1];
        }

//        if ( y < res.rows - 1 && res[y + 1][x] > maxValue ) {
//          maxValue = res[y + 1][x];
//        }

        // combine with mask
        // check if update is required
        if ( (maxValue = std::min(maxValue, mask[y][x])) <= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = maxValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_dilate(queue, mask, res, x + 1, y, maxValue);
        }

//        if ( y < res.rows - 1 ) {
//          update_queue_dilate(queue, mask, res, x, y + 1, maxValue);
//        }
      }
    }
    break;

  case 4 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( x < res.cols - 1 && res[y][x + 1] > maxValue ) {
          maxValue = res[y][x + 1];
        }

        if ( y < res.rows - 1 && res[y + 1][x] > maxValue ) {
          maxValue = res[y + 1][x];
        }

        // combine with mask
        // check if update is required
        if ( (maxValue = std::min(maxValue, mask[y][x])) <= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = maxValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_dilate(queue, mask, res, x + 1, y, maxValue);
        }

        if ( y < res.rows - 1 ) {
          update_queue_dilate(queue, mask, res, x, y + 1, maxValue);
        }
      }
    }
    break;

  case 8 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();
        const T currentValue = res[y][x];
        T maxValue = currentValue;

        if ( y < res.rows - 1 ) {
          // process the 3 values on the line below current pixel
          if ( x > 0 && res[y + 1][x - 1] > maxValue ) {
            maxValue = res[y + 1][x - 1];
          }

          if ( res[y + 1][x] > maxValue ) {
            maxValue = res[y + 1][x];
          }

          if ( x < res.cols - 1 && res[y + 1][x + 1] > maxValue ) {
            maxValue = res[y + 1][x + 1];
          }
        }

        if ( x < res.cols - 1 && res[y][x + 1] > maxValue ) {
          maxValue = res[y][x + 1];
        }

        // combine with mask
        // check if update is required
        if ( (maxValue = std::min(maxValue, mask[y][x])) <= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = maxValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_dilate(queue, mask, res, x + 1, y, maxValue);
        }

        if ( y < res.rows - 1 ) {
          if ( x > 0 ) {
            update_queue_dilate(queue, mask, res, x - 1, y + 1, maxValue);
          }
          update_queue_dilate(queue, mask, res, x, y + 1, maxValue);
          if ( x < res.cols - 1 ) {
            update_queue_dilate(queue, mask, res, x + 1, y + 1, maxValue);
          }
        }
      }
    }
    break;
  }

}




/**
 * Update result image using pixels in the lower-right neighborhood,
 * using the 4-adjacency.
 */
template<class T>
static void backward_erode(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, cv::Mat_<T> & res, int connectivity)
{
  switch ( connectivity ) {

  case 1 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

//        if ( x < res.cols - 1 && res[y][x + 1] < minValue ) {
//          minValue = res[y][x + 1];
//        }

        if ( y < res.rows - 1 && res[y + 1][x] < minValue ) {
          minValue = res[y + 1][x];
        }

        // combine with mask
        // check if update is required
        if ( (minValue = std::max(minValue, mask[y][x])) >= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = minValue;

        // eventually add lower-right neighbors to queue
//        if ( x < res.cols - 1 ) {
//          update_queue_erode(queue, mask, res, x + 1, y, minValue);
//        }

        if ( y < res.rows - 1 ) {
          update_queue_erode(queue, mask, res, x, y + 1, minValue);
        }
      }
    }
    break;
  case 2 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( x < res.cols - 1 && res[y][x + 1] < minValue ) {
          minValue = res[y][x + 1];
        }

//        if ( y < res.rows - 1 && res[y + 1][x] < minValue ) {
//          minValue = res[y + 1][x];
//        }

        // combine with mask
        // check if update is required
        if ( (minValue = std::max(minValue, mask[y][x])) >= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = minValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_erode(queue, mask, res, x + 1, y, minValue);
        }

//        if ( y < res.rows - 1 ) {
//          update_queue_erode(queue, mask, res, x, y + 1, minValue);
//        }
      }
    }
    break;

  case 4 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();

        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( x < res.cols - 1 && res[y][x + 1] < minValue ) {
          minValue = res[y][x + 1];
        }

        if ( y < res.rows - 1 && res[y + 1][x] < minValue ) {
          minValue = res[y + 1][x];
        }

        // combine with mask
        // check if update is required
        if ( (minValue = std::max(minValue, mask[y][x])) >= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = minValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_erode(queue, mask, res, x + 1, y, minValue);
        }

        if ( y < res.rows - 1 ) {
          update_queue_erode(queue, mask, res, x, y + 1, minValue);
        }
      }
    }
    break;

  case 8 :
    // Process regular lines
    for ( int y = res.rows - 1; y >= 0; --y ) {

      // Process pixels in the middle of the current line
      // consider pixels on the right and below
      for ( int x = res.cols - 1; x >= 0; --x ) {  // FIXME: res.channels();
        const T currentValue = res[y][x];
        T minValue = currentValue;

        if ( y < res.rows - 1 ) {
          // process the 3 values on the line below current pixel
          if ( x > 0 && res[y + 1][x - 1] < minValue ) {
            minValue = res[y + 1][x - 1];
          }

          if ( res[y + 1][x] < minValue ) {
            minValue = res[y + 1][x];
          }

          if ( x < res.cols - 1 && res[y + 1][x + 1] < minValue ) {
            minValue = res[y + 1][x + 1];
          }
        }

        if ( x < res.cols - 1 && res[y][x + 1] < minValue ) {
          minValue = res[y][x + 1];
        }

        // combine with mask
        // check if update is required
        if ( (minValue = std::max(minValue, mask[y][x])) >= currentValue ) {
          continue;
        }

        // update value of current pixel
        res[y][x] = minValue;

        // eventually add lower-right neighbors to queue
        if ( x < res.cols - 1 ) {
          update_queue_erode(queue, mask, res, x + 1, y, minValue);
        }

        if ( y < res.rows - 1 ) {
          if ( x > 0 ) {
            update_queue_erode(queue, mask, res, x - 1, y + 1, minValue);
          }
          update_queue_erode(queue, mask, res, x, y + 1, minValue);
          if ( x < res.cols - 1 ) {
            update_queue_erode(queue, mask, res, x + 1, y + 1, minValue);
          }
        }
      }
    }
    break;
  }

}





/**
 * Update result image using next pixel in the queue,
 * using the 6- or 8- adjacency and floating point values.
 */
template<class T>
static void process_queue_dilate(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, cv::Mat_<T> & res,
    int connectivity)
{
  switch ( connectivity ) {

  case 1 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      const int x = p.x, y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
//      if ( x > 0 && res[y][x - 1] > value ) {
//        value = res[y][x - 1];
//      }
//
//      if ( x < res.cols - 1 && res[y][x + 1] > value ) {
//        value = res[y][x + 1];
//      }

      if ( y > 0 && res[y - 1][x] > value ) {
        value = res[y - 1][x];
      }

      if ( y < res.rows - 1 && res[y + 1][x] > value ) {
        value = res[y + 1][x];
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::min(value, mask[y][x])) <= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
//      if ( x > 0 ) {
//        update_queue_dilate(queue, mask, res, x - 1, y, value);
//      }
//
//      if ( x < res.cols - 1 ) {
//        update_queue_dilate(queue, mask, res, x + 1, y, value);
//      }

      if ( y > 0 ) {
        update_queue_dilate(queue, mask, res, x, y - 1, value);
      }

      if ( y < res.rows - 1 ) {
        update_queue_dilate(queue, mask, res, x, y + 1, value);
      }
    }
    break;

  case 2 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      const int x = p.x, y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
      if ( x > 0 && res[y][x - 1] > value ) {
        value = res[y][x - 1];
      }

      if ( x < res.cols - 1 && res[y][x + 1] > value ) {
        value = res[y][x + 1];
      }

//      if ( y > 0 && res[y - 1][x] > value ) {
//        value = res[y - 1][x];
//      }

      if ( y < res.rows - 1 && res[y + 1][x] > value ) {
        value = res[y + 1][x];
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::min(value, mask[y][x])) <= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
      if ( x > 0 ) {
        update_queue_dilate(queue, mask, res, x - 1, y, value);
      }

      if ( x < res.cols - 1 ) {
        update_queue_dilate(queue, mask, res, x + 1, y, value);
      }

//      if ( y > 0 ) {
//        update_queue_dilate(queue, mask, res, x, y - 1, value);
//      }
//
//      if ( y < res.rows - 1 ) {
//        update_queue_dilate(queue, mask, res, x, y + 1, value);
//      }
    }
    break;

  case 4 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      const int x = p.x, y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
      if ( x > 0 && res[y][x - 1] > value ) {
        value = res[y][x - 1];
      }

      if ( x < res.cols - 1 && res[y][x + 1] > value ) {
        value = res[y][x + 1];
      }

      if ( y > 0 && res[y - 1][x] > value ) {
        value = res[y - 1][x];
      }

      if ( y < res.rows - 1 && res[y + 1][x] > value ) {
        value = res[y + 1][x];
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::min(value, mask[y][x])) <= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
      if ( x > 0 ) {
        update_queue_dilate(queue, mask, res, x - 1, y, value);
      }

      if ( x < res.cols - 1 ) {
        update_queue_dilate(queue, mask, res, x + 1, y, value);
      }

      if ( y > 0 ) {
        update_queue_dilate(queue, mask, res, x, y - 1, value);
      }

      if ( y < res.rows - 1 ) {
        update_queue_dilate(queue, mask, res, x, y + 1, value);
      }
    }
    break;

  case 8 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      int x = p.x;
      int y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compute bounds of neighborhood
      int xmin = std::max(x - 1, 0);
      int xmax = std::min(x + 1, res.cols - 1);
      int ymin = std::max(y - 1, 0);
      int ymax = std::min(y + 1, res.rows - 1);

      // compare with each one of the neighbors
      for ( int y2 = ymin; y2 <= ymax; y2++ ) {
        for ( int x2 = xmin; x2 <= xmax; x2++ ) {
          if ( res[y2][x2] > value ) {
            value = res[y2][x2];
          }
        }
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::min(value, mask[y][x])) <= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // compare with each one of the neighbors
      for ( int y2 = ymin; y2 <= ymax; y2++ ) {
        for ( int x2 = xmin; x2 <= xmax; x2++ ) {
          update_queue_dilate(queue, mask, res, x2, y2, value);
        }
      }
    }
    break;
  }
}




/**
 * Update result image using next pixel in the queue,
 * using the 6- or 8- adjacency and floating point values.
 */
template<class T>
static void process_queue_erode(std::deque<cv::Point> & queue, const cv::Mat_<T> & mask, cv::Mat_<T> & res,
    int connectivity)
{
  switch ( connectivity ) {

  case 1 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      int x = p.x;
      int y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
//      if ( x > 0 && res[y][x - 1] < value ) {
//        value = res[y][x - 1];
//      }
//
//      if ( x < res.cols - 1 && res[y][x + 1] < value ) {
//        value = res[y][x + 1];
//      }

      if ( y > 0 && res[y - 1][x] < value ) {
        value = res[y - 1][x];
      }

      if ( y < res.rows - 1 && res[y + 1][x] < value ) {
        value = res[y + 1][x];
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::max(value, mask[y][x])) >= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
//      if ( x > 0 ) {
//        update_queue_erode(queue, mask, res, x - 1, y, value);
//      }
//
//      if ( x < res.cols - 1 ) {
//        update_queue_erode(queue, mask, res, x + 1, y, value);
//      }

      if ( y > 0 ) {
        update_queue_erode(queue, mask, res, x, y - 1, value);
      }

      if ( y < res.rows - 1 ) {
        update_queue_erode(queue, mask, res, x, y + 1, value);
      }
    }
    break;

  case 2 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      int x = p.x;
      int y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
      if ( x > 0 && res[y][x - 1] < value ) {
        value = res[y][x - 1];
      }

      if ( x < res.cols - 1 && res[y][x + 1] < value ) {
        value = res[y][x + 1];
      }

//      if ( y > 0 && res[y - 1][x] < value ) {
//        value = res[y - 1][x];
//      }
//
//      if ( y < res.rows - 1 && res[y + 1][x] < value ) {
//        value = res[y + 1][x];
//      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::max(value, mask[y][x])) >= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
      if ( x > 0 ) {
        update_queue_erode(queue, mask, res, x - 1, y, value);
      }

      if ( x < res.cols - 1 ) {
        update_queue_erode(queue, mask, res, x + 1, y, value);
      }

//      if ( y > 0 ) {
//        update_queue_erode(queue, mask, res, x, y - 1, value);
//      }
//
//      if ( y < res.rows - 1 ) {
//        update_queue_erode(queue, mask, res, x, y + 1, value);
//      }
    }
    break;

  case 4 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      int x = p.x;
      int y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compare with each one of the four neighbors
      if ( x > 0 && res[y][x - 1] < value ) {
        value = res[y][x - 1];
      }

      if ( x < res.cols - 1 && res[y][x + 1] < value ) {
        value = res[y][x + 1];
      }

      if ( y > 0 && res[y - 1][x] < value ) {
        value = res[y - 1][x];
      }

      if ( y < res.rows - 1 && res[y + 1][x] < value ) {
        value = res[y + 1][x];
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::max(value, mask[y][x])) >= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // Eventually add each neighbor
      if ( x > 0 ) {
        update_queue_erode(queue, mask, res, x - 1, y, value);
      }

      if ( x < res.cols - 1 ) {
        update_queue_erode(queue, mask, res, x + 1, y, value);
      }

      if ( y > 0 ) {
        update_queue_erode(queue, mask, res, x, y - 1, value);
      }

      if ( y < res.rows - 1 ) {
        update_queue_erode(queue, mask, res, x, y + 1, value);
      }
    }
    break;

  case 8 :
    while ( !queue.empty() ) {

      cv::Point p = queue.front();
      queue.pop_front();

      int x = p.x;
      int y = p.y;

      // the maximal value around current pixel
      T value = res[y][x];

      // compute bounds of neighborhood
      int xmin = std::max(x - 1, 0);
      int xmax = std::min(x + 1, res.cols - 1);
      int ymin = std::max(y - 1, 0);
      int ymax = std::min(y + 1, res.rows - 1);

      // compare with each one of the neighbors
      for ( int y2 = ymin; y2 <= ymax; y2++ ) {
        for ( int x2 = xmin; x2 <= xmax; x2++ ) {
          if ( res[y2][x2] < value ) {
            value = res[y2][x2];
          }
        }
      }

      // bound with mask value
      // if no update is needed, continue to next item in queue
      if ( (value = std::max(value, mask[y][x])) >= res[y][x] ) {
        continue;
      }

      // update result for current position
      res[y][x] = value;

      // compare with each one of the neighbors
      for ( int y2 = ymin; y2 <= ymax; y2++ ) {
        for ( int x2 = xmin; x2 <= xmax; x2++ ) {
          update_queue_dilate(queue, mask, res, x2, y2, value);
        }
      }
    }
    break;
  }
}







template<class T>
static bool geo_reconstruction_dilate_(const cv::Mat_<T> & marker, const cv::Mat_<T> & mask, cv::Mat_<T> & dst, int connectivity)
{
  cv::Mat_<T> tmp;
  std::deque<cv::Point> queue;

  cv::Mat_<T> & res = marker.data == dst.data || mask.data == dst.data ? tmp : dst;

  cv::min((cv::Mat&) marker, (cv::Mat&) mask, (cv::Mat&) res);

  // forward iteration
  forward_dilate(mask, res, connectivity);

  // backward iteration
  backward_dilate(queue, mask, res, connectivity);
  process_queue_dilate(queue, mask, res, connectivity);

  if ( !tmp.empty() ) {
    dst = tmp;
  }

  return true;
}


template<class T>
static bool geo_reconstruction_erode_(const cv::Mat_<T> & marker, const cv::Mat_<T> & mask, cv::Mat_<T> & dst, int connectivity)
{
  cv::Mat_<T> tmp;
  std::deque<cv::Point> queue;

  cv::Mat_<T> & res = marker.data == dst.data || mask.data == dst.data ? tmp : dst;

  cv::max((const cv::Mat&) marker, (const cv::Mat&) mask, (cv::Mat&) res);

  // forward iteration
  forward_erode(mask, res, connectivity);

  // backward iteration
  backward_erode(queue, mask, res, connectivity);
  process_queue_erode(queue, mask, res, connectivity);

  if ( &res != &dst ) {
    dst = tmp;
  }

  return true;
}



bool geo_reconstruction_dilate(cv::InputArray _marker_image, cv::InputArray _reference_image,
    cv::Mat & reconstructed_image, int connectivity)
{
  const cv::Mat marker_image = _marker_image.getMat();
  const cv::Mat reference_image = _reference_image.getMat();

  // Check sizes and types are consistent
  if ( marker_image.size() != reference_image.size() || marker_image.type() != reference_image.type() ) {
    CF_FATAL("Invalid argument: marker and reference images must have the same size and type");
    return false;
  }

  // Check connectivity has a correct value
  if ( connectivity != 1 && connectivity != 2 && connectivity != 4 && connectivity != 8 ) {
    CF_FATAL("Invalid argument: Connectivity must be either 1, 2, 4 or 8, not %d", connectivity);
    return false;
  }

  switch ( marker_image.depth() ) {
  case CV_8U :
    return geo_reconstruction_dilate_((const cv::Mat_<uint8_t> &) marker_image,
        (const cv::Mat_<uint8_t> &) reference_image,
        (cv::Mat_<uint8_t> &) reconstructed_image,
        connectivity);
  case CV_8S :
    return geo_reconstruction_dilate_((const cv::Mat_<int8_t> &) marker_image,
        (const cv::Mat_<int8_t> &) reference_image,
        (cv::Mat_<int8_t> &) reconstructed_image,
        connectivity);
  case CV_16U :
    return geo_reconstruction_dilate_((const cv::Mat_<uint16_t> &) marker_image,
        (const cv::Mat_<uint16_t> &) reference_image,
        (cv::Mat_<uint16_t> &) reconstructed_image,
        connectivity);
  case CV_16S :
    return geo_reconstruction_dilate_((const cv::Mat_<int16_t> &) marker_image,
        (const cv::Mat_<int16_t> &) reference_image,
        (cv::Mat_<int16_t> &) reconstructed_image,
        connectivity);
  case CV_32S :
    return geo_reconstruction_dilate_((const cv::Mat_<int32_t> &) marker_image,
        (const cv::Mat_<int32_t> &) reference_image,
        (cv::Mat_<int32_t> &) reconstructed_image,
        connectivity);
  case CV_32F :
    return geo_reconstruction_dilate_((const cv::Mat_<float> &) marker_image,
        (const cv::Mat_<float> &) reference_image,
        (cv::Mat_<float> &) reconstructed_image,
        connectivity);
  case CV_64F :
    return geo_reconstruction_dilate_((const cv::Mat_<double> &) marker_image,
        (const cv::Mat_<double> &) reference_image,
        (cv::Mat_<double> &) reconstructed_image,
        connectivity);
  }

  CF_FATAL("Invalid argument: not supported pixel depth %d", marker_image.depth());
  return false;
}



bool geo_reconstruction_erode(cv::InputArray _marker_image, cv::InputArray _reference_image,
    cv::Mat & reconstructed_image, int connectivity)
{
  const cv::Mat marker_image = _marker_image.getMat();
  const cv::Mat reference_image = _reference_image.getMat();

  if ( marker_image.empty() ) {
    CF_FATAL("Invalid argument: marker image is empty");
    return false;
  }

  if ( reference_image.empty() ) {
    CF_FATAL("Invalid argument: reference image is empty");
    return false;
  }

  // Check sizes and types are consistent
  if ( marker_image.size() != reference_image.size() || marker_image.type() != reference_image.type() ) {
    CF_FATAL("Invalid argument: marker and reference images must have the same size and type");
    return false;
  }

  // Check connectivity has a correct value
  if ( connectivity != 1 && connectivity != 2 && connectivity != 4 && connectivity != 8 ) {
    CF_FATAL("Invalid argument: Connectivity must be either 1, 2, 4 or 8, not %d", connectivity);
    return false;
  }

  switch ( marker_image.depth() ) {
  case CV_8U :
    return geo_reconstruction_erode_((const cv::Mat_<uint8_t> &) marker_image,
        (const cv::Mat_<uint8_t> &) reference_image,
        (cv::Mat_<uint8_t> &) reconstructed_image,
        connectivity);
  case CV_8S :
    return geo_reconstruction_erode_((const cv::Mat_<int8_t> &) marker_image,
        (const cv::Mat_<int8_t> &) reference_image,
        (cv::Mat_<int8_t> &) reconstructed_image,
        connectivity);
  case CV_16U :
    return geo_reconstruction_erode_((const cv::Mat_<uint16_t> &) marker_image,
        (const cv::Mat_<uint16_t> &) reference_image,
        (cv::Mat_<uint16_t> &) reconstructed_image,
        connectivity);
  case CV_16S :
    return geo_reconstruction_erode_((const cv::Mat_<int16_t> &) marker_image,
        (const cv::Mat_<int16_t> &) reference_image,
        (cv::Mat_<int16_t> &) reconstructed_image,
        connectivity);
  case CV_32S :
    return geo_reconstruction_erode_((const cv::Mat_<int32_t> &) marker_image,
        (const cv::Mat_<int32_t> &) reference_image,
        (cv::Mat_<int32_t> &) reconstructed_image,
        connectivity);
  case CV_32F :
    return geo_reconstruction_erode_((const cv::Mat_<float> &) marker_image,
        (const cv::Mat_<float> &) reference_image,
        (cv::Mat_<float> &) reconstructed_image,
        connectivity);
  case CV_64F :
    return geo_reconstruction_erode_((const cv::Mat_<double> &) marker_image,
        (const cv::Mat_<double> &) reference_image,
        (cv::Mat_<double> &) reconstructed_image,
        connectivity);
  }

  CF_FATAL("Invalid argument: not supported pixel depth %d", marker_image.depth());
  return false;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Geodesic Opening by Reconstruction

bool geo_open(cv::InputArray src, cv::Mat & dst, cv::InputArray SE, int connectivity,
    const cv::Point & anchor, int borderType, const cv::Scalar & borderValue)
{
  cv::Mat eroded_src;

  cv::erode(src, eroded_src, SE, anchor, 1, borderType, borderValue);

  if ( !geo_reconstruction_dilate(eroded_src, src, dst, connectivity) ) {
    CF_FATAL("geo_reconstruction_dilate() fails");
    return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Geodesic Close by Reconstruction

bool geo_close(cv::InputArray src,  cv::Mat & dst, cv::InputArray SE, int connectivity,
    const cv::Point & anchor, int borderType, const cv::Scalar & borderValue)
{
  cv::Mat dilated_src;

  cv::dilate(src, dilated_src, SE, anchor, 1, borderType, borderValue);

  if ( !geo_reconstruction_erode(dilated_src, src, dst, connectivity) ) {
    CF_FATAL("geo_reconstruction_erode() fails");
    return false;
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Geodesic Hole Filling by Reconstruction

bool geo_fill_holes(cv::InputArray src, cv::Mat & dst, int connectivity)
{
  cv::Mat marker_image, inverted_source_image;
  double maxval;

  //
  // inverted_source_image = maxval - src
  //
  minMaxLoc(src, nullptr, &maxval);
  subtract(maxval, src, inverted_source_image);


  //
  //                |  inverted_source_image  on borders
  // marker_image = |
  //                |  0 otherwise
  //
  inverted_source_image.copyTo(marker_image);
  marker_image(cv::Rect(1, 1, src.cols() - 2, src.rows() - 2)).setTo(0);

  //
  // dst = maxval - geo_reconstruction_dilate(marker_image, inverted_source_image)
  //
  if ( !geo_reconstruction_dilate(marker_image, inverted_source_image, dst, connectivity) ) {
    CF_FATAL("geo_reconstruction_dilate() fails");
    return false;
  }

  subtract(maxval, dst, dst);

  return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get objects that touch the border of an image using morphological reconstruction by dilation.
// "Digital image processing using matlab" by Rafael C. Gonzales.
// 10.5.3  Clearing Border Objects
bool geo_get_components_touching_borders(cv::InputArray _src, cv::Mat & dst,
    int connectivity, double * optional_out_minval)
{
  const cv::Mat src = _src.getMat();
  double minval;
  cv::Mat marker_image;

  minMaxLoc(src, &minval);

  src.copyTo(marker_image);
  marker_image(cv::Rect(1, 1, src.cols - 2, src.rows - 2)).setTo(minval);

  if ( optional_out_minval ) {
    *optional_out_minval = minval;
  }

  if ( !geo_reconstruction_dilate(marker_image, src, dst, connectivity) ) {
    CF_FATAL("geo_reconstruction_dilate() fails");
    return false;
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Remove objects that touch the border of an image using morphological reconstruction by dilation.
// "Digital image processing using matlab" by Rafael C. Gonzales.
// 10.5.3  Clearing Border Objects
bool geo_remove_components_touching_borders(cv::InputArray _src, cv::Mat & dst,
    int connectivity, double * optional_out_minval)
{
  const cv::Mat src = _src.getMat();
  double minval;

  cv::Mat borders_image;

  if ( !geo_get_components_touching_borders(src, borders_image, connectivity, &minval) ) {
    CF_FATAL("geo_components_touching_borders() fails");
    return false;
  }

  subtract(src, borders_image, dst);

  if ( minval != 0 ) {
    add(dst, minval, dst);
  }

  if ( optional_out_minval ) {
    *optional_out_minval = minval;
  }

  return true;
}


