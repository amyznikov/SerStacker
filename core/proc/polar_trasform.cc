/*
 * polar_trasform.cc
 *
 *  Created on: Aug 20, 2023
 *      Author: amyznikov
 */

#include "polar_trasform.h"
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

template<class T>
static double distance(const cv::Point_<T> & a, const cv::Point_<T> & b)
{
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
template<class T>
static double distance_to_line(const cv::Point_<T> & p, const cv::Point_<T> & lp1, const cv::Point_<T> & lp2)
{
  const cv::Point_<T> & p1 = lp1;
  const cv::Point_<T> & p2 = lp2;

  const double x0 = p.x;
  const double y0 = p.y;
  const double x1 = p1.x;
  const double y1 = p1.y;
  const double x2 = p2.x;
  const double y2 = p2.y;

  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void create_epipolar_remap(const cv::Size src_size, const cv::Point2f & center, cv::Mat2f & output_rmap)
{
  /*
   *
   *    C1                      C8                         C7
   *
   *             A(0,0)                          B(w,0)
   *               ---------------------------------
   *               |                               |
   *               |                               |
   *               |                               |
   *   C2          |            C0                 |
   *               |                               |       C6
   *               |                               |
   *               |                               |
   *               ---------------------------------
   *              D(0,h)                          C(w,h)
   *
   *   C3                        C4                        C5
   *
   */

  static const auto max_corner_distance =
      [](const cv::Point2f & p, const cv::Point2f corners[4]) -> double {
        double rho_max = distance(p, corners[0]);
        for( int i = 1; i < 4; ++i ) {
          const double rho = distance(p, corners[i]);
          if( rho > rho_max ) {
            rho_max = rho;
          }
        }
        return rho_max;
      };

  const double x0 = center.x;
  const double y0 = center.y;
  const int w = src_size.width;
  const int h = src_size.height;

  const cv::Point2f corners[4] = {
      cv::Point2f(0, 0),
      cv::Point2f(w, 0),
      cv::Point2f(w, h),
      cv::Point2f(0, h),
  };

  double rho_min, rho_max, theta_min, theta_max;
  int ntheta = w + h;

  if( x0 < 0 ) {

    if( y0 < 0 ) {
      // Case C1
      rho_min = distance(center, corners[0]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[1].y - center.y, corners[1].x - center.x);
      theta_max = atan2(corners[3].y - center.y, corners[3].x - center.x);
      CF_DEBUG("C1 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }
    else if( y0 >= h ) {
      // Case C3
      rho_min = distance(center, corners[3]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[0].y - center.y, corners[0].x - center.x);
      theta_max = atan2(corners[2].y - center.y, corners[2].x - center.x);
      CF_DEBUG("C3 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }
    else {
      // Case C2
      rho_min = distance_to_line(center, corners[0], corners[3]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[0].y - center.y, corners[0].x - center.x);
      theta_max = atan2(corners[3].y - center.y, corners[3].x - center.x);
      CF_DEBUG("C2 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }
  }
  else if( x0 >= w ) {

    if( y0 < 0 ) {
      // Case C7
      rho_min = distance(center, corners[1]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[2].y - center.y, corners[2].x - center.x);
      theta_max = atan2(corners[0].y - center.y, corners[0].x - center.x);
      CF_DEBUG("C7 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }
    else if( y0 >= h ) {
      // Case C5
      rho_min = distance(center, corners[2]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[3].y - center.y, corners[3].x - center.x);
      theta_max = atan2(corners[2].y - center.y, corners[2].x - center.x);
      CF_DEBUG("C5 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }
    else {
      // Case C6
      rho_min = distance_to_line(center, corners[1], corners[2]);
      rho_max = max_corner_distance(center, corners);
      theta_min = atan2(corners[2].y - center.y, corners[2].x - center.x);
      theta_max = atan2(corners[1].y - center.y, corners[1].x - center.x) + 2 * CV_PI;
      CF_DEBUG("C6 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
    }

  }
  else if ( y0 < 0 ) {
    // case C8
    rho_min = distance_to_line(center, corners[0], corners[1]);
    rho_max = max_corner_distance(center, corners);
    theta_min = atan2(corners[1].y - center.y, corners[1].x - center.x);
    theta_max = atan2(corners[0].y - center.y, corners[0].x - center.x);
    CF_DEBUG("C8 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
  }
  else if ( y0 >= h ) {
    // case C4
    rho_min = distance_to_line(center, corners[2], corners[3]);
    rho_max = max_corner_distance(center, corners);
    theta_min = atan2(corners[3].y - center.y, corners[3].x - center.x);
    theta_max = atan2(corners[2].y - center.y, corners[2].x - center.x);
    CF_DEBUG("C4 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
  }
  else {
    // Case C0
    ntheta = (w + h);
    rho_min = 0;
    rho_max = max_corner_distance(center, corners);
    theta_min = 0;
    theta_max = 2 * CV_PI;
    CF_DEBUG("C0 rho_min=%g rho_max=%g theta_min=%g theta_max=%g", rho_min, rho_max, theta_min, theta_max);
  }

  if ( theta_max < theta_min ) {
    theta_max += 2 * CV_PI;
  }

  const cv::Size dst_size((int) (rho_max - rho_min) + 1, ntheta);
  const double theta_step = (theta_max - theta_min ) / ntheta;

  CF_DEBUG("dst_size= %dx%d theta_min=%g theta_max=%g theta_step=%g", dst_size.width, dst_size.height, theta_min, theta_max, theta_step);


  output_rmap.create(dst_size);
  output_rmap.setTo(-1);



#if HAVE_TBB
  tbb::parallel_for(0, dst_size.height, 1,
      [&output_rmap, w, h, dst_size, center, rho_min, theta_min, theta_max, theta_step](int t) {
#else
  for ( int t = 0; t < dst_size.height; ++t ) {
#endif

    const double theta = theta_min + t * theta_step;
    const double ct = cos(theta);
    const double st = sin(theta);

    for ( int r = 0; r < dst_size.width; ++r ) {

      const double rho = rho_min + r;

      const double x = rho * ct + center.x;
      if ( x >=0 && x < w ) {

        const double y = rho * st + center.y;
        if ( y >= 0 && y < h ) {
          output_rmap[t][r][0] = x;
          output_rmap[t][r][1] = y;
        }
      }
    }
#if HAVE_TBB
  });
#else
  }
#endif

}
