/*
 * planetary-disk-rotation.cc
 *
 *  Created on: Sep 9, 2021
 *      Author: amyznikov
 */

#include "planetary-disk-rotation.h"
#include <tbb/tbb.h>



void create_ellipse_rotation_remap(double rotation_angle,
    const cv::RotatedRect & E,
    const cv::Matx23d & R2I,
    const cv::Size & size,
    cv::Mat2f & rmap,
    cv::Mat1f & wmask)
{
  double sa, ca;
  sincos(E.angle * CV_PI / 180, &sa, &ca);

  rmap.create(size);
  wmask.create(size);

  typedef tbb::blocked_range<int> tbb_range;

  tbb::parallel_for(tbb_range(0, size.height, 64),
      [E, R2I, sa, ca, rotation_angle, &rmap, &wmask ](const tbb_range & r ) {

        const double A = 0.5 * E.size.width;
        const double B = 0.5 * E.size.height;
        const double x0 = E.center.x;
        const double y0 = E.center.y;

        for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
          for ( int x = 0; x < rmap.cols; ++x ) {

            double xe = ((x - x0) * ca + (y - y0) * sa) / A;
            double ye = (-(x - x0) * sa + (y - y0) * ca) / B;

            if ( xe * xe + ye * ye >= 1 ) {
              rmap[y][x][0] = -1;
              rmap[y][x][1] = -1;
              wmask[y][x] = 0;
              continue;
            }

            const double q = sqrt(1. - ye * ye);
            const double phi = asin(xe / q) + rotation_angle;

            if ( (phi <= -CV_PI / 2) || (phi >= CV_PI / 2) ) {
              rmap[y][x][0] = -1;
              rmap[y][x][1] = -1;
              wmask[y][x] = 0;
              continue;
            }

            xe = A * q * sin(phi);
            ye = B * ye;

            const double xx = xe * ca - ye * sa + x0;
            const double yy = xe * sa + ye * ca + y0;

            rmap[y][x][0] = R2I(0, 0) * xx + R2I(0, 1) * yy + R2I(0, 2);
            rmap[y][x][1] = R2I(1, 0) * xx + R2I(1, 1) * yy + R2I(1, 2);
            wmask[y][x] = cos(phi);
          }
        }
      });

}
