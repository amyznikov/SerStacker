/*
 * scale_sweep.cc
 *
 *  Created on: Sep 7, 2023
 *      Author: amyznikov
 */
#include "scale_sweep.h"
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
typedef tbb::blocked_range<int> tbb_range;
#endif // HAVE_TBB

/*
 * create_scale_compression_remap()
 *
 * This routine creates the map used to cv::remap() the train image
 * for given stereo matching iteration.
 */
void create_scale_compression_remap(int iteration,
    const cv::Size & image_size,
    const cv::Point2d & epipole_location,
    cv::Mat2f & cmap,
    cv::InputArray mask)
{
  INSTRUMENT_REGION("");

  const double max_epipole_distance =
      std::max( { abs(epipole_location.x), abs(epipole_location.x - image_size.width - 1),
          abs(epipole_location.y), abs(epipole_location.y - image_size.height - 1) });

  const double K =
      max_epipole_distance / (max_epipole_distance - iteration);

  cmap.create(image_size);

  if ( mask.empty() ) {

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [K, epipole_location, &cmap](const tbb_range & range) {
          for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];

            const float yy = (y - epipole_location.y) * K + epipole_location.y;

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
              cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
              cmp[x][1] = yy;
            }
          }
        });

  #else

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];

        const float yy = (y - epipole_location.y) * K + epipole_location.y;

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
          cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
          cmp[x][1] = yy;
        }
      }

#endif
  }
  else {

    const cv::Mat1b m =
        mask.getMat();

    cmap.setTo(-1);

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [K, epipole_location, &cmap, m](const tbb_range & range) {
          for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];
            const uint8_t * mp = m[y];

            const float yy = (y - epipole_location.y) * K + epipole_location.y;

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
              if ( mp[x] ) {
                cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
                cmp[x][1] = yy;
              }
            }
          }
        });

  #else

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];
        const uint8_t * mp = m[y];

        const float yy = (y - epipole_location.y) * K + epipole_location.y;

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
          if ( mp[x] ) {
            cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
            cmp[x][1] = yy;
          }
        }
      }

#endif

  }
}

