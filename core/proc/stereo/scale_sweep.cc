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
    cv::InputArray src_mask,
    cv::OutputArray dst_mask)
{
  INSTRUMENT_REGION("");

  const double max_epipole_distance =
      std::max( { abs(epipole_location.x), abs(epipole_location.x - image_size.width - 1),
          abs(epipole_location.y), abs(epipole_location.y - image_size.height - 1) });

  const double K =
      max_epipole_distance / (max_epipole_distance - iteration);

  cv::Mat1b dstm;

  cmap.create(image_size);

  if( dst_mask.needed() ) {
    dstm = cv::Mat1b(image_size, 0);
  }


  if ( src_mask.empty() ) {

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [K, epipole_location, &cmap, &dstm](const tbb_range & range) {
          for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];
            uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

            const float yy = (y - epipole_location.y) * K + epipole_location.y;

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
              cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
              cmp[x][1] = yy;
              if ( dstmp ) {
                dstmp[x] = 255;
              }
            }
          }
        });

  #else

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];
        uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

        const float yy = (y - epipole_location.y) * K + epipole_location.y;

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
          cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
          cmp[x][1] = yy;
          if ( dstmp ) {
            dstmp[x] = 255;
          }
        }
      }

#endif
  }
  else {

    const cv::Mat1b m =
        src_mask.getMat();

    cmap.setTo(-1);

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [K, epipole_location, &cmap, &m, &dstm](const tbb_range & range) {
          for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];
            const uint8_t * mp = m[y];
            uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

            const float yy = (y - epipole_location.y) * K + epipole_location.y;

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
              if ( mp[x] ) {
                cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
                cmp[x][1] = yy;
                if ( dstmp ) {
                  dstmp[x] = 255;
                }
              }
            }
          }
        });

  #else

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];
        const uint8_t * mp = m[y];
        uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

        const float yy = (y - epipole_location.y) * K + epipole_location.y;

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
          if ( mp[x] ) {
            cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
            cmp[x][1] = yy;
            if ( dstmp ) {
              dstmp[x] = 255;
            }
          }
        }
      }

#endif

  }
}


void create_scale_compression_remap(int iteration,
    const cv::Size & image_size,
    const cv::Point2d & epipole_location,
    const cv::Matx33d & H,
    cv::Mat2f & cmap,
    cv::InputArray src_mask,
    cv::OutputArray dst_mask)
{
  INSTRUMENT_REGION("");

  const double max_epipole_distance =
      std::max( { abs(epipole_location.x), abs(epipole_location.x - image_size.width - 1),
          abs(epipole_location.y), abs(epipole_location.y - image_size.height - 1) });

  const double K =
      max_epipole_distance / (max_epipole_distance - iteration);

  cv::Mat1b dstm;

  cmap.create(image_size);

  if( dst_mask.needed() ) {
    dstm = cv::Mat1b(image_size, 0);
  }


  if ( src_mask.empty() ) {

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [H, K, image_size, epipole_location, &cmap, &dstm](const tbb_range & range) {

        const int & w = image_size.width;
        const int & h = image_size.height;

        for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];
            uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {

              const double cw = (H(2,0) * x + H(2,1) * y + H(2,2));
              const double cx = (H(0,0) * x + H(0,1) * y + H(0,2)) / cw;
              const double cy = (H(1,0) * x + H(1,1) * y + H(1,2)) / cw;

              if ( cx >= 0 && cx < w && cy >= 0 && cy < h ) {
                cmp[x][0] = (cx - epipole_location.x) * K + epipole_location.x;
                cmp[x][1] = (cy - epipole_location.y) * K + epipole_location.y;
                if ( dstmp ) {
                  dstmp[x] = 255;
                }
              }
            }
          }
        });

#else

    const int & w = image_size.width;
    const int & h = image_size.height;

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];
        uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {

          const double cw = (H(2,0) * x + H(2,1) * y + H(2,2));
          const double cx = (H(0,0) * x + H(0,1) * y + H(0,2)) / cw;
          const double cy = (H(1,0) * x + H(1,1) * y + H(1,2)) / cw;

          if ( cx >= 0 && cx < w && cy >= 0 && cy < h ) {
            cmp[x][0] = (cx - epipole_location.x) * K + epipole_location.x;
            cmp[x][1] = (cy - epipole_location.y) * K + epipole_location.y;
            if ( dstmp ) {
              dstmp[x] = 255;
            }
          }
        }      }

#endif
  }
  else {

    const cv::Mat1b m =
        src_mask.getMat();

    cmap.setTo(-1);

#if HAVE_TBB

    tbb::parallel_for(tbb_range(0, cmap.rows, 64),
        [K, epipole_location, &cmap, &m, &dstm](const tbb_range & range) {
          for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {

            cv::Vec2f * cmp = cmap[y];
            const uint8_t * mp = m[y];
            uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

            const float yy = (y - epipole_location.y) * K + epipole_location.y;

            for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
              if ( mp[x] ) {
                cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
                cmp[x][1] = yy;
                if ( dstmp ) {
                  dstmp[x] = 255;
                }
              }
            }
          }
        });

  #else

      for ( int y = 0, ny = cmap.rows; y < ny; ++y ) {

        cv::Vec2f * cmp = cmap[y];
        const uint8_t * mp = m[y];
        uint8_t * dstmp = dstm.empty() ? nullptr : dstm[y];

        const float yy = (y - epipole_location.y) * K + epipole_location.y;

        for ( int x = 0, xmax = cmap.cols; x < xmax; ++x ) {
          if ( mp[x] ) {
            cmp[x][0] = (x - epipole_location.x) * K + epipole_location.x;
            cmp[x][1] = yy;
            if ( dstmp ) {
              dstmp[x] = 255;
            }
          }
        }
      }

#endif

  }
}

