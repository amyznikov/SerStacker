/*
 * c_hdl_motion_filter.cc
 *
 *  Created on: Dec 4, 2022
 *      Author: amyznikov
 */

#include "c_hdl_motion_filter.h"

#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

static inline double square(double x)
{
  return x * x;
}

c_hdl_motion_filter::c_hdl_motion_filter()
{
}

void c_hdl_motion_filter::initialize(const cv::Size & range_image_size)
{
  histogram_.clear();
  histogram_.shrink_to_fit();

  if( !(histogram_size_ = range_image_size).empty() ) {
    histogram_.resize(histogram_size_.area());
    clear();
  }
}

void c_hdl_motion_filter::clear()
{
  if( !histogram_.empty() ) {
    memset(histogram_.data(), 0, histogram_.size() * sizeof(histogram_[0]));
  }

  nb_frames_processed_ = 0;
}

void c_hdl_motion_filter::reset()
{
  initialize(cv::Size(0, 0));
}

const cv::Size& c_hdl_motion_filter::histogram_size() const
{
  return histogram_size_;
}

void c_hdl_motion_filter::set_enabled(bool v)
{
  enabeld_ = v;
}

bool c_hdl_motion_filter::enabled() const
{
  return enabeld_;
}

void c_hdl_motion_filter::set_max_distance(double v)
{
  max_distance_ = v;
}

double c_hdl_motion_filter::max_distance() const
{
  return max_distance_;
}

void c_hdl_motion_filter::set_bg_history(int v)
{
  bg_history_ = std::max(1, v);
}

int c_hdl_motion_filter::bg_history() const
{
  return bg_history_;
}

void c_hdl_motion_filter::set_var_threshold(double v)
{
  var_threshold_ = std::max(0.5, v);
}

double c_hdl_motion_filter::var_threshold() const
{
  return var_threshold_;
}

void c_hdl_motion_filter::set_var_init(double v)
{
  var_init_ = std::max(0.01, v);
}

double c_hdl_motion_filter::var_init() const
{
  return var_init_;
}

void c_hdl_motion_filter::set_var_min(double v)
{
  var_min_ = std::max(0.01, v);
}

double c_hdl_motion_filter::var_min() const
{
  return var_min_;
}

void c_hdl_motion_filter::set_var_max(double v)
{
  var_max_ = std::max(0.01, v);
}

double c_hdl_motion_filter::var_max() const
{
  return var_max_;
}

void c_hdl_motion_filter::set_motion_threshold(double v)
{
  motion_threshold_ = v;
}

double c_hdl_motion_filter::motion_threshold() const
{
  return motion_threshold_;
}

void c_hdl_motion_filter::set_enable_noise_reduction(bool v)
{
  enable_noise_reduction_ = v;
}

bool c_hdl_motion_filter::enable_noise_reduction() const
{
  return enable_noise_reduction_;
}

void c_hdl_motion_filter::set_denoise_filter_radius(const cv::Size & v)
{
  denoise_filter_radius_ = v;
}

const cv::Size& c_hdl_motion_filter::denoise_filter_radius() const
{
  return denoise_filter_radius_;
}

void c_hdl_motion_filter::set_denoise_filter_depth(double v)
{
  denoise_filter_depth_ = v;
}

double c_hdl_motion_filter::denoise_filter_depth() const
{
  return denoise_filter_depth_;
}

void c_hdl_motion_filter::set_denoise_filter_threshold(int v)
{
  denoise_filter_threshold_ = v;
}

int c_hdl_motion_filter::denoise_filter_threshold() const
{
  return denoise_filter_threshold_;
}

// check sort order
#define DBG_CHECK_ORDER() \
if ( true && gm.nmodes > 1 ) { \
  for( int i = 1; i < gm.nmodes; ++i ) { \
    if( gm.modes[i].m > gm.modes[i - 1].m ) { \
      CF_ERROR("Out of order: modes[%d].m=%g >= modes[%d].m=%g", i, gm.modes[i].m, i-1, gm.modes[i-1].m); \
      exit(1);/* raise (SIGTRAP); */\
    } \
  } \
}

#define DBG_CHECK_ZEROW() \
if ( false && gm.nmodes > 0 ) { \
  for( int i = 0; i < gm.nmodes; ++i ) { \
    if( gm.modes[i].w <= 0 ) { \
      CF_ERROR("zerow %g: at mode %d / %d", gm.modes[i].w, i, gm.nmodes); \
      /*exit(1);*/ raise (SIGTRAP); \
    } \
  } \
}

#define DUMP_MODES(_gm) \
if ( true ) { \
  std::string modes = ssprintf("modes: %d (", (_gm).nmodes); \
  for ( int i = 0; i < (_gm).nmodes; ++i ) { \
    modes += ssprintf("{w=%g m=%g s=%g}", \
        gm.modes[i].w, \
        gm.modes[i].m, \
        gm.modes[i].s); \
  }\
  modes += ")"; \
  CF_DEBUG("F %d x=%d y=%d %s", nb_frames_processed_-1, x, y, modes.c_str()); \
}

void c_hdl_motion_filter::remove_mode(struct GM & gm, int pos)
{
  if( pos < --gm.nmodes ) { // reduce number of modes (possible underflow MUST be checked by caller!)

    memcpy(&gm.modes[pos], // destination
        &gm.modes[pos + 1], // source
        (gm.nmodes - pos) * sizeof(gm.modes[0])); // number of bytes
  }
}

void c_hdl_motion_filter::insert_mode(struct GM & gm, int insert_pos)
{
  if( insert_pos < gm.nmodes ) { // increase number of modes (possible overflow MUST be checked by caller!)

    memmove(&gm.modes[insert_pos + 1], // destination
        &gm.modes[insert_pos], // source
        (gm.nmodes - insert_pos) * sizeof(gm.modes[0])); // number of bytes
  }

  ++gm.nmodes;
}

bool c_hdl_motion_filter::update(const cv::Mat1f & range_image, cv::OutputArray output_foreground_mask)
{
  INSTRUMENT_REGION("c_hdl_motion_filter");

  if( histogram_.empty() || range_image.size() != histogram_size_ ) {

    if( range_image.size().empty() ) {
      CF_ERROR("Empty range image specified");
      return false;
    }

    initialize(range_image.size());
  }

  //  CF_DEBUG("F %d sizeof(GM)=%u", nb_frames_processed_, (uint)sizeof(histogram_[0]));

  ++nb_frames_processed_;

  cv::Mat1b fgm;

  if( output_foreground_mask.needed() ) {
    output_foreground_mask.create(histogram_size_, CV_8UC1);
    fgm = output_foreground_mask.getMat();
    fgm.setTo(0);
  }

  const int nx =
      histogram_size_.width;

  const int ny =
      histogram_size_.height;

  const double T =
      square(var_threshold_);

  const int N = // data ratio normalization scale
      (std::min)(nb_frames_processed_,
          bg_history_);

  const double eps =
      1. / (N + 1);

  const double w_initial =
      2 * eps;

  const double smin =
      square(var_min_);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, ny, 256),
      [this, &range_image, &fgm, nx, ny, T, N, eps, w_initial, smin](const range & r) {
        for( int y = r.begin(); y != r.end(); ++y ) {

#else
        for( int y = 0; y < ny; ++y ) {
#endif

          const float *depthp = range_image[y];

          GM *const gmp = &histogram_[y * nx];

          for( int x = 0; x < nx; ++x ) {

            GM &gm = gmp[x];

            int update_mode_index = -1;

            const float depth = range_image[y][x];

            if( depth > 0 && depth < 1 ) { // ignore parasitic noise points in range 0..1 [m]
              continue;
            }

            bool fg = false;

            if( depth > 0 ) {

              if( !gm.nmodes ) { // just initialize initial mode
                gm.modes[0].w = w_initial;
                gm.modes[0].m = depth;
                gm.modes[0].s = square(var_init_);
                gm.nmodes = 1;
                fg = true;
              }
              else {

                // Search closest mode, data are sorted in decreasing depth order

                int closest_mode = 0;
                double s = square(depth - gm.modes[0].m);

                for( int i = 1; i < gm.nmodes; ++i ) {

                  const double dist = square(depth - gm.modes[i].m);
                  if( dist > s ) {
                    break;
                  }

                  s = dist;
                  closest_mode = i;
                }

                // Select position for update
                // Modes are sorted in decreasing distance order - most distant mode is first

                if( s < T * gm.modes[closest_mode].s ) { // Update existing mode

                  //  INSTRUMENT_REGION("UPDATE");

                  GMM &g =
                  gm.modes[update_mode_index =
                  closest_mode];

                  g.m = (1 - eps) * g.m + eps * depth;
                  g.s = (std::max)((1 - eps) * g.s + eps * s, smin);
                  g.w = (1 - eps) * g.w + eps;

                  if( g.w < motion_threshold_ ) {
                    fg = true;
                  }

                }
                else {

                  fg = true;

                  // INSTRUMENT_REGION("INSERT");

                  if( gm.nmodes == MAX_MODES ) {

                    //INSTRUMENT_REGION("OVERFOW");

                    // have no space to insert new mode - will replace oldest mode with new

                    int oldest_mode_index = 0;
                    double oldest_mode_weight = gm.modes[0].w;

                    for( int i = 1; i < gm.nmodes; ++i ) {
                      if( gm.modes[i].w < oldest_mode_weight ) {
                        oldest_mode_index = i;
                        oldest_mode_weight = gm.modes[i].w;
                      }
                    }

                    remove_mode(gm, oldest_mode_index);

                    if( closest_mode > oldest_mode_index ) {
                      --closest_mode;
                    }
                  }

                  // search position to insert new mode, data are sorted in decreasing depth order
                  const int insert_pos =
                  std::min(gm.nmodes,
                      (depth > gm.modes[closest_mode].m ? closest_mode :
                          closest_mode + 1));

                  insert_mode(gm, insert_pos);

                  gm.modes[insert_pos].w = w_initial;
                  gm.modes[insert_pos].m = depth;
                  gm.modes[insert_pos].s = square(var_init_);

                  update_mode_index =
                  insert_pos;
                }
              }
            }

            // update all modes except already updated one
            if( gm.nmodes ) {

              //INSTRUMENT_REGION("CLEANUP");

              const double E = N / (1. + N);

              for( int i = gm.nmodes - 1; i > update_mode_index; --i ) {

                if( (gm.modes[i].w *= E) < 1. / N ) {
                  // remove obsolete mode
                  remove_mode(gm, i);
                }
              }
            }

            // fuse near modes into one
            if( gm.nmodes > 1 ) {

              for( int i = gm.nmodes - 1; i > 0; --i ) {

                const double dist =
                square(gm.modes[i].m - gm.modes[i - 1].m);

                if( dist > 25 * (gm.modes[i].s + gm.modes[i - 1].s) ) {
                  continue;
                }

                const double w1 = gm.modes[i].w;
                const double w2 = gm.modes[i - 1].w;
                const double w = 1. / (w1 + w2);

                gm.modes[i - 1].m = (gm.modes[i].m * w1 + gm.modes[i - 1].m * w2) * w;
                gm.modes[i - 1].s = (gm.modes[i].s * w1 + gm.modes[i - 1].s * w2) * w;
                gm.modes[i - 1].w = (w1 * w1 + w2 * w2) * w;

                remove_mode(gm, i);
              }
            }

            if( fg && !fgm.empty() ) {
              fgm[y][x] = 255;
            }

          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
        });
#endif

          return true;
        }

        void c_hdl_motion_filter::reduce_motion_noise(const cv::Mat1f & range_image, cv::OutputArray output_foreground_mask)
const
{
  INSTRUMENT_REGION("c_hdl_motion_filter");

  if( denoise_filter_depth_ <= 0 || denoise_filter_threshold_ < 2 ) {
    return;
  }

  if( denoise_filter_radius_.width < 1 && denoise_filter_radius_.height < 1 ) {
    return;
  }

  cv::Mat1b mask =
      output_foreground_mask.getMat();

  const int nx =
      range_image.cols;

  const int ny =
      range_image.rows;

  const float dr =
      (float) denoise_filter_depth_;

  const int dx =
      denoise_filter_radius_.height;

  const int dy =
      denoise_filter_radius_.width;

  const int T =
      denoise_filter_threshold_;

#if HAVE_TBB && !defined(Q_MOC_RUN)
  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, ny, 256),
      [&range_image, &mask, nx, ny, dx, dy, dr, T](const range & r) {
        for( int y = r.begin(); y != r.end(); ++y ) {

#else
          for( int y = 0; y < ny; ++y ) {
#endif

          const int yymin = std::max(y - dy, 0);
          const int yymax = std::min(y + dy, ny - 1);

          for( int x = 0; x < nx; ++x ) {

            if ( !mask[y][x] ) {
              continue;
            }

            const float d0 =
            range_image[y][x];

            const int xxmin =
            std::max(x - dx, 0);

            const int xxmax =
            std::min(x + dx, nx - 1);

            int nmoving = 0;

            for ( int yy = yymin; yy <= yymax; ++yy ) {
              for ( int xx = xxmin; xx <= xxmax; ++xx ) {
                if( mask[yy][xx] ) {

                  const float d =
                  range_image[yy][xx];

                  if( d > 1 && fabsf(d - d0) < dr ) {
                    ++nmoving;
                  }
                }
              }
            }

            if ( nmoving < T ) {
              mask[y][x] = 0;
            }
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
        });
#endif

        }

        bool c_hdl_motion_filter::apply(const cv::Mat1f & range_image, cv::OutputArray output_foreground_mask)
{
  if( !update(range_image, output_foreground_mask) ) {
    CF_ERROR("c_hdl_motion_filter::update() fails");
    return false;
  }

  if( enable_noise_reduction_ ) {
    reduce_motion_noise(range_image,
        output_foreground_mask);
  }

  return true;
}

int c_hdl_motion_filter::nb_frames_processed() const
{
  return nb_frames_processed_;
}

const std::vector<c_hdl_motion_filter::GM>& c_hdl_motion_filter::current_histogram() const
{
  return histogram_;
}

const c_hdl_motion_filter::GM& c_hdl_motion_filter::current_histogram(int r, int c) const
{
  return histogram_[r * histogram_size_.width + c];
}

void c_hdl_motion_filter::create_background_image(const std::vector<c_hdl_motion_filter::GM> & histogram,
    const cv::Size & histogram_size,
    double motion_threshold,
    cv::OutputArray output_image)
{
  if( output_image.needed() ) {

    if( histogram_size.empty() ) {
      output_image.release();
      return;
    }

    output_image.create(histogram_size, CV_32FC1);
    output_image.setTo(0);

    cv::Mat1f image =
        output_image.getMat();

    for( int y = 0; y < histogram_size.height; ++y ) {

      const GM * gmp =
          &histogram[y * histogram_size.width];

      for( int x = 0; x < histogram_size.width; ++x ) {

        const GM & g =
            gmp[x];

        if( g.nmodes ) {
          for( int i = g.nmodes - 1; i >= 0; --i ) {
            if( g.modes[i].w >= motion_threshold ) {
              image[y][x] = g.modes[i].m;
              break;
            }
          }
        }
      }
    }
  }
}

void c_hdl_motion_filter::create_bgn_image(const std::vector<c_hdl_motion_filter::GM> & histogram,
    const cv::Size & histogram_size,
    cv::OutputArray output_image)
{
  if( output_image.needed() ) {

    if( histogram_size.empty() ) {
      output_image.release();
      return;
    }

    output_image.create(histogram_size, CV_32FC1);
    output_image.setTo(0);

    cv::Mat1f image =
        output_image.getMat();

    for( int y = 0; y < histogram_size.height; ++y ) {

      const GM * gmp =
          &histogram[y * histogram_size.width];

      for( int x = 0; x < histogram_size.width; ++x ) {

        const GM & g =
            gmp[x];

        image[y][x] =
            g.nmodes;

      }
    }
  }
}

void c_hdl_motion_filter::create_bgw_image(const std::vector<c_hdl_motion_filter::GM> & histogram,
    const cv::Size & histogram_size,
    cv::OutputArray output_image)
{
  if( output_image.needed() ) {

    if( histogram_size.empty() ) {
      output_image.release();
      return;
    }

    output_image.create(histogram_size, CV_32FC1);
    output_image.setTo(0);

    cv::Mat1f image =
        output_image.getMat();

    for( int y = 0; y < histogram_size.height; ++y ) {

      const GM * gmp =
          &histogram[y * histogram_size.width];

      for( int x = 0; x < histogram_size.width; ++x ) {

        const GM & g =
            gmp[x];

        if( g.nmodes ) {

          image[y][x] =
              g.modes[g.nmodes - 1].w;

        }
      }
    }
  }
}

void c_hdl_motion_filter::create_bgs_image(const std::vector<c_hdl_motion_filter::GM> & histogram,
    const cv::Size & histogram_size,
    cv::OutputArray output_image)
{
  if( output_image.needed() ) {

    if( histogram_size.empty() ) {
      output_image.release();
      return;
    }

    output_image.create(histogram_size, CV_32FC1);
    output_image.setTo(0);

    cv::Mat1f image =
        output_image.getMat();

    for( int y = 0; y < histogram_size.height; ++y ) {

      const GM * gmp =
          &histogram[y * histogram_size.width];

      for( int x = 0; x < histogram_size.width; ++x ) {

        const GM & g =
            gmp[x];

        if( g.nmodes ) {

          image[y][x] =
              sqrt(g.modes[0].s);
        }

      }
    }
  }
}

