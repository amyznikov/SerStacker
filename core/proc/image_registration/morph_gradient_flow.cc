/*
 * morph_gradient_flow.cc
 *
 *  Created on: Oct 5, 2023
 *      Author: amyznikov
 */

#include "morph_gradient_flow.h"
#include <core/io/save_image.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/stereo/scale_sweep.h>
#include <deque>
#include <core/proc/array2d.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace  {

class cvmat_deque:
    public std::deque<cv::Mat>
{
public:
  typedef cvmat_deque this_class;
  typedef std::deque<cv::Mat> base;

  void push(const cv::Mat & v)
  {
    base::emplace_back(v);
  }

  bool pop(cv::Mat * v)
  {
    if( !empty() ) {
      *v = base::front();
      base::pop_front();
      return true;
    }

    return false;
  }
};

static constexpr int MAX_BEST_EXTREMUMS = 3;

struct c_best_extremum
{
  int num_best_extremums = 0;
  float previous_cost = FLT_MAX;
  float pre_previous_cost = FLT_MAX;
  int plato_start = -1;

  struct E {
    float I = 0;
    float C = 0;
  } e[MAX_BEST_EXTREMUMS];
};

class c_best_grid :
    public c_array2d<c_best_extremum>
{
public:
  typedef c_best_grid this_class;
  typedef c_array2d<c_best_extremum> base;

};

} // namespace

template<class T>
static void compute_descriptors_(const cv::Mat & _image, cv::Mat & _descs, const int block_radius)
{
  const int nx =
      _image.cols;

  const int ny =
      _image.rows;

  const int cn =
      _image.channels();

  cv::Mat tmp = _image;

//  cv::copyMakeBorder(_image, tmp,
//      block_radius, block_radius, block_radius, block_radius,
//      cv::BORDER_REPLICATE);

  const cv::Mat_<T> image =
      tmp;

  const int block_size =
      2 * block_radius + 1;

  const int total_block_size =
      block_size * block_size * cn;

  cv::Mat_<T> descs (nx * ny, total_block_size, (T)(0));

  for( int y = block_radius; y < ny - block_radius; ++y ) {

    for( int x = block_radius; x < nx - block_radius; ++x ) {

      T *descp = descs[y * nx + x];
      int i = 0;

      for( int yy = y - block_radius; yy <= y + block_radius; ++yy ) {
        for( int xx = x - block_radius; xx <= x + block_radius; ++xx ) {
          for( int cc = 0; cc < cn; ++cc ) {
            //descp[i++] = image[yy + 1][(xx + 1) * cn + cc];
            descp[i++] = image[yy][xx * cn + cc];
          }
        }
      }

//      if( i != total_block_size ) {
//        CF_ERROR("APP BUG: i=%d total_block_size=%d", i, total_block_size);
//        exit(1);
//      }
    }
  }

  _descs = std::move(descs);
}


static void compute_descriptors(const cv::Mat & image, cv::Mat & descs, const int block_radius)
{
  switch (image.depth()) {
    case CV_8U:
      compute_descriptors_<uint8_t>(image, descs, block_radius);
      break;
    case CV_8S:
      compute_descriptors_<int8_t>(image, descs, block_radius);
      break;
    case CV_16U:
      compute_descriptors_<uint16_t>(image, descs, block_radius);
      break;
    case CV_16S:
      compute_descriptors_<int16_t>(image, descs, block_radius);
      break;
    case CV_32S:
      compute_descriptors_<int32_t>(image, descs, block_radius);
      break;
    case CV_32F:
      compute_descriptors_<float>(image, descs, block_radius);
      break;
    case CV_64F:
      compute_descriptors_<double>(image, descs, block_radius);
      break;
  }
}


template<class T>
static inline float compute_cost(const T cdesc[], const T rdesc[], int desc_size)
{
  float s = 0;
  for( int i = 0; i < desc_size; ++i ) {
    s += std::abs(cdesc[i] - rdesc[i]);
  }
  return s;
}

template<class T>
static void match_descriptors_(int x, int y, int nx, int ny,
    const cv::Mat & _cdescs, const cv::Mat & _rdescs,
    const cv::Mat2f & coarse_matches,
    int * output_best_match_x,
    int * output_best_match_y,
    float * output_best_match_cost )
{
  const cv::Mat_<T> cdescs = _cdescs;
  const cv::Mat_<T> rdescs = _rdescs;
  const int desc_size = rdescs.cols;

  const int coarse_x = x / 2;
  const int coarse_y = y / 2;

  const int search_radius = 2;

  int best_total_match_x = -1;
  int best_total_match_y = -1;
  float best_total_cost = FLT_MAX;

  int coarse_xx_min, coarse_xx_max;
  int coarse_yy_min, coarse_yy_max;

  if( y & 1 == 0 ) { // top
    coarse_yy_min = std::max(0, coarse_y - 1);
    coarse_yy_max = coarse_y;
  }
  else { // bottom
    coarse_yy_min = coarse_y;
    coarse_yy_max = std::min(ny - 1, coarse_y + 1);
  }

  if( x & 1 == 0 ) { // left
    coarse_xx_min = std::max(0, coarse_x - 1);
    coarse_xx_max = coarse_x;
  }
  else { // right
    coarse_xx_min = coarse_x;
    coarse_xx_max = std::min(nx - 1, coarse_x + 1);
  }


  for( int coarse_yy = coarse_yy_min; coarse_yy <= coarse_yy_max; ++coarse_yy ) {
    for( int coarse_xx = coarse_xx_min; coarse_xx <= coarse_xx_max; ++coarse_xx ) {

      const int hypx = coarse_matches[coarse_yy][coarse_xx][0];
      const int hypy = coarse_matches[coarse_yy][coarse_xx][1];

      if ( hypx < 0 || hypy < 0 ) {
        continue;
      }

      const int search_xmin = std::max(0, 2 * hypx - search_radius);
      const int search_xmax = std::min(x - 1, 2 * hypx + search_radius);
      const int search_ymin = std::max(0, 2 * hypy - search_radius);
      const int search_ymax = std::min(y - 1, 2 * hypy + search_radius);

      float best_cost = FLT_MAX;
      float best_match_x = -1;
      float best_match_y = -1;

      for( int search_y = search_ymin; search_y <= search_ymax; ++search_y ) {
        for( int search_x = search_xmin; search_x <= search_xmax; ++search_x ) {

          const float current_cost =
              compute_cost(cdescs[search_y * nx + search_x],
                  rdescs[y * nx + x],
                  desc_size);

          if ( current_cost < best_cost ) {
            best_match_x = search_x;
            best_match_y = search_y;
            if ( (best_cost = current_cost) <= 0 ) {
              break;
            }
          }
        }
      }

      if ( best_cost < best_total_cost ) {
        best_total_match_x = best_match_x;
        best_total_match_y = best_match_y;
        if ( (best_total_cost = best_cost) <= 0 ) {
          break;
        }
      }
    }
  }


  * output_best_match_x = best_total_match_x;
  * output_best_match_y = best_total_match_y;
  * output_best_match_cost =  best_total_cost;
}

static void match_descriptors(int x, int y, int nx, int ny,
    const cv::Mat & cdescs, const cv::Mat & rdescs,
    const cv::Mat2f & coarse_matches,
    int * best_match_x,
    int * best_match_y,
    float * best_cost)
{
  switch (cdescs.depth()) {
    case CV_8U:
      match_descriptors_<uint8_t>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_8S:
      match_descriptors_<int8_t>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_16U:
      match_descriptors_<uint16_t>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_16S:
      match_descriptors_<int16_t>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_32S:
      match_descriptors_<int32_t>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_32F:
      match_descriptors_<float>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
    case CV_64F:
      match_descriptors_<double>(x, y, nx, ny, cdescs, rdescs, coarse_matches, best_match_x, best_match_y, best_cost);
      break;
  }
}


bool morph_gradient_flow(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    int max_pyramid_level,
    int block_radius,
    int search_radius,
    double alpha,
    double beta,
    const cv::Point2f & E,
    cv::Mat1f & disp,
    cv::Mat1f & cost,
    const std::string & debug_path)
{

  INSTRUMENT_REGION("");

  std::vector<cv::Mat> current_layers;
  std::vector<cv::Mat> reference_layers;
  std::vector<cv::Mat2f> previous_matches, new_matches;
  std::vector<cv::Mat1f> new_best_costs;
  cv::Mat2f cmap;
  cv::Mat remapped_current_image;
  cv::Mat remapped_current_mask;
  cv::Mat current_descriptors;
  cv::Mat reference_descriptors;

  cvmat_deque cost_images;
  c_best_grid best_grid;

  if( current_image.size() != reference_image.size() ) {
    CF_ERROR("INPUT ERROR: current and reference image sizes not equal:\n"
        "current_image.size() = %dx%d reference_image.size() = %dx%d",
        current_image.cols(), current_image.rows(),
        reference_image.cols(), reference_image.rows());
    return false;
  }

#if 0
  cv::buildPyramid(current_image, current_layers, max_pyramid_level);
  cv::buildPyramid(reference_image, reference_layers, max_pyramid_level);
#else
  build_morph_gradient_pyramid(current_image, current_mask,
      current_layers,
      max_pyramid_level);
  build_morph_gradient_pyramid(reference_image, reference_mask,
      reference_layers,
      max_pyramid_level);
#endif

  const int nlayers =
      current_layers.size();

  if( nlayers != (int) reference_layers.size() ) {
    CF_ERROR("APP BUG: Number of pyramid layers not match:\n"
        "current_layers.size() = %zu reference_layers.size() = %zu",
        current_layers.size(),
        reference_layers.size());
    return false;
  }


  disp.create(reference_layers.back().size());
  disp.setTo(0);

  CF_DEBUG("E: x=%g y=%g alpha=%g beta=%g", E.x, E.y, alpha, beta);

  if( search_radius < 0 ) {
    const cv::Size size = reference_layers.back().size();
    search_radius = std::max(size.width, size.height) / 3;
  }

  for( int l = nlayers - 1; l >= 0; --l ) {

    if ( l < nlayers - 1 ) {
      break;
    }

    INSTRUMENT_REGION("BODY");

    const cv::Mat & current_image =
        current_layers[l];

    const cv::Mat & reference_image =
        reference_layers[l];

    const cv::Size reference_image_size =
        reference_image.size();

    const cv::Point2d epipole_location(E.x / (1 << l), E.y / (1 << l));


    if( !debug_path.empty() ) {

      std::string filename;

      filename =
          ssprintf("%s/current_layers/current_layer.%03d.tiff",
              debug_path.c_str(), l);
      if( !save_image(current_image, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

      filename =
          ssprintf("%s/reference_layers/reference_layer.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(reference_image, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

    }

    best_grid.create(reference_image_size);

    ////////////////

    for( int iteration = 0; iteration < search_radius; ++iteration ) {

      create_scale_compression_remap(iteration,
          reference_image_size,
          epipole_location,
          cmap,
          cv::noArray(),
          remapped_current_mask);

      cv::remap(current_layers[l], remapped_current_image,
          cmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REPLICATE);

      ////////
      cv::Mat current_cost_image;
      if( cost_images.size() > 2 ) {
        cost_images.pop(&current_cost_image);
      }
      cv::absdiff(reference_image, remapped_current_image,
          current_cost_image);
      cost_images.push(current_cost_image);
      ////////

      if ( block_radius > 0 ) {

        const int ksize  = 2 * block_radius + 1;

        cv::GaussianBlur(current_cost_image, current_cost_image,
            cv::Size(ksize, ksize ), 0, 0,
            cv::BORDER_REPLICATE);
      }

      if( !debug_path.empty() ) {

        std::string filename;

        filename =
            ssprintf("%s/costs/current_cost_image.%03d.%03d.tiff",
                debug_path.c_str(), l, iteration);

        if( !save_image(current_cost_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }

      if( current_cost_image.channels() > 1 ) {

        reduce_color_channels(current_cost_image, current_cost_image, cv::REDUCE_MIN);

        if( !debug_path.empty() ) {

          std::string filename;

          filename =
              ssprintf("%s/min_absdiff/min_absdiff_image.%03d.%03d.tiff",
                  debug_path.c_str(), l, iteration);

          if( !save_image(current_cost_image, filename) ) {
            CF_ERROR("save_image('%s') fails", filename.c_str());
            return false;
          }
        }
      }

      //////////////
      switch (cost_images.size()) {
        case 1: {

          const cv::Mat1f current_costs =
              current_cost_image;

          for( int y = 0; y < current_costs.rows; ++y ) {
            for( int x = 0; x < current_costs.cols; ++x ) {
              best_grid[y][x].previous_cost = current_costs[y][x];
            }
          }

          break;
        }
        case 2: {

          const cv::Mat1f current_costs =
              current_cost_image;

          for( int y = 0; y < current_costs.rows; ++y ) {
            for( int x = 0; x < current_costs.cols; ++x ) {

              c_best_extremum & pix =
                  best_grid[y][x];

              const float current_cost =
                  current_costs[y][x];

              if( current_cost < pix.previous_cost ) {
                // do nothing
              }
              else if( current_cost == pix.previous_cost ) {
                pix.plato_start = iteration - 1;
              }
              else { // current_cost > pix.previous_cost
                // do nothing
                const float current_plato_center = 0;

                const float current_plato_cost =
                    alpha * current_plato_center + beta * pix.previous_cost;

                pix.e[pix.num_best_extremums].C = current_plato_cost;
                pix.e[pix.num_best_extremums].I = current_plato_center;
                ++pix.num_best_extremums;
              }

              pix.pre_previous_cost = pix.previous_cost;
              pix.previous_cost = current_cost;
            }
          }

          break;
        }

        default: {
          const cv::Mat1f current_costs =
              current_cost_image;

          for( int y = 0; y < current_costs.rows; ++y ) {
            for( int x = 0; x < current_costs.cols; ++x ) {

              c_best_extremum & pix =
                  best_grid[y][x];

              const float current_cost =
                  current_costs[y][x];

              if( current_cost < pix.previous_cost ) {
                pix.plato_start = -1; // reset a plato if any
              }
              else if( current_cost == pix.previous_cost ) {
                if( pix.plato_start < 0 && pix.previous_cost < pix.pre_previous_cost ) {
                  pix.plato_start = iteration - 1;
                }
              }
              else { // current_cost > pix.previous_cost

                if( pix.plato_start < 0 && pix.previous_cost < pix.pre_previous_cost ) {
                  pix.plato_start = iteration - 1;
                }


                if ( pix.plato_start >= 0 ) {
                  // exit from a plato

                  const float current_plato_center =
                      pix.plato_start > 0 ?
                          (pix.plato_start + iteration - 1) / 2 :
                          0;

                  const float current_plato_cost =
                      alpha * current_plato_center + beta * pix.previous_cost;

                  pix.plato_start = -1;

                  if ( pix.num_best_extremums < MAX_BEST_EXTREMUMS ) {
                    pix.e[pix.num_best_extremums].C = current_plato_cost;
                    pix.e[pix.num_best_extremums].I = current_plato_center;
                    ++pix.num_best_extremums;
                  }
                  else {

                    int worst_index = 0;
                    float worst_cost = pix.e[0].C;

                    for( int i = 1; i < MAX_BEST_EXTREMUMS; ++i ) {
                      if( pix.e[i].C > worst_cost ) {
                        worst_cost = pix.e[i].C;
                        worst_index = i;
                      }
                    }

                    if( current_plato_cost < worst_cost ) {
                      pix.e[worst_index].C = current_plato_cost;
                      pix.e[worst_index].I = current_plato_center;
                    }
                  }
                }
              }

              pix.pre_previous_cost = pix.previous_cost;
              pix.previous_cost = current_cost;
            }
          }

          break;
        }
      }

      //////////////
    }

    previous_matches.resize(MAX_BEST_EXTREMUMS);
    for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
      previous_matches[i].create(reference_image_size);
      previous_matches[i].setTo(-1);
    }

    const double max_epipole_distance =
        std::max( { std::abs(epipole_location.x), std::abs(epipole_location.x - reference_image_size.width - 1),
            abs(epipole_location.y), abs(epipole_location.y - reference_image_size.height - 1) });

    for( int y = 0; y < reference_image_size.height; ++y ) {
      for( int x = 0; x < reference_image_size.width; ++x ) {

        c_best_extremum &pix =
            best_grid[y][x];

        for( int i = 0; i < pix.num_best_extremums; ++i ) {

          const double K =
              max_epipole_distance / (max_epipole_distance - pix.e[i].I);

          previous_matches[i][y][x][0] = (x - epipole_location.x) * K + epipole_location.x;
          previous_matches[i][y][x][1] = (y - epipole_location.y) * K + epipole_location.y;
        }
      }
    }

    ////////////////


    if( !debug_path.empty() ) {

      cv::Mat1f debug_image;
      std::string filename;

      debug_image.create(best_grid.size());
      for ( int y = 0; y < debug_image.rows; ++y ) {
        for ( int x = 0; x < debug_image.cols; ++x ) {
          debug_image[y][x] = best_grid[y][x].num_best_extremums;
        }
      }

      filename =
          ssprintf("%s/num_best_extremums/num_best_extremums.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(debug_image, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

      for( int y = 0; y < reference_image_size.height; ++y ) {
        for( int x = 0; x < reference_image_size.width; ++x ) {

          c_best_extremum &pix =
              best_grid[y][x];

          if( pix.num_best_extremums > 1 ) {
            std::sort(pix.e, pix.e + pix.num_best_extremums,
                [](const auto & prev, const auto & next) {
                  return prev.C < next.C;
                });
          }
        }
      }

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        debug_image.create(best_grid.size());
        debug_image.setTo(-1);

        for ( int y = 0; y < debug_image.rows; ++y ) {
          for ( int x = 0; x < debug_image.cols; ++x ) {

            const c_best_extremum & pix =
                best_grid[y][x];

            if ( i < pix.num_best_extremums ) {
              debug_image[y][x] = pix.e[i].C;
            }
          }
        }

        filename =
            ssprintf("%s/eC/eC.%03d.%02d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(debug_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        debug_image.create(best_grid.size());
        debug_image.setTo(-1);

        for ( int y = 0; y < debug_image.rows; ++y ) {
          for ( int x = 0; x < debug_image.cols; ++x ) {

            const c_best_extremum & pix =
                best_grid[y][x];

            if ( i < pix.num_best_extremums ) {
              debug_image[y][x] = pix.e[i].I;
            }
          }
        }

        filename =
            ssprintf("%s/eI/eI.%03d.%02d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(debug_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }
    }
  }
  ///////////////////

  for( int l = nlayers - 2; l >= 0; --l ) {

    const cv::Mat & current_image =
        current_layers[l];

    const cv::Mat & reference_image =
        reference_layers[l];

    const cv::Size reference_image_size =
        reference_image.size();

    if( !debug_path.empty() ) {

      std::string filename;

      filename =
          ssprintf("%s/current_layers/current_layer.%03d.tiff",
              debug_path.c_str(), l);
      if( !save_image(current_image, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

      filename =
          ssprintf("%s/reference_layers/reference_layer.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(reference_image, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }
    }

    compute_descriptors(current_image,
        current_descriptors,
        block_radius);

    compute_descriptors(reference_image,
        reference_descriptors,
        block_radius);

    new_matches.resize(MAX_BEST_EXTREMUMS);
    new_best_costs.resize(MAX_BEST_EXTREMUMS);
    for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
      new_matches[i].create(reference_image.size());
      new_matches[i].setTo(-1);
      new_best_costs[i].create(reference_image.size());
      new_best_costs[i].setTo(-1);
    }

    for( int y = 0; y < reference_image.rows; ++y ) {
      for( int x = 0; x < reference_image.cols; ++x ) {
        for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

          const cv::Mat2f &coarse_matches =
              previous_matches[i];

          int best_match_x = -1;
          int best_match_y = -1;
          float best_cost = FLT_MAX;

          match_descriptors(x, y, reference_image.cols, reference_image.rows,
              current_descriptors,
              reference_descriptors,
              coarse_matches,
              &best_match_x,
              &best_match_y,
              &best_cost);


          if ( best_match_x >= 0 ) {

            new_matches[i][y][x][0] = best_match_x;
            new_matches[i][y][x][1] = best_match_y;
            new_best_costs[i][y][x] = best_cost < FLT_MAX ? best_cost : -1;
          }
        }
      }
    }

    if( !debug_path.empty() ) {

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        cv::Mat2f optflow_image;
        std::string filename;

        filename =
            ssprintf("%s/best_costs/best_costs.%03d.%02d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(new_best_costs[i], filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }


        optflow_image.create(new_matches[i].size());
        optflow_image.setTo(cv::Scalar::all(-1));

        for( int y = 0; y < reference_image.rows; ++y ) {
          for( int x = 0; x < reference_image.cols; ++x ) {
            if ( new_matches[i][y][x][0] >= 0 ) {
              optflow_image[y][x][0] = new_matches[i][y][x][0] - x;
              optflow_image[y][x][1] = new_matches[i][y][x][1] - y;
            }
          }
        }

        filename =
            ssprintf("%s/optflow/optflow.%03d.%02d.flo",
                debug_path.c_str(), l, i);

        if( !save_image(optflow_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }

    }

    std::swap(new_matches, previous_matches);
  }


  return true;
}

#if 0
#if 1
    const double max_epipole_distance =
           std::max( { std::abs(epipole_location.x), std::abs(epipole_location.x - reference_image_size.width - 1),
               abs(epipole_location.y), abs(epipole_location.y - reference_image_size.height - 1) });

    for( int y = 0; y < reference_image_size.height; ++y ) {
      for( int x = 0; x < reference_image_size.width; ++x ) {

        c_best_extremum &pix =
            best_grid[y][x];

        for( int i = 0; i < pix.num_best_extremums; ++i ) {

          const double K =
              max_epipole_distance / (max_epipole_distance - pix.e[i].I);

          // cmap[y][x][0] = (x - epipole_location.x) * K + epipole_location.x;
          // cmap[y][x][1] = (y - epipole_location.y) * K + epipole_location.y;
        }
      }
    }
#endif



    template<class T>
    static void match_descriptors_(const cv::Mat & _cdescs, const cv::Mat & _rdescs,
        cv::Mat1f & disp, cv::Mat1f & cost, const cv::Point2f & E,
        const int search_forward, const int search_backward,
        int lvl )
    {
      const int nx =
          disp.cols;

      const int ny =
          disp.rows;

      const cv::Mat_<T> cdescs =
          _cdescs;

      const cv::Mat_<T> rdescs =
          _rdescs;

      const int block_size =
          rdescs.cols;

      const int block_radius =
          (sqrt(block_size) - 1) / 2;

      cost.create(disp.size());
      cost.setTo(0);

      constexpr int RX0 = 30;
      constexpr int RY0 = 25;


      for( int ry = block_radius; ry < ny - block_radius; ++ry ) {
        for( int rx = block_radius; rx < nx - block_radius; ++rx ) {

          const float rex = rx - E.x;
          const float rey = ry - E.y;
          const float rr = std::sqrt(rex * rex + rey * rey);
          const float sin = rey / rr;
          const float cos = rex / rr;
          const T * rdesc = rdescs[ry * nx + rx];


          const float cermin = std::max(0.f, rr + disp[ry][rx] - search_backward);
          const float cermax = std::max(0.f, rr + disp[ry][rx] + search_forward);
          float best_cost = FLT_MAX;

          for( float cr = cermin; cr <= cermax; cr += 1 ) {

            const int cx = (int) cvRound( (E.x + cr * cos) );
            const int cy = (int) cvRound( (E.y + cr * sin) );

            if( cx <= 0 || cx >= nx || cy <= 0 || cy >= ny ) {
              break;
            }

            const float ccost =
                compute_cost(cdescs[cy * nx + cx],
                    rdesc,
                    block_size);

            if ( lvl == 3 && rx == RX0 && ry == RY0 ) {
              CF_DEBUG("rx=%d ry=%d rr=%g cr=%g d=%g cx=%d cy=%d ccost=%g best=%g",
                  rx, ry, rr, cr, cr - rr, cx, cy, ccost,best_cost);
            }

            if ( ccost < best_cost ) {
              disp[ry][rx] = cr - rr;
              cost[ry][rx] = ccost;
              if ( (best_cost = ccost) <= 0 ) {
                break;
              }
            }
          }

        }
      }
    }


    static void match_descriptors(const cv::Mat & cdescs, const cv::Mat & rdescs,
        cv::Mat1f & disp, cv::Mat1f & cost, const cv::Point2f & E,
        const int search_forward, const int search_backward,
        int lvl )
    {
      switch (cdescs.depth()) {
        case CV_8U:
          match_descriptors_<uint8_t>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_8S:
          match_descriptors_<int8_t>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_16U:
          match_descriptors_<uint16_t>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_16S:
          match_descriptors_<int16_t>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_32S:
          match_descriptors_<int32_t>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_32F:
          match_descriptors_<float>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
        case CV_64F:
          match_descriptors_<double>(cdescs, rdescs, disp, cost, E, search_forward, search_backward, lvl);
          break;
      }
    }


#endif
