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

static constexpr int MAX_BEST_EXTREMUMS = 10;

struct c_best_extremum
{
  int num_best_extremums = 0;
  float previous_cost = FLT_MAX;
  float pre_previous_cost = FLT_MAX;
  float eI[MAX_BEST_EXTREMUMS] = {0};
  float eC[MAX_BEST_EXTREMUMS] = {0};
  int plato_start = -1;
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

  std::vector<cv::Mat> current_layers;
  std::vector<cv::Mat> reference_layers;
  cv::Mat2f cmap;
  cv::Mat remapped_current_image;
  cv::Mat remapped_current_mask;

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

                pix.eC[pix.num_best_extremums] = current_plato_cost;
                pix.eI[pix.num_best_extremums] = current_plato_center;
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
                    pix.eC[pix.num_best_extremums] = current_plato_cost;
                    pix.eI[pix.num_best_extremums] = current_plato_center;
                    ++pix.num_best_extremums;
                  }
                  else {

                    int worst_index = 0;
                    float worst_cost = pix.eC[0];

                    for( int i = 1; i < MAX_BEST_EXTREMUMS; ++i ) {
                      if( pix.eC[i] > worst_cost ) {
                        worst_cost = pix.eC[i];
                        worst_index = i;
                      }
                    }

                    if( current_plato_cost < worst_cost ) {
                      pix.eC[worst_index] = current_plato_cost;
                      pix.eI[worst_index] = current_plato_center;
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

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        debug_image.create(best_grid.size());
        debug_image.setTo(-1);

        for ( int y = 0; y < debug_image.rows; ++y ) {
          for ( int x = 0; x < debug_image.cols; ++x ) {

            const c_best_extremum & pix =
                best_grid[y][x];

            if ( i < pix.num_best_extremums ) {
              debug_image[y][x] = pix.eC[i];
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
    }


#if 0
    const double max_epipole_distance =
           std::max( { std::abs(epipole_location.x), std::abs(epipole_location.x - reference_image_size.width - 1),
               abs(epipole_location.y), abs(epipole_location.y - reference_image_size.height - 1) });


       cmap.create(reference_image_size);
       cmap.setTo(-1);

       for ( int y = 0; y < best_iteration.rows; ++y ) {
         for ( int x = 0; x < best_iteration.cols; ++x ) {

           const double K =
               max_epipole_distance / (max_epipole_distance - best_iteration[y][x]);

           cmap[y][x][0] = (x - epipole_location.x) * K + epipole_location.x;
           cmap[y][x][1] = (y - epipole_location.y) * K + epipole_location.y;
         }
       }

       cv::remap(current_image, remapped_current_image, cmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
       filename =
           ssprintf("%s/remapped_current_image/remapped_current_image.%03d.tiff",
               debug_path.c_str(), l);

       if( !save_image(remapped_current_image, filename) ) {
         CF_ERROR("save_image('%s') fails", filename.c_str());
         return false;
       }
#endif
  }



  return true;
}
