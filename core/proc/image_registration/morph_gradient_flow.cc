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
  INSTRUMENT_REGION("");
  float s = 0;
  for( int i = 0; i < desc_size; ++i ) {
    s += std::abs(cdesc[i] - rdesc[i]);
  }
  return s;
}

template<class T>
static void match_descriptors_(int x, int y, int nx, int ny,
    const cv::Point2d & E,
    const cv::Mat_<T> & cdescs,
    const cv::Mat_<T> & rdescs,
    const cv::Mat1f & coarse_disparity,
    double * output_best_match_disparity,
    double * output_best_match_cost)
{

  INSTRUMENT_REGION("");

  const int search_radius = 2;

//  const cv::Mat_<T> cdescs = _cdescs;
//  const cv::Mat_<T> rdescs = _rdescs;
  const int desc_size = rdescs.cols;
  const int block_radius = (sqrt(desc_size) - 1) / 2;

  const int coarse_x = x / 2;
  const int coarse_y = y / 2;

  const double epipole_distance =
      hypot(x - E.x, y - E.y);

  const double ce =
      (x - E.x) / epipole_distance;

  const double se =
      (y - E.y) / epipole_distance;

  int best_total_x = -1;
  int best_total_y = -1;
  float best_total_cost = FLT_MAX;

  const int coarse_xx_min =
      std::max(0, coarse_x - 1);

  const int coarse_xx_max =
      std::min(coarse_disparity.cols - 1, coarse_x + 1);

  const int coarse_yy_min =
      std::max(0, coarse_y - 1);

  const int coarse_yy_max =
      std::min(coarse_disparity.rows - 1, coarse_y + 1);

  for( int coarse_yy = coarse_yy_min; coarse_yy <= coarse_yy_max; ++coarse_yy ) {
    for( int coarse_xx = coarse_xx_min; coarse_xx <= coarse_xx_max; ++coarse_xx ) {

      if( coarse_disparity[coarse_yy][coarse_xx] < 0 ) {
        continue;
      }

      const int search_disparity_min =
          std::max(0, (int) (2 * coarse_disparity[coarse_yy][coarse_xx] - 2));

      const int search_disparity_max =
          (int) (2 * coarse_disparity[coarse_yy][coarse_xx] + 2);

      float best_cost = FLT_MAX;
      int best_current_x = -1;
      int best_current_y = -1;

      for( double current_disparity = search_disparity_min;
          current_disparity <= search_disparity_max;
          current_disparity += 1.5 ) {

        const int current_x =
            (int) ((epipole_distance + current_disparity) * ce + E.x);

        const int current_y =
            (int) ((epipole_distance + current_disparity) * se + E.y);

        if( current_x < 0 || current_x >= nx || current_y < 0 || current_y >= ny ) {
          break;
        }

        const float current_cost =
            compute_cost(cdescs[current_y * nx + current_x],
                rdescs[y * nx + x],
                desc_size);

        if( current_cost < best_cost ) {
          best_current_x = current_x;
          best_current_y = current_y;
          if( (best_cost = current_cost) <= 0 ) {
            break;
          }
        }
      }

      if( best_cost < best_total_cost ) {
        best_total_x = best_current_x;
        best_total_y = best_current_y;
        if( (best_total_cost = best_cost) <= 0 ) {
          break;
        }
      }
    }
  }

  if ( best_total_x >= 0 ) {
    *output_best_match_disparity = hypot(best_total_x - E.x, best_total_y - E.y) - epipole_distance;
    *output_best_match_cost = best_total_cost;
  }
  else {
    *output_best_match_disparity = -1;
    *output_best_match_cost = FLT_MAX;
  }
}
//
//static void match_descriptors(int x, int y, int nx, int ny,
//    const cv::Point2d & E,
//    const cv::Mat & cdescs, const cv::Mat & rdescs,
//    const cv::Mat1f & coarse_disparity,
//    double * best_match_disparity,
//    double * best_match_cost)
//{
//  switch (cdescs.depth()) {
//    case CV_8U:
//      match_descriptors_<uint8_t>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_8S:
//      match_descriptors_<int8_t>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_16U:
//      match_descriptors_<uint16_t>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_16S:
//      match_descriptors_<int16_t>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_32S:
//      match_descriptors_<int32_t>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_32F:
//      match_descriptors_<float>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//    case CV_64F:
//      match_descriptors_<double>(x, y, nx, ny, E, cdescs, rdescs, coarse_disparity, best_match_disparity, best_match_cost);
//      break;
//  }
//}

//
//static cv::Mat2f matches_to_optflow(const cv::Mat2f & matches)
//{
//  cv::Mat2f optflow(matches.size());
//
//  for ( int y = 0; y < optflow.rows; ++y ) {
//    for ( int x = 0; x < optflow.cols; ++x ) {
//      if ( matches[y][x][0] >= 0 ) {
//        optflow[y][x][0] = matches[y][x][0] - x;
//        optflow[y][x][1] = matches[y][x][1] - y;
//      }
//      else {
//        optflow[y][x][0] = -1;
//        optflow[y][x][1] = -1;
//      }
//    }
//  }
//  return optflow;
//}

template<class T>
static void update_fine_level_(const std::vector<cv::Mat1f> & coarse_disparities, int nx, int ny,
    const cv::Mat & _cdescs, const cv::Mat & _rdescs,
    const cv::Point2d & epipole_location,
    std::vector<cv::Mat1f> & fine_disparities,
    std::vector<cv::Mat1f> & fine_costs)
{
  const cv::Mat_<T> cdescs = _cdescs;
  const cv::Mat_<T> rdescs = _rdescs;

  CF_DEBUG("nx=%d ny=%d", nx, ny);

  for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
    for( int y = 0; y < ny; ++y ) {
      for( int x = 0; x < nx; ++x ) {

        const cv::Mat1f &coarse_disparity =
            coarse_disparities[i];

        double best_match_disparity = -1;
        double best_match_cost = FLT_MAX;

        match_descriptors_<T>(x, y, nx, ny,
            epipole_location,
            cdescs,
            rdescs,
            coarse_disparity,
            &best_match_disparity,
            &best_match_cost);

        if( best_match_disparity >= 0 ) {
          fine_disparities[i][y][x] = best_match_disparity;
          fine_costs[i][y][x] = best_match_cost < FLT_MAX ? best_match_cost : -1;
        }
      }
    }
  }
}

static void update_fine_level(const std::vector<cv::Mat1f> & coarse_disparities,
    int nx, int ny,
    const cv::Mat & cdescs, const cv::Mat & rdescs,
    const cv::Point2d & epipole_location,
    std::vector<cv::Mat1f> & fine_disparities,
    std::vector<cv::Mat1f> & fine_costs)
{

  switch (cdescs.depth()) {
    case CV_8U:
      update_fine_level_<uint8_t>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_8S:
      update_fine_level_<int8_t>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_16U:
      update_fine_level_<uint16_t>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_16S:
      update_fine_level_<int16_t>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_32S:
      update_fine_level_<int32_t>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_32F:
      update_fine_level_<float>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
      break;
    case CV_64F:
      update_fine_level_<double>(coarse_disparities, nx, ny, cdescs, rdescs, epipole_location, fine_disparities, fine_costs);
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
    double gradient_threshold,
    const cv::Point2f & E,
    cv::Mat1f & disp,
    cv::Mat1f & cost,
    const std::string & debug_path)
{

  INSTRUMENT_REGION("");

  std::vector<cv::Mat> current_layers;
  std::vector<cv::Mat> reference_layers;
  std::vector<cv::Mat1f> disparity_map, new_disparity_map;
  std::vector<cv::Mat1f> new_best_costs;

  cv::Mat1f final_disparity_map;
  cv::Mat1f final_best_costs;

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


    disparity_map.resize(MAX_BEST_EXTREMUMS);
    for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
      disparity_map[i].create(reference_image_size);
      disparity_map[i].setTo(cv::Scalar::all(-1));
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

          const double r = hypot(x - epipole_location.x, y - epipole_location.y);

          disparity_map[i][y][x] = r * (K - 1);
          //previous_matches[i][y][x][0] = (x - epipole_location.x) * K + epipole_location.x;
          //previous_matches[i][y][x][1] = (y - epipole_location.y) * K + epipole_location.y;
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

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        debug_image.create(best_grid.size());
        debug_image.setTo(cv::Scalar::all(-1));

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
        debug_image.setTo(cv::Scalar::all(-1));

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

//    if ( l < nlayers - 2 ) {
//      break;
//    }

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

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {


        filename =
            ssprintf("%s/previous_disparity/previous_disparity.%03d.%03d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(disparity_map[i], filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }
    }

    compute_descriptors(current_image,
        current_descriptors,
        block_radius);

    compute_descriptors(reference_image,
        reference_descriptors,
        block_radius);

    new_disparity_map.resize(MAX_BEST_EXTREMUMS);
    new_best_costs.resize(MAX_BEST_EXTREMUMS);

    for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
      new_disparity_map[i].create(reference_image.size());
      new_disparity_map[i].setTo(cv::Scalar::all(-1));
      new_best_costs[i].create(reference_image.size());
      new_best_costs[i].setTo(cv::Scalar::all(-1));
    }

    update_fine_level(disparity_map,
        reference_image.cols,
        reference_image.rows,
        current_descriptors,
        reference_descriptors,
        epipole_location,
        new_disparity_map,
        new_best_costs);


//    for( int y = 0; y < reference_image.rows; ++y ) {
//      for( int x = 0; x < reference_image.cols; ++x ) {
//        for( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
//
//          const cv::Mat1f &coarse_disparity =
//              disparity_map[i];
//
//          double best_match_disparity = -1;
//          double best_match_cost = FLT_MAX;
//
//          match_descriptors(x, y, reference_image.cols, reference_image.rows,
//              epipole_location,
//              current_descriptors,
//              reference_descriptors,
//              coarse_disparity,
//              &best_match_disparity,
//              &best_match_cost);
//
//
//          if ( best_match_disparity >= 0 ) {
//            new_disparity_map[i][y][x] = best_match_disparity;
//            new_best_costs[i][y][x] = best_match_cost < FLT_MAX ? best_match_cost : -1;
//          }
//        }
//      }
//    }

    if( !debug_path.empty() ) {

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {

        cv::Mat1f disparity_image;
        std::string filename;

        filename =
            ssprintf("%s/best_costs/best_costs.%03d.%02d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(new_best_costs[i], filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }

        new_disparity_map[i].copyTo(disparity_image);


        filename =
            ssprintf("%s/new_disparity/new_disparity.%03d.%03d.tiff",
                debug_path.c_str(), l, i);


        if( !save_image(disparity_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }

        cv::Mat mask;
        cv::compare(reference_image, cv::Scalar::all(gradient_threshold), mask, cv::CMP_LT);
        if ( mask.channels() > 1 ) {
          reduce_color_channels(mask, mask, cv::REDUCE_MAX);
        }

        disparity_image.setTo(cv::Scalar::all(-1), mask);

        filename =
            ssprintf("%s/new_disparity/new_disparity_gt.%03d.%03d.tiff",
                debug_path.c_str(), l, i);

        if( !save_image(disparity_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }
    }

    std::swap(new_disparity_map, disparity_map);
  }

  const cv::Mat & reference_layer =
      reference_layers.front();

  final_disparity_map.create(reference_layer.size());
  final_best_costs.create(reference_layer.size());

  for ( int y = 0; y < final_disparity_map.rows; ++y ) {
    for ( int x = 0; x < final_disparity_map.cols; ++x ) {

      float disparity_array[MAX_BEST_EXTREMUMS];
      float costs_array[MAX_BEST_EXTREMUMS];
      int num_hypothesis_computed = 0;

      for ( int i = 0; i < MAX_BEST_EXTREMUMS; ++i ) {
        if ( disparity_map[i][y][x] >= 0 ) {
          disparity_array[num_hypothesis_computed] = disparity_map[i][y][x];
          costs_array[num_hypothesis_computed] = new_best_costs[i][y][x];
          num_hypothesis_computed += 1;
        }
      }

      if ( num_hypothesis_computed < 1 ) {
        final_disparity_map[y][x] = -1;
        final_best_costs[y][x] = -1;
      }
      else if ( num_hypothesis_computed == 1 ) {
        final_disparity_map[y][x] = disparity_array[0];
        final_best_costs[y][x] = costs_array[0];
      }
      else {

        int best_cost_index = 0;

        for ( int i = 1; i < num_hypothesis_computed; ++i ) {
          if ( costs_array[i] < costs_array[best_cost_index] ) {
            best_cost_index = i;
          }
        }

        final_disparity_map[y][x] = disparity_array[best_cost_index];
        final_best_costs[y][x] = costs_array[best_cost_index];
      }
    }
  }

  if ( gradient_threshold > 0 ) {
    cv::Mat mask;
    cv::compare(reference_layer, cv::Scalar::all(gradient_threshold), mask, cv::CMP_LT);
    if ( mask.channels() > 1 ) {
      reduce_color_channels(mask, mask, cv::REDUCE_MAX);
    }
    final_disparity_map.setTo(cv::Scalar::all(-1), mask);
    final_best_costs.setTo(cv::Scalar::all(-1), mask);
  }

  if( !debug_path.empty() ) {

    std::string filename;

    filename =
        ssprintf("%s/final/final_disparity_map.tiff",
            debug_path.c_str());

    if( !save_image(final_disparity_map, filename) ) {
      CF_ERROR("save_image('%s') fails", filename.c_str());
      return false;
    }

    filename =
        ssprintf("%s/final/final_best_costs.tiff",
            debug_path.c_str());

    if( !save_image(final_best_costs, filename) ) {
      CF_ERROR("save_image('%s') fails", filename.c_str());
      return false;
    }

  }

  disp = final_disparity_map;
  cost = final_best_costs;

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
