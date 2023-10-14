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

#include <core/ssprintf.h>
#include <core/debug.h>

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
  cv::Mat absdiff_image;
  cv::Mat1f best_iteration;
  cv::Mat best_cost;
  cv::Mat1b excounter;

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

  CF_DEBUG("E: x=%g y=%g", E.x, E.y);

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

    best_iteration = cv::Mat1f(reference_image_size, 0);
    best_cost = cv::Mat(reference_image_size, reference_image.depth(), cv::Scalar::all(255));
    excounter = cv::Mat1b(reference_image_size, 0);

    cv::Mat selection_mask;

    for( int iteration = 0; iteration < search_radius; ++iteration ) {

      create_scale_compression_remap(iteration, reference_image_size, epipole_location, cmap, cv::noArray(), remapped_current_mask);

      cv::remap(current_layers[l], remapped_current_image, cmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

      cv::absdiff(reference_image, remapped_current_image, absdiff_image);

      if ( block_radius > 0 ) {
        const int ksize  = 2 * block_radius + 1;
        cv::GaussianBlur(absdiff_image, absdiff_image, cv::Size(ksize, ksize ), 0, 0, cv::BORDER_REPLICATE);
      }

      if( !debug_path.empty() ) {

        std::string filename;

        filename =
            ssprintf("%s/absdiff/absdiff_image.%03d.%03d.tiff",
                debug_path.c_str(), l, iteration);

        if( !save_image(absdiff_image, filename) ) {
          CF_ERROR("save_image('%s') fails", filename.c_str());
          return false;
        }
      }

      if( absdiff_image.channels() > 1 ) {

        reduce_color_channels(absdiff_image, absdiff_image, cv::REDUCE_MIN);

        if( !debug_path.empty() ) {

          std::string filename;

          filename =
              ssprintf("%s/min_absdiff/min_absdiff_image.%03d.%03d.tiff",
                  debug_path.c_str(), l, iteration);

          if( !save_image(absdiff_image, filename) ) {
            CF_ERROR("save_image('%s') fails", filename.c_str());
            return false;
          }
        }
      }


      cv::compare(absdiff_image, best_cost, selection_mask, cv::CMP_LT);
      absdiff_image.copyTo(best_cost, selection_mask);
      best_iteration.setTo(iteration, selection_mask);
      cv::add(excounter, 1, excounter, selection_mask);
    }


    if( !debug_path.empty() ) {

      std::string filename;

      filename =
          ssprintf("%s/best/best_cost.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(best_cost, best_cost < FLT_MAX, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

      filename =
          ssprintf("%s/best/best_iteration.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(best_iteration, best_cost < FLT_MAX, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

      filename =
          ssprintf("%s/best/excounter.%03d.tiff",
              debug_path.c_str(), l);

      if( !save_image(excounter, filename) ) {
        CF_ERROR("save_image('%s') fails", filename.c_str());
        return false;
      }

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


//      filename =
//          ssprintf("%s/best/best_disp.%03d.tiff",
//              debug_path.c_str(), l);
//
//      if( !save_image(best_iteration, best_cost < FLT_MAX, filename) ) {
//        CF_ERROR("save_image('%s') fails", filename.c_str());
//        return false;
//      }
//      if ( l > 0 ) {
//        cv::resize(best_iteration, best_iteration, reference_layers[l-1].size(), 0,0, cv::INTER_NEAREST);
//        cv::multiply(best_iteration, 2, best_iteration);
//
//        filename =
//            ssprintf("%s/best/best_disp_up.%03d.tiff",
//                debug_path.c_str(), l);
//
//        if( !save_image(best_iteration, filename) ) {
//          CF_ERROR("save_image('%s') fails", filename.c_str());
//          return false;
//        }
//      }



    }

  }

  return true;
}

//
//bool morph_gradient_flow(cv::InputArray current_image, cv::InputArray current_mask,
//    cv::InputArray reference_image, cv::InputArray reference_mask,
//    int max_pyramid_level,
//    int block_radius,
//    int search_radius,
//    const cv::Point2f & E,
//    cv::Mat1f & disp,
//    cv::Mat1f & cost,
//    const std::string & debug_path)
//{
//
//  std::vector<cv::Mat> current_layers;
//  std::vector<cv::Mat> reference_layers;
//
//  if( current_image.size() != reference_image.size() ) {
//    CF_ERROR("INPUT ERROR: current and reference image sizes not equal:\n"
//        "current_image.size() = %dx%d reference_image.size() = %dx%d",
//        current_image.cols(), current_image.rows(),
//        reference_image.cols(), reference_image.rows());
//    return false;
//  }
//
//#if 0
//  cv::buildPyramid(current_image, current_layers, max_pyramid_level);
//  cv::buildPyramid(reference_image, reference_layers, max_pyramid_level);
//#else
//  build_morph_gradient_pyramid(current_image, current_mask,
//      current_layers,
//      max_pyramid_level);
//  build_morph_gradient_pyramid(reference_image, reference_mask,
//      reference_layers,
//      max_pyramid_level);
//#endif
//
//  const int nlayers =
//      current_layers.size();
//
//  if( nlayers != (int) reference_layers.size() ) {
//    CF_ERROR("APP BUG: Number of pyramid layers not match:\n"
//        "current_layers.size() = %zu reference_layers.size() = %zu",
//        current_layers.size(),
//        reference_layers.size());
//    return false;
//  }
//
//
//  disp.create(reference_layers.back().size());
//  disp.setTo(0);
//
//  CF_DEBUG("E: x=%g y=%g", E.x, E.y);
//
//  if( block_radius < 1 ) {
//    block_radius = 1;
//  }
//
//  if( search_radius < 1 ) {
//    search_radius = 7;
//  }
//
//  for( int l = nlayers - 1; l >= 0; --l ) {
//
////    const int block_radius =
////      l == nlayers - 1 ? 1 :
////          3;
//
////    const int search_radius = 5;
//////        l == nlayers - 1 ? 7 :
//////            5;
//
//    int search_forward, search_backward;
//
//    if ( l == nlayers - 1 ) {
//      search_forward = search_radius;
//      search_backward = 0;
//    }
//    else {
//      search_forward = 2;
//      search_backward = 2;
//    }
//
//
//    cv::Mat rdescs, cdescs;
//
//    compute_descriptors(reference_layers[l], rdescs,
//        block_radius);
//
//    compute_descriptors(current_layers[l], cdescs,
//        block_radius);
//
//    const int nx =
//        reference_layers[l].cols;
//
//    const int ny =
//        reference_layers[l].rows;
//
//    const cv::Point2f El(E.x / (1 << l), E.y / (1 << l));
//
//    CF_DEBUG("[%d] layer size: %dx%d El: x=%g y=%g", l, nx, ny, El.x, El.y);
//
//    match_descriptors(cdescs, rdescs,
//        disp,
//        cost,
//        El,
//        search_forward,
//        search_backward,
//        l);
//
//
//    if( !debug_path.empty() ) {
//
//      std::string filename;
//
//      filename =
//          ssprintf("%s/current_layers/current_layer.%03d.tiff",
//              debug_path.c_str(), l);
//      if( !save_image(current_layers[l], filename) ) {
//        CF_ERROR("save_image('%s') fails", filename.c_str());
//        return false;
//      }
//
//      filename =
//          ssprintf("%s/reference_layers/reference_layer.%03d.tiff",
//              debug_path.c_str(), l);
//
//      if( !save_image(reference_layers[l], filename) ) {
//        CF_ERROR("save_image('%s') fails", filename.c_str());
//        return false;
//      }
//
//      filename =
//          ssprintf("%s/disp/disp.%03d.tiff",
//              debug_path.c_str(), l);
//
//      if( !save_image(disp, filename) ) {
//        CF_ERROR("save_image('%s') fails", filename.c_str());
//        return false;
//      }
//
//      filename =
//          ssprintf("%s/cost/cost.%03d.tiff",
//              debug_path.c_str(), l);
//
//      if( !save_image(cost, filename) ) {
//        CF_ERROR("save_image('%s') fails", filename.c_str());
//        return false;
//      }
//
//    }
//
//    if ( l > 0 ) {
//      cv::resize(disp, disp, reference_layers[l-1].size(), 0,0, cv::INTER_NEAREST);
//      //cv::pyrUp(disp, disp, reference_layers[l-1].size());
//      cv::multiply(disp, cv::Scalar::all(2), disp);
//
//      if( !debug_path.empty() ) {
//
//        std::string filename;
//
//        filename =
//            ssprintf("%s/disp/disp-upscaled.%03d.tiff",
//                debug_path.c_str(), l);
//        if( !save_image(disp, filename) ) {
//          CF_ERROR("save_image('%s') fails", filename.c_str());
//          return false;
//        }
//      }
//
//    }
//  }
//
//  cv::Mat m;
//  cv::compare(reference_layers[0], 15, m, cv::CMP_GT);
//  if ( m.channels() > 1 ) {
//    reduce_color_channels(m, m, cv::REDUCE_MAX);
//  }
//
//  disp.setTo(0, ~m);
//  disp.setTo(0, disp < 0);
//
//  if( !debug_path.empty() ) {
//
//    std::string filename;
//
//    filename =
//        ssprintf("%s/disp.tiff",
//            debug_path.c_str());
//
//    if( !save_image(disp, filename) ) {
//      CF_ERROR("save_image('%s') fails", filename.c_str());
//      return false;
//    }
//
//  }
//
//  return true;
//}
