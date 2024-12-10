/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
//#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>

#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>
#include <core/proc/fit_exponential.h>
#include <core/proc/fit_decreasing_exponent.h>
#include <core/proc/extract_channel.h>

#include <limits>
#include <core/debug.h>

namespace {

//
//class c_place
//{
//public:
//
//  // ?????????
//
//};
//
//
//int recognise_place_using_lgd_algorithm(const c_place & unknown_place, const std::vector<c_place> & database_of_known_places )
//{
//
//  int index_of_recognozed_place = 20;
//
//  return index_of_recognozed_place;
//}
//
//
//void test()
//{
//  std::vector<c_place> database_of_known_places;
//
//  while ( 42 ) {
//
//    c_place new_place = get_current_place();
//
//    int index = recognise_place_using_lgd_algorithm( new_place, database_of_known_places);
//    if ( index > 0 ) {
//      // recognized !!!!
//    }
//    else {
//      // NOT recognized !!!!
//      database_of_known_places.emplace_back(new_place);
//    }
//
//
//  }
//
//  /*
//
//  coarse       middle               fine
//  GPS     / Global Descriptors / Local Descriptors
//
//
//  GPS Positions
//      | ---------------- Midle
//      | ---------------- Midle
//      | ---------------- Midle
//      | ---------------- Midle
//                           |-------------------- Fine
//                           |-------------------- Fine
//                           |-------------------- Fine      V
//      |  ...
//      |
//      | ---------------- Midle
//                           |-------------------- Fine      V
//                           |-------------------- Fine
//                           |-------------------- Fine
//                           |-------------------- Fine
//                           |-------------------- Fine
//                           |-------------------- Fine
//                           |  .......
//                           |
//                           |-------------------- Fine
//                                                   |  Point Cloud     X,Y,Z
//
//
//
//  */
//
//
//}

}



int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  typedef uint16_t datatype;


  datatype databits = std::numeric_limits<datatype>::digits;
  datatype datamask = (datatype)(-1);
  datatype masku = (datatype )(datamask << (databits/2));
  datatype maskl = ~(datatype )(datamask << (databits/2));



  printf("digits=%d is_integer=%d databits=%u masku=0x%0X maskl=0x%0X\n",
      std::numeric_limits<datatype>::digits,
      std::numeric_limits<datatype>::is_integer,
      databits,
      masku,
      maskl);


  return 0;
}

