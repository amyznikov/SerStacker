/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
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

#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>

#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>

#include <core/debug.h>

namespace {

}

int main(int argc, char *argv[])
{

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

//  std::string input_file_name =
//      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/2011_09_26_drive_0005_sync.03.avi";

  std::string input_file_name =
      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0009_sync/2011_09_26_drive_0009_sync.03.avi";

//  std::string input_file_name =
//      "/home/data/zenuity/000006_romeo-undistorted.avi";

  c_ffmpeg_input_source::sptr source =
      c_ffmpeg_input_source::create(input_file_name);

  if ( !source->open() ) {
    CF_ERROR("source->open() fails for '%s'", input_file_name.c_str());
    return 1;
  }

  source->seek(215); // 321

  cv::Mat images[3];

  for( int i = 0; i < 2; ++i ) {

    if( !source->read(images[i], nullptr, nullptr) ) {
      CF_ERROR("source->read(images[%d]) fails for '%s'", i, input_file_name.c_str());
      return 1;
    }

    if( images[i].channels() != 1 ) {
      cv::cvtColor(images[i], images[i],
          cv::COLOR_BGR2GRAY);
    }

  }

  source->close();

  c_eccflow f;
  cv::Mat2f rmap;
  cv::Mat2f flow;

  f.set_support_scale(3);
  f.set_min_image_size(4);
  f.set_noise_level(1e-3);
  f.set_scale_factor(0.75);
  f.set_max_iterations(3);

  f.compute(images[1], images[0], rmap,
      cv::noArray(), cv::noArray());

  cv::remap(images[1], images[2],
      rmap, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_REPLICATE);

  ecc_remap_to_optflow(rmap, flow);

  save_image(images[0], ssprintf("alpha-debug/pyramid/reference_image.tiff"));
  save_image(images[1], ssprintf("alpha-debug/pyramid/current_image.tiff"));
  save_image(images[2], ssprintf("alpha-debug/pyramid/remapped_image.tiff"));
  save_image(flow, ssprintf("alpha-debug/pyramid/optflow.flo"));


  cv::Mat fchannels[2], fmag;
  cv::split(flow, fchannels);
  save_image(fchannels[0], ssprintf("alpha-debug/pyramid/optflow_x.tiff"));
  save_image(fchannels[1], ssprintf("alpha-debug/pyramid/optflow_y.tiff"));
  cv::magnitude(fchannels[0], fchannels[1], fmag);
  save_image(fmag, ssprintf("alpha-debug/pyramid/optflow_mag.tiff"));

  cv::phase(fchannels[0], fchannels[1], fmag);
  save_image(fmag, ssprintf("alpha-debug/pyramid/optflow_phase.tiff"));

  return 0;
}


#if 0

int main(int argc, char *argv[])
{

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

//  std::string input_file_name =
//      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/2011_09_26_drive_0005_sync.03.avi";

  std::string input_file_name =
      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0009_sync/2011_09_26_drive_0009_sync.03.avi";

//  std::string input_file_name =
//      "/home/data/zenuity/000006_romeo-undistorted.avi";

  c_ffmpeg_input_source::sptr source =
      c_ffmpeg_input_source::create(input_file_name);

  if ( !source->open() ) {
    CF_ERROR("source->open() fails for '%s'", input_file_name.c_str());
    return 1;
  }

  source->seek(321);

  cv::Mat src_images[2];
  cv::Mat images[2];

  for ( int i = 0; i < 2; ++i ) {

    if ( !source->read(src_images[i], nullptr, nullptr) ) {
      CF_ERROR("source->read(images[%d]) fails for '%s'", i, input_file_name.c_str());
      return 1;
    }

    cv::cvtColor(src_images[i], images[i], cv::COLOR_BGR2GRAY);
    cv::morphologyEx(images[i], images[i], cv::MORPH_GRADIENT, cv::Mat1b(3,3,255));
    images[i].convertTo(images[i], CV_32F, 1./255, 0);
  }

  source->close();

  c_eccflow f;

  f.set_support_scale(3);
  f.set_min_image_size(4);
  f.set_noise_level(1e-3);
  f.set_scale_factor(0.75);
  f.set_update_multiplier(2);

  f.set_reference_image(images[0], cv::noArray());

  const auto & pyramid =
      f.current_pyramid();

  CF_DEBUG("H: pyramid.size()=%zu", pyramid.size());
  for( int i = 0, n = pyramid.size(); i < n; ++i ) {
    save_image(pyramid[i].reference_image, ssprintf("alpha-debug/pyramid/L%03d.reference_mg.tiff", i));
  }

  cv::Mat2f rmap;
  cv::Mat2f optflow;

  f.compute(images[1], rmap,  cv::noArray());
  for( int i = 0, n = pyramid.size(); i < n; ++i ) {
    save_image(pyramid[i].current_image, ssprintf("alpha-debug/pyramid/L%03d.current_mg.tiff", i));

    ecc_remap_to_optflow(pyramid[i].rmap, optflow);
    save_image(optflow, ssprintf("alpha-debug/pyramid/optflow.L%03d.flo", i));
  }

  ecc_remap_to_optflow(rmap, optflow);
  save_image(optflow, ssprintf("alpha-debug/pyramid/optflow.final.flo"));

  save_image(images[0], ssprintf("alpha-debug/pyramid/reference_image.tiff"));
  save_image(images[1], ssprintf("alpha-debug/pyramid/current_image.tiff"));
  cv::remap(images[1], images[1], rmap, cv::noArray(), cv::INTER_LINEAR);
  save_image(images[1], ssprintf("alpha-debug/pyramid/remapped_image.tiff"));

  cv::Mat absdiff_image, mask1, mask2, mask;

//  if ( f.use_usharp() ) {
//    cv::compare(pyramid.front().reference_mg, cv::Scalar::all(0.1), mask1, cv::CMP_GT);
//  }
//  save_image(pyramid.front().reference_mg, mask1, ssprintf("alpha-debug/pyramid/reference_mg.tiff"));
//
//  cv::remap(pyramid.front().current_mg, pyramid.front().current_mg, rmap, cv::noArray(), cv::INTER_LINEAR);
//  if ( f.use_usharp() ) {
//    cv::compare(pyramid.front().current_mg, cv::Scalar::all(0.1), mask2, cv::CMP_GT);
//  }
//  save_image(pyramid.front().current_mg, mask2, ssprintf("alpha-debug/pyramid/remapped_current_mg.tiff"));
//
//  if ( f.use_usharp() ) {
//    cv::bitwise_and(mask1, mask2, mask);
//    morphological_smooth_close(mask, mask, cv::Mat1b(3, 3, 255));
//  }

  cv::remap(src_images[1], src_images[1], rmap, cv::noArray(), cv::INTER_LINEAR);
  save_image(src_images[0], mask, ssprintf("alpha-debug/pyramid/masked_src_image0.tiff"));
  save_image(src_images[1], mask, ssprintf("alpha-debug/pyramid/masked_src_image1.tiff"));



//  std::string options;
//
//  for ( int i = 1; i < argc; ++i ) {
//    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
//      fprintf(stdout, "Usage: alpha path_to_pcap_file.pcap\n");
//      return 0;
//
//    }
//
//    if ( address.empty() ) {
//      address = argv[i];
//      continue;
//    }
//
//    fprintf(stderr, "Invalid arg: '%s'\n", argv[i]);
//    return 1;
//  }



  return 0;
}
#endif


