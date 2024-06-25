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

#include <core/proc/image_registration/c_ecclm.h>

#include <core/debug.h>

namespace {

void ecclm_normalize(cv::Mat1f & image, cv::Mat1b & mask)
{
  cv::Mat m, s;

  const double eps =
      100;

  const double sigma =
      15;

  const int ksize =
      2 * ((int) (sigma * 3)) + 1;

  cv::Mat G =
      cv::getGaussianKernel(ksize, sigma,
          CV_32F);

  cv::sepFilter2D(image, m, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(image, m, m, cv::noArray(), CV_32F);

  cv::sepFilter2D(cv::abs(m), s, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::divide(m, s, image);
}

}



int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  const std::string input_filenames[2] = {
      "/home/data/ecclm/F1.png",
      "/home/data/ecclm/F2.png",
  };

  cv::Mat input_images[2];

  for ( int i = 0; i < 2; ++i ) {

    if ( !load_image(input_filenames[i], input_images[i]) ) {
      CF_ERROR("load_image(%s) fails", input_filenames[i].c_str());
      return 1;
    }

    //cv::morphologyEx(input_image, input_image, cv::MORPH_GRADIENT, cv::Mat1b(5,5, 255));
  }


  cv::Size size(input_images[0].size());

  cv::Mat reference_image,  current_image;
  cv::Mat1b reference_mask, current_mask;

  cv::cvtColor(input_images[0], reference_image, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(reference_image, reference_image, cv::Size(), 2, 2);

  cv::cvtColor(input_images[1], current_image, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(current_image, current_image, cv::Size(), 2, 2);


  if( !save_image(reference_image, reference_mask, "debug_ecclm/reference_image.tiff") ) {
    CF_ERROR("save_image(reference_image) fails");
    return 1;
  }

  if( !save_image(current_image, current_mask, "debug_ecclm/current_image.tiff") ) {
    CF_ERROR("save_image(current_image) fails");
    return 1;
  }


  cv::Mat1f reference_image_normalized, current_image_normalized;
  cv::Mat1b reference_mask_normalized, current_mask_normalized;

  ecc_convert_input_image(reference_image, reference_mask,
      reference_image_normalized, reference_mask_normalized);

  ecc_convert_input_image(current_image, current_mask,
      current_image_normalized, current_mask_normalized);

  ecclm_normalize(reference_image_normalized, reference_mask_normalized);
  ecclm_normalize(current_image_normalized, current_mask_normalized);

  if( !save_image(reference_image_normalized, reference_mask_normalized, "debug_ecclm/reference_image_normalized.tiff") ) {
    CF_ERROR("save_image(reference_image_normalized) fails");
    return 1;
  }

  if( !save_image(current_image_normalized, current_mask_normalized, "debug_ecclm/current_image__normalized.tiff") ) {
    CF_ERROR("save_image(current_image_normalized) fails");
    return 1;
  }


  //c_translation_image_transform transform;
  c_affine_image_transform transform;
  //c_homography_image_transform transform;

  c_ecch ecclmp(&transform);

  //transform.set_translation(cv::Vec2f(0,0));
  //transform.set_matrix(cv::Matx23d::eye());
  //transform.set_matrix(cv::Matx33d::eye());
  //transform.set_matrix(cv::Matx26f::eye());

  ecclmp.set_epsx(1e-3);
  ecclmp.set_reference_image(reference_image_normalized, reference_mask_normalized);

  //  transform.set_matrix(cv::Matx23f(
  //      1, 0, -5,
  //      0, 1, 40));

  if ( true ) {
    for ( int i = 0; i < 1; ++i ) {
      INSTRUMENT_REGION("ecclmp.align");
      //transform.reset();


      ecclmp.align(current_image_normalized, current_mask_normalized);
    }
  }

//  cv::Matx33d A =
//      transform.matrix();

//  cv::Matx26f A =
//      transform.matrix();

//  cv::Matx23d A =
//      transform.matrix();

  cv::Vec2f T =
      transform.translation();

  CF_DEBUG("H:\n"
      "T= {\n"
      "  %+g %+g\n"
      "}\n"
      "\n",
      T(0), T(01));

 // cv::Matx23d Ainv;
  //  cv::invertAffineTransform(A, Ainv);
  //  CF_DEBUG("H:\n"
  //      "A= {\n"
  //      "  %+g %+g %+g\n"
  //      "  %+g %+g %+g\n"
  //      "}\n"
  //      "Ainv = {\n"
  //      "  %+g %+g %+g\n"
  //      "  %+g %+g %+g\n"
  //      "}\n"
  //      "\n",
  //      A(0, 0), A(0, 1), A(0, 2),
  //      A(1, 0), A(1, 1), A(1, 2),
  //      Ainv(0, 0), Ainv(0, 1), Ainv(0, 2),
  //      Ainv(1, 0), Ainv(1, 1), Ainv(1, 2)
  //      );

//  cv::Matx33d Ainv = A.inv();
//
//  CF_DEBUG("H:\n"
//      "A= {\n"
//      "  %+g %+g %+g\n"
//      "  %+g %+g %+g\n"
//      "  %+g %+g %+g\n"
//      "}\n"
//      "Ainv = {\n"
//      "  %+g %+g %+g\n"
//      "  %+g %+g %+g\n"
//      "  %+g %+g %+g\n"
//      "}\n"
//      "\n",
//      A(0, 0), A(0, 1), A(0, 2),
//      A(1, 0), A(1, 1), A(1, 2),
//      A(2, 0), A(2, 1), A(2, 2),
//      Ainv(0, 0), Ainv(0, 1), Ainv(0, 2),
//      Ainv(1, 0), Ainv(1, 1), Ainv(1, 2),
//      Ainv(2, 0), Ainv(2, 1), Ainv(2, 2)
//    );


//  CF_DEBUG("H:\n"
//      "A= {\n"
//      "  %+g %+g %+g %+g %+g %+g\n"
//      "  %+g %+g %+g %+g %+g %+g\n"
//      "}\n"
//      "\n",
//      A(0, 0), A(0, 1), A(0, 2), A(0, 3), A(0, 4), A(0, 5),
//      A(1, 0), A(1, 1), A(1, 2), A(1, 3), A(1, 4), A(1, 5));


  transform.remap(input_images[1], current_mask, size,
      input_images[1], current_mask);

  if( !save_image(input_images[0], current_mask, "debug_ecclm/input_reference_image.tiff") ) {
    CF_ERROR("save_image(input_reference_image) fails");
    return 1;
  }

  if( !save_image(input_images[1], current_mask, "debug_ecclm/input_current_image_remapped.tiff") ) {
    CF_ERROR("save_image(input_current_image_remapped) fails");
    return 1;
  }

  return 0;
}


//
//int main(int argc, char *argv[])
//{
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//  const std::string input_filename = "/home/data/photo_2022-04-09_19-36-23.jpg";
//
//  cv::Mat input_image;
//
//  if ( !load_image(input_filename, input_image) ) {
//    CF_ERROR("load_image() fails");
//    return 1;
//  }
//
//  cv::cvtColor(input_image, input_image, cv::COLOR_BGR2GRAY);
//
//  CF_DEBUG("H");
//
//  cv::Size size(input_image.size());
//  cv::Vec2d T(-3, 2);
//
//  c_ecclm_translation model;
//
//
//  cv::Mat1f reference_image, current_image;
//  cv::Mat1b current_mask;
//
//  input_image.convertTo(reference_image,
//      reference_image.depth());
//
//  cv::GaussianBlur(reference_image, reference_image, cv::Size(), 1, 1);
//
//
//  c_ecclm_translation::remap_image(size, T,
//      reference_image, cv::Mat1b(),
//      current_image, current_mask);
//
//  if( !save_image(reference_image, "debug_ecclm/reference_image.tiff") ) {
//    CF_ERROR("save_image(reference_image) fails");
//    return 1;
//  }
//
//  CF_DEBUG("H");
//  if( !save_image(current_image, current_mask, "debug_ecclm/current_image.tiff") ) {
//    CF_ERROR("save_image(current_image) fails");
//    return 1;
//  }
//
//
//  CF_DEBUG("H");
//  c_ecclm ecclm(&model);
//
//  CF_DEBUG("H");
//  ecclm.set_reference_image(reference_image);
//  CF_DEBUG("H");
//  ecclm.align_to_reference(current_image, current_mask);
//
//  T = model.translation();
//
//  CF_DEBUG("H: T= {%+g %+g}", T(0), T(1));
//
//  c_ecclm_translation::remap_image(size, T,
//      current_image, current_mask,
//      current_image, current_mask);
//
//  if( !save_image(current_image, current_mask, "debug_ecclm/current_image_remapped_back.tiff") ) {
//    CF_ERROR("save_image(current_image_image_remapped_) fails");
//    return 1;
//  }
//
//  return 0;
//}
//



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

  c_eccflow2 f;

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


