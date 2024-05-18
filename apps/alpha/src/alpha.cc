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

//
//void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
//{
//  static thread_local cv::Mat Kx, Ky;
//  if( Kx.empty() ) {
//    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
//    Kx *= M_SQRT2;
//    Ky *= M_SQRT2;
//  }
//
//  if( ddepth < 0 ) {
//    ddepth = std::max(src.depth(), CV_32F);
//  }
//
//  cv::sepFilter2D(src, gx, -1, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  cv::sepFilter2D(src, gy, -1, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//}
//
//
//bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
//{
//  const cv::Size inputSize = image.size();
//
//  if( inputSize != dstSize ) {
//
//    std::vector<cv::Size> sizes;
//
//    sizes.emplace_back(dstSize);
//
//    while (42) {
//
//      const cv::Size nextSize((sizes.back().width + 1) / 2,
//          (sizes.back().height + 1) / 2);
//
//      if( nextSize == inputSize ) {
//        break;
//      }
//
//      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
//        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
//            nextSize.width, nextSize.height,
//            inputSize.width, inputSize.height);
//        return false;
//      }
//
//      sizes.emplace_back(nextSize);
//    }
//
//    for( int i = sizes.size() - 1; i >= 0; --i ) {
//      cv::pyrUp(image, image, sizes[i]);
//    }
//  }
//
//  return true;
//}
//

static void correlate(const cv::Mat & src1, const cv::Mat & src2, cv::Mat & dst)
{
  cv::Mat m1, m2;
  cv::Mat s1, s2, cov, norm;


  static const cv::Mat1f G =
      cv::getGaussianKernel(7, 0, CV_32F);

  cv::sepFilter2D(src1, m1, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src2, m2, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::subtract(src1, m1, s1, cv::noArray(), CV_32F);
  cv::subtract(src2, m2, s2, cv::noArray(), CV_32F);
  cv::sepFilter2D(s1.mul(s2), cov, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::sepFilter2D(s1.mul(s1), s1, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(s2.mul(s2), s2, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sqrt(s1.mul(s2), norm);
  cv::add(norm, cv::Scalar::all(1e-4), norm);

  cv::divide(cov, norm, dst);
  //norm.copyTo(dst);
  // dst.setTo(cv::Scalar::all(0), dst < 0.75);

}


static void extract_pixel_matches(const cv::Mat2f & rmap, const cv::Mat1b & mask,
    std::vector<cv::Point2f> & cpts, std::vector<cv::Point2f> & rpts)
{

  for ( int y = 0; y < mask.rows; ++y ) {
    for ( int x = 0; x < mask.cols; ++x ) {
      if ( mask[y][x] ) {
        rpts.emplace_back(x,y);
        cpts.emplace_back(rmap[y][x]);
      }
    }
  }

}

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

  source->seek(258); // 218 321

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

    cv::morphologyEx(images[i], images[i], cv::MORPH_GRADIENT,
        cv::Mat1b(3, 3, (uint8_t)255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);

    images[i].convertTo(images[i], CV_32F, 1./255);

  }

  source->close();

  c_eccflow f;
  cv::Mat2f rmap;
  cv::Mat2f flow;
  cv::Mat corr;
  cv::Mat1b mask;

  f.set_support_scale(3);
  f.set_min_image_size(4);
  f.set_noise_level(1e-3);
  f.set_scale_factor(0.75);
  f.set_max_iterations(3);
  f.set_downscale_method(c_eccflow::DownscaleRecursiveResize);

  f.compute(images[1], images[0], rmap,
      cv::noArray(), cv::noArray());

  cv::remap(images[1], images[2],
      rmap, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT);

  correlate(images[0], images[2], corr);

  cv::compare(corr, cv::Scalar::all(0.7), mask, cv::CMP_GE );

  ecc_remap_to_optflow(rmap, flow);

  save_image(images[0], mask, ssprintf("alpha-debug/pyramid/reference_image.tiff"));
  save_image(images[1], ssprintf("alpha-debug/pyramid/current_image.tiff"));
  save_image(images[2], mask, ssprintf("alpha-debug/pyramid/remapped_image.tiff"));
  save_image(corr, mask, ssprintf("alpha-debug/pyramid/corr.tiff"));
  save_image(flow, ssprintf("alpha-debug/pyramid/optflow.flo"));


  cv::subtract(flow, cv::mean(flow), flow);
  save_image(flow, ssprintf("alpha-debug/pyramid/optflow2.flo"));

  cv::Mat fchannels[2], fmag;
  cv::split(flow, fchannels);
  save_image(fchannels[0], mask, ssprintf("alpha-debug/pyramid/optflow_x.tiff"));
  save_image(fchannels[1], mask, ssprintf("alpha-debug/pyramid/optflow_y.tiff"));
  cv::magnitude(fchannels[0], fchannels[1], fmag);
  save_image(fmag, mask, ssprintf("alpha-debug/pyramid/optflow_mag.tiff"));

  cv::phase(fchannels[0], fchannels[1], fmag);
  save_image(fmag, mask, ssprintf("alpha-debug/pyramid/optflow_phase.tiff"));


  std::vector<cv::Point2f> cpts, rpts;
  cv::Vec3d A(0, 0, 0);
  cv::Vec3d T(0, 0, 1);

  extract_pixel_matches(rmap, mask, cpts, rpts);

  const c_lm_camera_pose_options lm_opts = {
      .robust_threshold = 5.0,
      .epsf = 1e-5,
      .epsx = 1e-5,
      .max_iterations = 3,
      .max_levmar_iterations = 100,
      .direction = EPIPOLAR_MOTION_FORWARD,
  };

  const cv::Matx33d camera_matrix(
      7.215377e+02, 0.000000e+00, 6.095593e+02,
      0.000000e+00, 7.215377e+02, 1.728540e+02,
      0.000000e+00, 0.000000e+00, 1.000000e+00);

  cv::Mat1b inliers((int) cpts.size(), 1, (uint8_t) (255));

  bool fOk =
      lm_refine_camera_pose2(A, T,
      camera_matrix,
      cpts,
      rpts,
      inliers,
      &lm_opts);

  if ( !fOk ) {
    CF_ERROR("lm_refine_camera_pose2() fails");
  }

  const cv::Matx33d R =
      build_rotation(A);

  const cv::Matx33d H =
      camera_matrix * R * camera_matrix.inv();

  const cv::Matx33d E =
      compose_essential_matrix(R, T);

  const cv::Matx33d F =
      compose_fundamental_matrix(E, camera_matrix) * H.inv() ;

  cv::Point2d e[2];

  compute_epipoles(F, e);

  CF_DEBUG("INLIERS: %d / %d", cv::countNonZero(inliers), inliers.size().area());
  CF_DEBUG("A: (%g %g %g)", A(0)* 180 / CV_PI, A(1)* 180 / CV_PI, A(2)* 180 / CV_PI);
  CF_DEBUG("T: (%g %g %g)", T(0), T(1), T(2));
  CF_DEBUG("e: (%g %g)  (%g %g)", e[0].x, e[0].y, e[1].x, e[1].y);



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


