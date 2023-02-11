/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/proc/eccalign.h>
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
#include <core/proc/jupiter.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/ecc2.h>
#include <core/debug.h>



int main(int argc, char *argv[])
{
  std::string input_file_names[2];
  std::string output_directory;
  std::string output_file_name;

  cv::Mat images[2];
  cv::Mat grays[2];
  cv::Mat masks[2];

  bool coarse_to_fine = false;
  bool fix_translation = false;
  bool fix_rotation = false;
  bool fix_scale = false;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Test for ecc2 align.\n"
          "Usage:\n"
          "   alpha <input_image1> <input_image2> [-o <output_directory>] [-h] [-t] [-r] [-s] \n"
          "\n");

      return 0;
    }

    if( strcmp(argv[i], "-h") == 0 ) {
      coarse_to_fine = true;
      continue;
    }

    if( strcmp(argv[i], "-t") == 0 ) {
      fix_translation = true;
      continue;
    }

    if( strcmp(argv[i], "-r") == 0 ) {
      fix_rotation = true;
      continue;
    }

    if( strcmp(argv[i], "-s") == 0 ) {
      fix_scale = true;
      continue;
    }

    if( strcmp(argv[i], "-o") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Output directory name expected after %s argument\n", argv[i - 1]);
        return 1;
      }

      output_directory = argv[i];
      continue;
    }

    if ( input_file_names[0].empty() ) {
      input_file_names[0] = argv[i];
      continue;
    }

    if ( input_file_names[1].empty() ) {
      input_file_names[1] = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if( input_file_names[0].empty() || input_file_names[1].empty() ) {
    fprintf(stderr, "Two input images expected\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  for ( int i  = 0; i < 2; ++i ) {
    if ( !load_image(input_file_names[i], images[i], masks[i]) ) {
      CF_ERROR("load_image('%s') fails", input_file_names[i].c_str());
      return 1;
    }

    if ( images[i].channels() == 3 ) {
      cv::cvtColor(images[i], grays[i], cv::COLOR_BGR2GRAY);
    }
    else {
      grays[i] = images[i];
    }
  }

  if ( output_directory.empty() ) {
    output_directory = "./ecc2";
  }

  cv::Vec2f T;

  if ( true ) {

    c_ecc_translation_transform transform(T);
    c_ecc2_forward_additive ecc(&transform);
    c_ecch2 ecch(&ecc);
    ecch.set_minimum_image_size(2);

    if( !ecch.set_reference_image(grays[0], masks[0]) ) {
      CF_ERROR("ecch.set_reference_image() fails");
      return 1;
    }

    if( !ecch.align(grays[1], masks[1]) ) {
      CF_ERROR("ecch.set_reference_image() fails");
      return 1;
    }

    T = transform.translation();

    CF_DEBUG("tx=%g ty=%g\n===========================\n", T[0], T[1]);
  }


  //c_ecc_translation_transform transform(0, 0);
  c_ecc_euclidean_transform transform(0, 0, 0, 1);
  //c_ecc_affine_transform transform;
   //c_ecc_homography_transform transform;
  //c_ecc_quadratic_transform transform;

  transform.set_translation(-T);
  transform.set_fix_translation(fix_translation);
  transform.set_fix_rotation(fix_rotation);
  transform.set_fix_scale(fix_scale);

  c_ecc2_forward_additive ecc(&transform);
  ecc.set_min_rho(0.5);
  ecc.set_update_step_scale(1);

  if( coarse_to_fine ) {

    c_ecch2 ecch(&ecc);

    ecch.set_minimum_image_size(4);

    if( !ecch.set_reference_image(grays[0], masks[0]) ) {
      CF_ERROR("ecch.set_reference_image() fails");
      return 1;
    }

    if( !ecch.align(grays[1], masks[1]) ) {
      CF_ERROR("ecch.set_reference_image() fails");
    }
  }
  else {

    if( !ecc.set_reference_image(grays[0], masks[0]) ) {
      CF_ERROR("ecc.set_reference_image() fails");
      return 1;
    }

    if( !ecc.align_to_reference(grays[1], masks[1]) ) {
      CF_ERROR("ecc.align_to_reference() fails");
      CF_DEBUG("iterations=%d / %d rho = %g / %g  eps=%g / %g",
          ecc.num_iterations(), ecc.max_iterations(),
          ecc.rho(), ecc.min_rho(),
          ecc.current_eps(), ecc.eps());

    }
  }

  CF_DEBUG("ecc: iterations=%d / %d rho = %g / %g  eps=%g / %g",
      ecc.num_iterations(), ecc.max_iterations(),
      ecc.rho(), ecc.min_rho(),
      ecc.current_eps(), ecc.eps());


  T = transform.translation();
  CF_DEBUG("Tx=%g Ty=%g angle=%g scale=%g\n"
      "===========================\n",
      T[0], T[1],
      transform.rotation() * 180 / CV_PI,
      transform.scale());

  if ( !ecc.current_remap().empty() ) {

    cv::remap(images[1], images[1],
        ecc.current_remap(),
        cv::noArray(),
        ecc.interpolation(),
        cv::BORDER_CONSTANT);

    if( !save_image(images[0], ssprintf("%s/image0.tiff", output_directory.c_str())) ) {
      CF_ERROR("save_image(images[0]) fails");
      return 1;
    }

    if( !save_image(images[1], ssprintf("%s/image1.tiff", output_directory.c_str())) ) {
      CF_ERROR("save_image(images[1]) fails");
      return 1;
    }
  }

  return 0;
}


#if 0

int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_file_name;
  std::string output_directory;

  cv::Mat image;
  std::vector<cv::Mat> pyramid;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Test for Laplacian pyramid.\n"
          "Usage:\n"
          "   alpha <input_image> [-o <output_directory>]\n"
          "\n");

      return 0;
    }

    if( strcmp(argv[i], "-o") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Output directory name expected after %s argument\n", argv[i - 1]);
        return 1;
      }

      output_directory = argv[i];
      continue;
    }

    if ( input_file_name.empty() ) {
      input_file_name = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if( input_file_name.empty() ) {
    fprintf(stderr, "No input file specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !load_image(input_file_name, image) ) {
    CF_ERROR("load_image('%s') fails", input_file_name.c_str());
    return 1;
  }


  build_laplacian_pyramid(image, pyramid);

  if ( output_directory.empty() ) {
    output_directory = "./pyramid";
  }

  for ( int i = 0, n = pyramid.size(); i < n; ++i ) {

    output_file_name =
        ssprintf("%s/L%03d.tiff",
            output_directory.c_str(), i);

    if ( !save_image(pyramid[i], output_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_file_name.c_str());
      return 1;
    }
  }


  cv::Mat reconstructed_image;
  reconstruct_laplacian_pyramid(reconstructed_image, pyramid);

  output_file_name =
      ssprintf("%s/reconstructed_image.tiff", output_directory.c_str());

  if ( !save_image(reconstructed_image, output_file_name) ) {
    CF_ERROR("save_image('%s') fails", output_file_name.c_str());
    return 1;
  }

  return 0;
}
#endif

