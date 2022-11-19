/*
 * hdrblend.cc
 *
 *  Created on: Nov 14, 2022
 *      Author: amyznikov
 */

#include <core/io/load_image.h>
#include <core/io/save_image.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>



int main(int argc, char *argv[])
{
  std::string input_file_names[2];
  std::string output_file_name;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
      fprintf(stdout, "Usage:\n"
          " hdrblend input_image1.tiff input_image2.tiff -o output_image.tiff\n");
      return 0;
    }

    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( ++i >= argc ) {
        fprintf(stderr, "output file name epxected after %s argument\n",
            argv[i-1]);
        return 1;
      }

      output_file_name =
          argv[i];

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

    fprintf(stderr, "Invalid argument %s\n", argv[i]);
    return 1;
  }


  if ( input_file_names[0].empty() || input_file_names[1].empty() ) {
    fprintf(stderr, "two input images expected\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    output_file_name  = "hdrblend_output.tiff";
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  cv::Mat3f input_images[2];
  cv::Mat1f gray_images[2];
  cv::Mat1b input_masks[2];
  cv::Mat3f input_logs[2];
  cv::Mat nosaturated_pixels_masks[2];
  cv::Mat common_compute_mask;
  cv::Mat mask;

  for( int i = 0; i < 2; ++i ) {

    if( !load_image(input_file_names[i], input_images[i], input_masks[i]) ) {
      fprintf(stderr, "load_image(%s) fails\n", input_file_names[i].c_str());
      return 1;
    }

    cv::cvtColor(input_images[i], gray_images[i], cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_images[i], gray_images[i], cv::Size(), 1, 1);
    cv::normalize(gray_images[i], gray_images[i], 0.0, 1, cv::NORM_MINMAX);
    save_image(gray_images[i], ssprintf("gray_images.%d.tiff", i));

    cv::compare(input_images[i], 0.8, mask, cv::CMP_LT);
    reduce_color_channels(mask, nosaturated_pixels_masks[i], cv::REDUCE_MAX);
    cv::dilate(nosaturated_pixels_masks[i], nosaturated_pixels_masks[i], cv::Mat1b(15,15, 255));

    save_image(nosaturated_pixels_masks[i], ssprintf("nosaturated_pixels.%d.png", i));

    cv::log(input_images[i], input_logs[i]);
  }

  cv::bitwise_and(nosaturated_pixels_masks[0], nosaturated_pixels_masks[1], common_compute_mask);
  save_image(common_compute_mask, ssprintf("common_compute_mask.png"));


  cv::Scalar m =
      cv::mean(input_logs[1] - input_logs[0],
          common_compute_mask);

  double delta = 0;
  for ( int i = 0; i < 3; ++i ) {
    delta += m[i];
  }
  delta /= 3;

  input_logs[1] -= cv::Scalar::all(delta);

  for ( int i = 0; i < 2; ++i ) {
    cv::exp(input_logs[i], input_logs[i]);
    save_image(input_logs[i], ssprintf("logs.%d.tiff", i));
  }

  cv::Mat3f final_image(input_logs[0].size(), cv::Vec3f(0.f, 0.f, 0.f));

  for( int y = 0; y < final_image.rows; ++y ) {
    for( int x = 0; x < final_image.cols; ++x ) {

      double w0 = gray_images[1][y][x];
      double w1 = (1 - gray_images[1][y][x]);
      double w = 1. / (w0*w0 + w1*w1);
      final_image[y][x] = w * (w0 * w0 * input_logs[0][y][x] + w1 * w1 * input_logs[1][y][x]);
    }
  }

  save_image(final_image, ssprintf("final_image.tiff"));


  return 0;
}
