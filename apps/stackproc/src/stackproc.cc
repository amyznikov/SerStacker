/*
 * stackproc.cc
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 */

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <core/readdir.h>
#include <core/io/save_image.h>
#include <core/io/rgbamix.h>
#include <tbb/tbb.h>
#include <core/get_time.h>
#include <core/debug.h>
#include "../../../core/improc/c_image_processor.h"

static bool convertTofp32(const cv::Mat & src, cv::Mat & dst)
{
  constexpr int ddepth = CV_32F;
  switch (src.depth()) {
    case CV_8S:
      src.convertTo(dst, ddepth, 1./INT8_MAX, 0.5);
      break;
    case CV_8U:
      src.convertTo(dst, ddepth, 1./UINT8_MAX, 0);
      break;
    case CV_16S:
      src.convertTo(dst, ddepth, 1./INT16_MAX, 0.5);
      break;
    case CV_16U:
      src.convertTo(dst, ddepth, 1./UINT16_MAX, 0);
      break;
    case CV_32S:
      src.convertTo(dst, ddepth, 1./INT32_MAX, 0.5);
      break;
    case CV_32F:
      if ( src.data != dst.data ) {
        src.copyTo(dst);
      }
      break;
    case CV_64F:
      src.convertTo(dst, ddepth);
      break;
    default:
      return  false;;
  }

  return true;
}


int main(int argc, char *argv[])
{
  std::string input_file_name, output_file_name, output_suffix;
  cv::Mat image;
  cv::Mat mask;
  bool overwrite_confirmed = false;
  double min, max;

  c_image_processor processor("processor");

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  for ( int i = 1; i < argc; ++i ) {
    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
      printf("Usage:\n"
          "  stack-postproc input_image [-o output_image] \n"
          "   [-overwrite]                    confirm overwrite input image if output file name is the same\n"
          "   [-unsharp sigma:amount]         apply usharp_mask(sigma, amout=(0..1))\n"
          "   [-clip min:max]                 clip data range to [min..max]\n"
          "   [-normalize min:max]            apply cv::normalize(min, max)\n"

          "   [-a <T|E|A|H|Q>:rho:eps:rc]     align color channels\n"
          "   [-wb]                           apply lsbc white balance\n"
          "   [-wbm]                          apply meanstdev white balance\n"
          "   [-wbh  <low:high>]              apply histogram based white balance\n"
          "   [-wbc  <r:g:b>]                 apply avg color balance\n"
          "   [-g gamma]                      apply pow(image, gamma)\n"
          "   [-cs a:b]                       scale color saturation: s = a + b * s\n"
          "   [-ct T]                         adjust color temperature\n"
          "   [-ul sigma:amount]              apply usharp_mask(sigma, amout) to luminance channel only\n"
          "   [-ulog sigma:amount]            apply usharp_mask_log(sigma, amout)\n"
          "   [-sob sigma:lambda]             apply SOB sharpen\n"
          "   [-wiener sigma:noise]           apply wiener deconvolution with gaussian psf\n"
          "   [-upoly v1:v2:..]               apply fft poly hpass\n"
          "   [-fft-profile out_file_name]    save fft radial profile\n"
          "   [-rl sigma:max-iterations]      apply richardson-lucy deconvolution with gaussian psf\n"
          "   [-blind lambda:ksize:maxiters]  apply blind_deconv\n"
          "   [-gf sigma:eps]                 apply (auto) guided filter\n"
          "   [-gb sigma]                     apply gaussian blur\n"
          "   [-mb r]                         apply median blur\n"
          "   [-wmb r:sigma]                  apply weighted median filter\n"
          "   [-nr iterations]                apply GIMP noise reduction\n"
          "   [-hs low:high]                  apply histogram strecth with clipping\n"
          "   [-csmooth mb:gs]                smooth colors with median and / or gaussian blur\n"
          "   [-pyrdown]                      apply pyrDonw()\n"
          "   [-resize <fx:fy>]               apply cv::resize()\n"

          "\n"
          );

      return 0;
    }


    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( ++i >= argc ) {
        fprintf(stderr, "Command line error: No output image specified\n");
        return 1;
      }
      output_file_name = argv[i];
    }

    else if ( strcmp(argv[i], "-overwrite") == 0 ) {
      overwrite_confirmed = true;
    }

    else if ( strcmp(argv[i], "-unsharp") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Command line error: No unsharp mask sharpen parameters specified\n");
        return 1;
      }

      double sigma = 0, amount = 0;

      if ( sscanf(argv[i], "%lf:%lf", &sigma, &amount) != 2 || sigma <= 0 || amount <= 0 ) {
        fprintf(stderr, "Command line error: invalid unsharp mask parameters specified: %s\n", argv[i]);
        return 1;
      }

      processor.emplace_back(c_unsharp_mask_routine::create(
              sigma, amount));

    }

    else if ( strcmp(argv[i], "-hwb") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Command line error: No lclip hclip range specified\n");
        return 1;
      }

      double lclip = 1, hclip = 99.9;

      if ( sscanf(argv[i], "%lf:%lf", &lclip, &hclip) != 2 ) {
        fprintf(stderr, "Command line error: invalid lclip hclip range specified: %s\n", argv[i]);
        return 1;
      }

      if ( lclip < 0 || lclip > 100 ) {
        fprintf(stderr, "Command line error: invalid lclip value specified: %s\n", argv[i]);
        return 1;
      }

      if ( hclip < lclip || hclip > 100 ) {
        fprintf(stderr, "Command line error: invalid hclip value specified: %s\n", argv[i]);
        return 1;
      }

      processor.emplace_back(c_histogram_white_balance_routine::create(
              lclip, hclip));

    }


    else if ( strcmp(argv[i], "-clip") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Command line error: No min max range specified\n");
        return 1;
      }

      double min = 0, max = 1;

      if ( sscanf(argv[i], "%lf:%lf", &min, &max) != 2 ) {
        fprintf(stderr, "Command line error: invalid min.max range specified: %s\n", argv[i]);
        return 1;
      }

      processor.emplace_back(c_rangeclip_routine::create(
              min, max));

    }


    else if ( strcmp(argv[i], "-normalize") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Command line error: No min max range specified\n");
        return 1;
      }

      double min = 0, max = 1;

      if ( sscanf(argv[i], "%lf:%lf", &min, &max) != 2 ) {
        fprintf(stderr, "Command line error: invalid min.max range specified: %s\n", argv[i]);
        return 1;
      }

      processor.emplace_back(c_range_normalize_routine::create(
              min, max));

    }



    else if ( input_file_name.empty() ) {
      input_file_name = argv[i];
    }

    else {
      fprintf(stderr, "Command line error: invalid argument '%s'\n", argv[i]);
      return 1;
    }
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "Command line error: No input image specified\n");
    return 1;
  }

  if ( !output_file_name.empty() && output_file_name == input_file_name && !overwrite_confirmed ) {
    fprintf(stderr, "Input and output file names are the same. Use -overwrite option to confirm overwriting\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    set_file_suffix(output_file_name =
        get_file_name(input_file_name),
        "-PP.tiff");
  }

  output_suffix =
      get_file_suffix(output_file_name);


  if ( !(image = cv::imread(input_file_name, cv::IMREAD_UNCHANGED)).data ) {
    CF_FATAL("cv::imread(%s) fails", input_file_name.c_str());
    return 1;
  }

  cv::minMaxLoc(image, &min, &max);

  CF_DEBUG("input: %dx%d channels=%d depth=%d min=%g max=%g",
        image.cols, image.rows,
        image.channels(), image.depth(),
        min, max);

  switch ( image.channels() ) {
  case 1 :
    break;
  case 2 : {
    cv::Mat channels[2];
    cv::split(image, channels);
    image = channels[0];
    cv::compare(channels[1], 0, mask, cv::CMP_GT);
    CF_DEBUG("HAVE MASK");
    break;
  }
  case 3 :
    break;
  case 4 : // Assuming BGRA
    cv::extractChannel(image, mask, 3);
    cv::compare(mask, 0, mask, cv::CMP_GT);
    cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
    CF_DEBUG("HAVE MASK");
    break;
  default :
    CF_FATAL("Invalid input: RGB or grayscale image expected, image.channels()=%d rgb_image.depth()=%d",
        image.channels(), image.depth());
    return 1;
    break;
  }

  convertTofp32(image,
      image);

  cv::minMaxLoc(image, &min, &max);
  CF_DEBUG("Output range: min=%f max=%f", min, max);

  processor.process(image, mask);

  cv::minMaxLoc(image, &min, &max);
  CF_DEBUG("Output range: min=%f max=%f", min, max);

  if ( strcasecmp(output_suffix.c_str(), ".tiff") != 0 && strcasecmp(output_suffix.c_str(), ".tif") != 0 ) {

    clip_range(image, 0, 1, mask);

    if ( strcasecmp(output_suffix.c_str(), "png") == 0 ) {
      image.convertTo(image, CV_16U, UINT16_MAX);
    }
    else {
      image.convertTo(image, CV_8U, UINT8_MAX);
    }
  }

  if ( !mask.empty() ) {
    image.setTo(0, ~mask);
    mergebgra(image, mask, image);
  }

  if ( !save_image(image, output_file_name) ) {
    CF_ERROR("save_image('%s') fails", output_file_name.c_str());
    return 1;
  }

  return 0;
}
