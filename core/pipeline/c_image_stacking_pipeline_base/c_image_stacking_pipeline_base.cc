/*
 * c_image_stacking_pipeline_base.cc
 *
 *  Created on: Mar 20, 2026
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline_base.h"
#include <core/proc/inpaint.h>

bool serialize_base_image_stacking_input_options(c_config_setting section, bool save,
    c_image_stacking_pipeline_base_input_options & opts)
{
  c_config_setting subsection;

  SERIALIZE_OPTION(section, save, opts, debayer_method);
  SERIALIZE_OPTION(section, save, opts, darkbayer_filename);
  SERIALIZE_OPTION(section, save, opts, flatbayer_filename);
  SERIALIZE_OPTION(section, save, opts, missing_pixel_mask_filename);
  SERIALIZE_OPTION(section, save, opts, missing_pixels_marked_black);
  SERIALIZE_OPTION(section, save, opts, inpaint_missing_pixels);
  SERIALIZE_OPTION(section, save, opts, filter_bad_pixels);
  SERIALIZE_OPTION(section, save, opts, detect_bad_asi_frames);
  SERIALIZE_OPTION(section, save, opts, bad_pixels_variation_threshold);
  SERIALIZE_OPTION(section, save, opts, enable_color_maxtrix);
  SERIALIZE_OPTION(section, save, opts, start_frame_index);
  SERIALIZE_OPTION(section, save, opts, max_input_frames);

  SERIALIZE_OPTION(section, save, opts, enable_bground_normalization);
  if( (subsection = SERIALIZE_GROUP(section, save, "bground_normalization")) ) {
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, norm_type);
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, stretch);
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, offset);
  }

  return true;
}

static void remove_bad_pixels(cv::Mat & image,
    double bad_pixels_variation_threshold,
    bool isbayer)
{
  INSTRUMENT_REGION("");

  cv::Mat medianImage, variationImage, meanVariationImage;
  double minimal_mean_variation_for_very_smooth_images;

  // threshold = estimate_noise(image);
  if ( image.depth() == CV_32F || image.depth() == CV_64F ) {
    minimal_mean_variation_for_very_smooth_images = 1e-3;
  }
  else {
    minimal_mean_variation_for_very_smooth_images = 1;
  }

  cv::medianBlur(image, medianImage, isbayer ? 3 : 5);
  cv::absdiff(image, medianImage, variationImage);

  static float K[3*3] = {
      1./8, 1./8, 1./8,
      1./8, 0.0,  1./8,
      1./8, 1./8, 1./8,
  };

  static const cv::Mat1f SE(3,3, K);

  cv::filter2D(variationImage, meanVariationImage, -1, SE);
  cv::max(meanVariationImage, minimal_mean_variation_for_very_smooth_images, meanVariationImage);

  medianImage.copyTo(image, variationImage > bad_pixels_variation_threshold * meanVariationImage);
}


c_image_stacking_pipeline_base::c_image_stacking_pipeline_base(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_image_stacking_pipeline_base::initialize_pipeline()
{
  if ( true ) {
    lock_guard lock(mutex());
    _missing_pixel_mask.release();
    _darkbayer.release();
    _flatbayer.release();
    _raw_bayer_image.release();
  }

  if ( !base::initialize_pipeline() ) {
      CF_ERROR("base::initialize_pipeline() fails");
      return false;
  }

  return true;
}

void c_image_stacking_pipeline_base::cleanup_pipeline()
{
  if( true ) {
    lock_guard lock(mutex());
    _darkbayer.release();
    _flatbayer.release();
    _raw_bayer_image.release();
  }

  base::cleanup_pipeline();
}


bool c_image_stacking_pipeline_base::read_input_frame(const c_input_sequence::sptr & input_sequence,
    const c_image_stacking_pipeline_base_input_options & input_options,
    cv::Mat & output_image, cv::Mat & output_mask,
    bool is_external_master_frame,
    bool save_raw_bayer)
{
  //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  if( !is_external_master_frame ) {

    if( !_darkbayer.empty() ) {

      if( _darkbayer.size() != output_image.size() || _darkbayer.channels() != output_image.channels() ) {
        CF_FATAL("darkbayer (%dx%d*%d) and input frame (%dx%d*%d) not match",
            _darkbayer.cols, _darkbayer.rows, _darkbayer.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::subtract(output_image, _darkbayer,
          output_image);
    }


    if( !_flatbayer.empty() ) {

      if( _flatbayer.size() != output_image.size() || _flatbayer.channels() != output_image.channels() ) {
        CF_FATAL("flatbayer_ (%dx%d*%d) and input frame (%dx%d*%d) not match",
            _flatbayer.cols, _flatbayer.rows, _flatbayer.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::divide(output_image, _flatbayer,
          output_image, output_image.depth());
    }


    if ( input_options.enable_bground_normalization ) {
      nomalizeImageHistogram(output_image, output_mask, output_image,
          input_options.background_normalization_options,
          input_sequence->colorid());
    }
  }

  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }

    if ( input_options.filter_bad_pixels ) {
      remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, false);
    }

    if( output_image.depth() != CV_32F ) {
      output_image.convertTo(output_image, CV_32F,
          1. / ((1 << input_sequence->bpp())));
    }

  }
  else {

    const DEBAYER_ALGORITHM algo =
        input_options.debayer_method;

    if ( save_raw_bayer ) {

      _raw_bayer_colorid =
          input_sequence->colorid();

      if( output_image.depth() == CV_32F ) {
        output_image.copyTo(_raw_bayer_image);
      }
      else {
        output_image.convertTo(_raw_bayer_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      if( input_options.filter_bad_pixels && input_options.bad_pixels_variation_threshold > 0 ) {
        if( !bayer_denoise(_raw_bayer_image, input_options.bad_pixels_variation_threshold) ) {
          CF_ERROR("bayer_denoise() fails");
          return false;
        }
      }
    }


    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, input_sequence->colorid(), algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if ( input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, true);
        }
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }
        if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if( input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, true);
        }

        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }

        if ( !nninterpolation(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toCString(algo));
        return false;
    }
  }

  if( input_options.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

//  if ( anscombe_.method() != anscombe_none ) {
//    anscombe_.apply(output_image, output_image);
//  }

  if ( !_missing_pixel_mask.empty() ) {

    if ( output_image.size() != _missing_pixel_mask.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          _missing_pixel_mask.cols, _missing_pixel_mask.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      _missing_pixel_mask.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, _missing_pixel_mask,
          output_mask);
    }
  }

  if ( !output_mask.empty() && input_options.inpaint_missing_pixels ) {
#if 1
    linear_interpolation_inpaint(output_image, output_mask,
        output_image);
#else
    average_pyramid_inpaint(output_image, output_mask,
        output_image);
#endif
  }


  return true;
}
