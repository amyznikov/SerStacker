/*
 * c_image_stacking_pipeline_base.cc
 *
 *  Created on: Mar 20, 2026
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline_base.h"
#include <core/proc/bad_pixels.h>
#include <core/proc/inpaint.h>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

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

  if( save ) {
    save_settings(section, "input_image_processor", opts.input_image_processor ?
        opts.input_image_processor->name() : "");
  }
  else {
    std::string s;
    if( load_settings(section, "input_image_processor", &s) && !s.empty() ) {
      opts.input_image_processor = c_image_processor_collection::default_instance()->get(s);
    }
  }

  return true;
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
  INSTRUMENT_REGION("");

  //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  const enum COLORID colorid = input_sequence->colorid();

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

      CF_DEBUG("darkbayer: %dx%d channels=%d depth=%d",
          _darkbayer.cols, _darkbayer.rows, _darkbayer.channels(), _darkbayer.depth());

      cv::subtract(output_image, _darkbayer,
          output_image);
    }


    if( !_flatbayer.empty() ) {

      if( _flatbayer.size() != output_image.size() || _flatbayer.channels() != output_image.channels() ) {
        CF_FATAL("flatbayer (%dx%d*%d) and input frame (%dx%d*%d) not match",
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
          colorid);
    }
  }

  if( input_options.detect_bad_asi_frames ) {
    INSTRUMENT_REGION("detect_bad_asi_frames");
    if ( is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }
  }

  if ( input_options.filter_bad_pixels && input_options.bad_pixels_variation_threshold > 0 ) {
    INSTRUMENT_REGION("filter_bad_pixels");

    if ( is_bayer_pattern(colorid) ) {
      bayer_denoise(output_image,
          input_options.bad_pixels_variation_threshold,
          colorid,
          false);
    }
    else {
      median_filter_bad_pixels(output_image,
          input_options.bad_pixels_variation_threshold,
          colorid);
    }
  }

  if ( is_bayer_pattern(colorid) ) {
    INSTRUMENT_REGION("debayer");

    if ( save_raw_bayer ) {
      _raw_bayer_colorid = colorid;
      if( output_image.depth() == CV_32F ) {
        output_image.copyTo(_raw_bayer_image);
      }
      else {
        output_image.convertTo(_raw_bayer_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }
    }

    if( !debayer(output_image, output_image, colorid, input_options.debayer_method) ) {
      CF_ERROR("debayer() fails");
      return false;
    }
  }

  if( !output_mask.empty() && output_mask.size() != output_image.size() ) {
    if( output_mask.depth() == CV_8U ) {
      cv::resize(output_mask, output_mask, output_image.size(), 0, 0, cv::INTER_NEAREST);
    }
    else {
      cv::resize(output_mask, output_mask, output_image.size(), 0, 0, cv::INTER_AREA);
    }
  }

  if (  !_missing_pixel_mask.empty() ) {
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
    else if ( output_mask.depth() == CV_8U ) {
      cv::bitwise_and(output_mask, _missing_pixel_mask, output_mask);
    }
    else {
      output_mask.setTo(0, ~_missing_pixel_mask);
    }
  }


  if ( !output_mask.empty() && input_options.inpaint_missing_pixels ) {
    INSTRUMENT_REGION("inpaint_missing_pixels");
    linear_interpolation_inpaint(output_image, output_mask);
  }

  if( input_options.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

  if( output_image.depth() != CV_32F ) {
    INSTRUMENT_REGION("convertTo_32F");
    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->bpp())));
  }


  return true;
}


int c_image_stacking_pipeline_base::select_master_frame(const c_input_sequence::sptr & master_sequence, int master_source_index,
    const c_image_stacking_pipeline_base_input_options & input_opts,
    const c_master_frame_selection_options & selection_opts)
{
  INSTRUMENT_REGION("");

  int selected_global_master_frame_index = 0;

  synchronized([this]() {
    _current_master_frame_candidate.release();
    _current_master_frame_candidate_mask.release();
  });

  master_source_index = std::clamp(master_source_index, 0, (int)master_sequence->sources().size() - 1);

  switch (selection_opts.master_selection_method) {

    case master_frame_specific_index:
      selected_global_master_frame_index = master_sequence->global_pos(master_source_index, selection_opts.master_frame_index);
      break;

    case master_frame_middle_index:
      selected_global_master_frame_index = master_sequence->global_pos(master_source_index, master_sequence->size() / 2);
      break;

    case master_frame_best_of_100_in_middle: {

      cv::Mat tmp;

      const int max_frames_to_scan =
          std::max(3, selection_opts.max_frames_to_scan);

      CF_DEBUG("Scan %d frames around of middle %d",
          max_frames_to_scan, master_sequence->size() / 2);

      int start_pos, end_pos, backup_current_pos;

      if( master_sequence->size() <= max_frames_to_scan ) {
        start_pos = 0;
        end_pos = master_sequence->size();
      }
      else {
        start_pos = master_sequence->size() / 2 - max_frames_to_scan / 2;
        end_pos = std::min(master_sequence->size(), start_pos + max_frames_to_scan / 2);
      }

      //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
      master_sequence->set_auto_apply_color_matrix(false);

      backup_current_pos = master_sequence->current_pos();
      master_sequence->seek(start_pos);

      cv::Mat currentImage, currentMask;
      int current_index, best_index = 0;
      double current_metric, best_metric = 0;

      _total_frames = end_pos - start_pos;
      _processed_frames = 0;
      _accumulated_frames = 0;

      on_frame_processed();

      for( current_index = 0; _processed_frames < _total_frames;
          _processed_frames = ++current_index, on_frame_processed() ) {

        if ( canceled() ) {
          CF_DEBUG("cancel requested");
          return -1;
        }

        if( is_bad_frame_index(master_sequence->current_pos()) ) {
          CF_DEBUG("Skip frame %d as blacklisted", master_sequence->current_pos());
          master_sequence->seek(master_sequence->current_pos() + 1);
          continue;
        }

        if ( !master_sequence->read(currentImage, currentMask) ) {
          CF_FATAL("input_sequence->read() fails\n");
          break;
        }

        if( selection_opts.input_image_preprocessor ) {
          if( !selection_opts.input_image_preprocessor->process(currentImage, currentMask) ) {
            CF_DEBUG("input_image_preprocessor fails for frame %d", master_sequence->current_pos() - 1);
            continue;
          }
        }

        if ( !is_bayer_pattern(master_sequence->colorid()) ) {
          tmp = currentImage;
        }
        else {
          average_bayer_planes(currentImage, tmp);
        }

        current_metric = compute_local_variance_map(tmp,
            selection_opts.quality_estimation);

        if( current_metric > best_metric ) {

          best_metric = current_metric;
          best_index = current_index;

          synchronized([&]() {
            currentImage.copyTo(_current_master_frame_candidate);
            currentMask.copyTo(_current_master_frame_candidate_mask);
          });

          set_status_msg(ssprintf("SELECT REFERENCE FRAME...\n"
              "BEST: INDEX=%d METRIC: %g",
              best_index + start_pos,
              best_metric));
        }
      }

      selected_global_master_frame_index = best_index + start_pos;
      master_sequence->seek(backup_current_pos);
      break;
    }

  }


  return selected_global_master_frame_index;
}

c_input_sequence::sptr c_image_stacking_pipeline_base::select_master_frame2(const c_input_sequence::sptr & input_sequence,
    const c_image_stacking_pipeline_base_input_options & input_opts,
    const c_master_frame_selection_options & selection_opts,
    int * output_source_index,
    int * output_master_frame_global_index)
{
  const std::string & master_filename =
      selection_opts.master_fiename;

  c_input_sequence::sptr master_sequence;
  int master_source_index = -1;
  int master_frame_global_index = -1;

  for( int i = 0, n = _input_sequence->sources().size(); i < n; ++i ) {
    const auto & source = _input_sequence->source(i);
    if( master_filename == source->filename() ) {
      master_source_index = i;
      break;
    }
  }

  if( master_source_index >= 0 ) {
    master_sequence = _input_sequence;
  }
  else if( (master_sequence = c_input_sequence::create(master_filename)) ) {
    master_source_index = 0;
  }
  else {
    CF_ERROR("c_input_sequence::create(master_filename='%s') fails", master_filename.c_str());
    return nullptr;
  }

  if ( !master_sequence->is_open() && !master_sequence->open() ) {
    CF_ERROR("master_sequence->open(master_filename='%s) fails", master_filename.c_str());
    return nullptr;
  }

  switch(selection_opts.master_selection_method)
  {
    case master_frame_specific_index:
      master_frame_global_index = master_sequence->global_pos(master_source_index,
          selection_opts.master_frame_index);
      break;

    case master_frame_middle_index: {
      const int master_source_size = master_sequence->source(master_source_index)->size();
      master_frame_global_index = master_sequence->global_pos(master_source_index, master_source_size / 2);
      break;
    }

    case master_frame_best_of_100_in_middle: {

      cv::Mat tmp;

      const int max_frames_to_scan =
          std::max(3, selection_opts.max_frames_to_scan);

      CF_DEBUG("Scan %d frames around of middle %d",
          max_frames_to_scan, master_sequence->size() / 2);

      int start_pos, end_pos;

      if( master_sequence->size() <= max_frames_to_scan ) {
        start_pos = 0;
        end_pos = master_sequence->size();
      }
      else {
        start_pos = master_sequence->size() / 2 - max_frames_to_scan / 2;
        end_pos = std::min(master_sequence->size(), start_pos + max_frames_to_scan / 2);
      }

      //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
      master_sequence->set_auto_apply_color_matrix(false);
      master_sequence->seek(start_pos);

      cv::Mat currentImage, currentMask;
      int current_index, best_index = 0;
      double current_metric, best_metric = 0;

      _total_frames = end_pos - start_pos;
      _processed_frames = 0;
      _accumulated_frames = 0;

      on_frame_processed();

      for( current_index = 0; _processed_frames < _total_frames;
          _processed_frames = ++current_index, on_frame_processed() ) {

        if ( canceled() ) {
          CF_DEBUG("cancel requested");
          return nullptr;
        }

        if( is_bad_frame_index(master_sequence->current_pos()) ) {
          CF_DEBUG("Skip frame %d as blacklisted", master_sequence->current_pos());
          master_sequence->seek(master_sequence->current_pos() + 1);
          continue;
        }

        if ( !master_sequence->read(currentImage, currentMask) ) {
          CF_FATAL("input_sequence->read() fails\n");
          return nullptr;
        }

        if( selection_opts.input_image_preprocessor ) {
          if( !selection_opts.input_image_preprocessor->process(currentImage, currentMask) ) {
            CF_DEBUG("input_image_preprocessor fails for frame %d", master_sequence->current_pos() - 1);
            continue;
          }
        }

        if ( !is_bayer_pattern(master_sequence->colorid()) ) {
          tmp = currentImage;
        }
        else {
          average_bayer_planes(currentImage, tmp);
        }

        current_metric = compute_local_variance_map(tmp,
            selection_opts.quality_estimation);

        if( current_metric > best_metric ) {

          best_metric = current_metric;
          best_index = current_index;

          synchronized([&]() {
            currentImage.copyTo(_current_master_frame_candidate);
            currentMask.copyTo(_current_master_frame_candidate_mask);
          });

          set_status_msg(ssprintf("SELECT REFERENCE FRAME...\n"
              "BEST: INDEX=%d METRIC: %g",
              best_index + start_pos,
              best_metric));
        }
      }

      master_frame_global_index = best_index + start_pos;
      break;
    }
  }

  if( master_frame_global_index < 0 || master_frame_global_index >= master_sequence->size() ) {
    CF_ERROR("BAD master_frame_global_index=%d. master_sequence->size()=%zu",
        master_frame_global_index, master_sequence->size());
    return nullptr;
  }

  * output_source_index = master_source_index;
  * output_master_frame_global_index = master_frame_global_index;
  return master_sequence;
}
