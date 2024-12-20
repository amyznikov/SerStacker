/*
 * c_running_average_pipeline.cc
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#include "c_running_average_pipeline.h"
#include <core/proc/unsharp_mask.h>
#include <core/io/load_image.h>

c_running_average_pipeline::c_running_average_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_running_average_pipeline::serialize(c_config_setting settings, bool save)
{
//  static const auto get_group =
//      [](c_config_setting setting, bool save, const std::string & name) {
//        return save ? setting.add_group(name) : setting[name];
//      };

  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {
    SERIALIZE_OPTION(section, save, registration_options_, double_align_moode);
    SERIALIZE_OPTION(section, save, registration_options_, reference_unsharp_sigma_);
    SERIALIZE_OPTION(section, save, registration_options_, reference_unsharp_alpha_);
    SERIALIZE_OPTION(section, save, registration_options_, enable_ecc);
    SERIALIZE_OPTION(section, save, registration_options_, ecc_motion_type);
    SERIALIZE_OPTION(section, save, registration_options_, min_rho);

    SERIALIZE_PROPERTY(section, save, ecch_, method);
    SERIALIZE_PROPERTY(section, save, ecch_, minimum_image_size);
    SERIALIZE_PROPERTY(section, save, ecch_, maxlevel);
    SERIALIZE_PROPERTY(section, save, ecch_, max_iterations);
    SERIALIZE_PROPERTY(section, save, ecch_, epsx);
    SERIALIZE_PROPERTY(section, save, ecch_, min_rho);
    SERIALIZE_PROPERTY(section, save, ecch_, interpolation);
    SERIALIZE_PROPERTY(section, save, ecch_, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, ecch_, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, ecch_, update_step_scale);

    SERIALIZE_OPTION(section, save, registration_options_, enable_eccflow);
    SERIALIZE_PROPERTY(section, save, eccflow_, downscale_method);
    SERIALIZE_PROPERTY(section, save, eccflow_, min_image_size);
    SERIALIZE_PROPERTY(section, save, eccflow_, max_pyramid_level);
    SERIALIZE_PROPERTY(section, save, eccflow_, noise_level);
    SERIALIZE_PROPERTY(section, save, eccflow_, support_scale);
    SERIALIZE_PROPERTY(section, save, eccflow_, max_iterations);
    SERIALIZE_PROPERTY(section, save, eccflow_, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, eccflow_, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, eccflow_, update_multiplier);
    SERIALIZE_PROPERTY(section, save, eccflow_, scale_factor);

  }

  if( (section = SERIALIZE_GROUP(settings, save, "average_options")) ) {

    SERIALIZE_OPTION(section, save, average_options_, running_weight);
    SERIALIZE_OPTION(section, save, average_options_, reference_weight);

    if( (subsection = SERIALIZE_GROUP(section, save, "lpg")) ) {
      SERIALIZE_OPTION(subsection, save, average_options_.lpg, k);
      SERIALIZE_OPTION(subsection, save, average_options_.lpg, p);
      SERIALIZE_OPTION(subsection, save, average_options_.lpg, dscale);
      SERIALIZE_OPTION(subsection, save, average_options_.lpg, uscale);
    }
  }


  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_accumulated_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_reference_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_reference_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_reference_video_options);
    }


    SERIALIZE_OPTION(section, save, output_options_, display_scale);
  }

  return true;
}

bool c_running_average_pipeline::ecc_ctls_enabled() const
{
  return (registration_options_.enable_ecc || registration_options_.double_align_moode);
}

bool c_running_average_pipeline::eccflow_ctls_enabled() const
{
  return (registration_options_.enable_eccflow || registration_options_.double_align_moode);
}

const std::vector<c_image_processing_pipeline_ctrl> & c_running_average_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
    POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image registration", "");

      PIPELINE_CTL(ctrls, registration_options_.double_align_moode, "double_align_moode", "");
      PIPELINE_CTLC(ctrls, registration_options_.reference_unsharp_sigma_, "ref_unsharp_sigma", "", _this->registration_options_.double_align_moode);
      PIPELINE_CTLC(ctrls,  registration_options_.reference_unsharp_alpha_, "ref_unsharp_alpha", "", _this->registration_options_.double_align_moode);

      PIPELINE_CTL_GROUP(ctrls, "ECC", "");
        PIPELINE_CTL(ctrls, registration_options_.enable_ecc, "enabled", "");

        PIPELINE_CTLC(ctrls, registration_options_.ecc_motion_type, "motion_type", "", (_this->ecc_ctls_enabled()));

        PIPELINE_CTLPC2(ctrls, ecch_, method, "ecch method", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, minimum_image_size, "ecch_minimum_image_size", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, maxlevel, "maxlevel ", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, max_iterations, "max_iterations", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, max_eps, "ecc_max_eps", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, min_rho, "ecc_min_rho", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, interpolation, "ecc_interpolation", "", (_this->ecc_ctls_enabled()));

        PIPELINE_CTLPC2(ctrls, ecch_, input_smooth_sigma, "input_smooth_sigma", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, reference_smooth_sigma, "reference_smooth_sigma", "", (_this->ecc_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, ecch_, update_step_scale, "update_step_scale", "", (_this->ecc_ctls_enabled()));
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "ECCFLOW", "");
        PIPELINE_CTL(ctrls, registration_options_.enable_eccflow, "enabled", "");
        PIPELINE_CTLPC2(ctrls, eccflow_, support_scale, "support_scale", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, downscale_method, "downscale_method", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, min_image_size, "min_image_size", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, max_pyramid_level, "max_pyramid_level", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, scale_factor, "scale_factor", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, max_iterations, "max_iterations", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, noise_level, "noise_level", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, input_smooth_sigma, "input_smooth_sigma", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, reference_smooth_sigma, "reference_smooth_sigma", "", (_this->eccflow_ctls_enabled()));
        PIPELINE_CTLPC2(ctrls, eccflow_, update_multiplier, "update_multiplier", "", (_this->eccflow_ctls_enabled()));
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTLC(ctrls, registration_options_.min_rho, "min_rho", "", (_this->ecc_ctls_enabled() || _this->eccflow_ctls_enabled()));

    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Average options", "");
    PIPELINE_CTLC(ctrls, average_options_.reference_weight, "reference running_weight", "", (_this->registration_options_.double_align_moode));
    PIPELINE_CTL(ctrls, average_options_.running_weight, "running_weight", "");
      PIPELINE_CTL_GROUP(ctrls, "Shapness measure", "");
        PIPELINE_CTL(ctrls, average_options_.lpg.k, "k", "");
        PIPELINE_CTL(ctrls, average_options_.lpg.p, "p", "power");
        PIPELINE_CTL(ctrls, average_options_.lpg.dscale, "dscale", "");
        PIPELINE_CTL(ctrls, average_options_.lpg.uscale, "uscale", "");
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, output_options_.display_scale, "display_scale", "");
    PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");


    PIPELINE_CTL_GROUP(ctrls, "Save accumulated video", "");
      PIPELINE_CTL(ctrls, output_options_.save_accumulated_video, "save_accumulated_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_accumulated_video_options,
          (_this->output_options_.save_accumulated_video));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "save_reference_video", "");
      PIPELINE_CTL(ctrls, output_options_.save_reference_video, "save_reference_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_reference_video_options,
          (_this->output_options_.save_reference_video));
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

///

void c_running_average_pipeline::set_double_align_moode(bool v)
{
  registration_options_.double_align_moode = v;
}

bool c_running_average_pipeline::double_align_moode() const
{
  return registration_options_.double_align_moode;
}

///

bool c_running_average_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

//  if ( average1_.compute(display_frame, display_mask) ) {
//    return true;
//  }

  if ( registration_options_.double_align_moode ) {
    if ( average2_.compute(display_frame, display_mask) ) {
      return true;
    }
  }
  else {
    if ( average1_.compute(display_frame, display_mask) ) {
      return true;
    }
  }

  return false;
}

bool c_running_average_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_running_average_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->registration_options_ = this->registration_options_;
  p->average_options_ = this->average_options_;
  p->output_options_ = this->output_options_;
  p->ecch_.copy_parameters(ecch_);
  //p->ecc_.copy_parameters(ecc_);
  p->eccflow_.copy_parameters(eccflow_);

  return true;
}


bool c_running_average_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path =
      create_output_path(output_options_.output_directory);

  average1_.clear();
  average2_.clear();
  current_image_.release();
  current_mask_.release();

  if ( !_input_options.darkbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.darkbayer_filename, _darkbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.darkbayer_filename.c_str());
      return false;
    }
  }

  if ( !_input_options.flatbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.flatbayer_filename, _flatbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.flatbayer_filename.c_str());
      return false;
    }
  }



  if ( !_input_options.missing_pixel_mask_filename.empty() ) {

    if ( !load_image(_input_options.missing_pixel_mask_filename, _missing_pixel_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( _missing_pixel_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !_input_options.missing_pixels_marked_black ) {
      cv::invert(_missing_pixel_mask, _missing_pixel_mask);
    }
  }


  if ( registration_options_.enable_ecc ) {

    ecc_tramsform_ =
        create_image_transform(registration_options_.ecc_motion_type);

    ecch_.set_image_transform(ecc_tramsform_.get());
    //ecch_.set_method(registration_options_.ecc_method);
  }



  CF_DEBUG("Output path='%s'", this->_output_path.c_str());


  return true;
}

void c_running_average_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  ecch_.clear();
  ecc_tramsform_.reset();

  current_image_.release();
  current_mask_.release();


}

bool c_running_average_pipeline::run_pipeline()
{
  if ( !start_pipeline(_input_options.start_frame_index, _input_options.max_input_frames) ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, ++_accumulated_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    const bool fOk =
        read_input_frame(_input_sequence,
            _input_options,
            current_image_, current_mask_,
            false,
            false);

    if( !fOk ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if ( current_image_.empty() ) {
      // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
      continue;
    }

    if( _input_options.input_image_processor && !_input_options.input_image_processor->empty() ) {
      if( !_input_options.input_image_processor->process(current_image_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if ( registration_options_.double_align_moode ) {

      if( !process_current_frame2() ) {
        CF_ERROR("process_current_frame2() fails");
        return false;
      }

    }
    else {

      if( !process_current_frame1() ) {
        CF_ERROR("process_current_frame1() fails");
        return false;
      }
    }

  }

  return true;
}

bool c_running_average_pipeline::average_add(c_running_frame_average & average, const cv::Mat & src, const cv::Mat & srcmask,
    double avgw, const cv::Mat2f * rmap)
{
  if( average_options_.lpg.k < 0 ) {

    lock_guard lock(mutex());

    if( !average.add(src, srcmask, avgw, rmap) ) {
      CF_ERROR("average.add() fails");
      return false;
    }
  }
  else {
    cv::Mat1f w;

    compute_weights(src, srcmask, w);

    lock_guard lock(mutex());

    if( !average.add(src, w, avgw, rmap) ) {
      CF_ERROR("average.add() fails");
      return false;
    }
  }

  return true;
}

bool c_running_average_pipeline::process_current_frame1()
{
  cv::Mat image1, mask1, image2, mask2;
  cv::Mat2f rmap;

  bool has_updates = true;

  const bool enable_registration =
      registration_options_.enable_ecc ||
      registration_options_.enable_eccflow;

  static const auto mkgrayscale =
      [](const cv::Mat & src, cv::Mat & dst) {
        if( src.channels() != 1 ) {
          cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        }
        else if ( &src != &dst ) {
          dst = src;
        }
      };


  if( !enable_registration || average1_.accumulated_frames() < 1 ) {

    if( !average_add(average1_, current_image_, current_mask_, average_options_.running_weight) ) {
      CF_ERROR("average_add() fails");
      return false;
    }
  }
  else {


    rmap.release();

    if( enable_registration ) {
      average1_.compute(image1, mask1);
      mkgrayscale(image1, image1);
      mkgrayscale(current_image_, image2);
    }


    if ( registration_options_.enable_ecc ) {

      ecch_.set_reference_image(image2, mask2);

      if( (has_updates = ecch_.align(image1, mask1)) ) {
        ecch_.image_transform()->create_remap(ecch_.reference_image().size(), rmap);
      }
    }


    if( registration_options_.enable_eccflow ) {
      eccflow_.set_reference_image(image2, mask2);
      has_updates = eccflow_.compute(image1, rmap, mask1);
    }

    if( has_updates ) {
      if( !average_add(average1_, current_image_, current_mask_, average_options_.running_weight, &rmap) ) {
        CF_ERROR("average_add() fails");
        return false;
      }
    }
  }

  if ( has_updates && output_options_.save_accumulated_video  ) {

    const bool fOK =
        add_output_writer(accumulated_video_writer_,
            output_options_.output_accumulated_video_options,
            "accw",
            ".ser");

    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          accumulated_video_writer_.filename().c_str());
      return false;
    }

    average1_.compute(image1, mask1);

    if( !accumulated_video_writer_.write(image1, mask1) ) {
      CF_ERROR("accumulated_video_writer_.write() fails");
      return false;
    }
  }

  return true;
}

bool c_running_average_pipeline::process_current_frame2()
{

  cv::Mat image1, mask1, image2, mask2;
  cv::Mat2f rmap;

  bool has_updates = true;

  const double W1 =
      average_options_.reference_weight;

  const double W2 =
      average_options_.running_weight;

  static const auto mkgrayscale =
      [](const cv::Mat & src, const cv::Mat & src_mask, cv::Mat & dst, cv::Mat & dst_mask) {
        if( src.channels() != 1 ) {
          cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        }
        else if ( &src != &dst ) {
          dst = src;
        }
        if ( &src_mask != &dst_mask) {
          dst_mask = src_mask;
        }
  };


  if ( !registration_options_.enable_ecc ) {
    CF_ERROR("ECC must be enabled for double align mode");
    return false;
  }

  ////
  mkgrayscale(current_image_, current_mask_,
      image2, mask2);

  if( average1_.accumulated_frames() < 1 ) {

    if( !average_add(average1_, image2, mask2, W1) ) {
      CF_ERROR("average_add() fails");
      return false;
    }

  }
  else {
    average1_.compute(image1, mask1);

    ecch_.set_reference_image(image2, mask2);
    if( !ecch_.align(image1, mask1) ) {
      has_updates = false;
    }
    else {

      if( !average_add(average1_, image2, mask2, W1, &(rmap = ecch_.create_remap())) ) {
        CF_ERROR("average_add() fails");
        return false;
      }

    }
  }

  ////

  if ( has_updates && average1_.accumulated_frames() > W1 )  {

    cv::Mat2f rmap2;

    if( registration_options_.enable_eccflow ) {

      average1_.compute(image1, mask1);

      mkgrayscale(image1, mask1,
          image1, mask1);

      if( registration_options_.reference_unsharp_sigma_ > 0 && registration_options_.reference_unsharp_alpha_ > 0 ) {

        unsharp_mask(image1, mask1, image1, registration_options_.reference_unsharp_sigma_,
            registration_options_.reference_unsharp_alpha_);
      }


      if ( image2.empty() ) {
        mkgrayscale(current_image_, current_mask_,
            image2, mask2);
      }


      rmap2 = rmap.clone();

      eccflow_.set_reference_image(image1, mask1);

      if( !eccflow_.compute(image2, rmap2, mask2) ) {
        has_updates = false;
      }

    }

    if ( has_updates ) {

      if ( rmap2.empty() ) {
       if ( !average_add(average2_, current_image_, current_mask_, W2, &rmap) ) {
          CF_ERROR("average_add(average2_) fails");
          return false;
        }
      }
      else {

        cv::remap(current_image_, image2, rmap2, cv::noArray(), cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

        CF_DEBUG("current_mask_: %dx%d nnz = %d / %d", current_mask_.cols, current_mask_.rows,
            cv::countNonZero(current_mask_),
            current_mask_.size().area());

        if ( current_mask_.empty() ) {
          mask2.release();
          //          cv::remap(cv::Mat1b(current_image_.size(), 255), mask2, rmap2, cv::noArray(),
          //              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else {

          cv::remap(current_mask_, mask2, rmap2, cv::noArray(),
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);

          cv::compare(mask2,  254, mask2, cv::CMP_GE);
        }


        if ( !average_add(average2_, image2, mask2, W2, &rmap) ) {
          CF_ERROR("average_add() fails");
          return false;
        }

      }
    }
  }

  if ( has_updates ) {

    if ( output_options_.save_reference_video && average1_.accumulated_frames() > 0 ) {

      const bool fOK =
          add_output_writer(reference_video_writer_,
              output_options_.output_reference_video_options,
              "accr",
              ".ser");

      if( !fOK ) {
        CF_ERROR("Can not open output writer '%s'",
            reference_video_writer_.filename().c_str());
        return false;
      }

      average1_.compute(image1, mask1);

      if( registration_options_.reference_unsharp_sigma_ > 0 && registration_options_.reference_unsharp_alpha_ > 0 ) {

        unsharp_mask(image1, mask1, image1, registration_options_.reference_unsharp_sigma_,
            registration_options_.reference_unsharp_alpha_);
      }

      if( !reference_video_writer_.write(image1, mask1) ) {
        CF_ERROR("reference_video_writer_.write() fails");
        return false;
      }
    }

    if ( output_options_.save_accumulated_video && average2_.accumulated_frames() > 0 ) {

      const bool fOK =
          add_output_writer(accumulated_video_writer_,
              output_options_.output_accumulated_video_options,
              "accw",
              ".ser");

      if( !fOK ) {
        CF_ERROR("Can not open output writer '%s'",
            accumulated_video_writer_.filename().c_str());
        return false;
      }

      average2_.compute(image1, mask1);

      if( !accumulated_video_writer_.write(image1, mask1) ) {
        CF_ERROR("accumulated_video_writer_.write() fails");
        return false;
      }
    }

  }

  return true;
}

void c_running_average_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  c_lpg_sharpness_measure::create_map(src, dst, average_options_.lpg);
  if( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }
}


