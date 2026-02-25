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
    SERIALIZE_OPTION(section, save, _registration_options, double_align_moode);
    SERIALIZE_OPTION(section, save, _registration_options, reference_unsharp_sigma);
    SERIALIZE_OPTION(section, save, _registration_options, reference_unsharp_alpha);
    SERIALIZE_OPTION(section, save, _registration_options, enable_ecc);
    SERIALIZE_OPTION(section, save, _registration_options, ecc_motion_type);
    SERIALIZE_OPTION(section, save, _registration_options, min_rho);

    SERIALIZE_PROPERTY(section, save, _ecch, method);
    SERIALIZE_PROPERTY(section, save, _ecch, minimum_image_size);
    SERIALIZE_PROPERTY(section, save, _ecch, maxlevel);
    SERIALIZE_PROPERTY(section, save, _ecch, max_iterations);
    SERIALIZE_PROPERTY(section, save, _ecch, epsx);
    SERIALIZE_PROPERTY(section, save, _ecch, min_rho);
    SERIALIZE_PROPERTY(section, save, _ecch, interpolation);
    SERIALIZE_PROPERTY(section, save, _ecch, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _ecch, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _ecch, update_step_scale);

    SERIALIZE_OPTION(section, save, _registration_options, enable_eccflow);
    SERIALIZE_PROPERTY(section, save, _eccflow, downscale_method);
    SERIALIZE_PROPERTY(section, save, _eccflow, min_image_size);
    SERIALIZE_PROPERTY(section, save, _eccflow, max_pyramid_level);
    SERIALIZE_PROPERTY(section, save, _eccflow, noise_level);
    SERIALIZE_PROPERTY(section, save, _eccflow, support_scale);
    SERIALIZE_PROPERTY(section, save, _eccflow, max_iterations);
    SERIALIZE_PROPERTY(section, save, _eccflow, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _eccflow, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, _eccflow, update_multiplier);
    SERIALIZE_PROPERTY(section, save, _eccflow, scale_factor);

  }

  if( (section = SERIALIZE_GROUP(settings, save, "average_options")) ) {

    SERIALIZE_OPTION(section, save, _average_options, running_weight);
    SERIALIZE_OPTION(section, save, _average_options, reference_weight);

    if( (subsection = SERIALIZE_GROUP(section, save, "lpg")) ) {
      SERIALIZE_OPTION(subsection, save, _average_options.lpg, k);
      SERIALIZE_OPTION(subsection, save, _average_options.lpg, p);
      SERIALIZE_OPTION(subsection, save, _average_options.lpg, dscale);
      SERIALIZE_OPTION(subsection, save, _average_options.lpg, uscale);
    }
  }


  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_accumulated_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_reference_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_reference_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_reference_video_options);
    }


    SERIALIZE_OPTION(section, save, _output_options, display_scale);
  }

  return true;
}

bool c_running_average_pipeline::ecc_ctls_enabled() const
{
  return (_registration_options.enable_ecc || _registration_options.double_align_moode);
}

bool c_running_average_pipeline::eccflow_ctls_enabled() const
{
  return (_registration_options.enable_eccflow || _registration_options.double_align_moode);
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_input_options> & ctx)
{
  using S = c_running_average_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
  ctlbind(ctls, "input_image_processor", ctx(&S::input_image_processor));
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_registration_options> & ctx)
{
  using S = c_running_average_registration_options;

  ctlbind(ctls, "double_align_moode", ctx(&S::double_align_moode), "");
  ctlbind(ctls, "ref_unsharp_sigma", ctx(&S::reference_unsharp_sigma), ""); // _this->_registration_options.double_align_moode);
  ctlbind(ctls, "ref_unsharp_alpha", ctx(&S::reference_unsharp_alpha), ""); // _this->_registration_options.double_align_moode);

  ctlbind_expandable_group(ctls, "ECC", "");
    ctlbind(ctls, "enable ecc", ctx(&S::enable_ecc), "");
    ctlbind(ctls, "motion_type", ctx(&S::ecc_motion_type), "");
    ctlbind(ctls, ctx(&S::ecch));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "ECCFLOW", "");
    ctlbind(ctls, "enable eccflow", ctx(&S::enable_eccflow), "");
    ctlbind(ctls, ctx(&S::eccflow));
  ctlbind_end_group(ctls);

  ctlbind(ctls, "min_rho", ctx(&S::min_rho), "");// , (_this->ecc_ctls_enabled() || _this->eccflow_ctls_enabled()));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_update_options> & ctx)
{
  using S = c_running_average_update_options;

  ctlbind(ctls, "reference running_weight", ctx(&S::reference_weight), ""); // , "", (_this->_registration_options.double_align_moode));
  ctlbind(ctls, "running_weight", ctx(&S::running_weight), "");
  ctlbind_expandable_group(ctls, "Shapness measure", "");
    ctlbind(ctls, ctx(&S::lpg));
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_output_options> & ctx)
{
  using S = c_running_average_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "display_scale", ctx(&S::display_scale), "");
  ctlbind_browse_for_directory(ctls, "output_directory", ctx(&S::output_directory), "");

  ctlbind_expandable_group(ctls, "Save accumulated video");
    ctlbind(ctls, "save_accumulated_video", ctx(&S::save_accumulated_video), "");
    ctlbind(ctls, ctx(&S::output_accumulated_video_options));//  (_this->_output_options.save_accumulated_video));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save reference video", "");
    ctlbind(ctls, "save_reference_video", ctx(&S::save_reference_video), "");
    ctlbind(ctls, ctx(&S::output_reference_video_options)); //  (_this->_output_options.save_reference_video));
  ctlbind_end_group(ctls);
}

const c_ctlist<c_running_average_pipeline> & c_running_average_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Image registration", "");
      ctlbind(ctls, ctx(&this_class::_registration_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Average options", "");
      ctlbind(ctls, ctx(&this_class::_average_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}

//
//const std::vector<c_image_processing_pipeline_ctrl> & c_running_average_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
//
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////    POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Image registration", "");
////
////      PIPELINE_CTL(ctrls, _registration_options.double_align_moode, "double_align_moode", "");
////      PIPELINE_CTLC(ctrls, _registration_options.reference_unsharp_sigma, "ref_unsharp_sigma", "", _this->_registration_options.double_align_moode);
////      PIPELINE_CTLC(ctrls,  _registration_options.reference_unsharp_alpha, "ref_unsharp_alpha", "", _this->_registration_options.double_align_moode);
////
////      PIPELINE_CTL_GROUP(ctrls, "ECC", "");
////        PIPELINE_CTL(ctrls, _registration_options.enable_ecc, "enabled", "");
////
////        PIPELINE_CTLC(ctrls, _registration_options.ecc_motion_type, "motion_type", "", (_this->ecc_ctls_enabled()));
////
////        PIPELINE_CTLPC2(ctrls, _ecch, method, "ecch method", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, minimum_image_size, "ecch_minimum_image_size", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, maxlevel, "maxlevel ", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, max_iterations, "max_iterations", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, max_eps, "ecc_max_eps", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, min_rho, "ecc_min_rho", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, interpolation, "ecc_interpolation", "", (_this->ecc_ctls_enabled()));
////
////        PIPELINE_CTLPC2(ctrls, _ecch, input_smooth_sigma, "input_smooth_sigma", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, reference_smooth_sigma, "reference_smooth_sigma", "", (_this->ecc_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _ecch, update_step_scale, "update_step_scale", "", (_this->ecc_ctls_enabled()));
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "ECCFLOW", "");
////        PIPELINE_CTL(ctrls, _registration_options.enable_eccflow, "enabled", "");
////        PIPELINE_CTLPC2(ctrls, _eccflow, support_scale, "support_scale", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, downscale_method, "downscale_method", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, min_image_size, "min_image_size", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, max_pyramid_level, "max_pyramid_level", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, scale_factor, "scale_factor", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, max_iterations, "max_iterations", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, noise_level, "noise_level", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, input_smooth_sigma, "input_smooth_sigma", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, reference_smooth_sigma, "reference_smooth_sigma", "", (_this->eccflow_ctls_enabled()));
////        PIPELINE_CTLPC2(ctrls, _eccflow, update_multiplier, "update_multiplier", "", (_this->eccflow_ctls_enabled()));
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTLC(ctrls, _registration_options.min_rho, "min_rho", "", (_this->ecc_ctls_enabled() || _this->eccflow_ctls_enabled()));
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Average options", "");
////    PIPELINE_CTLC(ctrls, _average_options.reference_weight, "reference running_weight", "", (_this->_registration_options.double_align_moode));
////    PIPELINE_CTL(ctrls, _average_options.running_weight, "running_weight", "");
////      PIPELINE_CTL_GROUP(ctrls, "Shapness measure", "");
////        PIPELINE_CTL(ctrls, _average_options.lpg.k, "k", "");
////        PIPELINE_CTL(ctrls, _average_options.lpg.p, "p", "power");
////        PIPELINE_CTL(ctrls, _average_options.lpg.dscale, "dscale", "");
////        PIPELINE_CTL(ctrls, _average_options.lpg.uscale, "uscale", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////    PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
////    PIPELINE_CTL(ctrls, _output_options.display_scale, "display_scale", "");
////    PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////
////
////    PIPELINE_CTL_GROUP(ctrls, "Save accumulated video", "");
////      PIPELINE_CTL(ctrls, _output_options.save_accumulated_video, "save_accumulated_video", "");
////      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_accumulated_video_options,
////          (_this->_output_options.save_accumulated_video));
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "save_reference_video", "");
////      PIPELINE_CTL(ctrls, _output_options.save_reference_video, "save_reference_video", "");
////      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_reference_video_options,
////          (_this->_output_options.save_reference_video));
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}

///

void c_running_average_pipeline::set_double_align_moode(bool v)
{
  _registration_options.double_align_moode = v;
}

bool c_running_average_pipeline::double_align_moode() const
{
  return _registration_options.double_align_moode;
}

///

bool c_running_average_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  bool fOk = false;

  if ( true ) {
    lock_guard lock(mutex());

    if ( _registration_options.double_align_moode ) {
      if ( _average2.compute(display_frame, display_mask) ) {
        fOk = true;
      }
    }
    else {
      if ( _average1.compute(display_frame, display_mask) ) {
        fOk = true;
      }
    }
  }

  if ( fOk && _input_bpp > 1 ) {
    cv::multiply(display_frame, cv::Scalar::all(1 << _input_bpp), display_frame);
  }

  return fOk;
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
  p->_registration_options = this->_registration_options;
  p->_average_options = this->_average_options;
  p->_output_options = this->_output_options;
  p->_ecch.copy_parameters(_ecch);
  //p->ecc_.copy_parameters(ecc_);
  p->_eccflow.copy_parameters(_eccflow);

  return true;
}


bool c_running_average_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path =
      create_output_path(_output_options.output_directory);

  _average1.clear();
  _average2.clear();
  _current_image.release();
  _current_mask.release();

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


  if ( _registration_options.enable_ecc ) {
    _ecc_tramsform = create_image_transform(_registration_options.ecc_motion_type);
    _ecch.set_image_transform(_ecc_tramsform.get());
    _ecch.options() = _registration_options.ecch;
  }

  if( _registration_options.enable_eccflow ) {
    _eccflow.options() = _registration_options.eccflow;
  }

  CF_DEBUG("Output path='%s'", this->_output_path.c_str());


  return true;
}

void c_running_average_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  _ecch.clear();
  _ecc_tramsform.reset();

  _current_image.release();
  _current_mask.release();


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
            _current_image, _current_mask,
            false,
            false);

    if( !fOk ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if ( _current_image.empty() ) {
      // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
      continue;
    }

    _input_bpp = _input_sequence->bpp();

    if( _input_options.input_image_processor && !_input_options.input_image_processor->empty() ) {
      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if ( _registration_options.double_align_moode ) {

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
  if( _average_options.lpg.k < 0 ) {

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
      _registration_options.enable_ecc ||
      _registration_options.enable_eccflow;

  static const auto mkgrayscale =
      [](const cv::Mat & src, cv::Mat & dst) {
        if( src.channels() != 1 ) {
          cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        }
        else if ( &src != &dst ) {
          dst = src;
        }
      };


  if( !enable_registration || _average1.accumulated_frames() < 1 ) {

    if( !average_add(_average1, _current_image, _current_mask, _average_options.running_weight) ) {
      CF_ERROR("average_add() fails");
      return false;
    }
  }
  else {


    rmap.release();

    if( enable_registration ) {
      _average1.compute(image1, mask1);
      mkgrayscale(image1, image1);
      mkgrayscale(_current_image, image2);
    }


    if ( _registration_options.enable_ecc ) {

      _ecch.set_reference_image(image2, mask2);

      if( (has_updates = _ecch.align(image1, mask1)) ) {
        _ecch.image_transform()->create_remap(_ecch.reference_image().size(), rmap);
      }
    }


    if( _registration_options.enable_eccflow ) {
      _eccflow.set_reference_image(image2, mask2);
      has_updates = _eccflow.compute(image1, rmap, mask1);
    }

    if( has_updates ) {
      if( !average_add(_average1, _current_image, _current_mask, _average_options.running_weight, &rmap) ) {
        CF_ERROR("average_add() fails");
        return false;
      }
    }
  }

  if ( has_updates && _output_options.save_accumulated_video  ) {

    const bool fOK =
        add_output_writer(_accumulated_video_writer,
            _output_options.output_accumulated_video_options,
            "accw",
            ".ser");

    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          _accumulated_video_writer.filename().c_str());
      return false;
    }

    _average1.compute(image1, mask1);

    if( !_accumulated_video_writer.write(image1, mask1) ) {
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
      _average_options.reference_weight;

  const double W2 =
      _average_options.running_weight;

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


  if ( !_registration_options.enable_ecc ) {
    CF_ERROR("ECC must be enabled for double align mode");
    return false;
  }

  ////
  mkgrayscale(_current_image, _current_mask,
      image2, mask2);

  if( _average1.accumulated_frames() < 1 ) {

    if( !average_add(_average1, image2, mask2, W1) ) {
      CF_ERROR("average_add() fails");
      return false;
    }

  }
  else {
    _average1.compute(image1, mask1);

    _ecch.set_reference_image(image2, mask2);
    if( !_ecch.align(image1, mask1) ) {
      has_updates = false;
    }
    else {

      if( !average_add(_average1, image2, mask2, W1, &(rmap = _ecch.create_remap())) ) {
        CF_ERROR("average_add() fails");
        return false;
      }

    }
  }

  ////

  if ( has_updates && _average1.accumulated_frames() > W1 )  {

    cv::Mat2f rmap2;

    if( _registration_options.enable_eccflow ) {

      _average1.compute(image1, mask1);

      mkgrayscale(image1, mask1,
          image1, mask1);

      if( _registration_options.reference_unsharp_sigma > 0 && _registration_options.reference_unsharp_alpha > 0 ) {

        unsharp_mask(image1, mask1, image1, _registration_options.reference_unsharp_sigma,
            _registration_options.reference_unsharp_alpha);
      }


      if ( image2.empty() ) {
        mkgrayscale(_current_image, _current_mask,
            image2, mask2);
      }


      rmap2 = rmap.clone();

      _eccflow.set_reference_image(image1, mask1);

      if( !_eccflow.compute(image2, rmap2, mask2) ) {
        has_updates = false;
      }

    }

    if ( has_updates ) {

      if ( rmap2.empty() ) {
       if ( !average_add(_average2, _current_image, _current_mask, W2, &rmap) ) {
          CF_ERROR("average_add(average2_) fails");
          return false;
        }
      }
      else {

        cv::remap(_current_image, image2, rmap2, cv::noArray(), cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

        CF_DEBUG("current_mask_: %dx%d nnz = %d / %d", _current_mask.cols, _current_mask.rows,
            cv::countNonZero(_current_mask),
            _current_mask.size().area());

        if ( _current_mask.empty() ) {
          mask2.release();
          //          cv::remap(cv::Mat1b(current_image_.size(), 255), mask2, rmap2, cv::noArray(),
          //              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else {

          cv::remap(_current_mask, mask2, rmap2, cv::noArray(),
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);

          cv::compare(mask2,  254, mask2, cv::CMP_GE);
        }


        if ( !average_add(_average2, image2, mask2, W2, &rmap) ) {
          CF_ERROR("average_add() fails");
          return false;
        }

      }
    }
  }

  if ( has_updates ) {

    if ( _output_options.save_reference_video && _average1.accumulated_frames() > 0 ) {

      const bool fOK =
          add_output_writer(_reference_video_writer,
              _output_options.output_reference_video_options,
              "accr",
              ".ser");

      if( !fOK ) {
        CF_ERROR("Can not open output writer '%s'",
            _reference_video_writer.filename().c_str());
        return false;
      }

      _average1.compute(image1, mask1);

      if( _registration_options.reference_unsharp_sigma > 0 && _registration_options.reference_unsharp_alpha > 0 ) {

        unsharp_mask(image1, mask1, image1, _registration_options.reference_unsharp_sigma,
            _registration_options.reference_unsharp_alpha);
      }

      if( !_reference_video_writer.write(image1, mask1) ) {
        CF_ERROR("reference_video_writer_.write() fails");
        return false;
      }
    }

    if ( _output_options.save_accumulated_video && _average2.accumulated_frames() > 0 ) {

      const bool fOK =
          add_output_writer(_accumulated_video_writer,
              _output_options.output_accumulated_video_options,
              "accw",
              ".ser");

      if( !fOK ) {
        CF_ERROR("Can not open output writer '%s'",
            _accumulated_video_writer.filename().c_str());
        return false;
      }

      _average2.compute(image1, mask1);

      if( !_accumulated_video_writer.write(image1, mask1) ) {
        CF_ERROR("accumulated_video_writer_.write() fails");
        return false;
      }
    }

  }

  return true;
}

void c_running_average_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  c_lpg_sharpness_measure::create_map(src, dst, _average_options.lpg);
  if( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }
}


