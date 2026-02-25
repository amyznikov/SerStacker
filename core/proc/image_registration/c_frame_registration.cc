/*
 * c_frame_registration.cc
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#include "c_frame_registration.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/normalize.h>
#include <core/io/save_image.h>
#include <core/get_time.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<planetary_disk_derotation_type>()
{
  static const c_enum_member members[] = {
      {planetary_disk_derotation_disabled, "disable", },
      {planetary_disk_derotation_jovian, "jovian", },
      {planetary_disk_derotation_saturn, "saturn", },
      {planetary_disk_derotation_disabled},
  };

  return members;
}


namespace {

void extract_matched_positions(const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch > & matches,
    std::vector<cv::Point2f> * current_positions,
    std::vector<cv::Point2f> * reference_positions )
{
  current_positions->clear(), current_positions->reserve(matches.size());
  reference_positions->clear(), reference_positions->reserve(matches.size());

  for ( const auto & m : matches ) {
    current_positions->emplace_back(current_keypoints[m.queryIdx].pt);
    reference_positions->emplace_back(reference_keypoints[m.trainIdx].pt);
  }
}


/*
 * Pyramid down to specific level
 */
bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);
  for ( int l = 1; l < level; ++l ) {
    cv::pyrDown(dst, dst, cv::Size(), border_mode);
  }
  return true;
}

/*
 * Pyramid up to specific size
 */
bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize =
      image.size();

  if ( inputSize != dstSize ) {

    std::vector<cv::Size> spyramid;

    spyramid.emplace_back(dstSize);

    while ( 42 ) {

      const cv::Size nextSize((spyramid.back().width + 1) / 2,
          (spyramid.back().height + 1) / 2);

      if ( nextSize == inputSize ) {
        break;
      }

      if ( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {

        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);

        return false;
      }

      spyramid.emplace_back(nextSize);
    }

    for ( int i = spyramid.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, spyramid[i]);
    }
  }

  return true;
}

} // namespace

c_frame_registration::c_frame_registration()
{
}

c_frame_registration::c_frame_registration(const c_image_registration_options & options) :
    _options(options)
{
}

const c_image_registration_status & c_frame_registration::status() const
{
  return _current_status;
}

c_image_registration_options & c_frame_registration::options()
{
  return _options;
}

const c_image_registration_options & c_frame_registration::options() const
{
  return _options;
}

void c_frame_registration::set_image_transform(const c_image_transform::sptr & transform)
{
  if( (_image_transform = transform) ) {
    _image_transform_defaut_parameters =
        _image_transform->parameters();
  }

  _ecch.set_image_transform(_image_transform.get());
}

const c_image_transform::sptr & c_frame_registration::image_transform() const
{
  return _image_transform;
}

void c_frame_registration::set_current_timestamp(double v, bool valid)
{
  _current_timestamp = v;
  _has_valid_current_timestamp = valid;
}

double c_frame_registration::current_timestamp() const
{
  return _current_timestamp;
}

bool c_frame_registration::has_valid_current_timestamp() const
{
  return _has_valid_current_timestamp;
}

void c_frame_registration::set_reference_timestamp(double v, bool valid)
{
  _reference_timestamp = v;
  _has_valid_reference_timestamp = valid;
}

double c_frame_registration::reference_timestamp() const
{
  return _reference_timestamp;
}

bool c_frame_registration::has_valid_reference_timestamp() const
{
  return _has_valid_reference_timestamp;
}



bool c_frame_registration::create_image_transfrom()
{
  if( !_image_transform ) {

    _ecch.set_image_transform(nullptr);

    if( !(_image_transform = ::create_image_transform(_options.motion_type)) ) {

      CF_ERROR("create_image_transform(type=%s (%d)) fails",
          toCString(_options.motion_type),
          _options.motion_type);

      return false;
    }

    _image_transform->parameters().copyTo(_image_transform_defaut_parameters);

    if( _options.enable_ecc_registration && _options.ecc.scale > 0 ) {
      _ecch.set_image_transform(_image_transform.get());
    }
  }


  return true;
}


const cv::Mat & c_frame_registration::reference_feature_image() const
{
  return _reference_feature_image;
}

const cv::Mat & c_frame_registration::reference_feature_mask() const
{
  return _reference_feature_mask;
}

const cv::Mat & c_frame_registration::reference_ecc_image() const
{
  return _ecch.reference_image();
}

const cv::Mat & c_frame_registration::reference_ecc_mask() const
{
  return _ecch.reference_mask();
}

const cv::Mat & c_frame_registration::current_feature_image() const
{
  return _current_feature_image;
}

const cv::Mat & c_frame_registration::current_feature_mask() const
{
  return _current_feature_mask;
}

const cv::Mat & c_frame_registration::current_ecc_image() const
{
  return _ecch.current_image();
}

const cv::Mat & c_frame_registration::current_ecc_mask() const
{
  return _ecch.current_mask();
}

//const cv::Mat1f & c_frame_registration::current_transform() const
//{
//  return current_transform_;
//}

const cv::Mat2f & c_frame_registration::current_remap() const
{
  return _current_remap;
}

const c_sparse_feature_extractor_and_matcher::sptr & c_frame_registration::create_sparse_feature_extractor_and_matcher()
{
  if ( _options.enable_feature_registration && !_sparse_feature_extractor_and_matcher ) {

    _sparse_feature_extractor_and_matcher =
        c_sparse_feature_extractor_and_matcher::create(_options.feature_registration.sparse_feature_extractor_and_matcher);

    if ( !_sparse_feature_extractor_and_matcher ) {
      CF_FATAL("c_sparse_feature_extractor_and_matcher::create() fails");
    }
  }

  return _sparse_feature_extractor_and_matcher;
}

const c_sparse_feature_extractor_and_matcher::sptr & c_frame_registration::sparse_feature_extractor_and_matcher() const
{
  return _sparse_feature_extractor_and_matcher;
}

//c_sparse_feature_extractor::sptr c_frame_registration::create_keypoints_extractor() const
//{
//  return create_sparse_feature_extractor(options_.feature_registration.sparse_feature_extractor);
//}
//
//const c_sparse_feature_extractor::sptr & c_frame_registration::set_keypoints_extractor(const c_sparse_feature_extractor::sptr & extractor)
//{
//  return this->keypoints_extractor_ = extractor;
//}
//
//const c_sparse_feature_extractor::sptr & c_frame_registration::keypoints_extractor() const
//{
//  return this->keypoints_extractor_;
//}

//const c_ecc_forward_additive & c_frame_registration::ecc() const
//{
//  return this->ecc_;
//}

const c_ecch & c_frame_registration::ecch() const
{
  return this->_ecch;
}

const c_eccflow & c_frame_registration::eccflow() const
{
  return this->_eccflow;
}

//const c_jovian_derotation & c_frame_registration::jovian_derotation() const
//{
//  return this->jovian_derotation_;
//}
//
//c_jovian_derotation & c_frame_registration::jovian_derotation()
//{
//  return this->jovian_derotation_;
//}

const c_jovian_derotation2 & c_frame_registration::jovian_derotation() const
{
  return this->_jovian_derotation;
}

c_jovian_derotation2 & c_frame_registration::jovian_derotation()
{
  return this->_jovian_derotation;
}

const c_saturn_derotation & c_frame_registration::saturn_derotation() const
{
  return this->_saturn_derotation;
}

c_saturn_derotation & c_frame_registration::saturn_derotation()
{
  return this->_saturn_derotation;
}

void c_frame_registration::set_ecc_image_preprocessor(const ecc_image_preprocessor_function & func)
{
  _ecc_image_preprocessor = func;
}

const c_frame_registration::ecc_image_preprocessor_function & c_frame_registration::ecc_image_preprocessor() const
{
  return _ecc_image_preprocessor;
}

//void c_frame_registration::set_enable_debug(bool v)
//{
//  enable_debug_ = v;
//}
//
//bool c_frame_registration::enable_debug() const
//{
//  return enable_debug_;
//}

void c_frame_registration::set_debug_path(const std::string & v)
{
  _debug_path = v;
//  if ( !debug_path_.empty() && options_.eccflow.enable_debug ) {
//    eccflow_.set_debug_path(ssprintf("%s/eccflow", debug_path_.c_str()));
//  }
//  else {
//    eccflow_.set_debug_path("");
//  }
}

const std::string& c_frame_registration::debug_path() const
{
  return _debug_path;
}

bool c_frame_registration::setup_reference_frame(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;
  cv::Mat eccflow_mask;

  _reference_frame_size = reference_image.size();
  memset(&_current_status.timings, 0, sizeof(_current_status.timings));

  if( _options.enable_feature_registration && _options.feature_registration.image_scale > 0 ) {

    if( !create_feature_image(reference_image, reference_mask, _reference_feature_image, _reference_feature_mask) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    if( !extract_reference_features(_reference_feature_image, _reference_feature_mask) ) {
      CF_ERROR("extract_reference_features() fails");
      return false;
    }
  }

  if( _options.enable_eccflow_registration || (_options.enable_ecc_registration && _options.ecc.scale > 0) ) {

    if( !create_reference_ecc_image(reference_image, reference_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("create_reference_ecc_image() fails");
      return false;
    }

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( _options.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
    }

    if( _options.enable_ecc_registration && _options.ecc.scale > 0 ) {

      _ecch.set_method(_options.ecc.ecc_method);
      _ecch.set_max_eps(_options.ecc.eps);
      _ecch.set_min_rho(_options.ecc.min_rho);
      _ecch.set_input_smooth_sigma(_options.ecc.input_smooth_sigma);
      _ecch.set_reference_smooth_sigma(_options.ecc.reference_smooth_sigma);
      _ecch.set_update_step_scale(_options.ecc.update_step_scale);
      _ecch.set_max_iterations(_options.ecc.max_iterations);
      _ecch.set_maxlevel(_options.ecc.ecch_max_level);
      _ecch.set_minimum_image_size(_options.ecc.ecch_minimum_image_size);

      cv::Mat1f reference_ecc_image;
      cv::Mat1b reference_ecc_mask;

      if( _options.ecc.scale == 1 ) {
        reference_ecc_image = ecc_image;
        reference_ecc_mask = ecc_mask;
      }
      else if( _options.ecc.scale < 1 ) {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), _options.ecc.scale, _options.ecc.scale, cv::INTER_AREA);
      }
      else {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), _options.ecc.scale, _options.ecc.scale,
            cv::INTER_LINEAR_EXACT);
      }

      if( !ecc_mask.empty() && reference_ecc_mask.size() != reference_ecc_image.size() ) {
        cv::resize(ecc_mask, reference_ecc_mask, reference_ecc_image.size(), 0, 0, cv::INTER_AREA);
        cv::compare(reference_ecc_mask, 255, reference_ecc_mask, cv::CMP_GE);
      }

      if( !_ecch.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
        CF_ERROR("ecch_.set_reference_image() fails");
        return false;
      }

    }

    if( _options.enable_eccflow_registration ) {

      _eccflow.set_update_multiplier(_options.eccflow.update_multiplier);
      _eccflow.set_max_iterations(_options.eccflow.max_iterations);
      _eccflow.set_support_scale(_options.eccflow.support_scale);
      _eccflow.set_min_image_size(_options.eccflow.min_image_size);
      _eccflow.set_max_pyramid_level(_options.eccflow.max_pyramid_level);
      _eccflow.set_downscale_method(_options.eccflow.downscale_method);
      _eccflow.set_scale_factor(_options.eccflow.scale_factor);
      _eccflow.set_input_smooth_sigma(_options.eccflow.input_smooth_sigma);
      _eccflow.set_reference_smooth_sigma(_options.eccflow.reference_smooth_sigma);
      _eccflow.set_noise_level(_options.eccflow.noise_level);
      _eccflow.set_scale_factor(_options.eccflow.scale_factor);


      if (eccflow_mask.empty() ) {
        eccflow_mask = ecc_mask;
      }

      if( !_eccflow.set_reference_image(ecc_image, eccflow_mask) ) {
        CF_ERROR("eccflow_.set_reference_image() fails");
        return false;
      }
    }

  }


  switch (_options.planetary_disk_derotation.derotation_type) {
    case planetary_disk_derotation_jovian: {

      const c_jovian_derotation_options & opts =
          _options.planetary_disk_derotation.jovian_derotation;

//      jovian_derotation_.set_detector_options(opts.detector_options);
//      jovian_derotation_.set_min_rotation(opts.min_rotation);
//      jovian_derotation_.set_max_rotation(opts.max_rotation);
//      jovian_derotation_.set_max_pyramid_level(opts.max_pyramid_level);
//      jovian_derotation_.set_num_orientations(opts.num_orientations);
//
//      jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
//          ssprintf("%s/derotation-reference-frame", debug_path_.c_str()));
//      if( !jovian_derotation_.setup_reference_image(reference_image, reference_mask) ) {
//        CF_ERROR("jovian_derotation_.setup_reference_image() fails");
//        return false;
//      }

      _jovian_derotation.detector_options() = opts.detector_options;
      _jovian_derotation.detector_options().auto_location = false;
      if( !_jovian_derotation.detect(reference_image, reference_mask) ) {
        CF_ERROR("jovian_derotation_.detect() fails");
        return false;
      }


      break;
    }


    case planetary_disk_derotation_saturn : {

      const c_saturn_derotation_options & opts =
          _options.planetary_disk_derotation.saturn_derotation;

//      saturn_derotation_.set_detector_options(opts.detector_options);
//
//      if( !saturn_derotation_.setup_reference_image(reference_image, reference_mask) ) {
//        CF_ERROR("saturn_derotation_.setup_reference_image() fails");
//        return false;
//      }
//
//      CF_ERROR("ERROR: planetary_disk_derotation_saturn still not implemented");

      break;
    }

    case planetary_disk_derotation_disabled:
    default:
      break;
  }


  return true;
}

bool c_frame_registration::register_frame(cv::InputArray current_image, cv::InputArray current_mask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  INSTRUMENT_REGION("");

  cv::Mat ecc_image;
  cv::Mat ecc_mask;
  cv::Mat eccflow_mask;
  double rho = -1;

  double start_time = 0, total_time = 0;
  double t0, t1;

  bool have_transform = false;

  start_time = get_realtime_ms();


  if( !create_image_transfrom() ) {
    CF_ERROR("create_image_transfrom() fails");
    return false;
  }

  _image_transform->set_parameters(_image_transform_defaut_parameters);
//  if ( true ) {
//    cv::Vec2f  xT = image_transform_->translation();
//    CF_DEBUG("XXXX: BEG: xT=%g %g", xT(0), xT(1));
//  }

  _current_frame_size = current_image.size();
  memset(&_current_status.timings, 0, sizeof(_current_status.timings));


  /////////////////////////////////////////////////////////////////////////////
  if( _options.enable_feature_registration && _options.feature_registration.image_scale > 0 ) {

    t0 = get_realtime_ms();
    if( !create_feature_image(current_image, current_mask, _current_feature_image, _current_feature_mask) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    _current_status.timings.extract_feature_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( !estimate_feature_transform(_current_feature_image, _current_feature_mask, _image_transform.get()) ) {
      CF_ERROR("estimate_feature_transform() fails");
      return false;
    }

//    if ( true ) {
//      cv::Vec2f  xT = image_transform_->translation();
//      CF_DEBUG("XXXX: FEAT: xT=%g %g", xT(0), xT(1));
//    }

    _current_status.timings.estimate_feature_transform =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }


  /////////////////////////////////////////////////////////////////////////////
  if( _options.enable_eccflow_registration || (_options.enable_ecc_registration && _options.ecc.scale > 0) ) {

    if( !create_current_ecc_image(current_image, current_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("create_current_ecc_image() fails");
      return false;
    }

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( _options.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
    }

//    {
//      double min, max;
//      cv::minMaxLoc(ecc_image, &min, &max);
//      CF_DEBUG("ECC IMAGE: min=%g max=%g", min, max);
//    }

  }

  /////////////////////////////////////////////////////////////////////////////

  if( _options.enable_ecc_registration && _options.ecc.scale > 0 ) {

    t0 = get_realtime_ms();

    _current_status.timings.extract_ecc_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( have_transform && _options.ecc.scale != 1 ) {
      _image_transform->scale_transfrom(_options.ecc.scale);
    }

    cv::Mat1f current_ecc_image;
    cv::Mat1b current_ecc_mask;

    if( _options.ecc.scale == 1 ) {
      current_ecc_image = ecc_image;
      current_ecc_mask = ecc_mask;
    }
    else if( _options.ecc.scale < 1 ) {

      cv::resize(ecc_image, current_ecc_image,
          cv::Size(),
          _options.ecc.scale,
          _options.ecc.scale,
          cv::INTER_AREA);
    }
    else {

      cv::resize(ecc_image, current_ecc_image,
          cv::Size(),
          _options.ecc.scale,
          _options.ecc.scale,
          cv::INTER_LINEAR_EXACT);
    }

    if( !ecc_mask.empty() && current_ecc_mask.size() != current_ecc_image.size() ) {

      cv::resize(ecc_mask, current_ecc_mask,
          current_ecc_image.size(),
          0, 0,
          cv::INTER_LINEAR);

      cv::compare(current_ecc_mask, 255, current_ecc_mask,
          cv::CMP_GE);

    }


    const bool estimate_translation_first =
        _options.motion_type != IMAGE_MOTION_TRANSLATION &&
        _options.ecc.ecch_estimate_translation_first &&
        _options.ecc.ecch_max_level != 0 ;

    if( estimate_translation_first ) {

      c_translation_image_transform transform(_image_transform->translation());
      _ecch.set_image_transform(&transform);

      if( !_ecch.align(current_ecc_image, current_ecc_mask) ) {

        CF_ERROR("ERROR: ecch_.align() fails for translation estimate: "
            "eps=%g/%g iterations=%d/%d",
            _ecch.eps(), _ecch.max_eps(),
            _ecch.num_iterations(), _ecch.max_iterations()
            );

        _ecch.set_image_transform(_image_transform.get());

        return false;
      }

      rho =
          compute_correlation(_ecch.current_image(), _ecch.current_mask(),
              _ecch.reference_image(), _ecch.reference_mask(),
              _ecch.create_remap());

//      if ( true ) {
//        const cv::Vec2f T = image_transform_->translation();
//        CF_DEBUG("ECCH translation first:  rho = %g / %g  (%+g %+g)", rho, ecch_.min_rho(), T[0], T[1]);
//      }

      if ( rho < 0.75 * _ecch.min_rho() ) {
        CF_ERROR("Poor image correlation after align : rho = %g / %g", rho, _ecch.min_rho());
        _ecch.set_image_transform(_image_transform.get());
        return false;
      }

      _image_transform->set_translation(transform.translation());
      _ecch.set_image_transform(_image_transform.get());
    }

    if( !_ecch.align(current_ecc_image, current_ecc_mask) ) {

      CF_ERROR("ecch_.align() fails: eps=%g/%g iterations=%d/%d",
          _ecch.eps(), _ecch.max_eps(),
          _ecch.num_iterations(), _ecch.max_iterations());

      return false;
    }

    rho =
        compute_correlation(_ecch.current_image(), _ecch.current_mask(),
            _ecch.reference_image(), _ecch.reference_mask(),
            _ecch.create_remap());

//    if( true ) {
//      const cv::Vec2f T = image_transform_->translation();
//      CF_DEBUG("ECCH translation first:  rho = %g / %g  (%+g %+g)", rho, ecch_.min_rho(), T[0], T[1]);
//    }

    if( rho < _ecch.min_rho() ) {
      CF_ERROR("Poor image correlation after align : rho = %g / %g", rho, _ecch.min_rho());
      return false;
    }


    if( _options.ecc.scale != 1 ) {
      _image_transform->scale_transfrom(1. / _options.ecc.scale);
    }

    _current_status.timings.ecc_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }

  /////////////////////////////////////////////////////////////////////////////

  // CF_DEBUG("current_remap_: %dx%d", current_remap_.cols, current_remap_.rows);

  t0 = get_realtime_ms();


  if( !_image_transform->create_remap(_reference_frame_size, _current_remap) ) {
    CF_ERROR("image_transform_->create_remap(size=%dx%d) fails",
        _reference_frame_size.width, _reference_frame_size.height);
    return false;
  }

  if( _current_remap.size() != _reference_frame_size ) {
    CF_ERROR("APP BUG: image_transform_->create_remap(size=%dx%d) returns unexpected map size %d%d",
        _reference_frame_size.width, _reference_frame_size.height,
        _current_remap.cols, _current_remap.rows);
    return false;
  }

  _current_status.timings.create_remap =
      get_realtime_ms() - t0;

  ///////////////

  if( _options.planetary_disk_derotation.derotation_type != planetary_disk_derotation_disabled ) {

    cv::Mat tmp_image, tmp_mask;

    cv::remap(current_image,
        tmp_image,
        _current_remap,
        cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::remap(current_mask.empty() ? cv::Mat1b(current_image.size(), 255) : current_mask,
        tmp_mask,
        _current_remap,
        cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::compare(tmp_mask, 250, tmp_mask,
        cv::CMP_GE);

    cv::Mat2f derotation_remap;
    cv::Mat1f current_wmask;
    cv::Mat1b current_bmask;

    switch (_options.planetary_disk_derotation.derotation_type) {
      case planetary_disk_derotation_jovian: {

//        jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
//            ssprintf("%s/derotation", debug_path_.c_str()));

//        if ( !jovian_derotation_.compute(tmp_image, tmp_mask) ) {
//          CF_ERROR("jovian_derotation_.compute() fails");
//          return false;
//        }

//        derotation_remap =
//            jovian_derotation_.current_derotation_remap();
//
//        current_wmask =
//            jovian_derotation_.current_wmask();

        const double current_tstamp_sec =
            _current_timestamp;

        const double reference_tstamp_sec =
            _reference_timestamp;

        CF_DEBUG("current_tstamp_sec=%g reference_tstamp_sec=%g", current_tstamp_sec, reference_tstamp_sec);

        if ( !_jovian_derotation.compute(current_tstamp_sec, reference_tstamp_sec) ) {
          CF_ERROR("jovian_derotation_.compute() fails");
          return false;
        }


        derotation_remap =
            _jovian_derotation.current_derotation_remap();

        CF_DEBUG("derotation_remap: %dx%d", derotation_remap.cols, derotation_remap.rows);

        current_wmask =
            _jovian_derotation.current_wmask();

        CF_DEBUG("current_wmask: %dx%d", current_wmask.cols, current_wmask.rows);

        current_bmask =
            _jovian_derotation.current_bmask();

        CF_DEBUG("current_bmask: %dx%d", current_bmask.cols, current_bmask.rows);

        break;
      }

      case planetary_disk_derotation_saturn: {

        const double current_tstamp_sec = _current_timestamp;
        const double reference_tstamp_sec = _reference_timestamp;

        if ( !_saturn_derotation.compute(current_tstamp_sec, reference_tstamp_sec) ) {
          CF_ERROR("saturn_derotation_.compute() fails");
          return false;
        }


        derotation_remap =
            _saturn_derotation.current_derotation_remap();

        current_wmask =
            _saturn_derotation.current_wmask();

        break;
      }

      default:
        break;
    }

    for( int y = 0; y < _current_remap.rows; ++y ) {
      for( int x = 0; x < _current_remap.cols; ++x ) {
        if( current_wmask[y][x] > 0 ) {
          _current_remap[y][x][0] += derotation_remap[y][x][0] - x;
          _current_remap[y][x][1] += derotation_remap[y][x][1] - y;
        }
      }
    }

    if ( !_debug_path.empty() ) {
      save_image(_current_remap,
          ssprintf("%s/remap_debug/current_remap_after_derotation.flo",
              _debug_path.c_str()));
    }

  }




  ///////////////

  if( _options.enable_eccflow_registration ) {

    t0 = get_realtime_ms();

    _current_status.timings.extract_smflow_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( eccflow_mask.empty() ) {
      eccflow_mask = ecc_mask;
    }

    if( !_eccflow.compute(ecc_image, _current_remap, eccflow_mask) ) {
      CF_ERROR("smflow_.compute() fails");
      return false;
    }

    _current_status.timings.smflow_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;
  }

  ///////////////


  if( dst.needed() || dstmask.needed() ) {

    t0 = get_realtime_ms();

    if( !remap(current_image, dst, current_mask, dstmask) ) {
      CF_ERROR("c_frame_registration::remap() fails");
      return false;
    }

    _current_status.timings.remap =
        (t1 = get_realtime_ms()) - t0;
  }

  total_time = get_realtime_ms() - start_time;

  CF_DEBUG("\nREGISTER: %g ms\n"
      "extract_feature_image: %g ms\n"
      "estimate_feature_transform: %g ms\n"
      "extract_ecc_image: %g ms\n"
      "ecc: rho=%g/%g eps=%g/%g iterations=%d/%d  %g ms (%g ms/it)\n"
      "create_remap: %g ms\n"
      "extract_smflow_image: %g ms\n"
      "smflow_align: %g ms\n"
      "remap : %g ms\n"
      "",
      total_time,
      _current_status.timings.extract_feature_image,
      _current_status.timings.estimate_feature_transform,
      _current_status.timings.extract_ecc_image,

      rho, _ecch.min_rho(),
      _ecch.eps(), _ecch.max_eps(),
      _ecch.num_iterations(), _ecch.max_iterations(),
      _current_status.timings.ecc_align, _current_status.timings.ecc_align / (_ecch.num_iterations() + 1),

      _current_status.timings.create_remap,
      _current_status.timings.extract_smflow_image,
      _current_status.timings.smflow_align,

      _current_status.timings.remap
      );

  return true;
}

// Create image appropriate for sift/surf/orb/etc feature detection
// TODO: Check the Christopher Tsai "Effects of 2-D Preprocessing on Feature Extraction"
//    <https://pdfs.semanticscholar.org/185e/62d607becc8d0ee2409a224a36c58b084ab3.pdf>
bool c_frame_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk) const
{
  cv::Mat tmp;

  if ( src.depth() == CV_8U ) {
    tmp = src.getMat();
  }
  else {

    // dark frame subtraction could produce negative pixel values, star extractor may be sensitive.
    cv::max(src, 0, tmp);
    normalize_minmax(tmp, tmp, 0, 255, srcmsk);
    tmp.convertTo(tmp, CV_8U);
  }

  if ( _options.feature_registration.registration_channel !=  color_channel_dont_change ) {

    const bool fOk =
        extract_channel(tmp, dst, srcmsk, dstmsk,
            _options.ecc_registration_channel,
            _options.feature_registration.image_scale,
            CV_8U,
            1);

    if ( !fOk ) {
      CF_ERROR("extract_channel(channel_=%d scale=%g) fails",
          _options.ecc_registration_channel,
          _options.feature_registration.image_scale);
      return false;
    }
  }
  else if ( _options.feature_registration.image_scale == 0 || _options.feature_registration.image_scale == 1 ) {

    dst.move(tmp);
    srcmsk.copyTo(dstmsk);

  }
  else {

    const double f =
        _options.feature_registration.image_scale;

    cv::resize(tmp, dst, cv::Size(), f, f, cv::INTER_AREA);

    if ( srcmsk.empty() ) {
      dstmsk.release();
    }
    else {
      cv::resize(srcmsk.getMat(), dstmsk, cv::Size(), f, f, cv::INTER_AREA);
      cv::compare(dstmsk.getMat(), 252, dstmsk, cv::CMP_GE);
    }
  }

  return true;
}

bool c_frame_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if( !extract_channel(src, dst, srcmsk, dstmsk, _options.ecc_registration_channel, scale, CV_32F) ) {
    CF_ERROR("extract_channel(registration_channel_=%d) fails", _options.ecc_registration_channel);
    return false;
  }

  if ( _ecc_image_preprocessor ) {
    _ecc_image_preprocessor(dst.getMatRef(),
        dstmsk.getMatRef());
  }

  if( _options.ecc.normalization_scale > 0 && _options.ecc.normalization_noise > 0 ) {
    ecc_normalize_meanstdev(dst.getMat(), dstmsk, dst,
        _options.ecc.normalization_scale,
        _options.ecc.normalization_noise);
  }

  return true;
}

bool c_frame_registration::create_reference_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_frame_registration::create_current_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_frame_registration::insert_planetary_disk_shape(const cv::Mat & src_ecc_image, const cv::Mat & src_mask,
    cv::Mat & dst_ecc_image, cv::Mat & dst_ecc_mask) const
{
  cv::Mat planetary_disk_mask;

  bool fOk =
      simple_planetary_disk_detector(src_ecc_image, src_mask,
          2,
          _options.ecc.planetary_disk_mask_stdev_factor,
          _options.ecc.se_close_size,
          nullptr,
          nullptr,
          &planetary_disk_mask);

  if( !fOk ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }

//  if( options_.ecc.planetary_disk_mask_stdev_factor > 0 ) {
//
//    cv::Scalar m, s;
//
//    cv::meanStdDev(src_ecc_image, m, s, src_mask);
//
//    const double threshold =
//        s[0] * options_.ecc.planetary_disk_mask_stdev_factor;
//
//    cv::bitwise_and(planetary_disk_mask, src_ecc_image > threshold, planetary_disk_mask);
//  }
//  morphological_smooth_close(planetary_disk_mask, planetary_disk_mask, cv::Mat1b(3, 3, 255));
//  geo_fill_holes(planetary_disk_mask, planetary_disk_mask, 8);


  src_ecc_image.copyTo(dst_ecc_image);

  if ( !src_mask.empty() ) {
    cv::bitwise_and(src_mask, ~planetary_disk_mask, dst_ecc_mask );
  }
  else {
    cv::bitwise_not(planetary_disk_mask, dst_ecc_mask);
  }

  if( !_options.enable_eccflow_registration || _options.eccflow.support_scale < 1 ) {
    dst_ecc_image.setTo(1, planetary_disk_mask);
  }
  else {
    /*
     * My current ECC flow implementation produces bugged artifacts when tries to align flat image regions with no gradients.
     * Here is temporary workaround to draw artificial planetary disk with radial intensity gradient from center to edges.
     */
    double min, max;

    cv::distanceTransform(planetary_disk_mask, planetary_disk_mask, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
    cv::minMaxLoc(planetary_disk_mask, &min, &max);
    cv::multiply(planetary_disk_mask, planetary_disk_mask, planetary_disk_mask, 1. / (max * max));

    planetary_disk_mask.copyTo(dst_ecc_image, planetary_disk_mask > FLT_EPSILON);

  }
  return true;
}


bool c_frame_registration::extract_reference_features(cv::InputArray reference_feature_image, cv::InputArray reference_feature_mask)
{
  if( _options.feature_registration.sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk detection

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        _options.feature_registration.sparse_feature_extractor_and_matcher.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image, reference_feature_mask,
              detector_opts.gbsigma,
              0.5,
              2,
              &_planetary_disk_reference_centroid);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image, reference_feature_mask,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              detector_opts.se_close_size,
              nullptr,
              &_planetary_disk_reference_component_rect,
              &_planetary_disk_reference_component_mask,
              &_planetary_disk_reference_centroid);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !_debug_path.empty() ) {
        save_image(_planetary_disk_reference_component_mask,
            ssprintf("%s/reference_component_mask_.tiff",
                _debug_path.c_str()));
      }
    }

    if( _options.feature_registration.image_scale != 1 ) {
      _planetary_disk_reference_centroid /= _options.feature_registration.image_scale;
    }
  }
  else {

    // generic feature detection
    if( !create_sparse_feature_extractor_and_matcher() ) {
      CF_ERROR("create_sparse_feature_extractor_and_matcher() fails");
      return false;
    }

    CF_DEBUG("reference_feature_image: %dx%d %d channels %d depth",
        reference_feature_image.cols(), reference_feature_image.rows(),
        reference_feature_image.channels(),
        reference_feature_image.depth());

    CF_DEBUG("reference_feature_mask: %dx%d %d channels %d depth",
        reference_feature_mask.cols(), reference_feature_mask.rows(),
        reference_feature_mask.channels(),
        reference_feature_mask.depth());

    if( !_sparse_feature_extractor_and_matcher->setup_reference_frame(reference_feature_image, reference_feature_mask) ) {
      CF_ERROR("sparse_feature_extractor_and_matcher_->setup_reference_frame() fails");
      return false;
    }

    CF_DEBUG("reference_keypoints: %zu", _sparse_feature_extractor_and_matcher->referece_keypoints().size());

    if ( _sparse_feature_extractor_and_matcher->referece_keypoints().empty() ) {
      CF_ERROR("No sparse keypoints extracted");
      return false;
    }
  }

  return true;
}

bool c_frame_registration::estimate_feature_transform(cv::InputArray current_feature_image, cv::InputArray current_feature_mask,
    c_image_transform * current_transform)
{
  if( _options.feature_registration.sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk registration

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        _options.feature_registration.sparse_feature_extractor_and_matcher.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image, current_feature_mask,
              detector_opts.gbsigma,
              0.5,
              2,
              &_planetary_disk_current_centroid);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }
    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image, current_feature_mask,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              detector_opts.se_close_size,
              nullptr,
              &_planetary_disk_current_component_rect,
              &_planetary_disk_current_component_mask,
              &_planetary_disk_current_centroid);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !_debug_path.empty() ) {
        save_image(_planetary_disk_current_component_mask,
            ssprintf("%s/current_component_mask_.tiff",
                _debug_path.c_str()));
      }
    }

    if( _options.feature_registration.image_scale != 1 ) {
      _planetary_disk_current_centroid /= _options.feature_registration.image_scale;
    }

    const cv::Point2f T =
        _planetary_disk_current_centroid - _planetary_disk_reference_centroid;

    current_transform->set_translation(cv::Vec2f(T.x, T.y));


  }
  else {

    // generic
    bool fOk =
        _sparse_feature_extractor_and_matcher->match_current_frame(current_feature_image,
            current_feature_mask);

    if( !fOk ) {
      CF_ERROR("sparse_feature_extractor_and_matcher_->match_current_frame() fails");
      return false;
    }

    fOk =
        estimate_image_transform(current_transform,
            _sparse_feature_extractor_and_matcher->matched_current_positions(),
            _sparse_feature_extractor_and_matcher->matched_reference_positions(),
            &_options.feature_registration.estimate_options);

    if( !fOk ) {
      CF_ERROR("estimate_image_transform() fails");
      return false;
    }


    if( _options.feature_registration.triangle_eps > 0 ) {
      if( _sparse_feature_extractor_and_matcher->matcher_type() == FEATURE2D_MATCHER_TRIANGLES ) {

        CF_DEBUG("Estimated using %zu point pairs",
            _sparse_feature_extractor_and_matcher->matched_current_positions().size());

        // Get more matches if possible

        const size_t num_referece_keypoints = _sparse_feature_extractor_and_matcher->referece_keypoints().size();
        const size_t num_current_keypoints = _sparse_feature_extractor_and_matcher->current_keypoints().size();
        const size_t available_pts = std::min(num_referece_keypoints, num_current_keypoints);

        if( available_pts > _sparse_feature_extractor_and_matcher->matched_current_positions().size() ) {

          std::vector<cv::Point2f> rpos, rrpos;
          std::vector<cv::Point2f> mrpos, mcpos;

          rpos.reserve(_sparse_feature_extractor_and_matcher->referece_keypoints().size());
          for( const cv::KeyPoint & kp : _sparse_feature_extractor_and_matcher->referece_keypoints() ) {
            rpos.emplace_back(kp.pt);
          }

          if( !current_transform->remap(rpos, rrpos) ) {
            CF_ERROR("current_transform->remap(rpos, rrpos) fails");
            return false;
          }

          const std::vector<cv::KeyPoint> & cpts = _sparse_feature_extractor_and_matcher->current_keypoints();
          const double eps = _options.feature_registration.triangle_eps * _options.feature_registration.triangle_eps;

          for( int i = 0, ni = (int) rrpos.size(); i < ni; ++i ) {
            const cv::Point2f rrp = rrpos[i];

            double d2min = DBL_MAX;
            int jmin = -1;

            for( int j = 0, nj = (int) cpts.size(); j < nj; ++j ) {
              const cv::Point2f cp = cpts[j].pt;
              const double d2 = (cp.x - rrp.x) * (cp.x - rrp.x) + (cp.y - rrp.y) * (cp.y - rrp.y);
              if( d2 < d2min ) {
                d2min = d2;
                jmin = j;
              }
            }

            if( jmin >= 0 && d2min < eps ) {
              mrpos.emplace_back(rpos[i]);
              mcpos.emplace_back(cpts[jmin].pt);
            }
          }

          fOk =
              estimate_image_transform(current_transform, mcpos, mrpos,
                  &_options.feature_registration.estimate_options);

          if( !fOk ) {
            CF_ERROR("estimate_image_transform(mcpos, mrpos) fails");
            return false;
          }

          CF_DEBUG("Re-estimated using %zu point pairs", mcpos.size());
        }
      }
    }

    if( _options.feature_registration.image_scale != 1 ) {
      current_transform->scale_transfrom(1. / _options.feature_registration.image_scale);
    }
  }

  return true;
}

bool c_frame_registration::base_remap(const cv::Mat2f & rmap,
    cv::InputArray _src, cv::OutputArray dst,
    cv::InputArray _src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{
  cv::Mat src =
      _src.getMat();

  cv::Mat src_mask =
      _src_mask.getMat();

  cv::Size src_size;
  const cv::Scalar * border_value_ptr;

  if ( !src.empty() ) {
    src_size = src.size();
  }
  else if ( !src_mask.empty() ) {
    src_size = src_mask.size();
  }
  else {
    src_size = _current_remap.size();
  }

  if ( dst.needed() || dst_mask.needed() ) {
    if ( _current_remap.size() != src_size ) {
      CF_ERROR("WARNING: src size (%dx%d) and remap size (%dx%d) different",
          src_size.width, src_size.height,
          _current_remap.cols, _current_remap.rows);
    }
  }

  if ( interpolation_flags < 0 ) {
    interpolation_flags = _options.interpolation;
  }

  if ( border_mode >= 0 ) {
    border_value_ptr = &border_value;
  }
  else {
    border_mode = _options.border_mode;
    border_value_ptr = &_options.border_value;
  }



  if( dst.needed() ) {

    cv::remap(src, dst, rmap, cv::noArray(), interpolation_flags,
        border_mode, *border_value_ptr);
  }

  if ( dst_mask.needed() ) {

    if ( !src_mask.empty() ) {
      cv::remap(src_mask, dst_mask, rmap, cv::noArray(),
          interpolation_flags, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    }
    else {
      cv::remap(cv::Mat1b(src_size, 255), dst_mask, rmap, cv::noArray(),
          interpolation_flags, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    }

    cv::Mat & out_mask =
        dst_mask.getMatRef();

    if ( out_mask.depth() == CV_8U ) {
      // reduce mask edge artifacts
      cv::compare(out_mask, 255, out_mask, cv::CMP_GE);
      cv::erode(out_mask, out_mask, cv::Mat1b(5, 5, 255), cv::Point(-1, -1), 1,
          cv::BORDER_CONSTANT, cv::Scalar::all(255));
    }

    if ( _options.planetary_disk_derotation.derotation_type != planetary_disk_derotation_disabled ) {

      // size must be referece_image.size()
      cv::Mat new_mask;

      if ( out_mask.depth() == CV_32F ) {
        out_mask.copyTo(new_mask);
      }
      else {
        out_mask.convertTo(new_mask, CV_32F, 1./255);
      }

      switch (_options.planetary_disk_derotation.derotation_type) {
        case planetary_disk_derotation_jovian:
          _jovian_derotation.current_wmask().copyTo(new_mask,
              _jovian_derotation.planetary_disk_ellipse_mask());
          break;
        case planetary_disk_derotation_saturn:
          _saturn_derotation.current_wmask().copyTo(new_mask,
              _saturn_derotation.planetary_disk_ellipse_mask());
          break;
      }


      static int iitest = 0;

      if ( !_debug_path.empty() ) {
        save_image(out_mask, ssprintf("%s/remap_debug/orig_mask.%03d.tiff", _debug_path.c_str(), iitest));
        save_image(new_mask, ssprintf("%s/remap_debug/new_mask.%03d.tiff", _debug_path.c_str(), iitest));
      }

      cv::GaussianBlur(new_mask, new_mask, cv::Size(), 2, 2);
      new_mask.setTo(0, ~out_mask);
      if ( !_debug_path.empty() ) {
        save_image(new_mask, ssprintf("%s/remap_debug/new_maskz.%03d.tiff", _debug_path.c_str(), iitest));
      }

      dst_mask.move(new_mask);

      ++iitest;
    }


  }

  return true;
}

bool c_frame_registration::custom_remap(const cv::Mat2f & rmap,
    cv::InputArray _src, cv::OutputArray dst,
    cv::InputArray _src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{
  return base_remap(rmap,
      _src, dst,
      _src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);

}


bool c_frame_registration::remap(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{
  return custom_remap(_current_remap,
      src, dst,
      src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);
}

