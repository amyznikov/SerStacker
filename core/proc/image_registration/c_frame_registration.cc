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
const c_enum_member * members_of<master_frame_selection_method>()
{
  static const c_enum_member members[] = {
      {master_frame_specific_index, "specific_index", },
      {master_frame_middle_index, "middle_index", },
      {master_frame_best_of_100_in_middle, "best_of_100_in_middle", },
      {master_frame_specific_index },
  };

  return members;
}

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
    options_(options)
{
}

const c_image_registration_status & c_frame_registration::status() const
{
  return current_status_;
}

c_image_registration_options & c_frame_registration::options()
{
  return options_;
}

const c_image_registration_options & c_frame_registration::options() const
{
  return options_;
}

void c_frame_registration::set_image_transform(const c_image_transform::sptr & transform)
{
  if( (image_transform_ = transform) ) {
    image_transform_defaut_parameters_ =
        image_transform_->parameters();
  }

  ecch_.set_image_transform(image_transform_.get());
}

const c_image_transform::sptr & c_frame_registration::image_transform() const
{
  return image_transform_;
}

bool c_frame_registration::create_image_transfrom()
{
  if( !image_transform_ ) {

    ecch_.set_image_transform(nullptr);

    if( !(image_transform_ = ::create_image_transform(options_.motion_type)) ) {

      CF_ERROR("create_image_transform(type=%s (%d)) fails",
          toCString(options_.motion_type),
          options_.motion_type);

      return false;
    }

    image_transform_->parameters().copyTo(image_transform_defaut_parameters_);

    if( options_.ecc.enabled && options_.ecc.scale > 0 ) {
      ecch_.set_image_transform(image_transform_.get());
    }
  }


  return true;
}


const cv::Mat & c_frame_registration::reference_feature_image() const
{
  return reference_feature_image_;
}

const cv::Mat & c_frame_registration::reference_feature_mask() const
{
  return reference_feature_mask_;
}

const cv::Mat & c_frame_registration::reference_ecc_image() const
{
  return ecch_.reference_image();
}

const cv::Mat & c_frame_registration::reference_ecc_mask() const
{
  return ecch_.reference_mask();
}

const cv::Mat & c_frame_registration::current_feature_image() const
{
  return current_feature_image_;
}

const cv::Mat & c_frame_registration::current_feature_mask() const
{
  return current_feature_mask_;
}

const cv::Mat & c_frame_registration::current_ecc_image() const
{
  return ecch_.current_image();
}

const cv::Mat & c_frame_registration::current_ecc_mask() const
{
  return ecch_.current_mask();
}

//const cv::Mat1f & c_frame_registration::current_transform() const
//{
//  return current_transform_;
//}

const cv::Mat2f & c_frame_registration::current_remap() const
{
  return current_remap_;
}

const c_sparse_feature_extractor_and_matcher::sptr & c_frame_registration::create_sparse_feature_extractor_and_matcher()
{
  if ( options_.feature_registration.enabled && !sparse_feature_extractor_and_matcher_ ) {

    sparse_feature_extractor_and_matcher_ =
        c_sparse_feature_extractor_and_matcher::create(options_.feature_registration.sparse_feature_extractor_and_matcher);

    if ( !sparse_feature_extractor_and_matcher_ ) {
      CF_FATAL("c_sparse_feature_extractor_and_matcher::create() fails");
    }
  }

  return sparse_feature_extractor_and_matcher_;
}

const c_sparse_feature_extractor_and_matcher::sptr & c_frame_registration::sparse_feature_extractor_and_matcher() const
{
  return sparse_feature_extractor_and_matcher_;
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
  return this->ecch_;
}

const c_eccflow & c_frame_registration::eccflow() const
{
  return this->eccflow_;
}

const c_jovian_derotation & c_frame_registration::jovian_derotation() const
{
  return this->jovian_derotation_;
}

c_jovian_derotation & c_frame_registration::jovian_derotation()
{
  return this->jovian_derotation_;
}

const c_saturn_derotation & c_frame_registration::saturn_derotation() const
{
  return this->saturn_derotation_;
}

c_saturn_derotation & c_frame_registration::saturn_derotation()
{
  return this->saturn_derotation_;
}

void c_frame_registration::set_ecc_image_preprocessor(const ecc_image_preprocessor_function & func)
{
  ecc_image_preprocessor_ = func;
}

const c_frame_registration::ecc_image_preprocessor_function & c_frame_registration::ecc_image_preprocessor() const
{
  return ecc_image_preprocessor_;
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
  debug_path_ = v;
//  if ( !debug_path_.empty() && options_.eccflow.enable_debug ) {
//    eccflow_.set_debug_path(ssprintf("%s/eccflow", debug_path_.c_str()));
//  }
//  else {
//    eccflow_.set_debug_path("");
//  }
}

const std::string& c_frame_registration::debug_path() const
{
  return debug_path_;
}

bool c_frame_registration::setup_reference_frame(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;
  cv::Mat eccflow_mask;

  reference_frame_size_ = reference_image.size();
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));

  if( options_.feature_registration.enabled && options_.feature_registration.scale > 0 ) {

    if( !create_feature_image(reference_image, reference_mask, reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    if( !extract_reference_features(reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("extract_reference_features() fails");
      return false;
    }
  }

  if( options_.eccflow.enabled || (options_.ecc.enabled && options_.ecc.scale > 0) ) {

    if( !create_reference_ecc_image(reference_image, reference_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("create_reference_ecc_image() fails");
      return false;
    }

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( options_.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
    }

    if( options_.ecc.enabled && options_.ecc.scale > 0 ) {

      ecch_.set_method(options_.ecc.ecc_method);
      ecch_.set_max_eps(options_.ecc.eps);
      ecch_.set_min_rho(options_.ecc.min_rho);
      ecch_.set_input_smooth_sigma(options_.ecc.input_smooth_sigma);
      ecch_.set_reference_smooth_sigma(options_.ecc.reference_smooth_sigma);
      ecch_.set_update_step_scale(options_.ecc.update_step_scale);
      ecch_.set_max_iterations(options_.ecc.max_iterations);
      ecch_.set_maxlevel(options_.ecc.ecch_max_level);
      ecch_.set_minimum_image_size(options_.ecc.ecch_minimum_image_size);

      cv::Mat1f reference_ecc_image;
      cv::Mat1b reference_ecc_mask;

      if( options_.ecc.scale == 1 ) {
        reference_ecc_image = ecc_image;
        reference_ecc_mask = ecc_mask;
      }
      else if( options_.ecc.scale < 1 ) {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale, cv::INTER_AREA);
      }
      else {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale,
            cv::INTER_LINEAR_EXACT);
      }

      if( !ecc_mask.empty() && reference_ecc_mask.size() != reference_ecc_image.size() ) {
        cv::resize(ecc_mask, reference_ecc_mask, reference_ecc_image.size(), 0, 0, cv::INTER_AREA);
        cv::compare(reference_ecc_mask, 255, reference_ecc_mask, cv::CMP_GE);
      }

      if( !ecch_.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
        CF_ERROR("ecch_.set_reference_image() fails");
        return false;
      }

    }

    if( options_.eccflow.enabled ) {

      eccflow_.set_update_multiplier(options_.eccflow.update_multiplier);
      eccflow_.set_max_iterations(options_.eccflow.max_iterations);
      eccflow_.set_support_scale(options_.eccflow.support_scale);
      eccflow_.set_min_image_size(options_.eccflow.min_image_size);
      eccflow_.set_max_pyramid_level(options_.eccflow.max_pyramid_level);
      eccflow_.set_downscale_method(options_.eccflow.downscale_method);
      eccflow_.set_scale_factor(options_.eccflow.scale_factor);
      eccflow_.set_input_smooth_sigma(options_.eccflow.input_smooth_sigma);
      eccflow_.set_reference_smooth_sigma(options_.eccflow.reference_smooth_sigma);
      eccflow_.set_noise_level(options_.eccflow.noise_level);
      eccflow_.set_scale_factor(options_.eccflow.scale_factor);


      if (eccflow_mask.empty() ) {
        eccflow_mask = ecc_mask;
      }

      if( !eccflow_.set_reference_image(ecc_image, eccflow_mask) ) {
        CF_ERROR("eccflow_.set_reference_image() fails");
        return false;
      }
    }

  }


  switch (options_.planetary_disk_derotation.derotation_type) {
    case planetary_disk_derotation_jovian: {

      const c_jovian_derotation_options & opts =
          options_.planetary_disk_derotation.jovian_derotation;

      jovian_derotation_.set_detector_options(opts.detector_options);
      jovian_derotation_.set_min_rotation(opts.min_rotation);
      jovian_derotation_.set_max_rotation(opts.max_rotation);
      jovian_derotation_.set_max_pyramid_level(opts.max_pyramid_level);
      jovian_derotation_.set_num_orientations(opts.num_orientations);

      jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
          ssprintf("%s/derotation-reference-frame", debug_path_.c_str()));

      if( !jovian_derotation_.setup_reference_image(reference_image, reference_mask) ) {
        CF_ERROR("jovian_derotation_.setup_reference_image() fails");
        return false;
      }

      break;
    }


    case planetary_disk_derotation_saturn : {

      const c_saturn_derotation_options & opts =
          options_.planetary_disk_derotation.saturn_derotation;

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

  image_transform_->set_parameters(image_transform_defaut_parameters_);
//  if ( true ) {
//    cv::Vec2f  xT = image_transform_->translation();
//    CF_DEBUG("XXXX: BEG: xT=%g %g", xT(0), xT(1));
//  }

  current_frame_size_ = current_image.size();
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));


  /////////////////////////////////////////////////////////////////////////////
  if( options_.feature_registration.enabled && options_.feature_registration.scale > 0 ) {

    t0 = get_realtime_ms();
    if( !create_feature_image(current_image, current_mask, current_feature_image_, current_feature_mask_) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    current_status_.timings.extract_feature_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( !estimate_feature_transform(current_feature_image_, current_feature_mask_, image_transform_.get()) ) {
      CF_ERROR("estimate_feature_transform() fails");
      return false;
    }

//    if ( true ) {
//      cv::Vec2f  xT = image_transform_->translation();
//      CF_DEBUG("XXXX: FEAT: xT=%g %g", xT(0), xT(1));
//    }

    current_status_.timings.estimate_feature_transform =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }


  /////////////////////////////////////////////////////////////////////////////
  if( options_.eccflow.enabled || (options_.ecc.enabled && options_.ecc.scale > 0) ) {

    if( !create_current_ecc_image(current_image, current_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("create_current_ecc_image() fails");
      return false;
    }

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( options_.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
    }

//    {
//      double min, max;
//      cv::minMaxLoc(ecc_image, &min, &max);
//      CF_DEBUG("ECC IMAGE: min=%g max=%g", min, max);
//    }

  }

  /////////////////////////////////////////////////////////////////////////////

  if( options_.ecc.enabled && options_.ecc.scale > 0 ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_ecc_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( have_transform && options_.ecc.scale != 1 ) {
      image_transform_->scale_transfrom(options_.ecc.scale);
    }

    cv::Mat1f current_ecc_image;
    cv::Mat1b current_ecc_mask;

    if( options_.ecc.scale == 1 ) {
      current_ecc_image = ecc_image;
      current_ecc_mask = ecc_mask;
    }
    else if( options_.ecc.scale < 1 ) {

      cv::resize(ecc_image, current_ecc_image,
          cv::Size(),
          options_.ecc.scale,
          options_.ecc.scale,
          cv::INTER_AREA);
    }
    else {

      cv::resize(ecc_image, current_ecc_image,
          cv::Size(),
          options_.ecc.scale,
          options_.ecc.scale,
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
        options_.motion_type != IMAGE_MOTION_TRANSLATION &&
        options_.ecc.ecch_estimate_translation_first &&
        options_.ecc.ecch_max_level != 0 ;

    if( estimate_translation_first ) {

      c_translation_image_transform transform(image_transform_->translation());
      ecch_.set_image_transform(&transform);

      if( !ecch_.align(current_ecc_image, current_ecc_mask) ) {

        CF_ERROR("ERROR: ecch_.align() fails for translation estimate: "
            "eps=%g/%g iterations=%d/%d",
            ecch_.eps(), ecch_.max_eps(),
            ecch_.num_iterations(), ecch_.max_iterations()
            );

        ecch_.set_image_transform(image_transform_.get());

        return false;
      }

      rho =
          compute_correlation(ecch_.current_image(), ecch_.current_mask(),
              ecch_.reference_image(), ecch_.reference_mask(),
              ecch_.create_remap());

//      if ( true ) {
//        const cv::Vec2f T = image_transform_->translation();
//        CF_DEBUG("ECCH translation first:  rho = %g / %g  (%+g %+g)", rho, ecch_.min_rho(), T[0], T[1]);
//      }

      if ( rho < 0.75 * ecch_.min_rho() ) {
        CF_ERROR("Poor image correlation after align : rho = %g / %g", rho, ecch_.min_rho());
        ecch_.set_image_transform(image_transform_.get());
        return false;
      }

      image_transform_->set_translation(transform.translation());
      ecch_.set_image_transform(image_transform_.get());
    }

    if( !ecch_.align(current_ecc_image, current_ecc_mask) ) {

      CF_ERROR("ecch_.align() fails: eps=%g/%g iterations=%d/%d",
          ecch_.eps(), ecch_.max_eps(),
          ecch_.num_iterations(), ecch_.max_iterations());

      return false;
    }

    rho =
        compute_correlation(ecch_.current_image(), ecch_.current_mask(),
            ecch_.reference_image(), ecch_.reference_mask(),
            ecch_.create_remap());

//    if( true ) {
//      const cv::Vec2f T = image_transform_->translation();
//      CF_DEBUG("ECCH translation first:  rho = %g / %g  (%+g %+g)", rho, ecch_.min_rho(), T[0], T[1]);
//    }

    if( rho < ecch_.min_rho() ) {
      CF_ERROR("Poor image correlation after align : rho = %g / %g", rho, ecch_.min_rho());
      return false;
    }


    if( options_.ecc.scale != 1 ) {
      image_transform_->scale_transfrom(1. / options_.ecc.scale);
    }

    current_status_.timings.ecc_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }

  /////////////////////////////////////////////////////////////////////////////

  // CF_DEBUG("current_remap_: %dx%d", current_remap_.cols, current_remap_.rows);

  t0 = get_realtime_ms();


  if( !image_transform_->create_remap(reference_frame_size_, current_remap_) ) {
    CF_ERROR("image_transform_->create_remap(size=%dx%d) fails",
        reference_frame_size_.width, reference_frame_size_.height);
    return false;
  }

  if( current_remap_.size() != reference_frame_size_ ) {
    CF_ERROR("APP BUG: image_transform_->create_remap(size=%dx%d) returns unexpected map size %d%d",
        reference_frame_size_.width, reference_frame_size_.height,
        current_remap_.cols, current_remap_.rows);
    return false;
  }

  current_status_.timings.create_remap =
      get_realtime_ms() - t0;

  ///////////////

  if( options_.planetary_disk_derotation.derotation_type != planetary_disk_derotation_disabled ) {

    cv::Mat tmp_image, tmp_mask;

    cv::remap(current_image,
        tmp_image,
        current_remap_,
        cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::remap(current_mask.empty() ? cv::Mat1b(current_image.size(), 255) : current_mask,
        tmp_mask,
        current_remap_,
        cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::compare(tmp_mask, 250, tmp_mask,
        cv::CMP_GE);

    cv::Mat2f derotation_remap;
    cv::Mat1f current_wmask;

    switch (options_.planetary_disk_derotation.derotation_type) {
      case planetary_disk_derotation_jovian:

        jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
            ssprintf("%s/derotation", debug_path_.c_str()));

        if ( !jovian_derotation_.compute(tmp_image, tmp_mask) ) {
          CF_ERROR("jovian_derotation_.compute() fails");
          return false;
        }

        derotation_remap =
            jovian_derotation_.current_derotation_remap();

        current_wmask =
            jovian_derotation_.current_wmask();

        break;

      case planetary_disk_derotation_saturn:
        derotation_remap =
            saturn_derotation_.current_derotation_remap();

        current_wmask =
            saturn_derotation_.current_wmask();

        break;
      default:
        break;
    }

    for( int y = 0; y < current_remap_.rows; ++y ) {
      for( int x = 0; x < current_remap_.cols; ++x ) {
        if( current_wmask[y][x] > 0 ) {
          current_remap_[y][x][0] += derotation_remap[y][x][0] - x;
          current_remap_[y][x][1] += derotation_remap[y][x][1] - y;
        }
      }
    }

    if ( !debug_path_.empty() ) {
      save_image(current_remap_,
          ssprintf("%s/remap_debug/current_remap_after_derotation.flo",
              debug_path_.c_str()));
    }

  }




  ///////////////

  if( options_.eccflow.enabled ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_smflow_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( eccflow_mask.empty() ) {
      eccflow_mask = ecc_mask;
    }

    if( !eccflow_.compute(ecc_image, current_remap_, eccflow_mask) ) {
      CF_ERROR("smflow_.compute() fails");
      return false;
    }

    current_status_.timings.smflow_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;
  }

  ///////////////


  if( dst.needed() || dstmask.needed() ) {

    t0 = get_realtime_ms();

    if( !remap(current_image, dst, current_mask, dstmask) ) {
      CF_ERROR("c_frame_registration::remap() fails");
      return false;
    }

    current_status_.timings.remap =
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
      current_status_.timings.extract_feature_image,
      current_status_.timings.estimate_feature_transform,
      current_status_.timings.extract_ecc_image,

      rho, ecch_.min_rho(),
      ecch_.eps(), ecch_.max_eps(),
      ecch_.num_iterations(), ecch_.max_iterations(),
      current_status_.timings.ecc_align, current_status_.timings.ecc_align / (ecch_.num_iterations() + 1),

      current_status_.timings.create_remap,
      current_status_.timings.extract_smflow_image,
      current_status_.timings.smflow_align,

      current_status_.timings.remap
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
  }

  const bool fOk =
      extract_channel(tmp, dst, srcmsk, dstmsk,
          options_.registration_channel,
          options_.feature_registration.scale,
          CV_8U,
          1);

  if ( !fOk ) {
    CF_ERROR("extract_channel(channel_=%d scale=%g) fails",
        options_.registration_channel,
        options_.feature_registration.scale);
    return false;
  }

  return true;
}

bool c_frame_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if( !extract_channel(src, dst, srcmsk, dstmsk, options_.registration_channel, scale, CV_32F) ) {
    CF_ERROR("extract_channel(registration_channel_=%d) fails", options_.registration_channel);
    return false;
  }

  if ( ecc_image_preprocessor_ ) {
    ecc_image_preprocessor_(dst.getMatRef(),
        dstmsk.getMatRef());
  }

  if( options_.ecc.normalization_scale > 0 && options_.ecc.normalization_noise > 0 ) {
    ecc_normalize_meanstdev(dst.getMat(), dstmsk, dst,
        options_.ecc.normalization_scale,
        options_.ecc.normalization_noise);
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
          nullptr,
          2,
          options_.ecc.planetary_disk_mask_stdev_factor,
          options_.ecc.se_close_size,
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

  if( !options_.eccflow.enabled || options_.eccflow.support_scale < 1 ) {
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
  if( options_.feature_registration.sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk detection

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        options_.feature_registration.sparse_feature_extractor_and_matcher.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image,
              reference_feature_mask,
              &planetary_disk_reference_centroid_,
              detector_opts.gbsigma);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image,
              reference_feature_mask,
              nullptr,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              detector_opts.se_close_size,
              &planetary_disk_reference_component_rect_,
              &planetary_disk_reference_component_mask_,
              &planetary_disk_reference_centroid_);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !debug_path_.empty() ) {
        save_image(planetary_disk_reference_component_mask_,
            ssprintf("%s/reference_component_mask_.tiff",
                debug_path_.c_str()));
      }
    }

    if( options_.feature_registration.scale != 1 ) {
      planetary_disk_reference_centroid_ /= options_.feature_registration.scale;
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

    if( !sparse_feature_extractor_and_matcher_->setup_reference_frame(reference_feature_image, reference_feature_mask) ) {
      CF_ERROR("sparse_feature_extractor_and_matcher_->setup_reference_frame() fails");
      return false;
    }

    CF_DEBUG("reference_keypoints: %zu", sparse_feature_extractor_and_matcher_->referece_keypoints().size());

    if ( sparse_feature_extractor_and_matcher_->referece_keypoints().empty() ) {
      CF_ERROR("No sparse keypoints extracted");
      return false;
    }
  }

  return true;
}

bool c_frame_registration::estimate_feature_transform(cv::InputArray current_feature_image, cv::InputArray current_feature_mask,
    c_image_transform * current_transform)
{
  if( options_.feature_registration.sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk registration

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        options_.feature_registration.sparse_feature_extractor_and_matcher.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image,
              current_feature_mask,
              &planetary_disk_current_centroid_,
              detector_opts.gbsigma);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }
    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image,
              current_feature_mask,
              nullptr,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              detector_opts.se_close_size,
              &planetary_disk_current_component_rect_,
              &planetary_disk_current_component_mask_,
              &planetary_disk_current_centroid_);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !debug_path_.empty() ) {
        save_image(planetary_disk_current_component_mask_,
            ssprintf("%s/current_component_mask_.tiff",
                debug_path_.c_str()));
      }
    }

    if( options_.feature_registration.scale != 1 ) {
      planetary_disk_current_centroid_ /= options_.feature_registration.scale;
    }

    const cv::Point2f T =
        planetary_disk_current_centroid_ - planetary_disk_reference_centroid_;

    current_transform->set_translation(cv::Vec2f(T.x, T.y));


  }
  else {

    // generic
    bool fOk =
        sparse_feature_extractor_and_matcher_->match_current_frame(current_feature_image,
            current_feature_mask);

    if( !fOk ) {
      CF_ERROR("sparse_feature_extractor_and_matcher_->match_current_frame() fails");
      return false;
    }

    fOk =
        estimate_image_transform(current_transform,
            sparse_feature_extractor_and_matcher_->matched_current_positions(),
            sparse_feature_extractor_and_matcher_->matched_reference_positions(),
            &options_.feature_registration.estimate_options);
    if( !fOk ) {
      CF_ERROR("estimate_image_transform() fails");
      return false;
    }


    if( options_.feature_registration.scale != 1 ) {
      current_transform->scale_transfrom(1. / options_.feature_registration.scale);
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
  cv::Mat src = _src.getMat();
  cv::Mat src_mask = _src_mask.getMat();

  cv::Size src_size;
  const cv::Scalar * border_value_ptr;

  if ( !src.empty() ) {
    src_size = src.size();
  }
  else if ( !src_mask.empty() ) {
    src_size = src_mask.size();
  }
  else {
    src_size = current_remap_.size();
  }

  if ( dst.needed() || dst_mask.needed() ) {
    if ( current_remap_.size() != src_size ) {
      CF_ERROR("WARNING: src size (%dx%d) and remap size (%dx%d) different",
          src_size.width, src_size.height,
          current_remap_.cols, current_remap_.rows);
    }
  }

  if ( interpolation_flags < 0 ) {
    interpolation_flags = options_.interpolation;
  }

  if ( border_mode >= 0 ) {
    border_value_ptr = &border_value;
  }
  else {
    border_mode = options_.border_mode;
    border_value_ptr = &options_.border_value;
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

    if ( options_.planetary_disk_derotation.derotation_type != planetary_disk_derotation_disabled ) {

      // size must be referece_image.size()
      cv::Mat new_mask;

      if ( out_mask.depth() == CV_32F ) {
        out_mask.copyTo(new_mask);
      }
      else {
        out_mask.convertTo(new_mask, CV_32F, 1./255);
      }

      switch (options_.planetary_disk_derotation.derotation_type) {
        case planetary_disk_derotation_jovian:
          jovian_derotation_.current_wmask().copyTo(new_mask,
              jovian_derotation_.planetary_disk_ellipse_mask());
          break;
        case planetary_disk_derotation_saturn:
          saturn_derotation_.current_wmask().copyTo(new_mask,
              saturn_derotation_.planetary_disk_ellipse_mask());
          break;
      }


      static int iitest = 0;

      if ( !debug_path_.empty() ) {
        save_image(out_mask, ssprintf("%s/remap_debug/orig_mask.%03d.tiff", debug_path_.c_str(), iitest));
        save_image(new_mask, ssprintf("%s/remap_debug/new_mask.%03d.tiff", debug_path_.c_str(), iitest));
      }

      cv::GaussianBlur(new_mask, new_mask, cv::Size(), 2, 2);
      new_mask.setTo(0, ~out_mask);
      if ( !debug_path_.empty() ) {
        save_image(new_mask, ssprintf("%s/remap_debug/new_maskz.%03d.tiff", debug_path_.c_str(), iitest));
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
  return custom_remap(current_remap_,
      src, dst,
      src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);
}

