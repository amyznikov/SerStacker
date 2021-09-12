/*
 * c_frame_registration.cc
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#include "c_frame_registration.h"

#include <core/get_time.h>
#include <core/debug.h>

c_frame_registration::c_frame_registration()
{
}

c_frame_registration::c_frame_registration(const c_frame_registration_base_options & opts)
  : base_options_(opts)
{
}

const c_frame_registration_status & c_frame_registration::status() const
{
  return current_status_;
}

c_frame_registration_base_options & c_frame_registration::base_options()
{
  return base_options_;
}

const c_frame_registration_base_options & c_frame_registration::base_options() const
{
  return base_options_;
}

void c_frame_registration::set_motion_type(enum ECC_MOTION_TYPE v)
{
  base_options_.motion_type = v;
}

enum ECC_MOTION_TYPE c_frame_registration::motion_type() const
{
  return base_options_.motion_type;
}

void c_frame_registration::set_registration_channel(color_channel_type v)
{
  base_options_.registration_channel = v;
}

int c_frame_registration::registration_channel() const
{
  return base_options_.registration_channel;
}

void c_frame_registration::set_interpolation_flags(int v)
{
  base_options_.interpolation_flags = v;
}

int c_frame_registration::interpolation_flags() const
{
  return base_options_.interpolation_flags;
}

void c_frame_registration::set_remap_border_mode(int v)
{
  base_options_.remap_border_mode = v;
}

int c_frame_registration::remap_border_mode() const
{
  return base_options_.remap_border_mode;
}

void c_frame_registration::set_remap_border_value(const cv::Scalar & v)
{
  base_options_.remap_border_value = v;
}

const cv::Scalar & c_frame_registration::remap_border_value() const
{
  return base_options_.remap_border_value;
}

void c_frame_registration::set_feature_scale(double v)
{
  base_options_.feature_scale = v;
}

double c_frame_registration::feature_scale() const
{
  return base_options_.feature_scale;
}

void c_frame_registration::set_ecc_scale(double v)
{
  base_options_.ecc.scale = v;
}

double c_frame_registration::ecc_scale() const
{
  return base_options_.ecc.scale;
}

void c_frame_registration::set_ecc_normalization_scale(int v)
{
  base_options_.ecc.normalization_scale = v;
}

int c_frame_registration::ecc_normalization_scale() const
{
  return base_options_.ecc.normalization_scale;
}

void c_frame_registration::set_ecc_normalization_noise(double v)
{
  base_options_.ecc.normalization_noise = v;
}

double c_frame_registration::ecc_normalization_noise() const
{
  return base_options_.ecc.normalization_noise;
}

void c_frame_registration::set_eccflow_support_scale(int v)
{
  base_options_.eccflow.support_scale = v;
}

int c_frame_registration::eccflow_support_scale() const
{
  return base_options_.eccflow.support_scale;
}

void c_frame_registration::set_eccflow_normalization_scale(int v)
{
  base_options_.eccflow.normalization_scale = v;
}

int c_frame_registration::eccflow_normalization_scale() const
{
  return base_options_.eccflow.normalization_scale;
}


void c_frame_registration::set_enable_ecc(bool v)
{
  base_options_.enable_ecc = v;
}

bool c_frame_registration::enable_ecc() const
{
  return base_options_.enable_ecc;
}

void c_frame_registration::set_enable_eccflow(bool v)
{
  base_options_.enable_eccflow = v;
}

bool c_frame_registration::enable_eccflow() const
{
  return base_options_.enable_eccflow;
}

//c_ecc_forward_additive & c_frame_registration::ecc()
//{
//  return ecc_;
//}

const c_ecc_forward_additive & c_frame_registration::ecc() const
{
  return ecc_;
}

//c_ecch_flow & c_frame_registration::eccflow()
//{
//  return eccflow_;
//}

const c_ecch_flow & c_frame_registration::eccflow() const
{
  return eccflow_;
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
  return ecc_.reference_image();
}

const cv::Mat & c_frame_registration::reference_ecc_mask() const
{
  return ecc_.reference_mask();
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
  return ecc_.input_image();
}

const cv::Mat & c_frame_registration::current_ecc_mask() const
{
  return ecc_.input_mask();
}

//const cv::Rect & c_frame_registration::reference_ROI() const
//{
//  return reference_ROI_;
//}
//
//const cv::Rect & c_frame_registration::current_ROI() const
//{
//  return current_ROI_;
//}


const cv::Mat1f & c_frame_registration::current_transform() const
{
  return current_transform_;
}

const cv::Mat2f & c_frame_registration::current_remap() const
{
  return current_remap_;
}

bool c_frame_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if ( !extract_channel(src, dst, srcmsk, dstmsk, registration_channel(), scale, CV_32F) ) {
    CF_ERROR("extract_channel(registration_channel_=%d) fails", registration_channel());
    return false;
  }

  if ( ecc_normalization_scale() > 0 ) {

    cv::Mat1f & m = (cv::Mat1f &) dst.getMatRef();
    cv::Mat mean, stdev;
    cv::Mat mask;

    ecc_downscale(m, mean, ecc_normalization_scale(), cv::BORDER_REPLICATE);
    ecc_downscale(m.mul(m), stdev, ecc_normalization_scale(), cv::BORDER_REPLICATE);
    cv::absdiff(stdev, mean.mul(mean), stdev);
    cv::sqrt(stdev, stdev);

    ecc_upscale(mean, m.size());
    ecc_upscale(stdev, m.size());

    cv::add(stdev, ecc_normalization_noise(), stdev);
    cv::subtract(m, mean, m);
    cv::divide(m, stdev, m);
  }

  return true;
}


bool c_frame_registration::create_eccflow_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if ( !extract_channel(src, dst, srcmsk, dstmsk, registration_channel(), scale, CV_32F) ) {
    CF_ERROR("extract_channel(registration_channel_=%d) fails", base_options_.registration_channel);
    return false;
  }

  return true;
}


bool c_frame_registration::setup_referece_frame(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;

  reference_frame_size_ = reference_image.size();
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));

  if ( feature_scale() > 0 ) {

    if ( !create_feature_image(reference_image, reference_mask, reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("extract_feature_image() fails");
      return false;
    }

    if ( !extract_reference_features(reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("extract_reference_features() fails");
      return false;
    }
  }

  if ( enable_ecc() || enable_eccflow() ) {
    if ( !create_ecc_image(reference_image, reference_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("extract_ecc_image(reference_image) fails");
      return false;
    }
    //    const double sigma =
    //        base_options_.ecc.reference_smooth_sigma;
    //
    //    if ( sigma > 0 ) {
    //      if ( gaussian_filter_.sigmax() != sigma ) {
    //        gaussian_filter_ = c_gaussian_filter(sigma, sigma);
    //      }
    //      gaussian_filter_.apply(ecc_image, ecc_mask, ecc_image);
    //    }
  }


  if ( enable_ecc() && ecc_scale() > 0 ) {

    ecc_.set_eps(base_options_.ecc.eps);
    ecc_.set_min_rho(base_options_.ecc.min_rho);
    ecc_.set_input_smooth_sigma(base_options_.ecc.input_smooth_sigma);
    ecc_.set_reference_smooth_sigma(base_options_.ecc.reference_smooth_sigma);
    ecc_.set_update_step_scale(base_options_.ecc.update_step_scale);
    ecc_.set_max_iterations(base_options_.ecc.max_iterations);

    cv::Mat1f reference_ecc_image;
    cv::Mat1b reference_ecc_mask;

    if ( ecc_scale() == 1 ) {
      reference_ecc_image = ecc_image;
      reference_ecc_mask = ecc_mask;
    }
    else if ( ecc_scale() < 1 ) {
      cv::resize(ecc_image, reference_ecc_image, cv::Size(), ecc_scale(), ecc_scale(), cv::INTER_AREA);
    }
    else {
      cv::resize(ecc_image, reference_ecc_image, cv::Size(), ecc_scale(), ecc_scale(), cv::INTER_LINEAR_EXACT);
    }

    if ( !ecc_mask.empty() && reference_ecc_mask.size() != reference_ecc_image.size() ) {
      cv::resize(ecc_mask, reference_ecc_mask, reference_ecc_image.size(), 0, 0, cv::INTER_LINEAR);
      cv::compare(reference_ecc_mask, 255, reference_ecc_mask, cv::CMP_GE);
    }

    if ( !ecc_.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
      CF_ERROR("ecc_.set_reference_image() fails");
      return false;
    }
  }



  if ( enable_eccflow() ) {

    eccflow_.set_update_multiplier(base_options_.eccflow.update_multiplier);
    eccflow_.set_max_iterations(base_options_.eccflow.max_iterations);
    eccflow_.set_support_scale(base_options_.eccflow.support_scale);
    eccflow_.set_normalization_scale(base_options_.eccflow.normalization_scale);
    eccflow_.set_input_smooth_sigma(0/*base_options_.eccflow.input_smooth_sigma*/);
    eccflow_.set_reference_smooth_sigma(0/*base_options_.eccflow.reference_smooth_sigma*/);

    if ( !eccflow_.set_reference_image(ecc_image, ecc_mask) ) {
      CF_ERROR("eccflow_.set_reference_image() fails");
      return false;
    }
  }

  return true;
}


bool c_frame_registration::register_frame(cv::InputArray current_image, cv::InputArray current_mask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;

  double start_time = 0, total_time = 0;
  double t0, t1;

  bool have_transform = false;



  start_time = get_realtime_ms();

  current_frame_size_ = current_image.size();
  current_transform_ = createEyeTransform(motion_type());
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));


  /////////////////////////////////////////////////////////////////////////////
  if ( feature_scale() > 0 ) {

    t0 = get_realtime_ms();
    if ( !create_feature_image(current_image, current_mask, current_feature_image_, current_feature_mask_) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    current_status_.timings.extract_feature_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( !estimate_feature_transform(current_feature_image_, current_feature_mask_, &current_transform_) ) {
      CF_ERROR("estimate_feature_transform() fails");
      return false;
    }

    current_status_.timings.estimate_feature_transform =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }


  /////////////////////////////////////////////////////////////////////////////

  if ( enable_ecc() || enable_eccflow() ) {

    if ( !create_ecc_image(current_image, current_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("extract_ecc_image(current_image) fails");
      return false;
    }

    //    const double sigma =
    //        base_options_.ecc.input_smooth_sigma;
    //
    //    if ( sigma > 0 ) {
    //      if ( gaussian_filter_.sigmax() != sigma ) {
    //        gaussian_filter_ = c_gaussian_filter(sigma, sigma);
    //      }
    //      gaussian_filter_.apply(ecc_image, ecc_mask, ecc_image);
    //    }
  }


  /////////////////////////////////////////////////////////////////////////////

  if ( enable_ecc() && ecc_scale() > 0 ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_ecc_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( have_transform && ecc_scale() != 1 ) {

      scaleTransform(motion_type(),
          current_transform_,
          ecc_scale());

    }

    cv::Mat1f current_ecc_image;
    cv::Mat1b current_ecc_mask;

    if ( ecc_scale() == 1 ) {
      current_ecc_image = ecc_image;
      current_ecc_mask = ecc_mask;
    }
    else if ( ecc_scale() < 1 ) {
      cv::resize(ecc_image, current_ecc_image, cv::Size(), ecc_scale(), ecc_scale(), cv::INTER_AREA);
    }
    else {
      cv::resize(ecc_image, current_ecc_image, cv::Size(), ecc_scale(), ecc_scale(), cv::INTER_LINEAR_EXACT);
    }

    if ( !ecc_mask.empty() && current_ecc_mask.size() != current_ecc_image.size() ) {
      cv::resize(ecc_mask, current_ecc_mask, current_ecc_image.size(), 0, 0, cv::INTER_LINEAR);
      cv::compare(current_ecc_mask, 255, current_ecc_mask, cv::CMP_GE);
    }

    ecc_.set_motion_type(motion_type());

    if ( !ecc_.align_to_reference(current_ecc_image, current_transform_, current_ecc_mask)  ) {
      CF_ERROR("ecc_.align_to_reference() fails: rho=%g/%g eps=%g/%g iterations=%d/%d",
          ecc_.rho(), ecc_.min_rho(),
          ecc_.current_eps(), ecc_.eps(),
          ecc_.num_iterations(), ecc_.max_iterations());
      return false;
    }

    if ( ecc_scale() != 1 ) {
      scaleTransform(motion_type(),
          current_transform_,
          1. / ecc_scale());
    }

    current_status_.timings.ecc_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }

  /////////////////////////////////////////////////////////////////////////////

  t0 = get_realtime_ms();
  createRemap(motion_type(), current_transform_, current_remap_, reference_frame_size_);
  current_status_.timings.create_remap =
      get_realtime_ms() - t0;

  if ( enable_eccflow() ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_smflow_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;


    if ( !eccflow_.compute(ecc_image, current_remap_, ecc_mask) ) {
      CF_ERROR("smflow_.compute() fails");
      return false;
    }

    current_status_.timings.smflow_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;
  }


  if ( dst.needed() || dstmask.needed() ) {

    t0 = get_realtime_ms();

    if ( !remap(current_image, dst, current_mask, dstmask) ) {
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

      ecc_.rho(), ecc_.min_rho(),
      ecc_.current_eps(), ecc_.eps(),
      ecc_.num_iterations(), ecc_.max_iterations(),
      current_status_.timings.ecc_align, current_status_.timings.ecc_align / (ecc_.num_iterations() + 1),

      current_status_.timings.create_remap,
      current_status_.timings.extract_smflow_image,
      current_status_.timings.smflow_align,

      current_status_.timings.remap
      );


  return true;
}



bool c_frame_registration::custom_remap(const cv::Mat2f & rmap,
    cv::InputArray _src, cv::OutputArray dst,
    cv::InputArray _src_mask, cv::OutputArray dst_mask,
    int interpolation_flags,
    int border_mode,
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


  if ( dst.needed() || dst_mask.needed()) {
    if ( current_remap_.size() != src_size ) {
      CF_ERROR("Invalid argument: invalid src / remap size");
      //return false;
    }
  }

  if ( interpolation_flags < 0 ) {
    interpolation_flags = this->interpolation_flags();
  }

  if ( border_mode >= 0 ) {
    border_value_ptr = &border_value;
  }
  else {
    border_mode = remap_border_mode();
    border_value_ptr = &base_options_.remap_border_value;
  }

  if ( dst.needed() ) {
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
  }

  return true;
}


bool c_frame_registration::remap(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray src_mask, cv::OutputArray dst_mask,
    int interpolation_flags,
    int border_mode,
    const cv::Scalar & border_value) const
{
  return custom_remap(current_remap_,
      src, dst,
      src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);
}

