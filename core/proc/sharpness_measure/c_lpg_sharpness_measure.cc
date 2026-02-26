/*
 * c_lpg_sharpness_measure.cc
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#include "c_lpg_sharpness_measure.h"
#include <core/proc/lpg.h>
#include <core/debug.h>


bool load_settings(c_config_setting settings, c_lpg_options * opts)
{
  LOAD_OPTION(settings, *opts, k);
  LOAD_OPTION(settings, *opts, p);
  LOAD_OPTION(settings, *opts, dscale);
  LOAD_OPTION(settings, *opts, uscale);
  LOAD_OPTION(settings, *opts, avgchannel);
  return true;
}

bool save_settings(c_config_setting settings, const c_lpg_options & opts)
{
  SAVE_OPTION(settings, opts, k);
  SAVE_OPTION(settings, opts, p);
  SAVE_OPTION(settings, opts, dscale);
  SAVE_OPTION(settings, opts, uscale);
  SAVE_OPTION(settings, opts, avgchannel);
  return true;
}


//bool c_lpg_options::serialize(c_config_setting settings, bool save)
//{
//  SERIALIZE_OPTION(settings, save, *this, k);
//  SERIALIZE_OPTION(settings, save, *this, p);
//  SERIALIZE_OPTION(settings, save, *this, dscale);
//  SERIALIZE_OPTION(settings, save, *this, uscale);
//  SERIALIZE_OPTION(settings, save, *this, avgchannel);
//  return true;
//}

//std::string c_lpg_options::save_settings()
//{
//  c_config cfg;
//
//  if( !serialize(cfg.root().add_group("c_lpg_options"), true) ) {
//    CF_FATAL("c_lpg_options::serialize() fails");
//    return "";
//  }
//
//  return cfg.write_string();
//}
//
//bool c_lpg_options::load_settings(const std::string & text)
//{
//  c_config cfg;
//
//  if ( !cfg.read_string(text.c_str()) ) {
//    CF_FATAL("c_lpg_options: cfg.read_string() fails");
//    return false;
//  }
//
//  c_config_setting section = cfg.root()["c_lpg_options"];
//  if ( !section.isGroup() ) {
//    CF_FATAL("Group 'c_lpg_options' not found");
//    return false;
//  }
//
//  if( !serialize(section, false) ) {
//    CF_FATAL("c_lpg_options::serialize(c_lpg_options) fails");
//    return false;
//  }
//
//  return true;
//}


c_lpg_sharpness_measure::c_lpg_sharpness_measure()
{
}

c_lpg_sharpness_measure::c_lpg_sharpness_measure(const c_lpg_options & opts) :
    _opts(opts)
{
}

c_lpg_options & c_lpg_sharpness_measure::options()
{
  return _opts;
}

const c_lpg_options & c_lpg_sharpness_measure::options() const
{
  return _opts;
}

void c_lpg_sharpness_measure::set_k(double v)
{
  _opts.k = v;
}

double c_lpg_sharpness_measure::k() const
{
  return _opts.k;
}

void c_lpg_sharpness_measure::set_dscale(int v)
{
  _opts.dscale = v;
}

int c_lpg_sharpness_measure::dscale() const
{
  return _opts.dscale;
}

void c_lpg_sharpness_measure::set_uscale(int v)
{
  _opts.uscale = v;
}

int c_lpg_sharpness_measure::uscale() const
{
  return _opts.uscale;
}

void c_lpg_sharpness_measure::set_p(double v)
{
  _opts.p = v;
}

double c_lpg_sharpness_measure::p() const
{
  return _opts.p;
}

void c_lpg_sharpness_measure::set_avgchannel(bool v)
{
  _opts.avgchannel = v;
}

bool c_lpg_sharpness_measure::avgchannel() const
{
  return _opts.avgchannel;
}

cv::Scalar c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Scalar rv;
  compute(image, mask,
      cv::noArray(),
      _opts.k,
      _opts.p,
      _opts.dscale,
      _opts.uscale,
      _opts.avgchannel,
      &rv);
  return rv;
}

bool c_lpg_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return compute(image, cv::noArray(),
      output_map,
      _opts.k,
      _opts.p,
      _opts.dscale,
      _opts.uscale,
      _opts.avgchannel,
      nullptr);
}

bool c_lpg_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map,
    const c_lpg_options & opts)
{
  return compute(image, cv::noArray(),
      output_map,
      opts.k,
      opts.p,
      opts.dscale,
      opts.uscale,
      opts.avgchannel,
      nullptr);
}

bool c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
    double k, double p, int dscale, int uscale, bool avgchannel,
    cv::Scalar * output_sharpness_metric)
{
  return lpg(image, mask, output_map,
    k, p, dscale, uscale, avgchannel,
    output_sharpness_metric);
}

bool c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
    const c_lpg_options & opts,
    cv::Scalar * output_sharpness_metric)
{
  return lpg(image, mask, output_map,
      opts.k, opts.p, opts.dscale, opts.uscale, opts.avgchannel,
      output_sharpness_metric);
}

//bool c_lpg_sharpness_measure::serialize(c_config_setting settings, bool save)
//{
//  return _opts.serialize(settings, save);
//}
//
//std::string c_lpg_sharpness_measure::save_settings()
//{
//  return _opts.save_settings();
//}
//
//bool c_lpg_sharpness_measure::load_settings(const std::string & text)
//{
//  return _opts.load_settings(text);
//}

