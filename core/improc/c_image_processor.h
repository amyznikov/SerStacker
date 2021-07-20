/*
 * c_image_processor.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __c_image_processor_h__
#define __c_image_processor_h__

#include <opencv2/opencv.hpp>
#include <core/settings.h>
#include <core/histogram/c_pixinsight_midtones_transfer_function.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/c_anscombe_transform.h>
#include <core/proc/align_channels.h>
#include <core/proc/normalize.h>
#include <core/proc/autoclip.h>
#include <core/proc/smap.h>

class c_image_processor
{
public:
  typedef c_image_processor this_class;
  typedef std::shared_ptr<this_class> ptr;

  c_image_processor(const std::string & name, const std::string & display_name )
    : name_(name), display_name_(display_name), enabled_(true)
  {
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  const std::string & name() const
  {
    return name_;
  }

  const std::string & display_name() const
  {
    return display_name_;
  }


  static ptr create(const std::string & name);
  static ptr load(c_config_setting settings);

  virtual ~c_image_processor() = default;
  virtual void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) = 0;
  virtual bool save_settings(c_config_setting settings) const;
  virtual bool load_settings(c_config_setting settings);


protected:
  std::string name_;
  std::string display_name_;
  bool enabled_;
};

class c_image_processor_chain :
    public std::vector<c_image_processor::ptr>
{
  std::string name_;
  bool enabled_ = true;

public:
  typedef c_image_processor_chain this_class;
  typedef std::vector<c_image_processor::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create()
  {
    return ptr(new this_class());
  }

  void set_name(const std::string & v)
  {
    name_ = v;
  }

  const std::string & name() const
  {
    return name_;
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  bool save_settings(c_config_setting settings) const;
  bool load_settings(c_config_setting settings);

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) const
  {
    if ( enabled_ ) {
      for ( const c_image_processor::ptr & processor : *this ) {

      //        CF_DEBUG("processor=%p name='%s' enabled='%d'",
      //            processor.get(),
      //            processor ? processor->name().c_str() : "",
      //            processor ? processor->enabled() : 0);

        if ( processor && processor->enabled() ) {
          processor->process(image, mask);
        }
      }
    }
  }

};


class c_image_processor_chains :
    public std::vector<c_image_processor_chain::ptr>
{
public:
  typedef c_image_processor_chains this_class;
  typedef std::vector<c_image_processor_chain::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create()
  {
    return ptr(new this_class());
  }

  bool save_settings(c_config_setting settings) const;
  bool load_settings(c_config_setting settings);

};


class c_unsharp_mask_image_processor
    : public c_image_processor
{
public:
  typedef c_unsharp_mask_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_unsharp_mask_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "unsharp_mask";
  }

  static const char * default_display_name() {
    return "Unsharp Mask";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double sigma, double alpha = 0.9) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_sigma(sigma);
    obj->set_alpha(alpha);
    return obj;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    unsharp_mask(image, image, sigma_, alpha_, outmin_, outmax_);
    if ( mask.needed() && !mask.empty() ) {
      const int ksize = 2 * std::max(2, (int) (sigma_ * 5)) + 3;
      cv::erode(mask, mask, cv::Mat1b(ksize, ksize, 255), cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
      image.getMatRef().setTo(0, ~mask.getMat());
    }
  }

  void set_sigma(double v)
  {
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_alpha(double v)
  {
    alpha_ = v;
  }

  double alpha() const
  {
    return alpha_;
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;


protected:
  double sigma_ = 1, alpha_ = 0.9;
  double outmin_ = -1, outmax_ = -1;
};

class c_mtf_image_processor
    : public c_image_processor
{
public:
  typedef c_mtf_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_mtf_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "mtf";
  }

  static const char * default_display_name() {
    return "MTF";
  }

  static ptr create() {
    return ptr(new this_class(default_name(), default_display_name()));
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    mtf_->apply(image, image);
  }

  const c_pixinsight_midtones_transfer_function::ptr & mtf() const
  {
    return mtf_;
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  c_pixinsight_midtones_transfer_function::ptr mtf_ =
      c_pixinsight_midtones_transfer_function::create();
};

class c_align_color_channels_image_processor
    : public c_image_processor
{
public:
  typedef c_align_color_channels_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_align_color_channels_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "align_color_channels";
  }

  static const char * default_display_name() {
    return "Align Color Channels";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  void set_reference_channel(int v) {
    reference_channel_ = v;
  }

  int reference_channel() const {
    return reference_channel_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    algorithm_.align(reference_channel_, image, image, mask, mask);
  }

protected:
  c_align_color_channels algorithm_;
  int reference_channel_ = 0;
};


class c_rangeclip_image_processor
    : public c_image_processor
{
public:
  typedef c_rangeclip_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_rangeclip_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "rangeclip";
  }

  static const char * default_display_name() {
    return "Clip";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double min, double max, bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_min(min);
    obj->set_max(max);
    obj->set_enabled(enabled);
    return obj;
  }

  void set_min(double v)
  {
    min_ = v;
  }

  double min() const
  {
    return min_;
  }

  void set_max(double v)
  {
    max_ = v;
  }

  double max() const
  {
    return max_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    clip_range(image.getMatRef(), min_, max_, mask.getMat());
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double min_ = 0.0;
  double max_ = 1.0;
};

class c_autoclip_image_processor
    : public c_image_processor
{
public:
  typedef c_autoclip_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_autoclip_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "autoclip";
  }

  static const char * default_display_name() {
    return "Auto Clip";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double lclip, double hclip, bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_lclip(lclip);
    obj->set_hclip(hclip);
    obj->set_enabled(enabled);
    return obj;
  }

  void set_lclip(double v)
  {
    plo_ = v;
  }

  double lclip() const
  {
    return plo_;
  }

  void set_hclip(double v)
  {
    phi_ = v;
  }

  double hclip() const
  {
    return phi_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    double omin = 0, omax = 1;

    switch ( image.depth() ) {
    case CV_8U :
      omin = 0, omax = UINT8_MAX;
      break;
    case CV_8S :
      omin = INT8_MIN, omax = INT8_MAX;
      break;
    case CV_16U :
      omin = 0, omax = UINT16_MAX;
      break;
    case CV_16S :
      omin = INT16_MIN, omax = INT16_MAX;
      break;
    case CV_32S :
      omin = INT32_MIN, omax = INT32_MAX;
      break;
      break;
    }

    autoclip(image.getMatRef(), mask, plo_, phi_, omin, omax);
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double plo_ = 0.5;
  double phi_ = 99.5;
};


class c_range_normalize_image_processor
    : public c_image_processor
{
public:
  typedef c_range_normalize_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_range_normalize_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "normalize";
  }

  static const char * default_display_name() {
    return "Normalize";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double outmin, double outmax, bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_outmin(outmin);
    obj->set_outmax(outmax);
    obj->set_enabled(enabled);
    return obj;
  }

  void set_outmin(double v)
  {
    outmin_ = v;
  }

  double outmin() const
  {
    return outmin_;
  }

  void set_outmax(double v)
  {
    outmax_ = v;
  }

  double outmax() const
  {
    return outmax_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    //clip_range(image.getMatRef(), min_, max_, mask.getMat());
    normalize_minmax(image.getMatRef(), image.getMatRef(), outmin_, outmax_, mask, true);
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double outmin_ = 0.0;
  double outmax_ = 1.0;
};


class c_histogram_white_balance_image_processor
    : public c_image_processor
{
public:
  typedef c_histogram_white_balance_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_histogram_white_balance_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "smap";
  }

  static const char * default_display_name() {
    return "SMAP";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double lclip, double hclip, bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_lclip(lclip);
    obj->set_hclip(hclip);
    obj->set_enabled(enabled);
    return obj;
  }

  void set_lclip(double v)
  {
    lclip_ = v;
  }

  double lclip() const
  {
    return lclip_;
  }

  void set_hclip(double v)
  {
    hclip_ = v;
  }

  double hclip() const
  {
    return hclip_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    histogram_white_balance(image.getMatRef(),
        mask,
        image.getMatRef(),
        lclip_,
        hclip_);
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double lclip_ = 1;
  double hclip_ = 99;

};


class c_anscombe_image_processor
    : public c_image_processor
{
public:
  typedef c_anscombe_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_anscombe_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "anscombe";
  }

  static const char * default_display_name() {
    return "Anscombe transform";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    anscombe_.apply(image.getMatRef(), image.getMatRef());
  }

  void set_method(enum anscombe_method v)
  {
    anscombe_.set_method(v);
  }

  enum anscombe_method method() const
  {
    return anscombe_.method();
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  c_anscombe_transform anscombe_;
};




class c_noisemap_image_processor
    : public c_image_processor
{
public:
  typedef c_noisemap_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_noisemap_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "nosemap";
  }

  static const char * default_display_name() {
    return "NOISE MAP";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    create_noise_map(image.getMatRef(), image.getMatRef(), mask);
  }

  bool save_settings(c_config_setting settings) const override { return true; }
  bool load_settings(c_config_setting settings) override { return true; }

};




class c_smap_image_processor
    : public c_image_processor
{
public:
  typedef c_smap_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_smap_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "smap";
  }

  static const char * default_display_name() {
    return "SMAP";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  static ptr create(double minv, double scale = 1.0/16, bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_minv(minv);
    obj->set_scale(scale);
    obj->set_enabled(enabled);
    return obj;
  }

  void set_minv(double v)
  {
    minv_ = v;
  }

  double minv() const
  {
    return minv_;
  }

  void set_scale(double v)
  {
    scale_ =  v;
  }

  double scale() const
  {
    return scale_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    cv::Mat1f smap;
    compute_smap(image, smap, minv_, scale_);
    image.move(smap);
  }

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double minv_ = 0;
  double scale_ = 1.0/16;
};


class c_test_image_processor
    : public c_image_processor
{
public:
  typedef c_test_image_processor this_class;
  typedef c_image_processor base;
  typedef std::shared_ptr<this_class> ptr;

  c_test_image_processor(const std::string & name, const std::string & display_name)
    : base(name, display_name)
  {
  }

  static const char * default_name() {
    return "test";
  }

  static const char * default_display_name() {
    return "TEST";
  }

  static ptr create(bool enabled = true) {
    ptr obj(new this_class(default_name(), default_display_name()));
    obj->set_enabled(enabled);
    return obj;
  }

  void set_scale(double v) {
    scale_ = v;
  }

  double scale() const {
    return scale_;
  }

  void set_level(int v) {
    level_ = v;
  }

  int level() const {
    return level_;
  }

  void process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  bool save_settings(c_config_setting settings) const override;
  bool load_settings(c_config_setting settings) override;

protected:
  double scale_ = 1;
  int level_ = 0;
};

#endif /* __c_image_processor_h__ */
