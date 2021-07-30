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

///////////////////////////////////////////////////////////////////////////////

class c_image_processor_routine
{
public:
  typedef c_image_processor_routine this_class;
  typedef std::shared_ptr<this_class> ptr;
  typedef std::function<this_class::ptr()> factory;

  struct class_factory
  {
    std::string class_name;
    std::string display_name;
    std::string tooltip;
    std::function<c_image_processor_routine::ptr()> create_instance;

    class_factory(const std::string & _class_name,
        const std::string & _display_name,
        const std::string & _tooltip,
        const std::function<c_image_processor_routine::ptr()> & _create_instance);
  };

  struct class_list_guard_lock
  {
    class_list_guard_lock() {
      mtx().lock();
    }

    ~class_list_guard_lock() {
      mtx().unlock();
    }

    static std::mutex & mtx() {
      static std::mutex mtx_;
      return mtx_;
    }
  };

  static const std::vector<const class_factory*> & class_list();

  virtual ~c_image_processor_routine() = default;


  static ptr create(const std::string & class_name);
  static ptr create(c_config_setting settings);
  virtual bool load(c_config_setting settings);
  virtual bool save(c_config_setting settings) const;

  const std::string & class_name() const
  {
    return class_factory_->class_name;
  }

  const std::string & display_name() const
  {
    return class_factory_->display_name;
  }

  const std::string & tooltip() const
  {
    return class_factory_->tooltip;
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }


  virtual bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) = 0;


protected:
  c_image_processor_routine(const class_factory * _class_factory, bool enabled = true)
    : class_factory_(_class_factory), enabled_(enabled)
  {
  }

protected:
  const class_factory * class_factory_;
  bool enabled_;
};

class c_image_processor :
    public std::vector<c_image_processor_routine::ptr>
{
  std::string name_;
  bool enabled_ = true;

public:
  typedef c_image_processor this_class;
  typedef std::vector<c_image_processor_routine::ptr> base;
  typedef std::shared_ptr<this_class> ptr;


  c_image_processor(const std::string & objname);

  static ptr create(const std::string & objname);
  static ptr load(const std::string & filename);
  static ptr load(c_config_setting settings);
  bool save(const std::string & filename) const;
  bool save(c_config_setting settings) const;


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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) const
  {
    if ( enabled_ ) {
      for ( const c_image_processor_routine::ptr & processor : *this ) {
        if ( processor && processor->enabled() ) {
          if ( !processor->process(image, mask) ) {
            return false;
          }
        }
      }
    }
    return true;
  }

};


inline bool save_settings(c_config_setting settings, const c_image_processor::ptr & obj)
{
  return obj->save(settings);
}

inline bool save_settings(c_config_setting settings, const c_image_processor & obj)
{
  return obj.save(settings);
}


class c_image_processor_collection :
    public std::vector<c_image_processor::ptr>
{
public:
  typedef c_image_processor_collection this_class;
  typedef std::vector<c_image_processor::ptr> base;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create()
  {
    return ptr(new this_class());
  }

  static ptr create(c_config_setting settings)
  {
    ptr obj(new this_class());
    if ( obj->load(settings) ) {
      return obj;
    }
    return nullptr;
  }

  bool load(c_config_setting settings)
  {
    clear();

    if ( !settings ) {
      return false;
    }

    c_config_setting processors_list_item =
        settings["processors"];

    if ( !processors_list_item || !processors_list_item.isList() ) {
      return false;
    }

    const int num_processors = processors_list_item.length();

    reserve(num_processors);

    for ( int i = 0; i < num_processors; ++i ) {

      c_config_setting processor_item =
          processors_list_item.get_element(i);

      if ( processor_item && processor_item.isGroup() ) {
        c_image_processor::ptr processor = c_image_processor::load(processor_item);
        if ( processor ) {
          emplace_back(processor);
        }
      }
    }

    return true;
  }

  bool save(c_config_setting settings) const
  {
    if ( !settings ) {
      return false;
    }

    c_config_setting processors_list_item =
        settings.add_list("processors");

    for ( const c_image_processor::ptr & processor : *this ) {
      if ( processor && !processor->save(processors_list_item.add_element(CONFIG_TYPE_GROUP)) ) {
        return false;
      }
    }

    return true;
  }

};


inline bool save_settings(c_config_setting settings, const c_image_processor_collection::ptr & obj)
{
  return obj->save(settings);
}

inline bool save_settings(c_config_setting settings, const c_image_processor_collection & obj)
{
  return obj.save(settings);
}

///////////////////////////////////////////////////////////////////////////////



class c_unsharp_mask_routine
    : public c_image_processor_routine
{
public:
  typedef c_unsharp_mask_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("unsharp_mask", "unsharp mask", "unsharp mask",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_unsharp_mask_routine(bool enabled = true)
    : base( &class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  static ptr create(double sigma, double alpha = 0.9, bool enabled = true) {
    ptr obj(new this_class(enabled));
    obj->set_sigma(sigma);
    obj->set_alpha(alpha);
    return obj;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    unsharp_mask(image, image, sigma_, alpha_, outmin_, outmax_);
    if ( mask.needed() && !mask.empty() ) {
      const int ksize = 2 * std::max(2, (int) (sigma_ * 5)) + 3;
      cv::erode(mask, mask, cv::Mat1b(ksize, ksize, 255), cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
      image.getMatRef().setTo(0, ~mask.getMat());
    }
    return true;
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

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double sigma_ = 1, alpha_ = 0.9;
  double outmin_ = -1, outmax_ = -1;
};

class c_mtf_routine
    : public c_image_processor_routine
{
public:
  typedef c_mtf_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("mtf", "mtf", "mtf",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_mtf_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return mtf_->apply(image, image);
  }

  const c_pixinsight_midtones_transfer_function::ptr & mtf() const
  {
    return mtf_;
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  c_pixinsight_midtones_transfer_function::ptr mtf_ =
      c_pixinsight_midtones_transfer_function::create();
};

class c_align_color_channels_routine
    : public c_image_processor_routine
{
public:
  typedef c_align_color_channels_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("align_color_channels", "align_color_channels", "align_color_channels",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_align_color_channels_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  void set_reference_channel(int v)
  {
    reference_channel_ = v;
  }

  int reference_channel() const
  {
    return reference_channel_;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return algorithm_.align(reference_channel_, image, image, mask, mask);
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  c_align_color_channels algorithm_;
  int reference_channel_ = 0;
};


class c_rangeclip_routine
    : public c_image_processor_routine
{
public:
  typedef c_rangeclip_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("rangeclip", "rangeclip", "rangeclip",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_rangeclip_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr (new this_class(enabled));
  }

  static ptr create(double min, double max, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_min(min);
    obj->set_max(max);
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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return clip_range(image.getMatRef(), min_, max_, mask.getMat());
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double min_ = 0.0;
  double max_ = 1.0;
};

class c_autoclip_routine
    : public c_image_processor_routine
{
public:
  typedef c_autoclip_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("autoclip", "autoclip", "autoclip",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_autoclip_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  static ptr create(double lclip, double hclip, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_lclip(lclip);
    obj->set_hclip(hclip);
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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
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

    return autoclip(image.getMatRef(), mask, plo_, phi_, omin, omax);
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double plo_ = 0.5;
  double phi_ = 99.5;
};


class c_range_normalize_routine
    : public c_image_processor_routine
{
public:
  typedef c_range_normalize_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("normalize", "normalize", "normalize",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_range_normalize_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  static ptr create(double outmin, double outmax, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_outmin(outmin);
    obj->set_outmax(outmax);
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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    //clip_range(image.getMatRef(), min_, max_, mask.getMat());
    return normalize_minmax(image.getMatRef(), image.getMatRef(), outmin_, outmax_, mask, true);
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double outmin_ = 0.0;
  double outmax_ = 1.0;
};


class c_histogram_white_balance_routine
    : public c_image_processor_routine
{
public:
  typedef c_histogram_white_balance_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("histogram_white_balance", "histogram_white_balance", "histogram_white_balance",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_histogram_white_balance_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  static ptr create(double lclip, double hclip, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_lclip(lclip);
    obj->set_hclip(hclip);
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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return histogram_white_balance(image.getMatRef(),
        mask,
        image.getMatRef(),
        lclip_,
        hclip_);
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double lclip_ = 1;
  double hclip_ = 99;

};


class c_anscombe_routine
    : public c_image_processor_routine
{
public:
  typedef c_anscombe_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("anscombe", "anscombe transform", "anscombe transform",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_anscombe_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  static ptr create(enum anscombe_method m, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_method(m);
    return obj;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    anscombe_.apply(image.getMatRef(), image.getMatRef());
    return true;
  }

  void set_method(enum anscombe_method v)
  {
    anscombe_.set_method(v);
  }

  enum anscombe_method method() const
  {
    return anscombe_.method();
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  c_anscombe_transform anscombe_;
};




class c_noisemap_routine
    : public c_image_processor_routine
{
public:
  typedef c_noisemap_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("noisemap", "noise map", "noise map",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_noisemap_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr(new this_class(enabled));
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    create_noise_map(image.getMatRef(), image.getMatRef(), mask);
    return true;
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;
};




class c_smap_routine
    : public c_image_processor_routine
{
public:
  typedef c_smap_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("smap", "smap", "smap",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_smap_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr (new this_class(enabled));
  }

  static ptr create(double minv, double scale = 1.0 / 16, bool enabled = true)
  {
    ptr obj(new this_class(enabled));
    obj->set_minv(minv);
    obj->set_scale(scale);
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

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    cv::Mat1f smap;
    compute_smap(image, smap, minv_, scale_);
    image.move(smap);
    return true;
  }

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double minv_ = 0;
  double scale_ = 1.0/16;
};


class c_test_routine
    : public c_image_processor_routine
{
public:
  typedef c_test_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("test", "test", "test",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_test_routine(bool enabled = true)
    : base(&class_factory, enabled)
  {
  }

  static ptr create(bool enabled = true)
  {
    return ptr (new this_class(enabled));
  }

  void set_scale(double v)
  {
    scale_ = v;
  }

  double scale() const
  {
    return scale_;
  }

  void set_level(int v)
  {
    level_ = v;
  }

  int level() const
  {
    return level_;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  bool save(c_config_setting settings) const override;
  bool load(c_config_setting settings) override;

protected:
  double scale_ = 1;
  int level_ = 0;
};

#endif /* __c_image_processor_h__ */
