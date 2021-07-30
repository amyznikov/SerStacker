/*
 * c_image_processor.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include <functional>
#include "c_image_processor.h"

static std::vector<const c_image_processor_routine::class_factory*> c_image_processor_routine_class_list_;

c_image_processor_routine::class_factory::class_factory(const std::string & _class_name,
    const std::string & _display_name,
    const std::string & _tooltip,
    const std::function<c_image_processor_routine::ptr()> & _create_instance) :
        class_name(_class_name), display_name(_display_name), tooltip(_tooltip), create_instance(_create_instance)
{
  class_list_guard_lock guard_lock;
  c_image_processor_routine_class_list_.emplace_back(this);
}


const std::vector<const c_image_processor_routine::class_factory*> & c_image_processor_routine::class_list()
{
  return c_image_processor_routine_class_list_;
}



c_image_processor::c_image_processor(const std::string & objname) :
    name_(objname)
{
}

c_image_processor::ptr c_image_processor::create(const std::string & objname)
{
  return ptr(new this_class(objname));
}

c_image_processor::ptr c_image_processor::load(const std::string & filename)
{
  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return nullptr;
  }


  std::string object_class;
  if ( !load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("load_settings(object_class) fails", filename.c_str());
    return nullptr;
  }

  if ( object_class != "c_image_processor" ) {
    CF_FATAL("Incorect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return nullptr;
  }

  c_config_setting root = cfg.root().get("c_image_processor");
  if ( !root || !root.isGroup() ) {
    CF_FATAL("group 'c_image_processor' is not found in file '%s''",
        filename.c_str());
    return nullptr;
  }

  return load(root);
}

c_image_processor::ptr c_image_processor::load(c_config_setting settings)
{
  std::string objname;

  if ( !settings.get("name", &objname) || objname.empty() ) {
    CF_ERROR("c_image_processor: settings.get('name') fails");
    return nullptr;
  }

  ptr obj = create(objname);
  if ( !obj ) {
    CF_ERROR("c_image_processor::create(objname=%s) fails", objname.c_str());
    return nullptr;
  }

  settings.get("enabled", &obj->enabled_);

  c_config_setting chain =
      settings["chain"];

  if ( chain && !chain.isList() ) {
    CF_ERROR("c_image_processor::load(objname=%s): 'chain' item must be libconfig list", objname.c_str());
    return nullptr;
  }

  if ( chain ) {

    const int chain_length = chain.length();

    obj->reserve(chain_length);

    for ( int i = 0; i < chain_length; ++i ) {

      c_config_setting group =
          chain.get_element(i);

      if ( !group || !group.isGroup() ) {
        CF_ERROR("c_image_processor:  chain.get_element(objname=%s, index=%d, type=GROUP) fails", objname.c_str(), i);
        return nullptr;
      }

      c_image_processor_routine::ptr routine =
          c_image_processor_routine::create(group);

      if ( !routine ) {
        CF_ERROR("c_image_processor_routine::create(objname=%s, chain index=%d) fails", objname.c_str(), i);
        return nullptr;
      }

      obj->emplace_back(routine);
    }
  }

  return obj;
}


bool c_image_processor::save(const std::string & filename) const
{
  c_config cfg(filename);

  time_t t = time(0);

  if ( !save_settings(cfg.root(), "object_class", std::string("c_image_processor")) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save(cfg.root().add_group("c_image_processor"))) {
    CF_FATAL("save(c_image_processor) fails");
    return false;
  }

  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  return true;
}

bool c_image_processor::save(c_config_setting settings) const
{
  if ( !settings ) {
    CF_ERROR("c_image_processor::save() gets invalid argument: c_config_setting = null");
    return false;
  }

  settings.set("name", name_);
  settings.set("enabled", enabled_);

  c_config_setting chain =
      settings.add_list("chain");

  for ( const c_image_processor_routine::ptr & routine : *this ) {
    if ( routine && !routine->save(chain.add_element(CONFIG_TYPE_GROUP)) ) {
      return false;
    }
  }

  return true;
}


c_image_processor_routine::ptr c_image_processor_routine::create(c_config_setting settings)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine::create(c_config_setting): settings is null");
    return nullptr;
  }

  std::string objname;
  if ( !settings.get("routine", &objname) || objname.empty() ) {
    CF_ERROR("No 'routine' string found");
    return nullptr;
  }

  c_image_processor_routine::ptr routine =
      c_image_processor_routine::create(objname);

  if( !routine ) {
    CF_ERROR("c_image_processor_routine::create(routine=%s) fails", objname.c_str());
    return nullptr;
  }

  if ( !routine->load(settings) ) {
    CF_ERROR("routine->load(routine=%s, c_config_setting) fails", objname.c_str());
    return nullptr;
  }

  return routine;
}

c_image_processor_routine::ptr c_image_processor_routine::create(const std::string & processor_name)
{
  if ( processor_name.empty() ) {
    return nullptr;
  }

  class_list_guard_lock lock;
  c_image_processor_routine::ptr processor;
  const char * cname = processor_name.c_str();

  for ( const class_factory * f : c_image_processor_routine_class_list_ ) {
    if ( strcasecmp(cname, f->class_name.c_str()) == 0 ) {
      return f->create_instance();
    }
  }

  return nullptr;
}


bool c_image_processor_routine::load(c_config_setting settings)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine::load(c_config_setting) : settings is null");
    return false;
  }

  settings.get("enabled", &enabled_);

  return true;
}

bool c_image_processor_routine::save(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  settings.set("routine", class_name());
  settings.set("display_name", display_name());
  settings.set("tooltip", tooltip());
  settings.set("enabled", enabled());

  return true;
}



//

c_unsharp_mask_routine::c_class_factory c_unsharp_mask_routine::class_factory;

bool c_unsharp_mask_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("sigma", &sigma_);
  settings.get("alpha", &alpha_);

  return true;
}

bool c_unsharp_mask_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("sigma", sigma_);
  settings.set("alpha", alpha_);

  return true;
}

//

c_mtf_routine::c_class_factory c_mtf_routine::class_factory;

bool c_mtf_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  return true;
}

bool c_mtf_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  return true;
}


//

c_align_color_channels_routine::c_class_factory c_align_color_channels_routine::class_factory;

bool c_align_color_channels_routine::load(c_config_setting settings)
{
  return base::load(settings);
}

bool c_align_color_channels_routine::save(c_config_setting settings) const
{
  return base::save(settings);
}

//

c_rangeclip_routine::c_class_factory c_rangeclip_routine::class_factory;

bool c_rangeclip_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("min", &min_);
  settings.get("max", &max_);
  return true;
}

bool c_rangeclip_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("min", min_);
  settings.set("max", max_);
  return true;
}

//

c_autoclip_routine::c_class_factory c_autoclip_routine::class_factory;

bool c_autoclip_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("plo", &plo_);
  settings.get("phi", &phi_);

  return true;
}

bool c_autoclip_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("plo", plo_);
  settings.set("phi", phi_);

  return true;
}

//

c_histogram_white_balance_routine::c_class_factory c_histogram_white_balance_routine::class_factory;

bool c_histogram_white_balance_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("lclip", &lclip_);
  settings.get("hclip", &hclip_);

  return true;
}

bool c_histogram_white_balance_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("lclip", lclip_);
  settings.set("hclip", hclip_);

  return true;
}


//

c_range_normalize_routine::c_class_factory c_range_normalize_routine::class_factory;

bool c_range_normalize_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("outmin", &outmin_);
  settings.get("outmax", &outmax_);

  return true;
}

bool c_range_normalize_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("outmin", outmin_);
  settings.set("outmax", outmax_);

  return true;
}

//

c_anscombe_routine::c_class_factory c_anscombe_routine::class_factory;

bool c_anscombe_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  int v = anscombe_.method();
  if ( settings.get("method", &v) ) {
    anscombe_.set_method((anscombe_method) (v));
  }

  return true;
}

bool c_anscombe_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("method", (int)anscombe_.method());

  return true;
}


//

c_noisemap_routine::c_class_factory c_noisemap_routine::class_factory;

bool c_noisemap_routine::load(c_config_setting settings)
{
  return base::load(settings);
}

bool c_noisemap_routine::save(c_config_setting settings) const
{
  return base::save(settings);
}

//
c_smap_routine::c_class_factory c_smap_routine::class_factory;

bool c_smap_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("minv", &minv_);
  settings.get("scale", &scale_);

  return true;
}

//

bool c_smap_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("minv", minv_);
  settings.set("scale", scale_);

  return true;
}


//

c_test_routine::c_class_factory c_test_routine::class_factory;

bool c_test_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  return true;
}

bool c_test_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  return true;
}

///*
// * Pyramid down to specific level
// */
//static bool pdownscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
//{
//  cv::pyrDown(src, dst, cv::Size(), border_mode);
//  for ( int l = 1; l < level; ++l ) {
//    cv::pyrDown(dst, dst, cv::Size(), border_mode);
//  }
//  return true;
//}
//
///*
// * Pyramid up to specific size
// */
//static bool pupscale(cv::Mat & image, cv::Size dstSize)
//{
//  const cv::Size inputSize = image.size();
//
//  if ( inputSize != dstSize ) {
//
//    std::vector<cv::Size> spyramid;
//
//    spyramid.emplace_back(dstSize);
//
//    while ( 42 ) {
//      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
//      if ( nextSize == inputSize ) {
//        break;
//      }
//      if ( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
//        CF_DEBUG("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
//            nextSize.width, nextSize.height,
//            inputSize.width, inputSize.height);
//        return false;
//      }
//      spyramid.emplace_back(nextSize);
//    }
//
//    for ( int i = spyramid.size() - 1; i >= 0; --i ) {
//      cv::pyrUp(image, image, spyramid[i]);
//    }
//  }
//
//  return true;
//}


bool c_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat & src = image.getMatRef();
  cv::Mat tmp;


//  constexpr double Q = 1./sqrt(2.0);
//  constexpr double S = (4. + 4 * Q);
//
//  static float C[3 * 3] = {
//      -Q,   -1,    -Q,
//      -1,    S,    -1,
//      -Q,   -1,    -Q,
//
//  };
//
//  static const cv::Mat1f K = cv::Mat1f(3, 3, C) / (2 * S);

  constexpr double Q = 1./sqrt(0.1*0.1 + 0.1*0.1);
  constexpr double S = (4.*0.1 + 4 * Q);

  static float C[3 * 3] = {
       -Q,   -0.1,    -Q,
      -0.1,    S,    -0.1,
       -Q,   -0.1,    -Q,

  };

  static const cv::Mat1f K = cv::Mat1f(3, 3, C) / (2 * S);

  double min, max;

  if ( level_ <= 0 ) {
    cv::filter2D(src, tmp, -1, K);
  }
  else {
    pdownscale(src, tmp, level_, cv::BORDER_DEFAULT);
    cv::filter2D(tmp, tmp, -1, K);
    pupscale(tmp, src.size());
  }

  cv::minMaxLoc(src, &min, &max);
  CF_DEBUG("src: min=%g max=%g", min, max);

  cv::minMaxLoc(tmp, &min, &max);
  CF_DEBUG("tmp: min=%g max=%g", min, max);

  cv::minMaxLoc(K, &min, &max);
  CF_DEBUG("K: min=%g max=%g norm=%g sum=%g", min, max, cv::norm(K, cv::NORM_L1), cv::sum(K)[0]);


  CF_DEBUG("scale_/ (1 << level_): %g", scale_ / (1 << level_));

  cv::scaleAdd(tmp, scale_ / (1 << level_), src, src);

  return true;
}

