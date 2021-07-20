/*
 * c_image_processor.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "c_image_processor.h"
#include <functional>

// class factory items
namespace {

static const struct class_factory_item {
  const char * processor_name;
  std::function<c_image_processor::ptr()> create_instance;
} class_factory_items[] = {
    {
        c_unsharp_mask_image_processor::default_name(),
        []() {return c_unsharp_mask_image_processor::create();}
    },
    {
        c_mtf_image_processor::default_name(),
        []() {return c_mtf_image_processor::create();}
    },
    {
        c_autoclip_image_processor::default_name(),
        []() {return c_autoclip_image_processor::create();}
    },
    {
        c_smap_image_processor::default_name(),
        []() {return c_smap_image_processor::create();}
    },
};

}


c_image_processor::ptr c_image_processor::load(c_config_setting settings)
{
  if ( !settings ) {
    return nullptr;
  }

  std::string processor_name;
  if ( !settings.get("name", &processor_name) || processor_name.empty() ) {
    return nullptr;
  }

  c_image_processor::ptr processor =
      c_image_processor::create(processor_name);

  if( !processor ) {
    return nullptr;
  }

  if ( !processor->load_settings(settings) ) {
    return nullptr;
  }

  return processor;
}

c_image_processor::ptr c_image_processor::create(const std::string & processor_name)
{
  if ( processor_name.empty() ) {
    return nullptr;
  }

  c_image_processor::ptr processor;
  const char * cname = processor_name.c_str();

  for ( size_t i = 0; i < sizeof(class_factory_items) / sizeof(class_factory_items[0]); ++i ) {
    if ( strcasecmp(cname, class_factory_items[i].processor_name) == 0 ) {
      return class_factory_items[i].create_instance();
    }
  }

  return nullptr;
}


bool c_image_processor::save_settings(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  settings.set("name", this->name_);
  settings.set("display_name", this->display_name_);
  settings.set("enabled", this->enabled_);

  return true;
}

bool c_image_processor::load_settings(c_config_setting settings)
{
  if ( !settings ) {
    return false;
  }

  std::string processor_name;
  if ( !settings.get("name", &name_) || processor_name != this->name_ ) {
    return false;
  }

  settings.get("enabled", &this->enabled_);
  settings.get("display_name", &this->display_name_);

  return true;
}

//

bool c_image_processor_chain::save_settings(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  settings.set("name", this->name_);
  settings.set("enabled", this->enabled_);

  c_config_setting processors_item =
      settings.add_list("processors");

  for ( const c_image_processor::ptr & processor : *this ) {
    if ( processor && !processor->save_settings(processors_item.add_group("processor") ) ) {
      return false;
    }
  }

  return true;
}

bool c_image_processor_chain::load_settings(c_config_setting settings)
{
  if ( !settings ) {
    return false;
  }

  if ( !settings.get("name", &this->name_) ) {
    return false;
  }

  settings.get("enabled", &this->enabled_);

  c_config_setting processors_item =
      settings["processors"];

  if ( processors_item ) {

    if ( !processors_item.isList() ) {
      return false;
    }

    const int num_processors = processors_item.length();

    base::clear();
    base::reserve(num_processors);

    for ( int i = 0; i < num_processors; ++i ) {

      c_image_processor::ptr processor =
          c_image_processor::load(processors_item.get_element(i));

      if ( processor ) {
        base::emplace_back(processor);
      }
    }
  }

  return true;
}

//

bool c_image_processor_chains::save_settings(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  c_config_setting chains_item =
      settings.add_list("chains");

  for ( const c_image_processor_chain::ptr & chain : *this ) {
    if ( chain && !chain->save_settings(chains_item.add_group("chain")) ) {
      return false;
    }
  }

  return true;
}

bool c_image_processor_chains::load_settings(c_config_setting settings)
{
  if ( !settings ) {
    return false;
  }

  c_config_setting chains_item =
      settings["chains"];

  if ( !chains_item || !chains_item.isList() ) {
    return false;
  }

  const int num_chains = chains_item.length();

  base::clear();
  base::reserve(num_chains);

  for ( int i = 0; i < num_chains; ++i ) {

    c_config_setting chain_item =
        chain_item.get_element(i);

    if ( chain_item && chain_item.isGroup() ) {
      c_image_processor_chain::ptr chain = c_image_processor_chain::create();
      if ( chain && chain->load_settings(chain_item) ) {
        base::emplace_back(chain);
      }
    }
  }

  return true;
}

//

bool c_unsharp_mask_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("sigma", sigma_);
  settings.set("alpha", alpha_);

  return true;
}

bool c_unsharp_mask_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("sigma", &sigma_);
  settings.get("alpha", &alpha_);

  return true;
}

bool c_mtf_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  return true;
}

bool c_mtf_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  return true;
}

bool c_rangeclip_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("min", min_);
  settings.set("max", max_);
  return true;
}

bool c_rangeclip_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("min", &min_);
  settings.get("max", &max_);
  return true;
}


bool c_autoclip_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("plo", plo_);
  settings.set("phi", phi_);

  return true;
}

bool c_autoclip_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("plo", &plo_);
  settings.get("phi", &phi_);

  return true;
}


bool c_range_normalize_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("outmin", outmin_);
  settings.set("outmax", outmax_);

  return true;
}


bool c_histogram_white_balance_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("lclip", &lclip_);
  settings.get("hclip", &hclip_);

  return true;
}

bool c_histogram_white_balance_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("lclip", lclip_);
  settings.set("hclip", hclip_);

  return true;
}


bool c_range_normalize_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("outmin", &outmin_);
  settings.get("outmax", &outmax_);

  return true;
}


bool c_anscombe_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  int v = anscombe_.method();
  if ( settings.get("method", &v) ) {
    anscombe_.set_method((anscombe_method) (v));
  }

  return true;
}

bool c_anscombe_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("method", (int)anscombe_.method());

  return true;
}



bool c_smap_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  settings.set("minv", minv_);
  settings.set("scale", scale_);

  return true;
}

bool c_smap_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
    return false;
  }

  settings.get("minv", &minv_);
  settings.get("scale", &scale_);

  return true;
}


bool c_test_image_processor::save_settings(c_config_setting settings) const
{
  if ( !base::save_settings(settings) ) {
    return false;
  }

  return true;
}

bool c_test_image_processor::load_settings(c_config_setting settings)
{
  if ( !base::load_settings(settings) ) {
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


void c_test_image_processor::process(cv::InputOutputArray image, cv::InputOutputArray mask)
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
}

