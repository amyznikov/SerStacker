/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/proc/eccalign.h>
#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/registration/c_feature_based_registration.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/jupiter.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <core/registration/c_planetary_disk_registration.h>
#include <tbb/tbb.h>
#include <core/debug.h>
#include <variant>


enum new_anscombe_method
{
  new_anscombe_method_none = 0,
  new_anscombe_method_regular = 1,
  new_anscombe_method_sqrt = 2,
};


template<class enum_type>
class c_enum_combobox_base
{
public:
  c_enum_combobox_base()
  {
    populate_combo();
  }

  void populate_combo()
  {
//    const c_enum_member * members =
//        members_of<enum_type>();

//    while ( members->name ) {
//      ++members;
//    }
  }
};

class c_new_anscombe_method_combo:
     public c_enum_combobox_base<new_anscombe_method>
{
public:
  c_new_anscombe_method_combo()
  {
  }
};



class c_some_processor
{
public:

  virtual ~c_some_processor() = default;

  //virtual void get_parameters(std::vector<c_property> * params) const  = 0;
  virtual bool set_parameter(const std::string & name, const std::string & value) = 0;
  virtual bool get_parameter(const std::string & name, std::string * value) = 0;

};

class c_some_anscombe_processor
  : public c_some_processor
{
public:

  typedef c_some_anscombe_processor this_class;

  void set_some_value(int v)
  {
    CF_DEBUG("some_value_ = %d", v);
    some_value_ = v;
  }

  int some_value() const
  {
    return some_value_;
  }

  void set_method(new_anscombe_method v)
  {
    method_ = v;
  }

  new_anscombe_method method() const
  {
    return method_;
  }

//  void get_parameters(std::vector<c_property> * params) const
//  {
//    params->emplace_back("some_value", "this is some value");
//    params->emplace_back("method", "this is some method", members_of<decltype(method())>());
//  }

  bool ddxparam(bool getit, const std::string & name, std::string & value)
  {
//    STRDDX(some_value, name, value, getit);
//    STRDDX(method, name, value, getit);
    return false;
  }

  bool set_parameter(const std::string & name, const std::string & value)
  {
    return ddxparam(false, name, const_cast<std::string &>(value));
  }

  virtual bool get_parameter(const std::string & name, std::string * value)
  {
    return ddxparam(true, name, *value);
  }

protected:
  int some_value_ = 0;
  new_anscombe_method method_ = new_anscombe_method_sqrt;
};


class c_property_list_widget {
public:

  void populate(c_some_processor * p)
  {


  }
};

int main(int argc, char *argv[])
{
    cf_set_logfile(stderr);
    cf_set_loglevel(CF_LOG_DEBUG);

  c_new_anscombe_method_combo combo;
  c_some_anscombe_processor proc;

  proc.set_parameter("some_value", "10");

  return 0;



//  std::string filenames[2];
//  cv::Mat images[2], masks[2];
//  cv::Mat feature_image, feature_mask;
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name1.tiff> <input-file-name2.tiff> \n");
//      return 0;
//    }
//
//    else if ( filenames[0].empty() ) {
//      filenames[0] = argv[i];
//    }
//    else if ( filenames[1].empty() ) {
//      filenames[1] = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filenames[0].empty() /*|| filenames[1].empty()*/ ) {
//    fprintf(stderr, "Two input file names expected\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  for ( int i = 0; i < 1; ++i ) {
//    if ( !load_image(filenames[i], images[i], masks[i]) ) {
//      CF_ERROR("load_image(%s) fails", filenames[i].c_str());
//      return 1;
//    }
//  }

//  cv::Mat1f image(128, 512, 0.f);
//  cv::Mat_<std::complex<float>> spec;
//  cv::Mat1f spow;
//
//  cv::circle(image, cv::Point(image.cols/2, image.rows/2), 31, 1, -1, cv::LINE_8 );
//
//  save_image(image, "image.tiff");
//
//  cv::dft(image, spec, cv::DFT_COMPLEX_OUTPUT);
//  fftSwapQuadrants(spec);
//
//  cv::Mat channels[2];
//  cv::split(spec, channels);
//  cv::magnitude(channels[0], channels[1], spow);
//  cv::log(1 + spow, spow);
//  save_image(spow, "spow.tiff");


  return 0;
}

