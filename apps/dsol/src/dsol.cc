/*
 * dsol.cc
 *
 *  Created on: Jun 8, 2024
 *      Author: amyznikov
 */
//
#include "dso/FullSystem.h"
#include "dso/c_dso_dataset_reader.h"
#include "c_dso_display_opencv.h"
#include <core/settings.h>
#include <core/debug.h>

using namespace dso;

class c_FullSystem :
    public FullSystem
{
public:
  typedef c_FullSystem this_class;
  typedef FullSystem base;
  typedef std::unique_ptr<this_class> uptr;


  void onFrameHessianMakeImages(FrameHessian * fh) override;

protected:


};

void c_FullSystem::onFrameHessianMakeImages(FrameHessian * fh)
{
  const int id =
      fh->shell->id;

  char filename[PATH_MAX] = "";

  CF_DEBUG("pointHessians.size=%zu", fh->pointHessians.size());


//  for ( int l = 0; l < PYR_LEVELS; ++l ) {
//
//    if ( !fh->absSquaredGrad[l] || hG[l] < 1 ||  wG[l] < 1 ) {
//      break;
//    }
//
//    cv::Mat1f cvimg(hG[l], wG[l], fh->absSquaredGrad[l]);
//
//    sprintf(filename, "../mono-dataset/sequence_46/absSquaredGrad/absSquaredGrad.%06d.%03d.tiff", id, l);;
//
//    CF_DEBUG("l=%d %dx%d", l, hG[l], wG[l]);
//
//    if( !cv::imwrite(filename, cvimg) ) {
//      CF_ERROR("cv::imwrite('%s') fails", filename);
//      return;
//    }
//  }
}


struct c_dso_app_settings
{
  std::string files; // "/home/projects/dso/mono-dataset/sequence_46/images.zip"
  std::string calib; // "/home/projects/dso/mono-dataset/sequence_46/camera.txt"
  std::string gamma; // " /home/projects/dso/mono-dataset/sequence_46/pcalib.txt"
  std::string vignette;
};

bool load_settings(c_config_setting settings, c_dso_app_settings * cfg)
{
  LOAD_OPTION(settings, *cfg, files);
  LOAD_OPTION(settings, *cfg, calib);
  LOAD_OPTION(settings, *cfg, gamma);
  LOAD_OPTION(settings, *cfg, vignette);

  return true;
}

static bool load_settings(const std::string & filename, c_dso_app_settings * cfg)
{
  c_config config_file(filename);

  if ( !config_file.read() ) {
    CF_ERROR("config_file.read('%s') fails", filename.c_str());
    return false;
  }

  return load_settings(config_file.root(), cfg);
}

/**
 * ./apps/dsol/dsol --files /home/projects/dso/mono-dataset/sequence_46/images.zip --calib /home/projects/dso/mono-dataset/sequence_46/camera.txt --gamma /home/projects/dso/mono-dataset/sequence_46/pcalib.txt
 *
 */

int main(int argc, char *argv[])
{
  c_dso_app_settings cfg;
  //load_settings

//  std::string config_filename = "dsol.conf";
//  std::string vignette = "";
//  std::string gammaCalib = "";
//  std::string source = "";
//  std::string calib = "";


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  for( int i = 1; i < argc; ++i ) {
    if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Options:\n"
          " --config <path/to/config/file.cfg>\n"
          " --files <path_to_files_directory>\n"
          " --calib <path_to_calib_directory>\n"
          " --vignette <path_to_vignette_calib>\n"
          " --gamma <path_to_gamma_calib>\n"
          "\n"
          "\n"
          "\n");

      return 0;
    }


    if( strcmp(argv[i], "--config") == 0 ) {

      if( ++i >= argc ) {
        fprintf(stderr, "Missing mandatory argument after '%s' key\n", argv[i - 1]);
        return 1;
      }

      if ( !load_settings(argv[i], &cfg) ) {
        fprintf(stderr, "load_settings('%s') fails\n", argv[i]);
        return 1;
      }

      continue;
    }


    if( strcmp(argv[i], "--files") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Missing mandatory argument after '%s' key\n", argv[i - 1]);
        return 1;
      }

      cfg.files =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "--calib") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Missing mandatory argument after '%s' key\n", argv[i - 1]);
        return 1;
      }

      cfg.calib =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "--gamma") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Missing mandatory argument after '%s' key\n", argv[i - 1]);
        return 1;
      }

      cfg.gamma =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "--vignette") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Missing mandatory argument after '%s' key\n", argv[i - 1]);
        return 1;
      }

      cfg.vignette =
          argv[i];

      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n",
        argv[i]);
    return 1;
  }


  c_dso_dataset_reader::uptr reader(new c_dso_dataset_reader());

  if( !reader->open(cfg.files, cfg.calib, cfg.gamma, cfg.vignette) ) {
    CF_ERROR("c_dso_dataset_reader::open() fails");
    return 1;
  }

  reader->setGlobalCalibration();

  int numImages =
      reader->getNumImages();

  CF_DEBUG("NumImages=%d", numImages);

  c_FullSystem::uptr fullSystem(new c_FullSystem());
  fullSystem->setPhotometricGamma(reader->photometricGamma());

  c_dso_display_opencv::uptr opencv_display(new c_dso_display_opencv());
  fullSystem->display = opencv_display.get();

  c_image_and_exposure image;

  for ( int i = 0; i < numImages; ++i ) {

    if ( !reader->getImage(i, &image) ) {
      CF_ERROR("getImage(%d) fails", i);
      break;
    }

    CF_DEBUG("img[%d]   exposure=%g", i, image.exposure());

    fullSystem->addActiveFrame(image, i);
  }

  CF_DEBUG("OK");

  return 0;
}
