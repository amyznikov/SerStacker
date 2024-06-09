/*
 * dsol.cc
 *
 *  Created on: Jun 8, 2024
 *      Author: amyznikov
 */
//
//#include "odom.h"
//#include "extra.h"

#include "FullSystem.h"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include <core/settings.h>
#include <core/debug.h>


using namespace dso;

//typedef std::unique_ptr<ImageFolderReader>
//  ImageFolderReaderPtr;

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


//namespace sv::dsol {
//
//struct c_app_settings
//{
//  OdomCfg odom;
//  SelectCfg selection;
//  StereoCfg stereo;
//  DirectCfg align;
//  DirectCfg adjust;
//
//};
//
//}
//
//
//using namespace sv;
//using namespace sv::dsol;
//
//
//
//
//bool load_settings(c_config_setting settings, OdomCfg * cfg)
//{
//  LOAD_OPTION(settings, *cfg, marg);
//  LOAD_OPTION(settings, *cfg, num_kfs);
//  LOAD_OPTION(settings, *cfg, num_levels);
//  LOAD_OPTION(settings, *cfg, min_track_ratio);
//  LOAD_OPTION(settings, *cfg, vis_min_depth);
//
//  LOAD_OPTION(settings, *cfg, reinit);
//  LOAD_OPTION(settings, *cfg, init_depth);
//  LOAD_OPTION(settings, *cfg, init_stereo);
//  LOAD_OPTION(settings, *cfg, init_align);
//
//  LOAD_OPTION(settings, *cfg, tbb);
//  LOAD_OPTION(settings, *cfg, log);
//  LOAD_OPTION(settings, *cfg, vis);
//
//  return true;
//}
//
//bool load_settings(c_config_setting settings, SelectCfg * cfg)
//{
//  LOAD_OPTION(settings, *cfg, sel_level);
//  LOAD_OPTION(settings, *cfg, cell_size);
//  LOAD_OPTION(settings, *cfg, min_grad);
//  LOAD_OPTION(settings, *cfg, max_grad);
//  LOAD_OPTION(settings, *cfg, nms_size);
//  LOAD_OPTION(settings, *cfg, min_ratio);
//  LOAD_OPTION(settings, *cfg, max_ratio);
//  LOAD_OPTION(settings, *cfg, reselect);
//
//  return true;
//}
//
//
//bool load_settings(c_config_setting settings, StereoCfg * cfg)
//{
//  LOAD_OPTION(settings, *cfg, half_rows);
//  LOAD_OPTION(settings, *cfg, half_cols);
//  LOAD_OPTION(settings, *cfg, match_level);
//  LOAD_OPTION(settings, *cfg, refine_size);
//  LOAD_OPTION(settings, *cfg, min_zncc);
//  LOAD_OPTION(settings, *cfg, min_depth);
//  LOAD_OPTION(settings, *cfg, best_ratio);
//
//  return true;
//}
//
//bool load_settings(c_config_setting settings, DirectOptmCfg * cfg)
//{
//  LOAD_OPTION(settings, *cfg, init_level);
//  LOAD_OPTION(settings, *cfg, max_iters);
//  LOAD_OPTION(settings, *cfg, max_xs);
//  return true;
//}
//
//bool load_settings(c_config_setting settings, DirectCostCfg * cfg)
//{
//  LOAD_OPTION(settings, *cfg, affine);
//  LOAD_OPTION(settings, *cfg, stereo);
//  LOAD_OPTION(settings, *cfg, c2);
//  LOAD_OPTION(settings, *cfg, dof);
//  LOAD_OPTION(settings, *cfg, max_outliers);
//  LOAD_OPTION(settings, *cfg, grad_factor);
//  LOAD_OPTION(settings, *cfg, min_depth);
//  return true;
//}
//
//
//bool load_settings(c_config_setting settings, DirectCfg * cfg)
//{
//  c_config_setting section;
//
//  if( (section = GET_SETTINGS_SECTION(settings, "optm")) ) {
//    if( !load_settings(section, &cfg->optm) ) {
//      CF_ERROR("load_settings('optm') fails");
//      return false;
//    }
//  }
//
//  if( (section = GET_SETTINGS_SECTION(settings, "cost")) ) {
//    if( !load_settings(section, &cfg->cost) ) {
//      CF_ERROR("load_settings('cost') fails");
//      return false;
//    }
//  }
//
//  return true;
//}
//
//
//
//bool load_settings(c_config_setting settings, c_app_settings * cfg)
//{
//  c_config_setting section;
//
//  if( (section = GET_SETTINGS_SECTION(settings, "odometry")) ) {
//    if( !load_settings(section, &cfg->odom) ) {
//      CF_ERROR("load_settings('odometry') fails");
//      return false;
//    }
//  }
//
//  if( (section = GET_SETTINGS_SECTION(settings, "selection")) ) {
//    if( !load_settings(section, &cfg->selection) ) {
//      CF_ERROR("load_settings('selection') fails");
//      return false;
//    }
//  }
//
//  if( (section = GET_SETTINGS_SECTION(settings, "stereo")) ) {
//    if( !load_settings(section, &cfg->stereo) ) {
//      CF_ERROR("load_settings('stereo') fails");
//      return false;
//    }
//  }
//
//  if( (section = GET_SETTINGS_SECTION(settings, "align")) ) {
//    if( !load_settings(section, &cfg->align) ) {
//      CF_ERROR("load_settings('align') fails");
//      return false;
//    }
//  }
//
//  if( (section = GET_SETTINGS_SECTION(settings, "adjust")) ) {
//    if( !load_settings(section, &cfg->adjust) ) {
//      CF_ERROR("load_settings('adjust') fails");
//      return false;
//    }
//  }
//
//  return true;
//}
//
//
//static bool load_settings(const std::string & filename, c_app_settings * cfg)
//{
//  c_config config_file(filename);
//
//  if ( !config_file.read() ) {
//    CF_ERROR("config_file.read('%s') fails", filename.c_str());
//    return false;
//  }
//
//  return load_settings(config_file.root(), cfg);
//}


int main(int argc, char *argv[])
{
  std::string config_filename =
      "dsol.conf";




  for( int i = 1; i < argc; ++i ) {
    if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
      fprintf(stdout, "Usage:\n"
          "  dsol --config <path/to/dsol.conf\n"
          "\n");

      return 0;
    }

    if ( strcmp(argv[i], "--config") == 0 ) {
      if ( ++i > argc ) {
        fprintf(stderr, "Path to config file expected\n");
        return 1;
      }

      config_filename =
          argv[i];

      continue;
    }


    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }





  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_FullSystem::uptr fullSystem(new c_FullSystem());

  //fullSystem->addActiveFrame(nullptr, 0);

//
//
//
//  DirectOdometry odom_;
//  MotionModel motion_;
//  c_app_settings cfg;
//
//  if( !load_settings(config_filename, &cfg) ) {
//    CF_ERROR("load_settings('%s') fails", config_filename.c_str());
//    return 1;
//  }
//
//  odom_.Init(cfg.odom);
//
//  odom_.selector = PixelSelector(cfg.selection);
//  odom_.matcher = StereoMatcher(cfg.stereo);
//  odom_.aligner = FrameAligner(cfg.align);
//  odom_.adjuster = BundleAdjuster(cfg.adjust);
//  odom_.cmap = MakeCmapJet();
//
//  motion_.Init();

  CF_DEBUG("OK");

  return 0;
}
