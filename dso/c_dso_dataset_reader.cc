/*
 * c_dso_dataset_reader.cc
 *
 *  Created on: Jun 9, 2024
 *      Author: amyznikov
 */

#include "c_dso_dataset_reader.h"
#include <core/io/c_stdio_file.h>
#include <dirent.h>
#include <algorithm>
#include <core/readdir.h>
#include <core/io/load_image.h>
#include <core/settings.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace {

using c_image_and_exposure = c_dso_dataset_reader::c_image_and_exposure;
using Undistort = dso::c_image_undistort;


struct c_dso_dataset_settings
{
  std::string files; // "/home/projects/dso/mono-dataset/sequence_46/images.zip"
  std::string calib; // "/home/projects/dso/mono-dataset/sequence_46/camera.txt"
  std::string gamma; // " /home/projects/dso/mono-dataset/sequence_46/pcalib.txt"
  std::string vignette;
};

bool load_settings(c_config_setting settings, c_dso_dataset_settings * cfg)
{
  LOAD_OPTION(settings, *cfg, files);
  LOAD_OPTION(settings, *cfg, calib);
  LOAD_OPTION(settings, *cfg, gamma);
  LOAD_OPTION(settings, *cfg, vignette);

  return true;
}

static bool load_settings(const std::string & filename, c_dso_dataset_settings * cfg)
{
  c_config config_file(filename);

  if ( !config_file.read() ) {
    CF_ERROR("config_file.read('%s') fails", filename.c_str());
    return false;
  }

  return load_settings(config_file.root(), cfg);
}



static int getdir(std::string dir, std::vector<std::string> & files)
{
  DIR * dp;
  struct dirent * dirp;

  if( !(dp = opendir(dir.c_str())) ) {
    return -1;
  }

  while ((dirp = readdir(dp))) {

    const std::string name(dirp->d_name);

    if( name != "." && name != ".." ) {
      files.push_back(name);
    }
  }

  closedir(dp);

  std::sort(files.begin(), files.end());

  if( dir.at(dir.length() - 1) != '/' ) {
    dir = dir + "/";
  }

  for( size_t i = 0; i < files.size(); i++ ) {
    if( files[i].at(0) != '/' ) {
      files[i] = dir + files[i];
    }
  }

  return files.size();
}

} // namespace

c_dso_dataset_reader::c_dso_dataset_reader()
{
}

c_dso_dataset_reader::~c_dso_dataset_reader()
{
  close();
}

c_dso_dataset_reader * c_dso_dataset_reader::load(const std::string & config_filename)
{
  c_dso_dataset_settings cfg;

  if ( !load_settings(config_filename, &cfg) ) {
    CF_ERROR("load_settings('%s') fails", config_filename.c_str());
    return nullptr;
  }

  this_class * dataset =
      new this_class();


  const std::string directory =
      get_parent_directory(config_filename);

  if ( !directory.empty() ) {

    if ( !cfg.files.empty() && !is_absolute_path(cfg.files) ) {
      cfg.files = ssprintf("%s/%s", directory.c_str(), cfg.files.c_str());
    }

    if ( !cfg.calib.empty() && !is_absolute_path(cfg.calib) ) {
      cfg.calib = ssprintf("%s/%s", directory.c_str(), cfg.calib.c_str());
    }

    if ( !cfg.gamma.empty() && !is_absolute_path(cfg.gamma) ) {
      cfg.gamma = ssprintf("%s/%s", directory.c_str(), cfg.gamma.c_str());
    }

    if ( !cfg.vignette.empty() && !is_absolute_path(cfg.vignette) ) {
      cfg.vignette = ssprintf("%s/%s", directory.c_str(), cfg.vignette.c_str());
    }
  }

  if ( !dataset->open(cfg.files, cfg.calib, cfg.gamma, cfg.vignette) ) {
    CF_ERROR("dataset->open() fails for config file '%s'", config_filename.c_str());
    delete dataset;
    return nullptr;
  }

  return dataset;
}

void c_dso_dataset_reader::close()
{
#if HAS_ZIPLIB
  if( ziparchive ) {
    zip_close(ziparchive);
    ziparchive = nullptr;
  }

  if ( zip_databuffer ) {
    delete zip_databuffer;
    zip_databuffer = nullptr;
  }
#endif

  if ( undistort ) {
    undistort.reset();
  }
}

int c_dso_dataset_reader::numImages() const
{
  return files.size();
}

bool c_dso_dataset_reader::getImage(int id, c_image_and_exposure * image)
{
  return getImage_internal(id, image);
}

const float* c_dso_dataset_reader::photometricGamma() const
{
  return undistort ? undistort->photometricGamma() : nullptr;
}

double c_dso_dataset_reader::getTimestamp(int id) const
{
  if( timestamps.empty() ) {
    return id * 0.1f;
  }

  if( id < 0 || id >= (int) timestamps.size() ) {
    return 0;
  }

  return timestamps[id];
}

Eigen::VectorXf c_dso_dataset_reader::getOriginalCalib() const
{
  return undistort->getOriginalParameter().cast<float>();
}
Eigen::Vector2i c_dso_dataset_reader::getOriginalDimensions() const
{
  return undistort->getOriginalSize();
}

void c_dso_dataset_reader::getCalibMono(Eigen::Matrix3f & K, int & w, int & h) const
{
  K = undistort->getK().cast<float>();
  w = undistort->getSize()[0];
  h = undistort->getSize()[1];
}

void c_dso_dataset_reader::setGlobalCalibration()
{
  int w_out, h_out;
  Eigen::Matrix3f K;
  getCalibMono(K, w_out, h_out);
  dso::setGlobalCalib(w_out, h_out, K);
}

void c_dso_dataset_reader::prepImage(int /*id*/, bool /*as8U*/)
{
}



bool c_dso_dataset_reader::open(const std::string & path, const std::string & calibFile, const std::string & gammaFile, const std::string & vignetteFile)
{
  this->path = path;
  this->calibfile = calibFile;

  const bool isZipped =
      strcasecmp(get_file_suffix(path).c_str(), ".zip") == 0;

  if( !isZipped ) {
    getdir(path, files);
  }
  else {

#if HAS_ZIPLIB

    int ziperror = 0;

    if( !(ziparchive = zip_open(path.c_str(), ZIP_RDONLY, &ziperror)) || ziperror ) {

      CF_ERROR("ZIPERROR=%d reading zip archive '%s'", ziperror, path.c_str());

      if( ziparchive ) {
        zip_close(ziparchive);
        ziparchive = nullptr;
      }

      return false;
    }

    files.clear();

    const int numEntries =
        zip_get_num_entries(ziparchive, 0);

    for( int k = 0; k < numEntries; k++ ) {

      const char * name =
          zip_get_name(ziparchive, k,
          ZIP_FL_ENC_STRICT);

      const std::string nstr =
          std::string(name);

      if( nstr == "." || nstr == ".." ) {
        continue;
      }

      files.push_back(name);
    }

    CF_DEBUG("got %d entries and %zu files!", numEntries, files.size());

    std::sort(files.begin(), files.end());

#else
    CF_ERROR("ERROR: cannot read .zip archive, as compiled without ziplib!");
    return nullptr;
#endif
  }

  undistort.reset(Undistort::load(calibFile, gammaFile,
      vignetteFile));

  if( !undistort ) {
    CF_ERROR("Undistort::getUndistorterForFile() fails");
    close();
    return false;
  }

  const Eigen::Vector2i imageSize =
      undistort->getSize();

  const Eigen::Vector2i origSize =
      undistort->getOriginalSize();

  width = undistort->getSize()[0];
  height = undistort->getSize()[1];

  widthOrg = origSize[0];
  heightOrg = origSize[1];

  // load timestamps if possible.
  loadTimestamps();

  return true;
}

void c_dso_dataset_reader::loadTimestamps()
{
#if HAS_ZIPLIB
  const bool isZipped =
      ziparchive != nullptr;
#else
  const bool isZipped = false;
#endif

  c_stdio_file file;

  std::string parent_directory =
      get_parent_directory(path);

  std::string timestaps_filename =
      ssprintf("%s/times.txt", parent_directory.c_str());

  if( !file.open(timestaps_filename, "rt") ) {

    CF_ERROR("Can not read timestaps from '%s' : %s",
        timestaps_filename.c_str(),
        strerror(errno));

    parent_directory =
        get_parent_directory(parent_directory);

    timestaps_filename =
          ssprintf("%s/times.txt", parent_directory.c_str());

    file.open(timestaps_filename, "rt");
  }

  if( !file.is_open() ) {
    CF_ERROR("Can not read timestaps from '%s': %s",
        timestaps_filename.c_str(),
        strerror(errno));
  }
  else {

    CF_ERROR("read timestaps from '%s'",
        timestaps_filename.c_str());

    char buf[1000];

    while (fgets(buf, sizeof(buf), file)) {

      int id;
      double stamp;
      float exposure = 0;

      if( sscanf(buf, "%d %lf %f", &id, &stamp, &exposure) == 3 ) {
        timestamps.push_back(stamp);
        exposures.push_back(exposure);
      }

      else if( 2 == sscanf(buf, "%d %lf", &id, &stamp) ) {
        timestamps.push_back(stamp);
        exposures.push_back(exposure);
      }
    }
  }

  file.close();


  // check if exposures are correct, (possibly skip)
  bool exposuresGood =
      ((int) exposures.size() == (int) numImages());

  for( size_t i = 0, n = exposures.size(); i < n; ++i ) {

    if( exposures[i] == 0 ) {

      // fix!
      float sum = 0, num = 0;

      if( i > 0 && exposures[i - 1] > 0 ) {
        sum += exposures[i - 1];
        num++;
      }

      if( i + 1 < (int) exposures.size() && exposures[i + 1] > 0 ) {
        sum += exposures[i + 1];
        num++;
      }

      if( num > 0 ) {
        exposures[i] = sum / num;
      }
    }

    if( exposures[i] == 0 ) {
      exposuresGood = false;
    }
  }

  if( (int) numImages() != (int) timestamps.size() ) {
    CF_ERROR(" Number of images and timestamps not equal: set timestamps and exposures to zero!");
    exposures.clear();
    timestamps.clear();
  }

  if( (int) numImages() != (int) exposures.size() || !exposuresGood ) {
    CF_ERROR(" Number of images and exposures not equal: set exposures to zero!");
    exposures.clear();
  }

  CF_DEBUG("got %d images and %d timestamps and %d exposures.!",
      (int) numImages(),
      (int) timestamps.size(),
      (int) exposures.size());

}


bool c_dso_dataset_reader::getRawImage_internal(int id, cv::Mat * image)
{
#if HAS_ZIPLIB
  const bool isZipped =
      ziparchive != nullptr;
#else
  const bool isZipped = false;
#endif

  if( !isZipped ) {
    // return dso::IOWrap::readImageBW_8U(files[id]);

    if ( !load_image(files[id], *image) ) {
      CF_ERROR("load_image('%s') fails", files[id].c_str());
      return false;
    }

    return true;
  }

#if !HAS_ZIPLIB
  CF_ERROR("ERROR: cannot read .zip archive, as compiled without ziplib!");
  return false;
#else

  zip_file_t * zipfp =
      zip_fopen(ziparchive, files[id].c_str(), 0);

  if( !zipfp ) {
    CF_ERROR("zip_fopen('%z') fails", files[id].c_str());
    return false;
  }

  if( !zip_databuffer ) {
    zip_databuffer = new char[widthOrg * heightOrg * 6 + 10000];
  }

  long readbytes =
      zip_fread(zipfp, zip_databuffer,
          (long) widthOrg * heightOrg * 6 + 10000);

  zip_fclose(zipfp);

  if( readbytes > (long) widthOrg * heightOrg * 6 ) {

    CF_ERROR("read %ld/%ld bytes for file %s. increase buffer!!", readbytes,
        (long ) widthOrg * heightOrg * 6 + 10000, files[id].c_str());

    delete[] zip_databuffer;
    zip_databuffer = nullptr;

    if( !(zipfp = zip_fopen(ziparchive, files[id].c_str(), 0)) ) {
      CF_ERROR("zip_fopen('%z') fails", files[id].c_str());
      return false;
    }

    zip_databuffer = new char[(long) widthOrg * heightOrg * 30];
    readbytes = zip_fread(zipfp, zip_databuffer, (long) widthOrg * heightOrg * 30 + 10000);
    zip_fclose(zipfp);

    if( readbytes > (long) widthOrg * heightOrg * 30 ) {

      CF_ERROR("buffer still to small (read %ld/%ld). abort.", readbytes,
          (long ) widthOrg * heightOrg * 30 + 10000);

      delete zip_databuffer;
      zip_databuffer = nullptr;
      return false;
    }
  }

  if( (*image = cv::imdecode(cv::Mat(readbytes, 1, CV_8U, (void* )zip_databuffer), cv::IMREAD_GRAYSCALE)).empty() ) {
    CF_ERROR("cv::imdecode(zip_databuffer) fails");
    return false;
  }

  return true;
#endif

}

bool c_dso_dataset_reader::getImage_internal(int id, c_image_and_exposure * output_image)
{
  cv::Mat image;

  if ( !getRawImage_internal(id, &image) ) {
    CF_ERROR("getRawImage_internal(id=%d) fails", id);
    return false;
  }

  //  if ( image.depth() != CV_8U ) {
  //    image.convertTo(image, CV_8U);
  //  }

  if ( image.channels() != 1 ) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }

  return undistort->undistort(image, output_image,
      (exposures.size() == 0 ? 1.0f : exposures[id]),
      (timestamps.size() == 0 ? 0.0 : timestamps[id]));

}

