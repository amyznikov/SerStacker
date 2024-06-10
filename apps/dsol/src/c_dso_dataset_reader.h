/*
 * c_dso_dataset_reader.h
 *
 *  Created on: Jun 9, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dso_dataset_reader_h__
#define __c_dso_dataset_reader_h__

#include "dso/util/usettings.h"
#include "dso/util/globalFuncs.h"
#include "dso/util/globalCalib.h"
#include "dso/util/Undistort.h"
#include "IOWrapper/ImageRW.h"

#if HAS_ZIPLIB
# include <zip.h>
#endif


class c_dso_dataset_reader
{
public:
  typedef c_dso_dataset_reader this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  using MinimalImageB = dso::MinimalImageB;
  using c_image_and_exposure = dso::c_image_and_exposure;

  c_dso_dataset_reader();
  ~c_dso_dataset_reader();

  bool open(const std::string & path, const std::string & calibFile, const std::string & gammaFile, const std::string & vignetteFile);
  void close();

  int getNumImages() const;

  void setGlobalCalibration();

  float * getPhotometricGamma() const;
  void getCalibMono(Eigen::Matrix3f & K, int & w, int & h) const;

  Eigen::VectorXf getOriginalCalib() const;
  Eigen::Vector2i getOriginalDimensions() const;

  void prepImage(int id, bool as8U = false);

  bool getImage(int id, c_image_and_exposure * image);

  MinimalImageB* getRawImage(int id);
  double getTimestamp(int id) const;

protected:
  bool getImage_internal(int id, c_image_and_exposure * image);

  bool getRawImage_internal(int id, cv::Mat * image);
  MinimalImageB * getRawImage_internal(int id);
  void loadTimestamps();

protected:
  std::string path;
  std::string calibfile;

  std::vector<c_image_and_exposure*> preloadedImages;
  std::vector<std::string> files;
  std::vector<double> timestamps;
  std::vector<float> exposures;

  int width = 0;
  int height = 0;
  int widthOrg = 0;
  int heightOrg = 0;

  // undistorter. [0] always exists, [1-2] only when MT is enabled.
  dso::Undistort * undistort = nullptr;

#if HAS_ZIPLIB
  zip_t* ziparchive = nullptr;
  char * zip_databuffer = nullptr;
#endif

};

#endif /* __c_dso_dataset_reader_h__ */
