/*
 * camera_calibration.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "camera_calibration.h"
#include <core/ssprintf.h>
#include <core/debug.h>


/**
 * write_camera_intrinsics_yml()
 *
 * Write camera intrinsic parameters into yml file storage (intrinsics.yml)
 */
bool write_camera_intrinsics_yml(const c_camera_intrinsics & intrinsics, const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::WRITE);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not write the intrinsic parameters "
        "into output file '%s'", ymlfile.c_str());
    return false;
  }

  fs << "cameraResolution" << intrinsics.image_size;
  fs << "cameraMatrix" << intrinsics.camera_matrix;
  fs << "dist_coeffs" << intrinsics.dist_coeffs;
  fs.release();

  return true;
}

/**
 * read_camera_intrinsics_yml()
 *
 * Read camera intrinsic parameters from yml file storage (intrinsics.yml)
 */
bool read_camera_intrinsics_yml(c_camera_intrinsics * intrinsics, const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::READ);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not open the extrinsics parameters "
        "from input file '%s'", ymlfile.c_str());
    return false;
  }


  if( fs["cameraResolution"].empty() ) {
    CF_ERROR("'cameraResolution' not specified in YML file '%s'", ymlfile.c_str());
    return false;
  }
  try {
    fs["cameraResolution"] >> intrinsics->image_size;
  }
  catch( ... ) {
    CF_ERROR("Error while reading image size 'cameraResolution' from specified yml file '%s'",
        ymlfile.c_str());
    return false;
  }


  if( fs["cameraMatrix"].empty() ) {
    CF_ERROR("'cameraMatrix' not specified in YML file '%s'", ymlfile.c_str());
    return false;
  }
  try {
    fs["cameraMatrix"] >> intrinsics->camera_matrix;
  }
  catch( ... ) {
    CF_ERROR("Error while reading 'cameraMatrix' from specified yml file '%s'",
        ymlfile.c_str());
    return false;
  }


  if( fs["dist_coeffs"].empty() ) {
    CF_WARNING("WARNING: 'dist_coeffs' not specified in YML file '%s'", ymlfile.c_str());
    intrinsics->dist_coeffs.clear();
  }
  else {
    try {
      fs["dist_coeffs"] >> intrinsics->dist_coeffs;
    }
    catch( ... ) {
      CF_ERROR("Error while reading 'dist_coeffs' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }
  }

  return true;
}


/*
 * save_stereo_camera_intrinsics_yml()
 *
 * Write stereo camera intrinsic parameters into yml file stirage (intrinsics.yml)
 */
bool write_stereo_camera_intrinsics_yml(const c_stereo_camera_intrinsics & intrinsics,
    const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::WRITE);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not write the intrinsic parameters "
        "into output file '%s'", ymlfile.c_str());
    return false;
  }

  fs <<
      "S1" << intrinsics.camera[0].image_size <<
      "M1" << intrinsics.camera[0].camera_matrix <<
      "D1" << intrinsics.camera[0].dist_coeffs <<
      "S2" << intrinsics.camera[1].image_size <<
      "M2" << intrinsics.camera[1].camera_matrix <<
      "D2" << intrinsics.camera[1].dist_coeffs;

  fs.release();

  return true;
}

bool read_stereo_camera_intrinsics_yml(c_stereo_camera_intrinsics * intrinsics,
    const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::READ);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not open the extrinsics parameters "
        "from input file '%s'", ymlfile.c_str());
    return false;
  }



  if ( !fs["S"].empty() ) {
    // Assume the same image size for both cameras
    try {

      fs["S"] >> intrinsics->camera[0].image_size;

      intrinsics->camera[1].image_size =
          intrinsics->camera[0].image_size;
    }
    catch (...) {
      CF_ERROR("Error while reading image size 'S' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }
  }
  else {
    // Assume two image sizes S1 and S2 specified

    if ( !fs["S1"].empty() ) {
      try {
        fs["S1"] >> intrinsics->camera[0].image_size;
      }
      catch (...) {
        CF_ERROR("Error while reading image size 'S1' from specified yml file '%s'",
            ymlfile.c_str());
        return false;
      }
    }


    if ( !fs["S2"].empty() ) {
      try {
        fs["S2"] >> intrinsics->camera[1].image_size;
      }
      catch (...) {
        CF_ERROR("Error while reading image size 'S2' from specified yml file '%s'",
            ymlfile.c_str());
        return false;
      }
    }
  }





  if ( !fs["M"].empty() ) {
    // Assume the same camera matrix for both cameras
    try {

      fs["M"] >> intrinsics->camera[0].camera_matrix;

      intrinsics->camera[1].camera_matrix =
          intrinsics->camera[0].camera_matrix;
    }
    catch (...) {
      CF_ERROR("Error while reading 3x3 intrinsics camera matrix 'M' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }
  }
  else {
    // Assume two camera matrices M1 and M2 specified

    try {
      fs["M1"] >> intrinsics->camera[0].camera_matrix;
    }
    catch (...) {
      CF_ERROR("Error while reading 3x3 intrinsics camera matrix 'M1' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }

    try {
      fs["M2"] >> intrinsics->camera[1].camera_matrix;
    }
    catch (...) {
      CF_ERROR("Error while reading 3x3 intrinsics camera matrix 'M2' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }
  }




  if ( !fs["D"].empty() ) {
    // Assume the same distortion coefficients D for both cameras
    try {

      fs["D"] >> intrinsics->camera[0].dist_coeffs;

      intrinsics->camera[1].dist_coeffs =
          intrinsics->camera[0].dist_coeffs;
    }
    catch (...) {
      CF_ERROR("Error while reading distortion coefficients 'D' from specified yml file '%s'",
          ymlfile.c_str());
      return false;
    }
  }
  else {
    // Assume two distinct distortion coefficients  D1 and D2 specified

    if ( !fs["D1"].empty() ) {

      try {
        fs["D1"] >> intrinsics->camera[0].dist_coeffs;
      }
      catch (...) {
        CF_ERROR("Error while reading distortion coefficients 'D1' from specified yml file '%s'",
            ymlfile.c_str());
        return false;
      }
    }

    if ( !fs["D2"].empty() ) {

      try {
        fs["D2"] >> intrinsics->camera[1].dist_coeffs;
      }
      catch (...) {
        CF_ERROR("Error while reading distortion coefficients 'D2' from specified yml file '%s'",
            ymlfile.c_str());
        return false;
      }
    }
  }


  return true;
}

/*
 * save_stereo_camera_extrinsics_yml()
 *
 * Write stereo camera extrinsics parameters into yml file storage (extrinsics.yml)
 */
bool write_stereo_camera_extrinsics_yml(const c_stereo_camera_extrinsics & extrinsics,
    const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::WRITE);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not write the extrinsics parameters "
        "into output file '%s'", ymlfile.c_str());
    return false;
  }

  fs <<
      "R" << extrinsics.R <<
      "T" << extrinsics.T;

  return true;
}

/*
 * load_stereo_camera_extrinsics_yml()
 *
 * Read stereo camera extrinsics parameters into yml file storage (extrinsics.yml)
 */
bool read_stereo_camera_extrinsics_yml(c_stereo_camera_extrinsics * extrinsics,
    const std::string & ymlfile)
{
  cv::FileStorage fs(ymlfile, cv::FileStorage::READ);

  if ( !fs.isOpened() ) {
    CF_ERROR("Error: can not open the extrinsics parameters "
        "from input file '%s'", ymlfile.c_str());
    return false;
  }

  try {
    fs["R"] >> extrinsics->R;
  }
  catch (...) {
    CF_ERROR("Error while reading extrinsics 3x3 rotation matrix 'R' from specified yml file '%s'",
        ymlfile.c_str());
    return false;
  }

  try {
    fs["T"] >> extrinsics->T;
  }
  catch (...) {
    CF_ERROR("Error while reading extrinsics 3x1 translation vector 'T' from specified yml file '%s'",
        ymlfile.c_str());
    return false;
  }

  return true;
}
