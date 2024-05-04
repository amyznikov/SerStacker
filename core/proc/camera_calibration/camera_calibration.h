/*
 * camera_calibration.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __camera_calibration_h__
#define __camera_calibration_h__

#include <opencv2/opencv.hpp>


/**
 * Camera matrix and distortion coefficients
 * */
struct c_camera_intrinsics {
  cv::Size image_size;
  cv::Matx33d camera_matrix;
  std::vector<double> dist_coeffs;
};

/**
 * Two sets of intrinsics for each of left and right cameras
 * */
struct c_stereo_camera_intrinsics {
  c_camera_intrinsics camera[2];
};

/**
 * Stereo extrinsics
 * */
struct c_stereo_camera_extrinsics {
  cv::Matx33d R;
  cv::Vec3d T;
};


/**
 * write_camera_intrinsics_yml()
 *
 * Write camera intrinsic parameters into yml file storage (intrinsics.yml)
 */
bool write_camera_intrinsics_yml(const c_camera_intrinsics & intrinsics,
    const std::string & ymlfile);

/**
 * read_camera_intrinsics_yml()
 *
 * Read camera intrinsic parameters from yml file storage (intrinsics.yml)
 */
bool read_camera_intrinsics_yml(c_camera_intrinsics * intrinsics,
    const std::string & ymlfile);


/**
 * write_stereo_camera_intrinsics_yml()
 *
 * Write stereo camera intrinsic parameters into yml file storage (intrinsics.yml)
 */
bool write_stereo_camera_intrinsics_yml(const c_stereo_camera_intrinsics & intrinsics,
    const std::string & ymlfile);

/**
 * read_stereo_camera_intrinsics_yml()
 *
 * Read stereo camera intrinsic parameters from yml file storage (intrinsics.yml)
 */
bool read_stereo_camera_intrinsics_yml(c_stereo_camera_intrinsics * intrinsics,
    const std::string & ymlfile);


/**
 * write_stereo_camera_extrinsics_yml()
 *
 * Write stereo camera extrinsics parameters into yml file storage (extrinsics.yml)
 */
bool write_stereo_camera_extrinsics_yml(const c_stereo_camera_extrinsics & extrinsics,
    const std::string & ymlfile);

/**
 * load_stereo_camera_extrinsics_yml()
 *
 * Read stereo camera extrinsics parameters into yml file storage (extrinsics.yml)
 */
bool read_stereo_camera_extrinsics_yml(c_stereo_camera_extrinsics * extrinsics,
    const std::string & ymlfile);

#endif /* __camera_calibration_h__ */
