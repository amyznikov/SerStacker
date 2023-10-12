/*
 * triangulate.h
 *
 *  Created on: Nov 1, 2021
 *      Author: amyznikov
 *
 *  TODO: Read this <https://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code>
 */

#pragma once
#ifndef __triangulate_h__
#define __triangulate_h__

#include <opencv2/opencv.hpp>


/* pix2cam()
 *
 * Compute 3D camera coordinates of a given image pixel (xpix, ypix)
 * using given inverted camera matrix
 *  */
inline cv::Vec3d pix2cam(double xpix, double ypix, const cv::Matx33d & inverted_camera_matrix,
    bool normalize_to_unit_length = false)
{
  const cv::Vec3d cpos = inverted_camera_matrix * cv::Vec3d(xpix, ypix, 1);
  return normalize_to_unit_length ?  cpos /cv::norm(cpos) :  cpos / cpos(2);
}


/* triangulate_point()
 *
 * This routine triangulates single 2D point imaged at
 * pixel location (xpix0, ypix0) at first image and at
 * pixel location (xpix1, ypix1) at second image.
 *
 * Inverted camera matrix, epipole location in camera coordinates and
 * baseline length must be provided on input.
 *
 * The cv::norm(epipole_direction_in_camera_coordinates) is assumed to be normalzied to 1.
 */
bool triangulate_point(double xpix0, double ypix0,
    double xpix1, double ypix1,
    const cv::Matx33d & inverted_camera_matrix,
    const cv::Vec3d & epipole_direction_in_camera_coordinates,
    double Baseline,
    /*out, opt*/ double * output_depth,
    /*out, opt*/ cv::Vec3d * output_3dpos);


/* triangulate_stereo_disparity()
 *
 * This routine triangulates single 2D point
 * imaged on stereo pair pixel location (xpix, ypix) with given disparity disp.
 *
 * Inverted camera matrix, epipole direction in camera coordinate system
 * and baseline length must be provided on input.
 *
 * The cv::norm(epipole_direction_in_camera_coordinates) is assumed
 * to be normalzied to 1.
 */
bool triangulate_stereo_disparity(double xpix, double ypix, double disp,
    const cv::Matx33d & inverted_camera_matrix,
    double Baseline,
    /*out, opt*/ double * output_depth,
    /*out, opt*/ cv::Vec3d * output_3dpos);

/* triangulate_dense_matches()
 *
 * computes depthmap from given matches (correspondences) map.
 * the correspondences map may be computed using @ref scale_sweeping_stereo_matcher()
 *
 */
bool triangulate_dense_matches(cv::InputArray cmap,
    const cv::Matx33d & camera_matrix,
    const cv::Point2d & epipole_location_in_pixel_coordinates,
    double BaseLine,
    /* out, opt */ cv::OutputArray output_depthmap,
    /* out, opt */ cv::OutputArray output_cloud3d);


/* triangulate_stereo_disparity_map()
 *
 * computes depthmap from given stereo disparity map
 * returned cv::StereoMatcher::compute().
 *
 */
bool triangulate_stereo_disparity_map(cv::InputArray disparity_map,
    const cv::Matx33d & camera_matrix,
    double baseline,
    double min_disparity,
    /* out, opt */ cv::OutputArray output_depthmap,
    /* out, opt */ cv::OutputArray output_cloud3d = cv::noArray());

#endif /* __triangulate_h__ */
