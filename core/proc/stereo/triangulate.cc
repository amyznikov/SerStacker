/*
 * triangulate.cc
 *
 *  Created on: Nov 1, 2021
 *      Author: amyznikov
 */


#include "triangulate.h"
//#include <core/ssprintf.h>
#include <core/debug.h>

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
 *
 * http://cyclowiki.org/wiki/Расстояние_между_прямыми_в_трёхмерном_пространстве
 */
bool triangulate_point(double xpix0, double ypix0,
    double xpix1, double ypix1,
    const cv::Matx33d & inverted_camera_matrix,
    const cv::Vec3d & epipole_direction_in_camera_coordinates,
    double Baseline,
    /*out, opt*/ double * output_depth,
    /*out, opt*/ cv::Vec3d * output_3dpos)
{
  // Vector to the point direction in camera coordinates
  const cv::Vec3d u0 = pix2cam(xpix0, ypix0, inverted_camera_matrix);
  const cv::Vec3d u1 = pix2cam(xpix1, ypix1, inverted_camera_matrix);
  const double normu0 = cv::norm(u0);
  const double normu1 = cv::norm(u1);

  const double alpha0 = acos(epipole_direction_in_camera_coordinates.dot(u0) / normu0);
  const double alpha1 = M_PI - acos(epipole_direction_in_camera_coordinates.dot(u1) / normu1);

  double sin_gamma = sin(M_PI - alpha1 - alpha0);
  if ( sin_gamma > 2e-3 ) {

      const double distance0 =
          Baseline * sin(alpha1) / sin_gamma;

      const double depth =
          u0.dot(cv::Vec3d(0, 0, 1)) * distance0 / normu0;

    if ( depth > Baseline  ) {

      if ( output_depth  ) {
        * output_depth = depth ;
      }

      if ( output_3dpos ) {
        * output_3dpos = u0 * depth;
      }

      return  true;
    }
  }

  return false;
}


/* triangulate_disparity()
 *
 */
bool triangulate_stereo_disparity(double xpix, double ypix, double disp,
    const cv::Matx33d & inverted_camera_matrix,
    double Baseline,
    /*out, opt*/ double * output_depth,
    /*out, opt*/ cv::Vec3d * output_3dpos)
{
  static const cv::Vec3d epipole_direction(-1, 0, 0);

  return triangulate_point(xpix, ypix,
      xpix + disp, ypix,
      inverted_camera_matrix,
      epipole_direction,
      Baseline,
      output_depth,
      output_3dpos);
}

/* triangulate_dense_matches()
 *
 * computes depthmap from given matches (correspondences) map.
 * the correspondences map may be computed using @ref scale_sweeping_stereo_matcher()
 *
 */
bool triangulate_dense_matches(cv::InputArray cmap,
    const cv::Matx33d & camera_matrix,
    const cv::Point2d & epipole_location,
    double Baseline,
    /* out, opt */cv::OutputArray output_depthmap,
    /* out, opt */cv::OutputArray output_cloud3d)
{
  //
  // Check input arguments
  //

  if ( !output_depthmap.needed() && !output_cloud3d.needed() ) {
    CF_ERROR("None of output_depthmap nor output_cloud3d is specified");
    return false;
  }

  if ( output_depthmap.needed() && output_depthmap.fixedType() && output_depthmap.type() != CV_32FC1 ) {
    CF_ERROR("Fixed-type output depthmap image must have CV_32FC1 type");
    return false;
  }

  if ( output_cloud3d.needed() && output_cloud3d.fixedType() && output_cloud3d.type() != CV_32FC3 ) {
    CF_ERROR("Fixed-type output cloud3d image must have CV_32FC1 type");
    return false;
  }

  // Get query->train matches map
  const cv::Mat2f matches_map =
      cmap.getMat();



  // Pre-compute inversion of the camera matrix
  const cv::Matx33d inverted_camera_matrix =
      camera_matrix.inv();


  // Unit vector to epipole direction in camera coordinates
  const cv::Vec3d e =
      pix2cam(epipole_location.x, epipole_location.y,
          inverted_camera_matrix,
          true);


  cv::Mat1f depths_image;
  cv::Mat3f cloud_image;

  if ( output_depthmap.needed() ) {
    output_depthmap.create(matches_map.size(), CV_32FC1);
    depths_image = output_depthmap.getMatRef();
    depths_image.setTo(0);
  }

  if ( output_cloud3d.needed() ) {
    output_cloud3d.create(matches_map.size(), CV_32FC3);
    cloud_image = output_cloud3d.getMatRef();
    cloud_image.setTo(0);
  }

  for ( int y = 0; y < matches_map.rows; ++y ) {
    for ( int x = 0; x < matches_map.cols; ++x ) {

      if ( matches_map[y][x][0] < 0 || matches_map[y][x][1] < 0  ) {
        continue;
      }

      const double x0 = x;
      const double y0 = y;
      const double x1 = matches_map[y][x][0];
      const double y1 = matches_map[y][x][1];

//      const double x0 = matches_map[y][x][0];
//      const double y0 = matches_map[y][x][1];
//      const double x1 = x;
//      const double y1 = y;

      double depth;
      cv::Vec3d ppos;


      if ( !triangulate_point(x0, y0, x1, y1, inverted_camera_matrix, e, Baseline, &depth, &ppos) ) {
        continue;
      }

      if ( !depths_image.empty() ) {
        depths_image[y][x] = depth;
      }

      if ( !cloud_image.empty() ) {
        cloud_image[y][x] = ppos;
      }
    }
  }

  return true;
}


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
    /* out, opt */ cv::OutputArray output_cloud3d)
{

  //
  // Check input arguments
  //
  if ( output_depthmap.needed() ) {
    if ( output_depthmap.fixedType() &&  output_depthmap.type() != CV_32FC1 ) {
      CF_ERROR("Invalid arg: output_depthmap type must be CV_32FC1");
      return false;
    }
  }

  if ( output_cloud3d.needed() ) {
    if ( output_cloud3d.fixedType() &&  output_cloud3d.type() != CV_32FC3 ) {
      CF_ERROR("Invalid arg: output_cloud3d type must be CV_32FC3");
      return false;
    }
  }

  cv::Mat1f disps;
  cv::Mat1f depths;
  cv::Mat3f cloud3d;


  // These constands are copied from /opencv/modules/calib3d/src/stereobm.cpp
  static constexpr int DISPARITY_SHIFT_16S = 4;
  static constexpr int DISPARITY_SHIFT_32S = 8;

  switch ( disparity_map.type() ) {
  case CV_16SC1 :
    disparity_map.getMat().convertTo(disps, CV_32F, 1. / (1 << DISPARITY_SHIFT_16S), 0);
    break;
  case CV_32SC1 :
    disparity_map.getMat().convertTo(disps, CV_32F, 1. / (1 << DISPARITY_SHIFT_32S), 0);
    break;
  case CV_32FC1 :
    //disps = disparity_map.getMat();
    disparity_map.getMat().copyTo(disps);
    break;
  default :
    CF_ERROR("Invalid arg: the disparity map type=%d not supported. Must be one of CV_32FC1, CV_16SC1, CV_32SC1");
    return false;
  }



  if ( output_depthmap.needed() ) {
    output_depthmap.create(disps.size(), CV_32FC1);
    output_depthmap.setTo(0);
    depths = output_depthmap.getMatRef();
  }

  if ( output_cloud3d.needed() ) {
    output_cloud3d.create(disps.size(), CV_32FC3);
    output_cloud3d.setTo(0);
    cloud3d = output_cloud3d.getMatRef();
  }

  // Pre-compute inversion of the camera matrix
  const cv::Matx33d inverted_camera_matrix =
      camera_matrix.inv();

  for ( int y = 0; y < disps.rows; ++y ) {
    for ( int x = 0; x < disps.cols; ++x ) {

      if ( disps[y][x] < min_disparity ) {
        continue;
      }

      double depth;
      cv::Vec3d ppos;

      if ( !triangulate_stereo_disparity(x, y, disps[y][x], inverted_camera_matrix, baseline, &depth, &ppos) ) {
        continue;
      }

      if ( !depths.empty() ) {
        depths[y][x] = depth;
      }

      if ( !cloud3d.empty() ) {
        cloud3d[y][x] = ppos;
      }
    }
  }


  return true;
}
