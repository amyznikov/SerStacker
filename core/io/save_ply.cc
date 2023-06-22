/*
 * save_ply.cc
 *
 *  Created on: May 28, 2021
 *      Author: amyznikov
 */

#include "save_ply.h"
#include <core/readdir.h>
#include <core/debug.h>

bool save_ply(const std::vector<cv::Vec3d> & points,
    const std::vector<cv::Vec3b> & colors,
    const std::string & fname)
{
  const std::string parent_directory = get_parent_directory(fname);

  if ( !parent_directory.empty() && !create_path(parent_directory) ) {
    CF_ERROR("Can not create output directory for '%s': %s",
        fname.c_str(), strerror(errno));
    return false;
  }


  CF_DEBUG("SAVING 3D CLOUD TO %s", fname.c_str());


  FILE * fp = fopen(fname.c_str(), "w");
  if ( !fp ) {
    CF_FATAL("Can not write '%s' : %s", fname.c_str(), strerror(errno));
    return false;
  }

  const int n = points.size();

  fprintf(fp, "ply\n"
      "format ascii 1.0\n"
      "element vertex %d\n"
      "property double x\n"
      "property double y\n"
      "property double z\n"
      "property uchar red\n"
      "property uchar green\n"
      "property uchar blue\n"
      "end_header\n",
      n
      );

  for ( int i = 0; i < n; ++i ) {

    const cv::Vec3d & v = points[i];
    const cv::Vec3b & c = colors[i];

    fprintf(fp,
        "%+.15f %+.15f %+.15f "
         "%u %u %u\n",
        v(0), v(1), v(2),
        c(2), c(1), c(0));
  }

  fclose(fp);
  return true;
}



bool save_ply(const cv::Mat3f & points, const cv::Mat3b & colors, const cv::Mat1b & mask, const std::string & fname)
{
  const std::string parent_directory = get_parent_directory(fname);

  if ( !parent_directory.empty() && !create_path(parent_directory) ) {
    CF_ERROR("Can not create output directory for '%s': %s",
        fname.c_str(), strerror(errno));
    return false;
  }

  CF_DEBUG("SAVING 3D CLOUD TO %s", fname.c_str());

  FILE * fp = fopen(fname.c_str(), "w");
  if ( !fp ) {
    CF_FATAL("Can not write '%s' : %s", fname.c_str(), strerror(errno));
    return false;
  }

  const int n = cv::countNonZero(mask);

  fprintf(fp, "ply\n"
      "format ascii 1.0\n"
      "element vertex %d\n"
      "property double x\n"
      "property double y\n"
      "property double z\n"
      "property uchar red\n"
      "property uchar green\n"
      "property uchar blue\n"
      "end_header\n",
      n
      );

  for ( int y = 0; y < mask.rows; ++y ) {
    for ( int x = 0; x < mask.cols; ++x ) {

      if ( mask[y][x] ) {

        const cv::Vec3f & pos = points[y][x];
        const cv::Vec3b & color = colors[y][x];

        fprintf(fp,
            "%+.15f %+.15f %+.15f "
            "%u %u %u\n",
            (double)pos(0), (double)pos(1), (double)pos(2),
            (uint) color(2), (uint) color(1), (uint) color(0));
      }
    }
  }

  fclose(fp);
  return true;
}


