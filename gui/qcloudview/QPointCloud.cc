/*
 * QPointCloud.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QPointCloud.h"
#include <core/debug.h>

bool loadPlyFile(const QString & filename, QPointCloud * cloud)
{
  static const char * header_lines[] = {
      "ply\n",
      "format ascii 1.0\n",
      "element vertex",
      "property double x\n",
      "property double y\n",
      "property double z\n",
      "property uchar red\n",
      "property uchar green\n",
      "property uchar blue\n",
      "end_header\n"
  };


  const std::string stdfilename =
      filename.toStdString();

  const char * cfilename =
      stdfilename.c_str();

  std::vector<cv::Vec3f> points;
  std::vector<cv::Vec3b> colors;

  FILE * fp = nullptr;

  double xmin = 1e38, xmax = -1e38, ymin = 1e38, ymax = -1e38, zmin = 1e38, zmax = -1e38;

  char line[1024] = "";

  bool fOk = false;



  cloud->clear();

  if ( !(fp = fopen(cfilename, "r")) ) {
    CF_FATAL("fopen('%s') fails: %s", cfilename, strerror(errno));
    goto __end;
  }

  // read header
  for ( uint i = 0; i < sizeof(header_lines)/sizeof(header_lines[0]); ++i ) {
    if ( !fgets(line, sizeof(line), fp) ) {
      CF_FATAL("UNexpected EOF while reading %s: %s", cfilename, strerror(errno));
      goto __end;
    }

    if ( strncasecmp(line, header_lines[i], strlen(header_lines[i])) != 0 ) {
      CF_FATAL("Not supported format of %s: unparsed line '%s'", cfilename, line);
      goto __end;
    }
  }


  cloud->filename = filename;


  while ( fgets(line, sizeof(line), fp) ) {

    double x, y, z, r, g, b;

    if( sscanf(line, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &g, &b) == 6 ) {

      points.emplace_back(x, y, z);
      colors.emplace_back(r, g, b);

      if( x < xmin ) {
        xmin = x;
      }

      if( x > xmax ) {
        xmax = x;
      }

      if( y < ymin ) {
        ymin = y;
      }

      if( y > ymax ) {
        ymax = y;
      }

      if( z < zmin ) {
        zmin = z;
      }

      if( z > zmax ) {
        zmax = z;
      }
    }
  }

  cv::Mat(points).copyTo(cloud->points);
  cv::Mat(colors).copyTo(cloud->colors);

  fOk = true;

//  CF_DEBUG("%s: %zu points: xmin=%g xmax=%g ymin=%g ymax=%g zmin=%g zmax=%g",
//      cfilename,
//      cloud->points.size(),
//      xmin, xmax, ymin, ymax, zmin, zmax);

__end :

  if ( fp ) {
    fclose(fp);
  }

  return fOk;
}
