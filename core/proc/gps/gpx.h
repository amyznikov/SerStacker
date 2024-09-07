/*
 * gpx.h
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __gpx_h__
#define __gpx_h__

#include <vector>
#include <string>

struct c_gpx_point
{
  double ts;
  double lat;
  double lon;
  double elev;
};

struct c_gpx_track
{
  std::string name;
  std::string author;
  std::string src;
  std::string created;
  double length = 0;
  double duration = 0;

  std::vector<c_gpx_point> pts;
};


bool load_gpx_track_xml(const std::string & gpx_xml_file_name,
    c_gpx_track * gpx_track);

#endif /* __gpx_h__ */
