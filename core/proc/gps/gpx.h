/*
 * gpx.h
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __gpx_h__
#define __gpx_h__

#include <core/proc/gps/gps.h>
#include <vector>
#include <string>

struct c_gpx_track
{
  std::string name;
  std::string author;
  std::string src;
  std::string created;
  double length = 0;
  double duration = 0;

  std::vector<c_gps_position> pts;
};


bool load_gpx_track_xml(const std::string & gpx_xml_file_name,
    c_gpx_track * gpx_track);

#endif /* __gpx_h__ */
