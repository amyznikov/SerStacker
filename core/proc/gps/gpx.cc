/*
 * gpx.cc
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#include "gpx.h"
#include <core/io/c_stdio_file.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TINYXML2

#include <tinyxml2.h>
#include <string.h>
#include <time.h>
#include <core/ssprintf.h>
#include <algorithm>

using namespace tinyxml2;


/**
 * findElement()
 * Find child xml element using recursive path starting from given root node
 * */

static const XMLElement* findElement(const XMLElement * root, const std::string & name)
{
  std::vector<std::string> tokens =
      strsplit(name, ".");

  const XMLElement * child = nullptr;

  for( size_t i = 0, n = tokens.size(); i < n; ++i, root = child ) {
    if( !(child = root->FirstChildElement(tokens[i].c_str())) ) {
      break;
    }
  }

  return child;
}

/**
 * getValue()
 *
 * Parse xml node text
 * */
template<class T>
static bool getValue(const XMLElement * node, T * value)
{
  return node && node->GetText() && fromString(node->GetText(), value);
}

template<class T>
static bool getAttribute(const XMLElement * node, const char * name, T * value)
{
  if ( node ) {
    const char * attribute =
        node->Attribute(name);
    if ( attribute ) {
      return fromString(attribute, value);
    }
  }
  return false;
}


/**
 * getValue()
 *
 * Parse xml node text
 * */
template<class T>
static bool getValue(const XMLElement * root, const std::string & path, T * value)
{
  return getValue(findElement(root, path), value);
}

static bool getGPXTime(const XMLElement * root, const char * path, double * value)
{
  //2024-08-21T04:04:53Z

  std::string token;

  if( !getValue(root, path, &token) ) {
    CF_DEBUG("getValue fails");
  }
  else {

    int Y, M, D, h, m;
    double s;

    if( sscanf(token.c_str(), "%4d-%2d-%2dT%2d:%2d:%lf", &Y, &M, &D, &h, &m, &s) == 6 ) {

      struct tm tm = {
          .tm_sec = (int)s, /* Seconds. [0-60] (1 leap second) */
          .tm_min = m, /* Minutes. [0-59] */
          .tm_hour = h, /* Hours. [0-23] */
          .tm_mday = D, /* Day.   [1-31] */
          .tm_mon = M - 1, /* Month. [0-11] */
          .tm_year = Y - 1900, /* Year - 1900.  */
          .tm_wday = 0, /* Day of week. [0-6] */
          .tm_yday = 0, /* Days in year.[0-365] */
          .tm_isdst = 0, /* DST.   [-1/0/1]*/
      };

      *value = mktime(&tm) + (s - (int) s);
      return true;
    }
  }

  return false;
}

#endif


bool load_gpx_track_xml(const std::string & gpx_xml_file_name, c_gpx_track * gpx_track)
{
#if HAVE_TINYXML2

  XMLDocument xml;
  XMLError status;



  if( (status = xml.LoadFile(gpx_xml_file_name.c_str())) != XML_SUCCESS ) {
#if  (TINYXML2_MAJOR_VERSION >= 6)
    CF_ERROR("xml.LoadFile('%s') fails: %s",
        gpx_xml_file_name.c_str(), xml.ErrorStr());
#else
    CF_ERROR("xml.LoadFile('%s') fails: XMLError = %d (%s)",
        gpx_xml_file_name.c_str(), (int)status, xml.ErrorName());
#endif
    return false;
  }

  const XMLElement * gpx_element =
      xml.RootElement();
  if( !gpx_element || !gpx_element->Name() || strcasecmp(gpx_element->Name(), "gpx") != 0  ) {
    CF_ERROR("xml.RootElement('gpx') fails");
    return false;
  }

  const XMLElement * trk_element =
      findElement(gpx_element, "trk");
  if( !trk_element ) {
    CF_ERROR("findElement('gpx.trk') fails");
    return false;
  }

  getValue(gpx_element, "metadata.author.name", &gpx_track->author);
  getValue(trk_element, "name", &gpx_track->name);
  getValue(trk_element, "src", &gpx_track->src);

  const XMLElement * trkseg_element =
      findElement(trk_element, "trkseg");
  if( !trkseg_element ) {
    CF_ERROR("findElement('gpx.trk.trkseg') fails");
    return false;
  }

  const int sequence_size  =
      trkseg_element->ChildElementCount("trkpt");

  const XMLElement * trkpt_element =
      trkseg_element->FirstChildElement( "trkpt");

  c_gps_position p;

  for( ; trkpt_element; trkpt_element = trkpt_element->NextSiblingElement() ) {

    if ( !getAttribute(trkpt_element, "lat", &p.latitude) ) {
      CF_DEBUG("getAttribute(lat) fails at line %d", trkpt_element->GetLineNum());
      continue;
    }

    if ( !getAttribute(trkpt_element, "lon", &p.longitude) ) {
      CF_DEBUG("getAttribute(lon) fails at line %d", trkpt_element->GetLineNum());
      continue;
    }

    if ( !getValue(trkpt_element, "ele", &p.altitude) ) {
      CF_DEBUG("getValue(ele) fails at line %d", trkpt_element->GetLineNum());
      continue;
    }

    if ( !getGPXTime(trkpt_element, "time", &p.timestamp) ) {
      CF_DEBUG("getGPXTime fails at  at line %d", trkpt_element->GetLineNum());
      continue;
    }

    p.latitude *= CV_PI / 180;
    p.longitude *= CV_PI / 180;

    gpx_track->pts.emplace_back(p);
  }

  std::sort(gpx_track->pts.begin(), gpx_track->pts.end(),
      [](const c_gps_position & prev, const c_gps_position & next) {
        return prev.timestamp < next.timestamp;
      });


  return true;
#else  // HAVE_TINYXML2
  CF_ERROR("This app is built with no tinyxml2 support. Can not parse gpx xml file");
  return false;
#endif
}

bool load_gpx_track_csv(const std::string & csv_file_name, c_gpx_track * gpx_track)
{
  // Location.csv
  // time,seconds_elapsed,bearingAccuracy,speedAccuracy,verticalAccuracy,horizontalAccuracy,speed,bearing,altitude,longitude,latitude
  // time,seconds_elapsed,bearingAccuracy,speedAccuracy,verticalAccuracy,horizontalAccuracy,speed,bearing,altitude,longitude,latitude

  c_stdio_file fp;

  if( !fp.open(csv_file_name, "r") ) {
    CF_ERROR("Can not read '%s': %s ",
        csv_file_name.c_str(),
        strerror(errno));
    return false;
  }

  std::vector<char> line(2048, (char)(0));
  std::vector<std::string> tokens;
  c_gps_position gps;

  int time_column_index = -1;
  int seconds_elapsed_column_index = -1;
  int altitude_column_index = -1;
  int longitude_column_index = -1;
  int latitude_column_index = -1;


  if ( !fgets(line.data(), line.size(), fp) ) {
    CF_ERROR("Can not read header line from '%s': %s ",
        csv_file_name.c_str(),
        strerror(errno));
    return false;
  }


  CF_DEBUG("line='%s'", line.data());

  tokens = strsplit(line.data(), ", \t\n\r");

  CF_DEBUG("tokens.size=%zu", tokens.size());

  for( size_t i = 0, n = tokens.size(); i < n; ++i ) {

    CF_DEBUG("token[%zu]='%s'", i, tokens[i].c_str());

    if( strcasecmp(tokens[i].c_str(), "time") == 0 ) {
      time_column_index = (int) (i);
    }
    else if( strcasecmp(tokens[i].c_str(), "seconds_elapsed") == 0 ) {
      seconds_elapsed_column_index = (int) (i);
    }
    else if( strcasecmp(tokens[i].c_str(), "altitude") == 0 ) {
      altitude_column_index = (int) (i);
    }
    else if( strcasecmp(tokens[i].c_str(), "longitude") == 0 ) {
      longitude_column_index = (int) (i);
    }
    else if( strcasecmp(tokens[i].c_str(), "latitude") == 0 ) {
      latitude_column_index = (int) (i);
    }
  }

  if( longitude_column_index < 0 ) {
    CF_ERROR("Can not determine longitude column index from header line");
    errno = EINVAL;
    return false;
  }

  if( latitude_column_index < 0 ) {
    CF_ERROR("Can not determine latitude column index from header line");
    errno = EINVAL;
    return false;
  }


  for( int current_line_index = 2; fgets(line.data(), line.size(), fp); ++current_line_index ) {
    if( line[0] == '#' || line[0] == '/' ) {
      continue;
    }

    tokens = strsplit(line.data(), ", \t");
    const int num_tokens = (int)(tokens.size());


    if( seconds_elapsed_column_index >= 0 && seconds_elapsed_column_index < num_tokens) {
      if( sscanf(tokens[seconds_elapsed_column_index].c_str(), "%lf", &gps.timestamp) != 1 ) {
        CF_ERROR("Can parse 'seconds_elapsed' value at line %d: %s", current_line_index,
            tokens[seconds_elapsed_column_index].c_str());
        continue;
      }
    }
    else if( time_column_index >= 0 && time_column_index < num_tokens ) {
      if( sscanf(tokens[time_column_index].c_str(), "%lf", &gps.timestamp) != 1 ) {
        CF_ERROR("Can parse 'time' value at line %d: %s", current_line_index,
            tokens[time_column_index].c_str());
        continue;
      }
    }

    if( altitude_column_index >= 0 && altitude_column_index < num_tokens) {
      if( sscanf(tokens[altitude_column_index].c_str(), "%lf", &gps.altitude) != 1 ) {
        CF_ERROR("Can parse 'altitude' value at line %d: %s", current_line_index,
            tokens[altitude_column_index].c_str());
        continue;
      }
    }

    if( longitude_column_index >= 0 && longitude_column_index < num_tokens) {
      if( sscanf(tokens[longitude_column_index].c_str(), "%lf", &gps.longitude) != 1 ) {
        CF_ERROR("Can parse 'longitude' value at line %d: %s", current_line_index,
            tokens[longitude_column_index].c_str());
        continue;
      }

      gps.longitude *= CV_PI / 180;
    }

    if( latitude_column_index >= 0 && latitude_column_index < num_tokens) {
      if( sscanf(tokens[latitude_column_index].c_str(), "%lf", &gps.latitude) != 1 ) {
        CF_ERROR("Can parse 'latitude' value at line %d: %s", current_line_index,
            tokens[latitude_column_index].c_str());
        continue;
      }

      gps.latitude *= CV_PI / 180;
    }

    gpx_track->pts.emplace_back(gps);
  }

  gpx_track->name = get_file_name(csv_file_name);
  gpx_track->author = "Unknown Author";
  gpx_track->src = csv_file_name;

  return true;
}
