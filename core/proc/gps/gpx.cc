/*
 * gpx.cc
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#include "gpx.h"
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TINYXML2

#include <tinyxml2.h>
#include <string.h>
#include <time.h>

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

    int Y, M, D, h, m, s;

    if( sscanf(token.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d", &Y, &M, &D, &h, &m, &s) == 6 ) {

      struct tm tm = {
          .tm_sec = s, /* Seconds. [0-60] (1 leap second) */
          .tm_min = m, /* Minutes. [0-59] */
          .tm_hour = h, /* Hours. [0-23] */
          .tm_mday = D, /* Day.   [1-31] */
          .tm_mon = M - 1, /* Month. [0-11] */
          .tm_year = Y - 1900, /* Year - 1900.  */
          .tm_wday = 0, /* Day of week. [0-6] */
          .tm_yday = 0, /* Days in year.[0-365] */
          .tm_isdst = 0, /* DST.   [-1/0/1]*/
      };

      *value = mktime(&tm);
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

  for( ; trkpt_element; trkpt_element = trkpt_element->NextSiblingElement() ) {

    gpx_track->pts.emplace_back();

    c_gpx_point & p =
        gpx_track->pts.back();

    getAttribute(trkpt_element, "lat", &p.lat);
    getAttribute(trkpt_element, "lon", &p.lon);
    getValue(trkpt_element, "ele", &p.elev);
    getGPXTime(trkpt_element, "time", &p.ts);
  }



  return true;
#else  // HAVE_TINYXML2
  CF_ERROR("This app is built with no tinyxml2 support. Can not parse gpx xml file");
  return false;
#endif
}
