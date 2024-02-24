/*
 * hdl_lidar_specifcation_db_xml.cc
 *
 *  Created on: Mar 22, 2022
 *      Author: amyznikov
 *
 *  Each HDL-64E S2 unit comes with its own unique .XML file, called db.XML,
 *  that was generated as a result of the calibration performed at Velodyne’s factory.
 *  DSR uses this XML file to display points accurately.
 *  The .XML file also holds the key to interpreting the packet data for users
 *  that wish to create their own software applications.
 *
 *  HDL-64E S2 USER’S MANUAL
 *  HDL-64E_S2.pdf
 */

#include "hdl_lidar_specifcation_db_xml.h"
#include <algorithm>
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TINYXML2 // Must be set

#include <tinyxml2.h>
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
  return node && fromString(node->GetText(), value);
}

/**
 * getValue()
 *
 * Parse xml node text
 * */
template<class T>
static bool getValue(const XMLElement * root, const std::string & path, T * value)
{
  const XMLElement * node = findElement(root, path);
  return node && fromString(node->GetText(), value);
}

#endif // HAVE_TINYXML2

/**
 * load_hdl_lidar_specifcation_db_xml()
 *
 * Load Velodyne Lidar specification from db.xml file.
 *
 * */
bool load_hdl_lidar_specifcation_db_xml(const std::string & xmlfilename,
    c_hdl_specification * spec,
    HDLSensorType sensor_type /*= HDLSensor_unknown*/)
{
#if HAVE_TINYXML2

  XMLDocument xml;
  XMLError status;

  spec->sensor = sensor_type;
  spec->lasers.clear();

  /*
   * Open the xml file and search for "boost_serialization.DB" node
   * */

  status = xml.LoadFile(xmlfilename.c_str());
  if( status != XML_SUCCESS ) {
#if  (TINYXML2_MAJOR_VERSION >= 6)
    CF_ERROR("xml.LoadFile('%s') fails: %s",
        xmlfilename.c_str(), xml.ErrorStr());
#else
    CF_ERROR("xml.LoadFile('%s') fails: XMLError = %d (%s)",
        xmlfilename.c_str(), (int)status, xml.ErrorName());
#endif
    return false;
  }

  const XMLElement * boost_serialization = xml.RootElement();
  if( !boost_serialization || strcasecmp(boost_serialization->Value(), "boost_serialization") != 0 ) {
    CF_ERROR("xml.RootElement(boost_serialization) fails");
    return false;
  }

  const XMLElement * db = findElement(boost_serialization, "DB");
  if( !db ) {
    CF_ERROR("findElement(\"boost_serialization.DB\") fails");
    return false;
  }

  /*
   * Read distance resolution
   * */

  const XMLElement * distLSB_ = findElement(db, "distLSB_");
  if( !distLSB_ ) {
    spec->distance_resolution = 0.002; // [m]
    CF_WARNING("findElement(\"boost_serialization.DB.distLSB_\") fails. "
        "Using default distance resolution %g m",
        spec->distance_resolution);
  }
  else if( !getValue(distLSB_, &spec->distance_resolution) || spec->distance_resolution <= 0 ) {
    CF_ERROR("getValue(\"boost_serialization.DB.distLSB_\") fails");
    return false;
  }
  else {
    spec->distance_resolution *= 1e-2; // [cm] -> [m]
  }

  /*
   * search lasers table
   * */

  const XMLElement * points_ = findElement(db, "points_");
  if( !points_ ) {
    CF_ERROR("findElement(\"boost_serialization.DB.points_\") fails");
    return false;
  }

  /*
   * read the number of lasers
   * */

  int num_lasers = -1;
  if( !getValue(points_, "count", &num_lasers) || num_lasers < 1 ) {
    CF_ERROR("getValue(\"boost_serialization.DB.points_.count\") fails: count=%d",
        num_lasers);
    return false;
  }

  /*
   * read lasers table
   */

  const XMLElement * item = findElement(points_, "item");
  if( !item ) {
    CF_ERROR("findElement(\"DB.points_.item\") fails");
    return false;
  }

  spec->lasers.reserve(num_lasers);

  c_hdl_lasers_table table { 0 };

  for( int i = 0; i < num_lasers; ++i ) {

    const XMLElement * px = item->FirstChildElement("px");
    if( !px ) {
      CF_ERROR("[%d] FirstChildElement(\"DB.points_.item.px\") fails", i);
      return false;
    }

    /*
     * Mandatory data
     * */

    if( !getValue(px, "id_", &table.laser_id) ) {
      CF_DEBUG("getValue(id_) fails");
      return false;
    }

    if( !getValue(px, "rotCorrection_", &table.rot_correction) ) {
      CF_DEBUG("getValue(rotCorrection_) fails");
      return false;
    }

    if( !getValue(px, "vertCorrection_", &table.vert_correction) ) {
      CF_DEBUG("getValue(vertCorrection_) fails");
      return false;
    }

    /*
     * Optional data
     * */
    getValue(px, "distCorrection_", &table.distance_correction);
    getValue(px, "distCorrection_", &table.distance_correction);
    getValue(px, "distCorrectionX_", &table.dist_correction_x);
    getValue(px, "distCorrectionY_", &table.dist_correction_y);
    getValue(px, "vertOffsetCorrection_", &table.vert_offset);
    getValue(px, "horizOffsetCorrection_", &table.horz_offset);
    getValue(px, "focalDistance_", &table.focal_distance);
    getValue(px, "focalSlope_", &table.focal_slope);
    getValue(px, "closeSlope_", &table.close_slope);

    table.vert_offset *= 1e-2; // [cm]->[m]
    table.horz_offset *= 1e-2; // [cm]->[m]
    table.distance_correction *= 1e-2; // [cm] -> [m]
    table.dist_correction_x *= 1e-2; // [cm] -> [m]
    table.dist_correction_y *= 1e-2; // [cm] -> [m]
    table.focal_distance *= 1e-2; // [cm] -> [m]

    spec->lasers.emplace_back(table);

    item = item->NextSiblingElement();
  }

  /*
   * Veloview xml files contain extra items, populated with zeros,
   * we need detect and drop them
   */
  std::sort(spec->lasers.begin(), spec->lasers.end(),
      [](const c_hdl_lasers_table & prev, const c_hdl_lasers_table & next) {
        return prev.laser_id < next.laser_id;
      });

  static const auto all_zeros_after =
      [](const c_hdl_specification * spec, size_t start_index) -> bool {
        for ( size_t i = start_index, n = spec->lasers.size(); i < n; ++i ) {
          if ( spec->lasers[i].vert_correction != 0 ) {
            return false;
          }
        }
        return true;
      };

  const size_t n = spec->lasers.size();
  int dropfrom = -1;

  if( sensor_type != HDLSensor_unknown ) {
    switch (sensor_type) {
      case HDLSensor_VLP16:
        dropfrom = 16;
        break;
      case HDLSensor_VLP16HiRes:
        dropfrom = 16;
        break;
      case HDLSensor_VLP32AB:
        dropfrom = 32;
        break;
      case HDLSensor_VLP32C:
        dropfrom = 32;
        break;
      case HDLSensor_HDL32E:
        dropfrom = 32;
        break;
      case HDLSensor_HDL64:
        dropfrom = 64;
        break;
      case HDLSensor_VLS128:
        dropfrom = 128;
        break;
      default:
        break;
    }

    if( dropfrom > 0 && spec->lasers.size() < dropfrom ) {
      CF_ERROR("Invalid Sensor type or XML file: expected %d lasers but XML file has only %zu laser items",
          dropfrom, spec->lasers.size());
      return false;
    }
  }

  if( dropfrom < 0 ) {
    if( n > 16 ) {
      if( all_zeros_after(spec, 16) ) {
        dropfrom = 16;
      }
      else if( n > 32 ) {
        if( all_zeros_after(spec, 32) ) {
          dropfrom = 32;
        }
        else if( n > 64 ) {
          if( all_zeros_after(spec, 64) ) {
            dropfrom = 64;
          }
        }
      }
    }
  }

  if( dropfrom > 0 ) {
    spec->lasers.erase(spec->lasers.begin() + dropfrom,
        spec->lasers.end());
  }

  /*
   * Now sort by elevation and generate range image row numbers
   */

  std::sort(spec->lasers.begin(), spec->lasers.end(),
      [](const c_hdl_lasers_table & prev, const c_hdl_lasers_table & next) {
        return prev.vert_correction < next.vert_correction;
      });

  for( int i = 0; i < num_lasers; ++i ) {
    spec->lasers[i].laser_ring = i;
  }

  /*
   * Final sort back by laserid for correct packets parsing
   */
  std::sort(spec->lasers.begin(), spec->lasers.end(),
      [](const c_hdl_lasers_table & prev, const c_hdl_lasers_table & next) {
        return prev.laser_id < next.laser_id;
      });

  return true;

#else
  CF_ERROR("Application was built with no tinyxml2 support.XML reading is not available");
  return false;
#endif // HAVE_TINYXML2
}
