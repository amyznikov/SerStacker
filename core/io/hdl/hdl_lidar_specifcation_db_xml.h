/*
 * hdl_lidar_specifcation_db_xml.h
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
 *
 */

#ifndef __hdl_lidar_specifcation_db_xml_h__
#define __hdl_lidar_specifcation_db_xml_h__

#include <string>

#include "c_hdl_specification.h"

/**
 * load_hdl_lidar_specifcation_db_xml()
 *
 * Load Velodyne Lidar specification from db.xml file.
 * */
bool load_hdl_lidar_specifcation_db_xml(const std::string & xmlfilename,
    c_hdl_specification * spec,
    HDLSensorType sensor_type = HDLSensor_unknown);

#endif /* __hdl_lidar_specifcation_db_xml_h__ */
