/*
 * kitti-timestamp.h
 *
 *  Created on: Sep 16, 2018
 *      Author: amyznikov
 */

#pragma once
#ifndef __kitti_timestamp_h__
#define __kitti_timestamp_h__

#include <vector>
#include <string>

///////////////////////////////////////////////////////////////////////////////

//! @addtogroup kitti
//! @{

/** @brief Read velodine time stamps from specified file
 *
 * The time stamp format is like this:
 *  2011-09-29 12:19:46.230015023
 *
 *  It is converted to julian dates with floating-point seconds
 */
bool read_kitti_velodyne_timestamps(const std::string & tsfilename,
    std::vector<double> & stamps);

//! @} kitti

///////////////////////////////////////////////////////////////////////////////
#endif /* __kitti_timestamp_h__ */
