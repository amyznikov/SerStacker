/*
 * v4l2_list_devices.h
 *
 *  Created on: Dec 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __v4l2_list_devices_h__
#define __v4l2_list_devices_h__

#include <vector>
#include <string>

bool v4l2_list_devices(std::vector<std::string> * filenames);

#endif /* __v4l2_list_devices_h__ */
