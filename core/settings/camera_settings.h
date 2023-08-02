/*
 * camera_settings.h
 *
 *  Created on: Aug 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __camera_settings_h__
#define __camera_settings_h__

#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/settings/opencv_settings.h>

bool load_settings(c_config_setting settings,
    c_camera_intrinsics * c);

bool save_settings(c_config_setting settings,
    const c_camera_intrinsics & c);

#endif /* __camera_settings_h__ */
