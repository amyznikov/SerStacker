/*
 * parse_timestamp.h
 *
 *  Created on: Aug 18, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __parse_timestamp_h__
#define __parse_timestamp_h__

#include <string>

bool parse_timestamp_from_filename(const std::string & pathfilename,
    double * ooutput_timestamp_sec);


#endif /* __parse_timestamp_h__ */
