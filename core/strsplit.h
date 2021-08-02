/*
 * strsplit.h
 *
 *  Created on: Nov 21, 2017
 *      Author: amyznikov
 */

#pragma once
#ifndef __strsplit_h__
#define __strsplit_h__

#include <string>
#include <vector>

size_t strsplit(const std::string & s,
    std::vector<std::string> & tokens,
    const std::string & _delims);


#endif /* __strsplit_h__ */
