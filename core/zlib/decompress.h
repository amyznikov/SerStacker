/*
 * decompress.h
 *
 *  Created on: Apr 11, 2022
 *      Author: amyznikov
 *
 *  Based on https://github.com/mapbox/gzip-hpp
 */

#pragma once
#ifndef __decompress_h__
#define __decompress_h__

#ifndef ZLIB_CONST
# define ZLIB_CONST
#endif

#include <stdint.h>
#include <zlib.h>
#include <vector>

bool decompress(const void * data, uint32_t size,
    std::vector<uint8_t> * output);

#endif /* __decompress_h__ */
