/*
 * zlib_compress.h
 *
 *  Created on: Apr 11, 2022
 *      Author: amyznikov
 *
 *  Based on https://github.com/mapbox/gzip-hpp
 */

#pragma once
#ifndef __zlib_compress_h__
#define __zlib_compress_h__

#include <stdint.h>
#include <vector>

bool zlib_compress(const void * data, uint32_t size,
    std::vector<uint8_t> * output);

#endif /* __compress_h__ */
