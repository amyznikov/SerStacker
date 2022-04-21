/*
 * compress.h
 *
 *  Created on: Apr 11, 2022
 *      Author: amyznikov
 *
 *  Based on https://github.com/mapbox/gzip-hpp
 */

#pragma once
#ifndef __compress_h__
#define __compress_h__

#ifndef ZLIB_CONST
# define ZLIB_CONST
#endif

#include <stdint.h>
#include <zlib.h>
#include <vector>

bool compress(const void * data, uint size,
    std::vector<uint8_t> * output);

#endif /* __compress_h__ */
