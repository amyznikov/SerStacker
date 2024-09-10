/*
 * libzip_compress.h
 *
 *  Created on: Sep 8, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __libzip_compress_h__
#define __libzip_compress_h__

#include <stdint.h>
#include <vector>

bool libzip_compress(const void * data, uint32_t size,
    std::vector<uint8_t> * output);



#endif /* __libzip_compress_h__ */
