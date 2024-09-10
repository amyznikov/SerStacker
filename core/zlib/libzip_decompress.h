/*
 * libzip_decompress.h
 *
 *  Created on: Sep 8, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __libzip_decompress_h__
#define __libzip_decompress_h__


#include <stdint.h>
#include <vector>

bool libzip_decompress(const void * data, uint32_t size,
    std::vector<uint8_t> * output);



#endif /* __libzip_decompress_h__ */
