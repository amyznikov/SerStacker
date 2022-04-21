/*
 * decompress.cc
 *
 *  Created on: Apr 11, 2022
 *      Author: amyznikov
 *
 *  Based on https://github.com/mapbox/gzip-hpp
 */
#include "decompress.h"
#include <core/debug.h>

bool decompress(const void * data, uint size, std::vector<uint8_t> * output)
{
  z_stream inflate_s = { 0 };
  size_t size_decompressed = 0;

  constexpr uint max_bytes = 2000000000;

  // The windowBits parameter is the base two logarithm of the window size (the size of the history buffer).
  // It should be in the range 8..15 for this version of the library.
  // Larger values of this parameter result in better compression at the expense of memory usage.
  // This range of values also changes the decoding type:
  //  -8 to -15 for raw deflate
  //  8 to 15 for zlib
  // (8 to 15) + 16 for gzip
  // (8 to 15) + 32 to automatically detect gzip/zlib header
  constexpr int window_bits = 15 + 32; // auto with windowbits of 15

  if( inflateInit2(&inflate_s, window_bits) != Z_OK ) {
    CF_ERROR("inflateInit2() fails");
    return false;
  }

  inflate_s.next_in = (z_const Bytef*)(data);
  inflate_s.avail_in = static_cast<uint>(size);


  do {
    size_t resize_to = size_decompressed + 2 * size;
    if( resize_to > max_bytes ) {
      inflateEnd(&inflate_s);
      CF_ERROR("size of output string will use more memory then intended when decompressing");
      return false;
    }

    output->resize(resize_to);
    inflate_s.avail_out = static_cast<uint>(2 * size);
    inflate_s.next_out = reinterpret_cast<Bytef*>(&output[0] + size_decompressed);

    int ret = inflate(&inflate_s, Z_FINISH);
    if( ret != Z_STREAM_END && ret != Z_OK && ret != Z_BUF_ERROR ) {
      CF_ERROR("inflate() fails : %s", inflate_s.msg);
      inflateEnd(&inflate_s);
      return false;
    }

    size_decompressed += (2 * size - inflate_s.avail_out);
  } while (inflate_s.avail_out == 0);

  inflateEnd(&inflate_s);

  output->resize(size_decompressed);

  return true;
}

