/*
 * compress.cc
 *
 *  Created on: Apr 11, 2022
 *      Author: amyznikov
 *
 *  Based on https://github.com/mapbox/gzip-hpp
 */
#include "compress.h"
#include <core/debug.h>


bool compress(const void * data, uint size, std::vector<uint8_t> * output)
{
  z_stream deflate_s = {0};
  std::size_t size_compressed = 0;

  // The windowBits parameter is the base two logarithm of the window size (the size of the history buffer).
  // It should be in the range 8..15 for this version of the library.
  // Larger values of this parameter result in better compression at the expense of memory usage.
  // This range of values also changes the decoding type:
  //  -8 to -15 for raw deflate
  //  8 to 15 for zlib
  // (8 to 15) + 16 for gzip
  // (8 to 15) + 32 to automatically detect gzip/zlib header (decompression/inflate only)
  constexpr int window_bits = 15 + 16; // gzip with windowbits of 15

  // The memory requirements for deflate are (in bytes):
  // (1 << (window_bits+2)) +  (1 << (mem_level+9))
  // with a default value of 8 for mem_level and our window_bits of 15
  // this is 128Kb
  constexpr int mem_level = 8;

  if( deflateInit2(&deflate_s, Z_DEFAULT_COMPRESSION, Z_DEFLATED, window_bits, mem_level, Z_DEFAULT_STRATEGY) != Z_OK ) {
    CF_ERROR("deflateInit2() fails");
    return false;
  }

  deflate_s.next_in = (z_const Bytef *)data;
  deflate_s.avail_in = size;

  output->clear();

  do {

    size_t increase = size / 2 + 1024;
    if( output->size() < (size_compressed + increase) ) {
      output->resize(size_compressed + increase);
    }
    // There is no way we see that "increase" would not fit in an unsigned int,
    // hence we use static cast here to avoid -Wshorten-64-to-32 error
    deflate_s.avail_out = (uint)increase;
    deflate_s.next_out = (Bytef*)((output->data() + size_compressed));
    deflate(&deflate_s, Z_FINISH);

    size_compressed += (increase - deflate_s.avail_out);
  } while (deflate_s.avail_out == 0);

  deflateEnd(&deflate_s);

  output->resize(size_compressed);

  return true;
}

