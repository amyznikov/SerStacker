/*
 * bswap.h
 *
 *  Created on: Oct 14, 2023
 *      Author: amyznikov
 */

#ifndef __bswap_h__
#define __bswap_h__

#if __WIN32__ || __WIN64__ //  __MINGW32__ || __MINGW64__
// <cygwin/include/bits/byteswap.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline unsigned short __bswap_16 (unsigned short __x)
{
  return (__x >> 8) | (__x << 8);
}

static inline unsigned int __bswap_32 (unsigned int __x)
{
  return (__bswap_16 (__x & 0xffff) << 16) | (__bswap_16 (__x >> 16));
}

static inline unsigned long long __bswap_64 (unsigned long long __x)
{
  return (((unsigned long long) __bswap_32 (__x & 0xffffffffull)) << 32) | (__bswap_32 (__x >> 32));
}

#ifdef __cplusplus
}
#endif

#define bswap_16(x) __bswap_16(x)
#define bswap_32(x) __bswap_32(x)
#define bswap_64(x) __bswap_64(x)

#else
#	include <byteswap.h>
#endif

#endif /* __bswap_h__ */
