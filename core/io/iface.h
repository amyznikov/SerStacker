/*
 * iface.h
 *
 *  Created on: Sep 22, 2016
 *      Author: amyznikov
 *
 *  Update: Jan 30, 2022
 *     Adapt to glddm
 */

#pragma once
#ifndef __cuttle_iface_h__
#define __cuttle_iface_h__

#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <netdb.h>
#include <net/if.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef
struct ifaceinfo {
  char ifname[256];
  uint32_t ifaddress;
  uint32_t ifa_flags; /*!< IFF_* from net/if.h */
} ifaceinfo;

ssize_t cf_enumerate_ifaces(struct ifaceinfo ifaces[], size_t maxifaces);

char * cf_format_iface_flags(uint32_t ifa_flags, char string[256]);

bool cf_get_iface_address(const char * string, uint32_t * address, uint16_t * port);


#define cf_iflags_str(flags) \
  cf_format_iface_flags(flags, (char[256]){0})

#ifdef __cplusplus
}
#endif




#ifdef __cplusplus

#include <string>
#include <string.h>
#include <arpa/inet.h>

/* fixme: only IPv4 is implemented */
inline std::string cf_get_iface_address(const std::string & ifacename )
{
  char ipaddrs[INET_ADDRSTRLEN + 8] = "";
  uint32_t address = (uint32_t)(-1);
  uint16_t port = (uint16_t )(-1);

  if ( cf_get_iface_address(ifacename.c_str(), &address, &port) ) {
    struct in_addr in = {htonl(address)};
    inet_ntop(AF_INET, &in, ipaddrs, INET_ADDRSTRLEN);
    if ( port != (uint16_t)(-1)) {
      sprintf(ipaddrs + strlen(ipaddrs), ":%u", port);
    }
  }

  return ipaddrs;
}


#endif

#endif /* __cuttle_iface_h__ */
