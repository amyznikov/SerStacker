/*
 * iface.c
 *
 *  Created on: Sep 22, 2016
 *      Author: amyznikov
 *
 *  Update: Jan 30, 2022
 *     Adapt to glddm
 *
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <ifaddrs.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "iface.h"

#define INET_ADDR(a,b,c,d) \
  (uint32_t)((((uint32_t)(a))<<24)|(((uint32_t)(b))<<16)|(((uint32_t)(c))<<8)|(d))


static char * strappend(char * string, const char * substring)
{
  while ( *substring ) {
    *string++ = *substring++;
  }
  return string;
}


/**
 * enumerate_ifaces()
 * @see man getifaddrs
 */
ssize_t cf_enumerate_ifaces(struct ifaceinfo ifaces[], size_t maxifaces)
{
  struct ifaddrs *ifaddr, *ifa;
  ssize_t n = 0;

  if ( getifaddrs(&ifaddr) == -1 ) {
    return -1;
  }

  /* Walk through linked list, maintaining head pointer so we can free list later */
  for ( ifa = ifaddr; n < (ssize_t) maxifaces && ifa != 0; ifa = ifa->ifa_next ) {
    if ( ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET ) {
      strncpy(ifaces[n].ifname, ifa->ifa_name, sizeof(ifaces[n].ifname) - 1)[sizeof(ifaces[n].ifname) - 1] = 0;
      ifaces[n].ifa_flags = ifa->ifa_flags;
      ifaces[n].ifaddress = ntohl(((struct sockaddr_in *) ifa->ifa_addr)->sin_addr.s_addr);
      ++n;
    }
  }

  freeifaddrs(ifaddr);

  return n;
}

/**
 * format the output string with interface state flags for comfortable reading by human
 */
char * cf_format_iface_flags(uint32_t ifa_flags, char string[256])
{
  char * beg = string;
#define CHECKIFF( flag ) \
  if ( IFF_##flag & ifa_flags ) { \
    string = strappend(string, #flag), *string ++= ' '; \
  }

  CHECKIFF(UP);
  CHECKIFF(BROADCAST);
  CHECKIFF(DEBUG);
  CHECKIFF(LOOPBACK);
  CHECKIFF(POINTOPOINT);
  CHECKIFF(NOTRAILERS);
  CHECKIFF(RUNNING);
  CHECKIFF(NOARP);
  CHECKIFF(PROMISC);
  CHECKIFF(ALLMULTI);
  CHECKIFF(MASTER);
  CHECKIFF(SLAVE);
  CHECKIFF(MULTICAST);
  CHECKIFF(PORTSEL);
  CHECKIFF(AUTOMEDIA);
  CHECKIFF(DYNAMIC);
  *string = 0;

#undef CHECKIFF
  return beg;
}



/**
 * Parse string with address:port pair.
 *  The address may be present as interface name
 */
bool cf_get_iface_address(const char * string, uint32_t * address, uint16_t * port)
{
  char * tmp;
  char sdup[strlen(string) + 1];
  int use_iface_name;

  if ( port != NULL ) {
    *port = 0;
  }
  *address = (uint32_t)(-1);

  strcpy(sdup, string);

  if ( isdigit(*sdup) ) {
    use_iface_name = 0;
  }
  else if ( isalpha(*sdup) ) {
    use_iface_name = 1;
  }
  else {
    errno = EINVAL;
    return false;
  }

  if ( port != NULL && (tmp = strchr(sdup, ':')) != 0) {
    if ( sscanf(tmp + 1, "%hu", port) != 1 ) {
      errno = EINVAL;
      return false;
    }
    *tmp = 0; /* Fix C string with zero byte */
  }

  if ( !use_iface_name ) {
    uint8_t b1, b2, b3, b4;

    if ( sscanf(sdup, "%hhu.%hhu.%hhu.%hhu", &b1, &b2, &b3, &b4) != 4 ) {
      errno = EINVAL;
      return false;
    }

    *address = INET_ADDR(b1, b2, b3, b4);
  }
  else {
    /* Search the interface name requested and query it's ip address */
    struct ifaceinfo ifaces[64];
    size_t ifaces_count;
    int iface_found = 0;
    size_t i;

    if ( (ifaces_count = cf_enumerate_ifaces(ifaces, sizeof(ifaces) / sizeof(ifaces[0]))) == (size_t) -1 ) {
      return false;
    }

    if ( ifaces_count == 0 ) {
      errno = ENODEV;
      return false;
    }

    for ( i = 0; i < ifaces_count; ++i ) {
      if ( strcmp(ifaces[i].ifname, sdup) == 0 ) {
        *address = ifaces[i].ifaddress;
        iface_found = 1;
        break;
      }
    }

    if ( !iface_found ) {
      errno = ENXIO;
      return false;
    }
  }

  return true;
}

/* fixme: only IPv4 is implemented */
std::string cf_get_iface_address(const std::string & ifacename)
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
