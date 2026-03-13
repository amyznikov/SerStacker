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

#ifndef _WIN32

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <stdint.h>
#include <string.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <iphlpapi.h>
  #include <ws2tcpip.h>
   // Link with : -liphlpapi -lws2_32
#else
  #include <ifaddrs.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <net/if.h>
#endif

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
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


ssize_t cf_enumerate_ifaces(struct ifaceinfo ifaces[], size_t maxifaces)
{
#ifdef _WIN32
    PIP_ADAPTER_ADDRESSES pAddresses = NULL;
    ULONG outBufLen = 15000;
    ULONG flags = GAA_FLAG_INCLUDE_PREFIX;
    ssize_t n = 0;

    pAddresses = (IP_ADAPTER_ADDRESSES*)malloc(outBufLen);
    if (GetAdaptersAddresses(AF_INET, flags, NULL, pAddresses, &outBufLen) != NO_ERROR) {
        free(pAddresses);
        return -1;
    }

    for (PIP_ADAPTER_ADDRESSES pCurr = pAddresses; pCurr && (size_t)n < maxifaces; pCurr = pCurr->Next) {
        for (PIP_ADAPTER_UNICAST_ADDRESS pUnicast = pCurr->FirstUnicastAddress; pUnicast; pUnicast = pUnicast->Next) {
            if (pUnicast->Address.lpSockaddr->sa_family == AF_INET) {
                snprintf(ifaces[n].ifname, sizeof(ifaces[n].ifname), "%ls", pCurr->FriendlyName);

                sockaddr_in* sa_in = (sockaddr_in*)pUnicast->Address.lpSockaddr;
                ifaces[n].ifaddress = ntohl(sa_in->sin_addr.s_addr);

                ifaces[n].ifa_flags = 0;
                if (pCurr->OperStatus == IfOperStatusUp) ifaces[n].ifa_flags |= 0x1; // IFF_UP

                n++;
                break;
            }
        }
    }
    free(pAddresses);
    return n;
#else
    struct ifaddrs *ifaddr, *ifa;
    ssize_t n = 0;

    if (getifaddrs(&ifaddr) == -1) return -1;

    for (ifa = ifaddr; n < (ssize_t)maxifaces && ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
            strncpy(ifaces[n].ifname, ifa->ifa_name, sizeof(ifaces[n].ifname) - 1);
            ifaces[n].ifname[sizeof(ifaces[n].ifname) - 1] = 0;
            ifaces[n].ifa_flags = ifa->ifa_flags;
            ifaces[n].ifaddress = ntohl(((struct sockaddr_in *)ifa->ifa_addr)->sin_addr.s_addr);
            n++;
        }
    }
    freeifaddrs(ifaddr);
    return n;
#endif
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

bool cf_get_iface_address(const char * string, struct sockaddr_in * saddrs)
{
  uint32_t address = 0;
  uint16_t port = 0;

  if ( cf_get_iface_address(string, &address, &port) ) {

    memset(saddrs, 0, sizeof(saddrs));
    saddrs->sin_family = AF_INET;
    saddrs->sin_addr.s_addr = htonl(address);
    saddrs->sin_port = htons(port);
    return true;
  }

  return false;
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

#endif // _WIN32
