/*
 * sockopt.c
 *
 *  Created: Oct 6, 2015
 *      Author: amyznikov
 *
 *  Update: Jan 30, 2022
 *     Adapt to glddm
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include "sockopt.h"

int so_get_error(int so)
{
  int optval = 0;
  socklen_t optlen = sizeof(optval);

  if ( getsockopt(so, SOL_SOCKET, SO_ERROR, &optval, &optlen) < 0 ) {
    optval = errno;
  }
  else {
    setsockopt(so, SOL_SOCKET, SO_ERROR, &optval, optlen);
  }

  return optval;
}


socklen_t so_get_addrlen(const struct sockaddr * addr)
{
  switch ( addr->sa_family ) {
    case AF_INET :
      return sizeof(struct sockaddr_in);
    case AF_INET6 :
      return sizeof(struct sockaddr_in6);
    case AF_UNIX :
      return offsetof(struct sockaddr_un, sun_path) + strlen(((struct sockaddr_un*) addr)->sun_path);
  }
  return 0;
}

bool so_set_send_bufsize(int so, int size)
{
  return setsockopt(so, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)) == 0;
}

bool so_set_recv_bufsize(int so, int size)
{
  return setsockopt(so, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size)) == 0;
}

int so_get_send_bufsize(int so, int * size)
{
  socklen_t optlen = sizeof(*size);
  return getsockopt(so, SOL_SOCKET, SO_SNDBUF, size, &optlen);
}

int so_get_recv_bufsize(int so, int * size)
{
  socklen_t optlen = sizeof(*size);
  return getsockopt(so, SOL_SOCKET, SO_RCVBUF, size, &optlen);
}

bool so_set_send_timeout(int so, int sec)
{
  struct timeval timeout = { .tv_sec = sec, .tv_usec = 0 };
  return setsockopt(so, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) == 0;
}

bool so_set_recv_timeout(int so, int sec)
{
  struct timeval timeout = { .tv_sec = sec, .tv_usec = 0 };
  return setsockopt(so, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
}

int so_get_send_timeout(int so, int * sec)
{
  struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
  socklen_t optlen = sizeof(timeout);
  if ( getsockopt(so, SOL_SOCKET, SO_SNDTIMEO, &timeout, &optlen) != -1 ) {
    *sec = (int) (timeout.tv_sec);
    return 0;
  }
  return -1;
}

int so_get_recv_timeout(int so, int * sec)
{
  struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
  socklen_t optlen = sizeof(timeout);
  if ( getsockopt(so, SOL_SOCKET, SO_RCVTIMEO, &timeout, &optlen) != -1 ) {
    *sec = (int) (timeout.tv_sec);
    return 0;
  }
  return -1;
}

bool so_set_nodelay(int so, int optval)
{
  return setsockopt(so, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval)) == 0;
}

int so_get_nodelay(int so, int * optval)
{
  socklen_t optlen = sizeof(*optval);
  return getsockopt(so, IPPROTO_TCP, TCP_NODELAY, optval, &optlen);
}


bool so_set_reuse_addrs(int so, int optval)
{
  return setsockopt(so, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == 0;
}

int so_get_reuse_addrs(int so, int * optval)
{
  socklen_t optlen = sizeof(*optval);
  return getsockopt(so, SOL_SOCKET, SO_REUSEADDR, optval, &optlen);
}

bool so_is_listening(int so)
{
  int optval = 0;
  socklen_t optlen = sizeof(optval);
  getsockopt(so, SOL_SOCKET, SO_ACCEPTCONN, &optval, &optlen);
  return optval != 0;
}

bool so_set_non_blocking(int so, int optval)
{
  int flags, status;

  if ( (flags = fcntl(so, F_GETFL, 0)) < 0 ) {
    status = -1;
  }
  else if ( optval ) {
    status = fcntl(so, F_SETFL, flags | O_NONBLOCK);
  }
  else {
    status = fcntl(so, F_SETFL, flags & ~O_NONBLOCK);
  }

  return status != -1;
}


bool so_set_keepalive(int so, int keepalive, int keepidle, int keepintvl, int keepcnt)
{
  if ( keepalive != -1 && setsockopt(so, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) == -1 ) {
    return false;
  }

  if ( keepalive ) {
    if ( keepidle != -1 && setsockopt(so, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle)) == -1 ) {
      return false;
    }

    if ( keepintvl != -1 && setsockopt(so, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl)) == -1 ) {
      return false;
    }

    if ( keepcnt != -1 && setsockopt(so, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt)) == -1 ) {
      return false;
    }
  }

  return true;
}


bool so_get_keepalive(int so, int * keepalive, int * keepidle, int * keepintvl, int * keepcnt)
{
  if ( keepalive ) {
    socklen_t optlen = sizeof(*keepalive);
    if ( getsockopt(so, SOL_SOCKET, SO_KEEPALIVE, keepalive, &optlen) == -1 ) {
      return false;
    }
  }

  if ( keepidle ) {
    socklen_t optlen = sizeof(*keepidle);
    if ( setsockopt(so, IPPROTO_TCP, TCP_KEEPIDLE, keepidle, optlen) == -1 ) {
      return false;
    }
  }

  if ( keepintvl ) {
    socklen_t optlen = sizeof(*keepintvl);
    if ( setsockopt(so, IPPROTO_TCP, TCP_KEEPINTVL, keepintvl, optlen) == -1 ) {
      return false;
    }
  }

  if ( keepcnt ) {
    socklen_t optlen = sizeof(*keepcnt);
    if ( setsockopt(so, IPPROTO_TCP, TCP_KEEPCNT, keepcnt, optlen) == -1 ) {
      return false;
    }
  }

  return true;
}

/** SIOCOUTQ
 *    man 7 tcp
 *    http://stackoverflow.com/questions/9618150/linux-send-whole-message-or-none-of-it-on-tcp-socket
 */
int so_get_outq_size(int so)
{
  int size;
  return ioctl(so, TIOCOUTQ, &size) == -1 ? -1 : size;
}

bool so_close(int so, bool abort_conn)
{
  if ( so == -1 ) {
    errno = EINVAL;
    return false;
  }

  if ( abort_conn ) {
    struct linger lo = { .l_onoff = 1, .l_linger = 0 };
    setsockopt(so, SOL_SOCKET, SO_LINGER, &lo, sizeof(lo));
  }

  shutdown(so, SHUT_RDWR);
  return close(so) == 0;
}

void so_sockaddr_in(const char * addrs, uint16_t port, struct sockaddr_in * sin)
{
  memset(sin, 0, sizeof(*sin));
  sin->sin_family = AF_INET;
  inet_pton(AF_INET, addrs, &sin->sin_addr);
  sin->sin_port = htons(port);
}


int so_tcp_listen(const char * addrs, uint16_t port, struct sockaddr_in * _sin)
{
  bool fOk = false;
  struct sockaddr_in sin;
  int so = -1;

  if ((so = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1 ) {
    // CF_FATAL("socket() fails: %s", strerror(errno));
    goto __end;
  }

  so_sockaddr_in(addrs, port, &sin);

  if ( !so_set_reuse_addrs(so, true) ) {
    // CF_WARNING("so_set_reuse_addrs() fails: %s", strerror(errno));
  }

  if ( bind(so, (struct sockaddr*)&sin, sizeof(sin)) == -1 ) {
    //CF_FATAL("bind() fails: %s", strerror(errno));
    goto __end;
  }

  if ( listen(so, SOMAXCONN) == -1 ) {
    // CF_FATAL("listen() fails: %s", strerror(errno));
    goto __end;
  }

  if ( _sin ) {
    *_sin = sin;
  }

  fOk = true;

__end:

  if ( !fOk ) {
    if ( so != -1 ) {
      close(so), so = -1;
    }
  }

  return so;
}



int so_tcp_listen2(uint32_t addrs, uint16_t port, struct sockaddr_in * _sout)
{
  bool fOk = false;
  struct sockaddr_in sin;
  int so = -1;

  if ((so = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1 ) {
    // CF_FATAL("socket() fails: %s", strerror(errno));
    goto __end;
  }

  if ( !so_set_reuse_addrs(so, true) ) {
    // CF_WARNING("so_set_reuse_addrs() fails: %s", strerror(errno));
  }


  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = htonl(addrs);
  sin.sin_port = htons(port);

  if ( bind(so, (struct sockaddr*)&sin, sizeof(sin)) == -1 ) {
    //CF_FATAL("bind() fails: %s", strerror(errno));
    goto __end;
  }

  if ( listen(so, SOMAXCONN) == -1 ) {
    // CF_FATAL("listen() fails: %s", strerror(errno));
    goto __end;
  }

  if ( _sout ) {
    *_sout = sin;
  }

  fOk = true;

__end:

  if ( !fOk ) {
    if ( so != -1 ) {
      close(so), so = -1;
    }
  }

  return so;
}


int so_tcp_connect(struct sockaddr_in * addrs)
{
  bool fOk = false;
  int so = -1;

  if ((so = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1 ) {
    goto __end;
  }

  if ( connect(so, (struct sockaddr *)addrs, sizeof(*addrs)) == -1 ) {
    goto __end;
  }

  fOk = true;

__end:

  if ( !fOk ) {
    if ( so != -1 ) {
      close(so), so = -1;
    }
  }

  return so;
}

int so_tcp_connect2(const char * addrs, uint16_t port)
{
  struct sockaddr_in sin;
  so_sockaddr_in(addrs, port, &sin);
  return so_tcp_connect(&sin);
}

int so_tcp_connect3(const char * addrport)
{
  char addrs[256] = "";
  uint16_t port = 0;
  if ( sscanf(addrport, "%255[^:]:%hu", addrs, &port) != 2 || port == 0 ) {
    errno = EINVAL;
    return -1;
  }
  return so_tcp_connect2(addrs, port);
}
