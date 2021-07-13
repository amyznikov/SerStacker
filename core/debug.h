/*
 * debug.h
 *
 *  Created on: Aug 27, 2016
 *      Author: amyznikov
 */

#ifndef __cf_core_debug_h__
#define __cf_core_debug_h__

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  CF_LOG_FATAL   = 0,   /* system is unusable */
  CF_LOG_CRITICAL= 1,   /* critical conditions */
  CF_LOG_ERROR   = 2,   /* error conditions */
  CF_LOG_WARNING = 3,   /* warning conditions */
  CF_LOG_NOTICE  = 4,   /* normal but significant condition */
  CF_LOG_INFO    = 5,   /* informational */
  CF_LOG_DEBUG   = 6,   /* debug-level messages */
  CF_LOG_EVENT   = 0x8  /* custom event masks start here */
};


void cf_set_logfile(FILE * fp);
FILE * cf_get_logfile(void);

bool cf_set_logfilename(const char * fname, const char * mode/* = "a" */);
const char * cf_get_logfilename(void);

void cf_set_logfunc(void (*func)(void * context, const char * msg), void * context);


void cf_set_loglevel(uint32_t mask);
void cf_set_loglevel_str(const char * ll);
uint32_t cf_get_loglevel(void);

bool cf_setup_signal_handler(void);



pid_t get_tid();

void cf_plogv(int pri, const char * file, const char * func, int line, const char * format, va_list arglist);
void cf_plog(int pri, const char * file, const char * func, int line, const char * format, ...)
  __attribute__ ((__format__ (printf, 5, 6)));

void cf_log_put(const char * msg);

void cf_pbt(void);


#define CF_FATAL(...)     cf_plog(CF_LOG_FATAL  , __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_CRITICAL(...)  cf_plog(CF_LOG_CRITICAL,__FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_ERROR(...)     cf_plog(CF_LOG_ERROR  , __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_WARNING(...)   cf_plog(CF_LOG_WARNING, __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_NOTICE(...)    cf_plog(CF_LOG_NOTICE , __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_INFO(...)      cf_plog(CF_LOG_INFO   , __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_DEBUG(...)     cf_plog(CF_LOG_DEBUG  , __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)
#define CF_EVENT(e,...)   cf_plog(e, __FILE__,  __FUNCTION__, __LINE__, __VA_ARGS__)


// fixme: check the http://svn.pld-linux.org/svn/backtracexx/
#define CF_PBT()          cf_pbt()

#ifdef __cplusplus
}
#endif

#endif /* __cf_core_debug_h__ */


