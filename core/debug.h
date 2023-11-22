/*
 * debug.h
 *
 *  Created on: March 11, 2022
 *      Author: amyznikov
 *
 *  Simple debug logging utilities for qlidarview project.
 */

#ifndef __serstacker_debug_h__
#define __serstacker_debug_h__

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <vector>

#if _MSC_VER
typedef int pid_t;
#else
# include <unistd.h>
#endif

#define __DEBUG_H_INCLUDED__  1

/** logging verbosity levels */
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

bool cf_set_logfilename(const std::string fname, const std::string & mode = "a" );
const std::string & cf_get_logfilename(void);

void cf_set_logfunc(void (*func)(void * context, const char * msg), void * context);


void cf_set_loglevel(uint32_t mask);
void cf_set_loglevel_str(const char * ll);
uint32_t cf_get_loglevel(void);

bool cf_setup_signal_handler(void);


struct cf_error_log_entry {
  std::string time;
  std::string file;
  std::string func;
  std::string msg;
};

void cf_enable_error_log(bool);
void cf_get_error_log(std::vector<cf_error_log_entry> * errlog, bool clear = true);
void cf_clear_error_log();
std::string cf_get_last_error_msg();


pid_t get_tid();

void cf_plogv(int pri, const char * file, const char * func, int line, const char * format, va_list arglist);
void cf_plog(int pri, const char * file, const char * func, int line, const char * format, ...)
#if !_MSC_VER
  __attribute__ ((__format__ (printf, 5, 6)));
#else
;
#endif

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




#if ENABLE_CALL_GRAPH_INSTRUMENTATION

#include <vector>
#include <map>
#include <mutex>

class c_scope_timer
{
  friend class c_callgraph_dump;

  struct c_graph_node {
    const char * functon;
    const char * scope;
    double total_time = 0;
    int call_count = 0;

    c_graph_node * parent = nullptr;
    std::vector<c_graph_node *> childs;

    c_graph_node(const char * _functon, const char * _scope) :
         functon(_functon), scope(_scope)
    {
    }
  };

  struct c_callgraph {

    struct key {
      c_graph_node * parent;
      const char * func;
      const char * scope;

      key(c_callgraph * graph, const char * _func, const char * _scope) :
          parent(graph->current ? graph->current->node_ : nullptr) ,
          func(_func),
          scope(_scope)
      {
      }

      bool operator < (const key & rhs) const
      {
        if ( parent < rhs.parent ) {
          return true;
        }
        if ( parent == rhs.parent ) {
          if ( func < rhs.func ) {
            return true;
          }
          if ( func == rhs.func ) {
            if ( scope < rhs.scope ) {
              return true;
            }
          }
        }
        return false;
      }
    };

    c_callgraph(pid_t _pid) : pid(pid)
    {
    }

    std::map<key, c_graph_node*> nodes;
    c_scope_timer * current = nullptr;
    pid_t pid;
  };

public:
  c_scope_timer(const char * _func, const char * _scope);
  ~c_scope_timer();

protected:
  c_scope_timer * parent_;
  c_graph_node * node_;
  double tstamp_;

protected:
  static std::mutex mtx_;
  static std::vector<c_scope_timer::c_callgraph *> allgraphs_;
  static thread_local c_callgraph * mygraph_;
};


#define INSTRUMENT_REGION(scope) \
    c_scope_timer my_scope_timer(__FUNCTION__, scope)

#else // ENABLE_TIME_INSTRUMENTATION

# define INSTRUMENT_REGION(scope)

#endif  // ENABLE_TIME_INSTRUMENTATION

#endif /* __serstacker_debug_h__  */


