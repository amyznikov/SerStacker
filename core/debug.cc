/*
 * debug.c
 *
 *  Created on: Aug 27, 2016
 *      Author: amyznikov
 */

#include "debug.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef __APPLE__
# include <stdlib.h>
# include <thread>
#else
# include <malloc.h>
#endif

#include <string.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <mutex>
//#include <string>

#include <sys/syscall.h>
#include <ucontext.h>
#include <execinfo.h>

// current log time stamp
namespace {

  struct c_current_time {
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec;
    int msec;
  };

  static void get_current_time(struct c_current_time * ct)
  {
    struct timespec t;
    struct tm * tm;

    clock_gettime(CLOCK_REALTIME, &t);
    tm = gmtime(&t.tv_sec);

    ct->year = tm->tm_year + 1900;
    ct->month = tm->tm_mon + 1;
    ct->day = tm->tm_mday;
    ct->hour = tm->tm_hour;
    ct->min = tm->tm_min;
    ct->sec = tm->tm_sec;
    ct->msec = t.tv_nsec / 1000000;
  }

  static const char * get_current_time_string(char buf[32])
  {
    struct c_current_time ct;
    get_current_time(&ct);
    snprintf(buf, 31, "%.4d-%.2d-%.2d-%.2d:%.2d:%.2d.%.3d",
        ct.year, ct.month, ct.day, ct.hour, ct.min, ct.sec, ct.msec);
    return buf;
  }

}

static std::mutex mtx;
static void (*logfunc)(void * context, const char * msg) = NULL;
static void * logcontext = NULL;
static FILE * fplog = NULL;
static std::string logfilename;
static uint32_t logmask = CF_LOG_DEBUG;

static std::vector<cf_error_log_entry> g_error_log;
static bool enable_error_log_ = false;

void cf_enable_error_log(bool v)
{
  mtx.lock();
  enable_error_log_ = v;
  g_error_log.clear();
  mtx.unlock();
}

void cf_get_error_log(std::vector<cf_error_log_entry> * errlog, bool clear)
{
  mtx.lock();

  if ( errlog ) {
    * errlog = g_error_log;
  }

  if ( clear ) {
    g_error_log.clear();
  }

  mtx.unlock();
}

void cf_clear_error_log()
{
  mtx.lock();
  g_error_log.clear();
  mtx.unlock();
}

std::string cf_get_last_error_msg()
{
  std::string msg;

  mtx.lock();
  if( !g_error_log.empty() ) {
    msg = g_error_log.back().msg;
  }
  mtx.unlock();

  return msg;
}


pid_t get_tid() {
#ifdef _WIN32
  return 0; // fixme: return process id under win32
#elif __APPLE__
  return std::hash<std::thread::id>()(std::this_thread::get_id());
#else
  return (pid_t) syscall (SYS_gettid);
#endif
}

void cf_set_logfunc(void (*func)(void * context, const char * msg), void * context)
{
  mtx.lock();
  logfunc = func;
  logcontext = context;
  mtx.unlock();
}

void cf_set_logfile(FILE * fp)
{
  mtx.lock();

  if ( fplog && fplog != stderr && fplog != stdout ) {
    fclose(fplog), fplog = NULL;
  }
  fplog = fp;

  mtx.unlock();

}

FILE * cf_get_logfile(void)
{
  return fplog;
}

bool cf_set_logfilename(const std::string fname, const std::string & mode)
{
  bool fok = false;

  mtx.lock();

  logfilename.clear();

  if ( fplog && fplog != stderr && fplog != stdout ) {
    fclose(fplog), fplog = NULL;
  }

  if ( fname.empty() ) {
    fok = true;
  }
  else if ( strcasecmp(fname.c_str(), "stderr") == 0 ) {
    fplog = stderr;
  }
  else if ( strcasecmp(fname.c_str(), "stdout") == 0 ) {
    fplog = stdout;
  }
  else if ( !(fplog = fopen((logfilename = fname).c_str(), mode.empty() ? "a" : mode.c_str())) ) {
    fprintf(stderr, "fatal error: fopen(logfilename='%s', mode='%s') fails: %s\n",
        logfilename.c_str(), mode.c_str(),
        strerror(errno));
  }
  else {
    char ctime_string[32];
    get_current_time_string(ctime_string);
    fprintf(fplog, "\n\nNEW LOG STARTED AT %s\n", ctime_string);
    fok = true;
  }

  mtx.unlock();

  return fok;
}

const std::string & cf_get_logfilename(void)
{
  return logfilename;
}

void cf_set_loglevel(uint32_t mask)
{
  logmask = mask;
}

void cf_set_loglevel_str(const char * ll)
{
  if ( ll && *ll ) {
    static const struct {
      const char * name;
      uint32_t value;
    } llevels[] = {
        {"FATAL", CF_LOG_FATAL  },   /* system is unusable */
        {"CRITICAL", CF_LOG_CRITICAL},   /* critical conditions */
        {"ERROR", CF_LOG_ERROR  },   /* error conditions */
        {"WARNING", CF_LOG_WARNING},   /* warning conditions */
        {"NOTICE", CF_LOG_NOTICE },   /* normal but significant condition */
        {"INFO", CF_LOG_INFO  },   /* informational */
        {"DEBUG", CF_LOG_DEBUG },   /* debug-level messages */
    };

    for ( uint i = 0; i < sizeof(llevels) / sizeof(llevels[0]); ++i ) {
      if ( strcasecmp(llevels[i].name, ll) == 0 ) {
        cf_set_loglevel(llevels[i].value);
        break;
      }
    }
  }
}


uint32_t cf_get_loglevel(void)
{
  return logmask;
}

static char pric(int pri)
{
  char ch;

  switch ( pri ) {
    case CF_LOG_FATAL :
      ch = 'F';
    break;
    case CF_LOG_CRITICAL :
      ch = 'C';
    break;
    case CF_LOG_ERROR :
      ch = 'E';
    break;
    case CF_LOG_WARNING :
      ch = 'W';
    break;
    case CF_LOG_NOTICE :
      ch = 'N';
    break;
    case CF_LOG_INFO :
      ch = 'I';
    break;
    case CF_LOG_DEBUG :
      ch = 'D';
    break;
    default :
      ch = 'U';
    break;
  }
  return ch;
}


void cf_log_put(const char * msg)
{
  if ( fplog ) {
    fputs(msg, fplog);
  }
  if ( logfunc ) {
    logfunc(logcontext, msg);
  }
}



static inline constexpr auto * file_name(const char * const path)
{
  const auto * startPosition = path;
  for ( const auto * currentCharacter = path; *currentCharacter != '\0'; ++currentCharacter ) {
    if ( *currentCharacter == '/' ) { // || *currentCharacter == '\\'
      startPosition = currentCharacter;
    }
  }
  if ( startPosition != path ) {
    ++startPosition;
  }
  return startPosition;
}


static void do_plogv(int pri, const char * file, const char * func, int line, const char * format, va_list arglist)
{
  char ctime_string[32];
  char msgbuf[4096] = "";

  if ( logfunc || fplog || (enable_error_log_ && pri <= CF_LOG_ERROR) ) {
    get_current_time_string(ctime_string);
  }

  file = file_name(file);

  if ( fplog || logfunc ) {
    vsnprintf(msgbuf, sizeof(msgbuf) - 1, format, arglist);
  }

  if ( fplog ) {
    fprintf(fplog, "|%c|%6d|%s| %s %s() %4d : %s\n", pric(pri), (int) get_tid(), ctime_string, file, func, line, msgbuf);
    fflush(fplog);
  }

  if ( logfunc ) {
    char buf[4096] = "";
    snprintf(buf, sizeof(buf) - 1, "|%c|%6d|%s| %s %s() %4d : %s", pric(pri), (int) get_tid(), ctime_string, file, func, line, msgbuf);
    logfunc(logcontext, buf);
  }

  if( enable_error_log_ && pri <= CF_LOG_ERROR ) {

    while (g_error_log.size() > 16) {
      g_error_log.erase(g_error_log.begin());
    }

    cf_error_log_entry e = {
        .time = ctime_string,
        .file = file + std::string(" : ") + std::to_string(line),
        .func = func,
        .msg = msgbuf
    };

    g_error_log.emplace_back(e);
  }
}

void cf_plogv(int pri, const char * file, const char * func, int line, const char * format, va_list arglist)
{
  mtx.lock();
  if ( (fplog || logfunc) && (pri & 0x07) <= (logmask & 0x07) ) {
    do_plogv(pri, file, func, line, format, arglist);
  }
  mtx.unlock();
}

void cf_plog(int pri, const char * file, const char * func, int line, const char * format, ...)
{
  mtx.lock();

  if ( (fplog || logfunc) && (pri & 0x07) <= (logmask & 0x07) ) {

    if ( fplog && fplog != stderr && fplog != stdout && !logfilename.empty() ) {
      if ( ftell(fplog) > 10 * 1024 * 1024 ) {
        fclose(fplog);
        if ( !(fplog = fopen(logfilename.c_str(), "w")) ) {
          fprintf(stderr, "FATAL ERROR in cf_plog(): fopen(logfilename=%s, 'w') fails: %s\n", logfilename, strerror(errno));
          fplog = stderr;
        }
        else {
          char ctime_string[32];
          get_current_time_string(ctime_string);
          fprintf(fplog, "\n\nLOG TRUNCATED AT %s\n", ctime_string);
        }
      }
    }

    va_list arglist;
    va_start(arglist, format);
    do_plogv(pri, file, func, line, format, arglist);
    va_end(arglist);
  }

  mtx.unlock();
}



#if ENABLE_CALL_GRAPH_INSTRUMENTATION

#include <core/readdir.h>
#include <core/ssprintf.h>

std::mutex c_scope_timer::mtx_;
std::vector<c_scope_timer::c_callgraph *> c_scope_timer::allgraphs_;
thread_local c_scope_timer::c_callgraph * c_scope_timer::mygraph_;


// REALTIME ns
static inline double get_timestamp_ns(void)
{
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return (double) (t.tv_sec * 1e9 + t.tv_nsec);
}

c_scope_timer::c_scope_timer(const char * _func, const char * _scope)
  : tstamp_(get_timestamp_ns())
{
  // check if call graph for this thread was already created
  if ( !mygraph_ ) {

    mtx_.lock();

    allgraphs_.emplace_back(
        mygraph_ = new c_callgraph(
            get_tid()));

    mtx_.unlock();
  }


  // check if graph node for this scope was already created

  c_graph_node *& node =
      mygraph_->nodes[c_callgraph::key(mygraph_, _func, _scope)];

  if ( !node ) {

    // Create new node to track this scope calls.
    // Add it to the list of childs of parent node

    node =
        new c_graph_node(
            _func,
            _scope);

    if ( mygraph_->current != nullptr ) { // has parent
      node->parent = mygraph_->current->node_;
      node->parent->childs.emplace_back(node);
    }
  }

  // update current call counter and current scope track
  ++(this->node_ = node)->call_count;
  this->parent_ = mygraph_->current;
  mygraph_->current = this;

}

c_scope_timer::~c_scope_timer()
{
  // update current timer and current scope track

  node_->total_time +=
      get_timestamp_ns() - tstamp_;

  mygraph_->current =
      this->parent_;
}

// call graph dump utility
static struct c_callgraph_dump {

  using c_callgraph = c_scope_timer::c_callgraph;
  using c_graph_node = c_scope_timer::c_graph_node;

  ~c_callgraph_dump();

  static void dump();
  static void dump_node(FILE * fp,
      const c_graph_node * node,
      int level);

} _callgraph_dump;


c_callgraph_dump::~c_callgraph_dump()
{
  dump();
}

void c_callgraph_dump::dump()
{
  std::string filename =
      ssprintf("./%s.timings.txt",
          c_file_name(get_self_exe_pathname()));

  FILE * fp = fopen(filename.c_str(), "w");
  if ( !fp ) {
    CF_FATAL("ERROR: Can not write '%s' : %s",
        filename.c_str(),
        strerror(errno));
  }
  else {

    fprintf(fp, "COMMAND:\n%s\n\n",
        get_self_exe_cmdline().c_str());

    fprintf(fp, "scope\tcalls\ttotal_time\tavg_time\n");

    c_scope_timer::mtx_.lock();

    for ( const c_callgraph * graph : c_scope_timer::allgraphs_ ) {

      fprintf(fp, "PID: %u\n", (unsigned int) graph->pid);

      for ( const auto & ii : graph->nodes ) {
        const c_graph_node * node = ii.second;
        if ( !node->parent ) {

          dump_node(fp, node, 0);
        }
      }

      fprintf(fp, "\n");
    }

    c_scope_timer::mtx_.unlock();

    fclose(fp);
  }
}

void c_callgraph_dump::dump_node(FILE * fp, const c_graph_node * node, int level)
{
  static const auto indent =
      [](FILE * fp, int n) {
        if ( n > 0 ) {
          for ( int i = 0; i < 4 * n; ++i ) {
            fputc('.', fp);
          }
          fputc(' ', fp);
        }
      };

  indent(fp, level);


  if ( node->scope && *node->scope ) {
    fprintf(fp, "%s/%s", node->functon, node->scope);
  }
  else {
    fprintf(fp, "%s", node->functon);
  }

  const double time =
      node->total_time * 1e-6;

  fprintf(fp, "\t%12d\t%.3f\t%.3f\n",
      node->call_count,
      time,
      time / node->call_count);

  for ( const c_graph_node * child : node->childs ) {
    dump_node(fp, child, level+1);
  }

}

#endif // ENABLE_TIME_INSTRUMENTATION

/**
 * Custom signal handler
 */
static void my_signal_handler(int signum, siginfo_t *si, void * context)
{
  int ignore = 0;
  int status = 0;
  const ucontext_t * uc = (ucontext_t *) context;
  void * caller_address;

#if ( __aarch64__ )
  caller_address = (void *) uc->uc_mcontext.pc;
#elif ( __arm__ )
  caller_address = (void *) uc->uc_mcontext.arm_pc;
#else
  caller_address = (void *) uc->uc_mcontext.gregs[16]; // REG_RIP
#endif

  if ( signum != SIGWINCH && signum != 17 ) {
    CF_CRITICAL("SIGNAL %d (%s)", signum, strsignal(signum));
  }

  switch ( signum ) {
    case SIGINT :
    case SIGQUIT :
    case SIGTERM :
      status = 0;
    break;

    case SIGSEGV :
    case SIGSTKFLT :
    case SIGILL :
    case SIGBUS :
    case SIGSYS :
    case SIGFPE :
    case SIGABRT :
      status = EXIT_FAILURE;
      CF_FATAL("Fault address:%p from %p", si->si_addr, caller_address);
    break;

    default :
      ignore = 1;
    break;
  }

  if ( !ignore ) {
#if ENABLE_TIME_INSTRUMENTATION
    c_callgraph_dump::dump();
#endif // ENABLE_TIME_INSTRUMENTATION
    exit(status);
  }
}


/**
 * setup_signal_handler()
 *    see errno on failure
 */
bool cf_setup_signal_handler(void)
{
  struct sigaction sa;
  int sig;

  memset(&sa, 0, sizeof(sa));

  sa.sa_flags = SA_SIGINFO;
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = my_signal_handler;

  for ( sig = 1; sig < SIGRTMIN; ++sig ) { // SIGUNUSED
    /* skip unblockable signals */
    if ( sig != SIGKILL && sig != SIGSTOP && sig != SIGCONT && sigaction(sig, &sa, NULL) != 0 ) {
      return false;
    }
  }

  return true;
}

