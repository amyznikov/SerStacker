/*
 * readdir.cc
 *
 *  Created on: Nov 8, 2016
 *      Author: amyznikov
 */

#include <limits.h>
#include <string.h>
#include <ftw.h>
#include <ctype.h>
#include <errno.h>
#include <algorithm>


#include "readdir.h"
#include "ssprintf.h"
#include "debug.h"

#ifdef _WIN32
# include <windows.h>
# include <sys/stat.h>
#endif


#ifndef _WIN32
# include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
# include <fnmatch.h>
#else

# include <io.h>
# include "Shlwapi.h"

/*
 *  Match NAME against the filename pattern PATTERN,
 *  returning zero if it matches, FNM_NOMATCH if not.
 *  Link against shlwapi.lib
 */
static int fnmatch (const char *__pattern, const char *__name, int __flags) {
  (void)(__flags);
  return PathMatchSpec(__name,__pattern) ? 0 : -1;
}

#endif


///////////////////////////////////////////////////////////////////////////////

/**
 *  Check if file exists
 * */
bool file_exists(const std::string & path)
{
  return access(expand_path(path).c_str(), F_OK) == 0;
}

/* check for directory existence */
bool is_directory(const std::string & path)
{
  struct stat sb;
  return stat(expand_path(path).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
}

/* check for regular file existence */
bool is_regular_file(const std::string & path)
{
  struct stat sb;
  return stat(expand_path(path).c_str(), &sb) == 0 && S_ISREG(sb.st_mode);
}


/* check for link existence */
bool is_link(const std::string & path)
{
  struct stat sb;
  return stat(expand_path(path).c_str(), &sb) == 0 && S_ISLNK(sb.st_mode);
}


/**
 * Check caller has read access rights for the file
 * */
bool file_readable(const std::string & path)
{
  return access(expand_path(path).c_str(), R_OK) == 0;
}

/* get file name from full path name */
std::string get_file_name(const std::string & fullpathname)
{
#ifdef _WIN32
  const char delims[] = "\\/";
#else
  const char delims[] = "/";
#endif

  size_t pos = fullpathname.find_last_of(delims);
  if ( pos != std::string::npos ) {
    return fullpathname.substr(pos + 1);
  }

  return fullpathname;
}


/* get file name part from full path name (C-string version)*/
const char * c_file_name(const char * s)
{
#ifdef _WIN32
  const char delims[] = "\\/";
#else
  const char delims[] = "/";
#endif

  const char * eos = s + strlen(s);
  while ( eos > s && !strchr(delims, *(eos - 1)) ) {
    --eos;
  }

  return eos;
}

/* get file name part from full path name (C-string version)*/
const char * c_file_name(const std::string & ss)
{
#ifdef _WIN32
  const char delims[] = "\\/";
#else
  const char delims[] = "/";
#endif

  const char * s = ss.c_str();
  const char * eos = s + ss.size();
  while ( eos > s && !strchr(delims, *(eos - 1)) ) {
    --eos;
  }

  return eos;
}


/* get file suffix from full path name */
std::string get_file_suffix(const std::string & pathname)
{
  size_t pos = pathname.find_last_of(".");
  if ( pos != std::string::npos ) {
    return pathname.substr(pos);
  }
  return std::string();
}

/* get file suffix from full path name (C-string version) */
const char * c_file_suffix(const char * fname)
{
  if ( fname ) {
    const char * s = strrchr(fname, '.' );
    return s ? s : s + strlen(s);
  }
  return nullptr;
}


/* add new or replace existng file suffix */
void set_file_suffix(std::string & pathname, const std::string & suffix)
{
  size_t pos = pathname.find_last_of('.');
  if ( pos == std::string::npos ) {
    pathname.append(suffix);
  }
  else {
    pathname.replace(pathname.begin() + pos, pathname.end(), suffix);
  }
}

/* get directory part from full path name */
std::string get_parent_directory(const std::string & fullpathname)
{
#ifdef _WIN32
  const char delims[] = "\\/";
#else
  const char delims[] = "/";
#endif
  size_t pos = fullpathname.find_last_of(delims);
  if ( pos != std::string::npos ) {
    return fullpathname.substr(0, pos);
  }
  return "";
}

/* check if path is absolute or relative */
bool is_absolute_path(const std::string & fullpathname)
{
  if ( fullpathname.size() < 1 ) {
    return false;
  }

  if ( fullpathname[0] == '/' || fullpathname[0] == '~' ) {
    return true;
  }

#ifdef _WIN32
  if ( fullpathname.size() > 1 && isalpha(fullpathname[0]) && fullpathname[1] == ':' ) {
    return true;
  }
#endif

  return false;
}

/* split the fullpathname to parent directory and file name */
void split_path(const std::string & fullpathname,
    std::string * parent_directory,
    std::string * file_name,
    bool prune_backslash_in_parrent_directory)
{

  if ( fullpathname.empty() ) {
    if ( file_name ) {
      *file_name = "";
    }
    if ( parent_directory ) {
      *parent_directory = "";
    }
  }
  else {

    // skip tailing slashes if are exists
    const std::string::const_iterator beg = fullpathname.begin();
    std::string::const_iterator end = fullpathname.end();

    while ( end > beg && *(end - 1) == '/' ) {
      --end;
    }

    if ( end == beg ) {
      if ( file_name ) {
        *file_name = "";
      }
      if ( parent_directory ) {
        *parent_directory = "/";
      }
    }
    else {

      // save position where tailing slashes begin (in order to drop them later)
      const std::string::const_iterator ends = end;

      // find last '/' in remaining data
      while ( end > beg && *(end - 1) != '/' ) {
        --end;
      }


      // A pointer could point to the former fullpathname, avoid the destroy by aliasing
      const std::string name = fullpathname.substr(end - beg, ends - end);
      const std::string directory = fullpathname.substr(0, end - beg);

      if ( file_name ) {
        *file_name = name;
      }

      if ( parent_directory ) {

        *parent_directory = directory;

        if ( prune_backslash_in_parrent_directory ) {
          while ( !parent_directory->empty() && parent_directory->back() == '/' && parent_directory->size() > 1 ) {
            parent_directory->pop_back();
          }
        }

      }
    }
  }
}

/* split the fullpathname to parent directory file name, and suffix */
void split_pathfilename(const std::string & fullpathname,
    std::string * parent_directory,
    std::string * file_name,
    std::string * file_suffix,
    bool prune_backslash_in_parrent_directory)
{
  if ( fullpathname.empty() ) {
    if ( parent_directory ) {
      *parent_directory = "";
    }
    if ( file_name ) {
      *file_name = "";
    }
    if ( file_suffix ) {
      *file_suffix = "";
    }
  }
  else {

    // skip tailing slashes if are exists
    const std::string::const_iterator beg = fullpathname.begin();
    std::string::const_iterator end = fullpathname.end();

    while ( end > beg && *(end - 1) == '/' ) {
      --end;
    }

    if ( end == beg ) {
      if ( parent_directory ) {
        *parent_directory = "/";
      }
      if ( file_name ) {
        *file_name = "";
      }
      if ( file_suffix ) {
        *file_suffix = "";
      }
    }
    else {

      // save position where tailing slashes begin (in order to drop them later)
      std::string::const_iterator ends = end;

      if ( *end == '/' ) { // don't extract file suffix from directory name

        // find last '/' in remaining data
        while ( end > beg && *(end - 1) != '/' ) {
          --end;
        }

      }
      else {

        // find last one of '.' or '/' in remaining data
        while ( end > beg && *(end - 1) != '.' && *(end - 1) != '/' ) {
          --end;
        }

        if ( *(end - 1) == '.' ) {
          if ( file_suffix ) {
            *file_suffix = fullpathname.substr(end - beg - 1, ends - end + 1);
          }

          // find last '/' in remaining data
          ends = end - 1;
          while ( end > beg && *(end - 1) != '/' ) {
            --end;
          }
        }
      }

      // A pointer could point to the former fullpathname, avoid the destroy by aliasing
      const std::string name = fullpathname.substr(end - beg, ends - end);
      const std::string directory = fullpathname.substr(0, end - beg);

      if ( file_name ) {
        *file_name = name;
      }

      if ( parent_directory ) {

        *parent_directory = directory;

        if ( prune_backslash_in_parrent_directory ) {
          while ( !parent_directory->empty() && parent_directory->back() == '/' && parent_directory->size() > 1 ) {
            parent_directory->pop_back();
          }
        }

      }
    }
  }
}


int64_t last_modification_timestamp(const std::string & abspath)
{
  struct stat st;
#ifdef _WIN32
  return stat(abspath.c_str(), &st) == -1 ? -1 :
      ((int64_t) st.st_mtime) * 1000000000;
#elif __APPLE__
  return stat(abspath.c_str(), &st) == -1 ? -1 :
      ((int64_t) st.st_mtimespec.tv_sec) * 1000000000 + st.st_mtimespec.tv_nsec; // double check logic
#else
  return stat(abspath.c_str(), &st) == -1 ? -1 :
      ((int64_t) st.st_mtim.tv_sec) * 1000000000 + st.st_mtim.tv_nsec;
#endif
}

static void split(const std::string & s, std::vector<std::string> * tokens, char delim)
{
  std::string::size_type curpos = 0;
  std::string::size_type nextpos = 0;

  while ( (nextpos = s.find_first_of(delim, curpos)) != std::string::npos ) {
    if ( nextpos > curpos ) {
      tokens->emplace_back(s.substr(curpos, nextpos - curpos));
    }
    curpos = nextpos + 1;
  }

  if ( curpos < s.size() ) {
    tokens->emplace_back(s.substr(curpos));
  }

}

/* check for existence of given file in coma-delimited search path */
std::string search_file(const std::string file_name,
    const std::string & search_path)
{
  std::vector<std::string> subdirs;
  std::string found_path_filename;

  split(search_path, &subdirs, ';');

  if ( subdirs.empty() ) {
    // lookup in current directory only
    if ( file_exists(file_name) ) {
      found_path_filename = file_name;
    }
  }
  else {
    for ( const std::string & subdir : subdirs ) {
      const std::string tmp = ssprintf("%s/%s", subdir.c_str(), file_name.c_str());
      if ( file_exists(tmp) ) {
        found_path_filename = tmp;
        break;
      }
    }
  }

  return found_path_filename;
}


/**
 * Get home directory of current user.
 */
std::string get_home_directory()
{
  const char * home;

  if ( (home = getenv("HOME")) ) {
    return home;
  }


  struct passwd pwd;
  struct passwd * result;
  char buf[PATH_MAX] = "";
  int status;

  status = getpwuid_r(getuid(), &pwd, buf, sizeof(buf), &result);

  if ( result ) {
    return result->pw_dir;
  }

  if ( status == 0 ) {
    errno = ENOENT;
  }
  else {
    errno = status;
  }

  return "";
}

/**
 * Expand home directory (~) symbol in path
 */
std::string expand_path(const std::string & path)
{
  if ( ~path.empty() && path[0] == '~' ) {
    std::string abspath = path;
    abspath.replace(0, 1, get_home_directory());
    return abspath;
  }

  return path;
}

/**
 *  readdir(std::vector<std::string> * filelist, const std::string & path,
 *     const std::string & filemask)
 *
 * Read the directory specified by path,
 *  search files matching filemask (if not empty),
 *    fill the filelist array with full path file names
 * Return:
 *  Number of filenames added to the filelist, or -1 on error.
 *  For the error code see the errno value.
 */
int readdir(std::vector<std::string> * list, const std::string & _path,
    const std::string & filemask, bool fullpathname, uint8_t d_type)
{
  DIR * dir = 0;
  struct dirent * e = NULL;
  std::vector<std::string> masks;

  const std::string path = _path.empty() ? "." : expand_path(_path);

  split(filemask, &masks, '|');

  int n = -1;

  if ( (dir = ::opendir(path.c_str())) ) {

    n = 0;

    while ( (e = ::readdir(dir)) ) {

      if ( strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0 ) {
        continue;
      }

#ifdef _DIRENT_HAVE_D_TYPE
      if ( d_type != DT_UNKNOWN && e->d_type != d_type ) {
        continue;
      }
#else
      if ( d_type != DT_UNKNOWN ) {
        char pathname[PATH_MAX];
        struct stat st;

        sprintf(pathname, "%s/%s", path.c_str(), e->d_name);
        if ( stat(pathname, &st) == -1 ) {
          continue;
        }

        switch (d_type ) {
          case DT_FIFO:
          if ( !S_ISFIFO(st.st_mode )) {
            continue;
          }
          break;
          case DT_CHR:
          if ( !S_ISCHR(st.st_mode )) {
            continue;
          }
          break;
          case DT_DIR:
          if ( !S_ISDIR(st.st_mode )) {
            continue;
          }
          break;
          case DT_BLK:
          if ( !S_ISBLK(st.st_mode )) {
            continue;
          }
          break;
          case DT_REG:
          if ( !S_ISREG(st.st_mode )) {
            continue;
          }
          break;
          case DT_LNK:
#ifndef S_ISLNK
          continue;
#else
          if ( !S_ISLNK(st.st_mode )) {
            continue;
          }
          break;
#endif
          case DT_SOCK:
#ifndef S_ISSOCK
          continue;
#else
          if ( !S_ISSOCK(st.st_mode )) {
            continue;
          }
          break;
#endif
          case DT_WHT:
          continue;
        }
      }
#endif

      if ( !masks.empty() ) {
        bool match = false;
        for ( size_t i = 0, n = masks.size(); i < n; ++i ) {
          if ( (match = (fnmatch(masks[i].c_str(), e->d_name, FNM_CASEFOLD) == 0)) ) {
            break;
          }
        }
        if ( !match ) {
          continue;
        }
      }

      if ( list ) {
        if ( !fullpathname ) {
          list->emplace_back(e->d_name);
        }
        else {
          char outfilename[PATH_MAX];
          sprintf(outfilename, "%s/%s", path.c_str(), e->d_name);
          list->emplace_back(outfilename);
        }
      }

      ++n;
    }

    ::closedir(dir);
  }

  return n;
}







/*
 * Remove all occurences of file names which match the specified pattern.
 * Used to implement an 'exclusion' mask
 * */
void filter_out(std::vector<std::string> & fnames, const std::string & filemask)
{
  size_t i = 0;
  std::vector<std::string> masks;

  split(filemask, &masks, '|');

  if ( !masks.empty() ) {

    while ( i < fnames.size() ) {

      bool matchFound = false;
      for ( size_t j = 0, n = masks.size(); j < n; ++j ) {
        if ( fnmatch(masks[j].c_str(), fnames[i].c_str(), FNM_CASEFOLD) == 0 ) {
          matchFound = true;
          break;
        }
      }

      if ( !matchFound ) {
        ++i;
      }
      else {
        fnames.erase(fnames.begin() + i);
      }
    }
  }
}


int rmfiles(const std::string & path, const std::string & filemask)
{
  std::vector<std::string> flist;
  int n;

  errno = 0;

  if ( (n = readdir(&flist, path, filemask, true)) < 1 ) {
    if ( n < 0 ) {
      CF_DEBUG("readdir(%s/%s) fails: %s", path.c_str(), filemask.c_str(), strerror(errno));
    }
    return n;
  }

  n = 0;
  for ( size_t i = 0; i < flist.size(); ++i ) {
    const char * fname = flist[i].c_str();
    if ( remove(fname) != 0 ) {
      CF_DEBUG("remove(%s) fails: %s", fname, strerror(errno));
    }
    else {
      ++n;
    }
  }

  return n;
}



static int recursive_rmdir_callback(const char * path, const struct stat * st, int typeflag, struct FTW * ftw)
{
  (void) ftw;
  (void) st;

  int status;

  // CF_DEBUG("typeflag=%d path='%s'", typeflag, path);

  switch ( typeflag ) {
    case FTW_F :
    case FTW_SL :
    case FTW_SLN :
#ifdef __APPLE__
    status = (unlink(path) == 0 ? 0 : 1);
#else
    status = (unlink(path) == 0 ? FTW_CONTINUE : FTW_STOP);
#endif
    break;

    case FTW_DP:  /* Directory, all subdirs have been visited. */
    case FTW_D :  /* Directory, must be not here really, because of FTW_DEPTH flag */
#ifdef __APPLE__
    status = (rmdir(path) == 0 ? 0 : 1);
#else
    status = (rmdir(path) == 0 ? FTW_CONTINUE : FTW_STOP);
#endif
    break;

    case FTW_DNR :
    case FTW_NS :
#ifdef __APPLE__
    status = 1;
#else
    status = FTW_STOP;
#endif
    break;

    default :
#ifdef __APPLE__
    status = 1;
#else
    status = FTW_STOP;
#endif
    errno = EINVAL;
    break;
  }

  return status;
}

/* removes non-empty directory recursively */
int recursive_rmdir(const std::string & path)
{
  if ( path.empty() || path == "/" ) { /* fixme : check carefully that we don't deleting the root !!! */
     errno = EINVAL;
     return -1;
  }
#ifdef __APPLE__
  return nftw(path.c_str(), recursive_rmdir_callback, 1, FTW_DEPTH | FTW_PHYS);
#else
  return nftw(path.c_str(), recursive_rmdir_callback, 1, FTW_ACTIONRETVAL | FTW_DEPTH | FTW_PHYS);
#endif
}


/**
 * Create directory recursively
 *  on error see the errno value
 * */

#ifdef _WIN32

static void copy_convert_slashes_to_unix(const char * src, char dst[])
{
  for ( ;*src; ++src ) {
    if ( *src != '/' && * src != '\\' )  {
      *dst++ = *src;
    }
    else if ( *dst != '/' ) {
      *dst++ = '/';
    }
  }
  *dst = 0;
}

bool create_path(const std::string & path, mode_t mode)
{
  (void)(mode);

  size_t size = path.size();
  char tmp[size + 1];
  char * p;
  char c;
  bool has_leading_drive_letter = false;

// CF_DEBUG("input: '%s'", path.c_str());

  memset(tmp, 0, size + 1); // mandatory before copy_convert_slashes_to_unix()
  copy_convert_slashes_to_unix(path.c_str(), tmp);

// CF_DEBUG("copy: '%s'", tmp);

  size = strlen(tmp);
  if ( tmp[size - 1] == '/' ) {
    tmp[size - 1] = 0;
  }

  if ( !*(p = tmp) ) {
    return true;
  }

  if ( isalpha(*p) && *(p + 1) == ':' ) {
    has_leading_drive_letter = true;
    p += 2;
  }

  errno = 0;

  for ( ; *p; p++ ) {
    if ( (c = *p) == '/' ) {
      if ( has_leading_drive_letter ) {
        has_leading_drive_letter = false;
      }
      else {
        *p = 0;

  //  CF_DEBUG("mkdir('%s')", tmp);
        if ( mkdir(tmp) != 0 && errno != EEXIST ) {
          CF_DEBUG("mkdir('%s') fails: %s", tmp, strerror(errno));
          return false;
        }
        *p = c;
      }
    }
  }

  if ( mkdir(tmp) != 0 && errno != EEXIST ) {
    CF_DEBUG("mkdir('%s') fails: %s", tmp, strerror(errno));
    return false;
  }

  return true;
}

#else

bool create_path(const std::string & input_path, mode_t mode)
{
  size_t size;
  std::string path = input_path;

  if ( path[0] == '~' ) {

    std::string home = get_home_directory();
    if ( home.empty() ) {
      return -1;
    }

    path.replace(0, 1, home);
  }

  char tmp[(size = path.size()) + 1];

  if ( strcpy(tmp, path.c_str())[size - 1] == '/' ) {
    tmp[size - 1] = 0;
  }

  errno = 0;
  for ( char * p = tmp + 1; *p; p++ ) {
    if ( *p == '/' ) {
      *p = 0;
      if ( mkdir(tmp, mode) != 0 && errno != EEXIST ) {
        return false;
      }
      *p = '/';
    }
  }

  return mkdir(tmp, mode) == 0 || errno == EEXIST ? true : false;
}

#endif


bool copy_file(const std::string & source_path_name, const std::string & target_path_name)
{
  FILE * fpsrc = NULL;
  FILE * fptgt = NULL;

  uint8_t buf[100 * 1042];
  size_t cb;

  bool fOK = false;

  if ( !(fpsrc = fopen(source_path_name.c_str(), "rb")) ) {
    CF_FATAL("fopen('%s') fails: %s", source_path_name.c_str(), strerror(errno));
    goto __end;
  }

  if ( !(fptgt = fopen(target_path_name.c_str(), "wb")) ) {
    CF_FATAL("fopen('%s') fails: %s", target_path_name.c_str(), strerror(errno));
    goto __end;
  }



  errno = 0;
  while ( (cb = fread(buf, 1, sizeof(buf), fpsrc)) ) {
    if ( fwrite(buf, 1, cb, fptgt) != cb ) {
      CF_FATAL("FATAL: fwrite(%s) fails: %s", target_path_name.c_str(), strerror(errno));
      goto __end;
    }
  }
  if ( ferror(fpsrc) ) {
    CF_FATAL("FATAL: fread(%s) fails: %s", source_path_name.c_str(), strerror(errno));
    goto __end;
  }


  fOK = true;

__end:;

  if ( fpsrc ) {
    fclose(fpsrc);
  }

  if ( fptgt ) {
    fclose(fptgt);
  }

  return fOK;
}


/*
 * Move file from source to destination.
 * */
bool move_file(const std::string & source_path_name,
    const std::string & target_path_name)
{
  errno = 0;

  if ( rename(source_path_name.c_str(), target_path_name.c_str()) == 0 ) {
    /* Success */
    return true;
  }

  if ( errno == EXDEV ) {
    /*
     * EXDEV: oldpath and newpath are not on the same mounted filesystem.
     *
     * Linux permits a filesystem to be mounted at multiple points,
     * but rename() does not work across different mount points,
     * even if the same filesystem is mounted on both.
     *
     * Try to copy_file() in this case.
     */
    return copy_file(source_path_name, target_path_name);
  }


  CF_FATAL("rename(%s --> %s) fails: %s", source_path_name.c_str(),
      target_path_name.c_str(), strerror(errno));

  return false;
}


/*
 * recursive copy directory and it's items
 * */
bool copy_path(const std::string & src, const std::string & dst, bool ignore_errors)
{
  // CF_DEBUG("    %s --> %s", src.c_str(), dst.c_str());

  if ( !is_directory(src) ) {

    if ( !create_path(get_parent_directory(dst)) ) {
      CF_FATAL("create_path(%s) fails: %s", dst.c_str(), strerror(errno));
      return false;
    }

    if ( !copy_file(src, dst) ) {
      CF_FATAL("copy_file(%s -> %s) fails: %s", src.c_str(), dst.c_str(), strerror(errno));
      return false;
    }

  }

  else {

    // copy directory content
    std::vector<std::string> dir_items;

    if ( readdir(&dir_items, src, "", false, DT_UNKNOWN) < 0 ) {
      CF_FATAL("readdir(%s) fails: %s", src.c_str(), strerror(errno));
      return false;
    }

    if ( !create_path(dst) ) {
      CF_FATAL("create_path(%s) fails: %s", dst.c_str(), strerror(errno));
      return false;
    }

    for ( const std::string & item : dir_items ) {
      const std::string src_full_path = ssprintf("%s/%s", src.c_str(), item.c_str());
      const std::string dst_full_path = ssprintf("%s/%s", dst.c_str(), item.c_str());
      if ( !copy_path(src_full_path, dst_full_path) && !ignore_errors ) {
        return false;
      }
    }
  }

  return true;
}


/* Return full path name to self executable */
std::string get_self_exe_pathname(void)
{
  char pathname[PATH_MAX] = "";
#ifdef _WIN32
  int rv = GetModuleFileNameA(GetModuleHandle(""), pathname, sizeof(pathname) - 1);
#else
  int rv = readlink("/proc/self/exe", pathname, sizeof(pathname));
#endif
  return rv >= 0 ? pathname : std::string();
}

/* Return directory name to self executable */
std::string get_self_exe_path(void)
{
  return get_parent_directory(get_self_exe_pathname());
}


