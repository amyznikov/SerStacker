/*
 * readdir.h
 *
 *  Created on: Nov 8, 2016
 *      Author: amyznikov
 */
#pragma once
#ifndef __readdir_h__
#define __readdir_h__

#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#else
#include <dirent.h>
#endif // _MSC_VER

///////////////////////////////////////////////////////////////////////////////


#ifdef _WIN32
/* copied from linux version of dirent.h */
enum {
    DT_UNKNOWN = 0,
# define DT_UNKNOWN	DT_UNKNOWN
    DT_FIFO = 1,
# define DT_FIFO	DT_FIFO
    DT_CHR = 2,
# define DT_CHR		DT_CHR
    DT_DIR = 4,
# define DT_DIR		DT_DIR
    DT_BLK = 6,
# define DT_BLK		DT_BLK
    DT_REG = 8,
# define DT_REG		DT_REG
    DT_LNK = 10,
# define DT_LNK		DT_LNK
    DT_SOCK = 12,
# define DT_SOCK	DT_SOCK
    DT_WHT = 14
# define DT_WHT		DT_WHT
};

#endif

#ifdef _WIN32
  typedef int mode_t;
# define DEFAULT_MKDIR_MODE 0
# else
# define DEFAULT_MKDIR_MODE \
    (S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH)
#endif


/* check for file existence */
bool file_exists(const std::string & pathname);

/* check for directory existence */
bool is_directory(const std::string & path);

/* check for regular file existence */
bool is_regular_file(const std::string & path);

/* check for link existence */
bool is_link(const std::string & path);

/* check for read access */
bool file_readable(const std::string & path);

/* get file name part from full path name */
std::string get_file_name(const std::string & fullpathname);

/* get file name part from full path name (C-string version)*/
const char * c_file_name(const char * s);

/* get file name part from full path name (C-string version)*/
const char * c_file_name(const std::string & s);

/**
 * @brief Returns filename stem
 * 
 * @param pathname full/short filename
 * @param depth number of suffixes to be dropped (-1 for all suffixes)
 * @return std::string 
 */
std::string stem(const std::string &pathname, int depth = 1);

/* get file suffix from full path name */
std::string get_file_suffix(const std::string & pathname);

///* get file suffix from full path name (C-string version) */
//const char * c_file_suffix(const char * fname);

/* add new or replace existng file suffix */
void set_file_suffix(std::string & pathname, const std::string & suffix);

/* get directory part from full path name */
std::string get_parent_directory(const std::string & fullpathname);

/* check if path is absolute or relative */
bool is_absolute_path(const std::string & fullpathname);

/* split the fullpathname to parent directory and file name */
void split_path(const std::string & fullpathname,
    std::string * parent_directory,
    std::string * file_name,
    bool prune_backslash_in_parrent_directory = true);

/* split the fullpathname to parent directory file name, and suffix */
void split_pathfilename(const std::string & fullpathname,
    std::string * parent_directory,
    std::string * file_name,
    std::string * file_suffix,
    bool prune_backslash_in_parrent_directory = true);

/**
 * Get home directory of current user.
 */
std::string get_home_directory();

/**
 * Expand home directory (~) symbol in path
 */
std::string expand_path(const std::string & path);

/* Time of last modification in nanosec */
int64_t last_modification_timestamp(const std::string & abspath);

/* check for existence of given file in coma-delimited search path */
std::string search_file(const std::string file_name,
    const std::string & search_path);

/**
 * Create directory recursively
 *  on error see the errno value
 * */
bool create_path(const std::string & path, mode_t mode = DEFAULT_MKDIR_MODE);


/*
 * move file from source to destination
 * */
bool move_file(const std::string & source_path_name,
    const std::string & target_path_name);


/*
 * copy file from source to destination
 * */
bool copy_file(const std::string & source_path_name,
    const std::string & target_path_name);


/*
 * recursive copy directory and it's items
 * */
bool copy_path(const std::string & src, const std::string & dst,
    bool ignore_errors = false);



/**
 * Search directory contents for specified file mask.
 * Return number of items added to list.
 * On error -1 returned, see errno for details
 */
int readdir(std::vector<std::string> * list, const std::string & path,
    const std::string & filemask = std::string(),
    bool fullpathname = true,
    uint8_t d_type = DT_UNKNOWN);

/*
 * Remove all occurences of file names which match the specified pattern.
 * Used to implement an 'exclusion' mask
 * */
void filter_out(std::vector<std::string> & filenames,
    const std::string & filemask);

/* removes regular files from directory using specified file name pattern */
int rmfiles(const std::string & path, const std::string & filemask);


/* recursive remove non-empty directory */
int recursive_rmdir(const std::string & path);



/*
 * Return full path name to self executable
 * */
std::string get_self_exe_pathname(void);

/*
 * Return directory name to self executable
 * */
std::string get_self_exe_path(void);

/*
 * Return command line of the self executable
 * */
std::string get_self_exe_cmdline(void);

///////////////////////////////////////////////////////////////////////////////
#endif /* __readdir_h__ */
