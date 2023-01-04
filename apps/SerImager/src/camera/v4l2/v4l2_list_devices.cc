/*
 * v4l2_list_devices.cc
 *
 *  Created on: Dec 22, 2022
 *      Author: amyznikov
 *
 * Based on v4l2-ctl list_devices()
 */

#include "v4l2_list_devices.h"

#include <map>
#include <algorithm>
#include <cstring>

#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/sysmacros.h>

#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/media.h>

static const char *prefixes[] = {
    "video",
    "radio",
    "vbi",
    "swradio",
    "v4l-subdev",
    "v4l-touch",
    "media",
    nullptr
};

static bool is_v4l_dev(const char * name)
{
  for( unsigned i = 0; prefixes[i]; i++ ) {
    unsigned l = strlen(prefixes[i]);

    if( !memcmp(name, prefixes[i], l) ) {
      if( isdigit(name[l]) )
        return true;
    }
  }
  return false;
}

static int calc_node_val(const char * s)
{
  int n = 0;

  s = std::strrchr(s, '/') + 1;

  for( unsigned i = 0; prefixes[i]; i++ ) {
    unsigned l = strlen(prefixes[i]);

    if( !memcmp(s, prefixes[i], l) ) {
      n = i << 8;
      n += atol(s + l);
      return n;
    }
  }
  return 0;
}

static bool sort_on_device_name(const std::string & s1, const std::string & s2)
{
  int n1 = calc_node_val(s1.c_str());
  int n2 = calc_node_val(s2.c_str());

  return n1 < n2;
}

bool v4l2_list_devices(std::vector<std::string> * filenames)
{
  using dev_vec = std::vector<std::string>;
  using dev_map = std::map<std::string, std::string>;

  DIR *dp;
  struct dirent *ep;
  dev_vec files;
  dev_map links;
  dev_map cards;
  struct v4l2_capability vcap;

  dp = opendir("/dev");
  if( dp == nullptr ) {
    perror("Couldn't open the directory");
    return false;
  }
  while ((ep = readdir(dp)))
    if( is_v4l_dev(ep->d_name) )
      files.push_back(std::string("/dev/") + ep->d_name);
  closedir(dp);

  /* Find device nodes which are links to other device nodes */
  for( auto iter = files.begin();
      iter != files.end(); ) {
    char link[64 + 1];
    int link_len;
    std::string target;

    link_len = readlink(iter->c_str(), link, 64);
    if( link_len < 0 ) { /* Not a link or error */
      iter++;
      continue;
    }
    link[link_len] = '\0';

    /* Only remove from files list if target itself is in list */
    if( link[0] != '/' ) /* Relative link */
      target = std::string("/dev/");
    target += link;
    if( std::find(files.begin(), files.end(), target) == files.end() ) {
      iter++;
      continue;
    }

    /* Move the device node from files to links */
    if( links[target].empty() )
      links[target] = *iter;
    else
      links[target] += ", " + *iter;
    iter = files.erase(iter);
  }

  std::sort(files.begin(), files.end(), sort_on_device_name);

  for( const auto &file : files ) {
    int fd = open(file.c_str(), O_RDWR);
    std::string bus_info;
    std::string card;

    if( fd < 0 )
      continue;
    int err = ioctl(fd, VIDIOC_QUERYCAP, &vcap);
    if( err ) {
      struct media_device_info mdi;

      err = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &mdi);
      if( !err ) {
        if( mdi.bus_info[0] )
          bus_info = mdi.bus_info;
        else
          bus_info = std::string("platform:") + mdi.driver;
        if( mdi.model[0] )
          card = mdi.model;
        else
          card = mdi.driver;
      }
    }
    else {
      bus_info = reinterpret_cast<const char*>(vcap.bus_info);
      card = reinterpret_cast<const char*>(vcap.card);
    }
    close(fd);
    if( err )
      continue;

    filenames->emplace_back(file.c_str());

// printf("ITEM: card='%s' file: '%s' links[file]: '%s'\n", card.c_str(), file.c_str(), links[file].c_str());

//    if( cards[bus_info].empty() )
//      cards[bus_info] += card + " (" + bus_info + "):\n";
//    cards[bus_info] += "\t" + file;
//    if( !(links[file].empty()) )
//      cards[bus_info] += " <- " + links[file];
//    cards[bus_info] += "\n";
  }

//  for( const auto &card : cards ) {
//    printf("first: '%s' second: '%s'\n", card.first.c_str(), card.second.c_str());
//  }

  return true;
}

