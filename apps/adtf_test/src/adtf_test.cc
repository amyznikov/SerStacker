/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */
#include "c_ifhd_file.h"
#include <core/debug.h>

int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  c_ifhd_reader ifhd;

  if ( !ifhd.open("/mnt/data/vlo/") ) {

  }

  return 0;
}


