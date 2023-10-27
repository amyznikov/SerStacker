/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */
#include <core/io/c_ifhd_file.h>
#include <core/io/c_vlo_file.h>
#include <core/debug.h>

int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_ifhd_reader ifhd;

  if ( !ifhd.open("/mnt/data/vlo/20231019-085416_s_152_LUT_ACCU_v3_split-019.dat") ) {
    CF_ERROR("ifhd.open() fails");
    return 1;
  }


  for ( const auto & stream : ifhd.streams() ) {
    CF_DEBUG("stream '%s' : %zu frames", stream.header.stream_name,
        (size_t)stream.header.stream_index_count);
  }

  if ( !ifhd.select_stream("ScaLa 3-PointCloud") ) {
    CF_ERROR("Requested stream not found");
    return 1;
  }

  if ( !ifhd.seek(0) ) {
    CF_ERROR("ifhd.seek(0)");
    return 1;
  }

  uint8_t data[5000000];

  for ( int i = 0; i < 3; ++i ) {

    const ssize_t cb =
        ifhd.read_payload(data, sizeof(data));

    CF_DEBUG("ifhd.read_payload(%d): %zd bytes", i, cb);

    if ( cb <= 0 ) {
      CF_ERROR("ifhd.read_payload(i=%d) fails", i);
      break;
    }

  }


  return 0;
}


