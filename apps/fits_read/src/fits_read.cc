/*
 * fits_read.cc
 *
 *  Created on: Jan 9, 2021
 *      Author: amyznikov
 */

#include <core/io/c_fits_file.h>
#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/debug.h>

#if !HAVE_CFITSIO

int main()
{
  fprintf(stderr, "fits_read\n"
      "This application is built with no cfitsio support.\n"
      "Install please libraw package (ubuntu: libcfitsio-dev, archliux: cfitsio, windows: https://github.com/healpy/cfitsio),\n"
      "reconfigure and rebuild the project.\n");

  return 1;
}

#else // HAVE_CFITSIO


int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_file_name;
  std::string output_path;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   fits_read [OPTIONS] input_file.fits [-o output_file.tiff]\n"
          "\n"
          "OPTIONS:\n"
          "\n"
      );

      return 0;
    }

    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
        return 1;
      }
      output_file_name = argv[i];
    }

    else if ( input_file_name.empty() && is_regular_file(argv[i]) ) {
      input_file_name = argv[i];
    }

    else {
      fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
      return 1;
    }
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input FITS file specified\n");
    return 1;
  }

  if ( input_file_name == output_file_name ) {
    fprintf(stderr, "Input and Output file names can not be same\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_fits_reader fits;

  if ( !fits.open(input_file_name) ) {
    fprintf(stderr, "fits.open(%s) fails: %s\n",
        input_file_name.c_str(),
        fits.statusmsg());
    return 1;
  }

  const std::vector<c_fits_file::FKEY> & header =
      fits.header();

  for ( uint i = 0, n = header.size(); i < n; ++i ) {
    printf("%4d %s = %s  // %s\n", i,
        header[i].keyname.c_str(),
        header[i].value.c_str(),
        header[i].comment.c_str());
  }

  printf("\n");
  printf("num_hdus=%d\n", fits.num_hdus());

  cv::Mat image;
  if ( !fits.read(image) ) {
    CF_ERROR("fits.read() fails: %s", fits.statusmsg());
    return  1;
  }

  printf("image: %dx%dx%d depth=%d\n",
      image.cols, image.rows,
      image.channels(),
      image.depth());


  if ( output_file_name.empty() ) {
    output_file_name = "output.tiff";
  }

  save_image(image, output_file_name);


  return 0;
}

#endif // HAVE_CFITSIO
