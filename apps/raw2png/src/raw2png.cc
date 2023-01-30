/*
 * raw2png.cc
 *
 *  Created on: Jun 5, 2021
 *      Author: amyznikov
 */
#include <core/io/c_raw_file.h>
#include <core/io/save_image.h>
#include <core/proc/autoclip.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if !HAVE_LIBRAW

int main()
{
  fprintf(stderr, "raw2png\n"
      "This application is built with no libraw support.\n"
      "Install please libraw package (ubuntu: libraw-dev, archliux: libraw, windows: https://www.libraw.org/),\n"
      "reconfigure and rebuild the project.\n");

  return 1;
}

#else // HAVE_LIBRAW


static void create_gamma_lut16(double g, cv::Mat1w & lut)
{
  lut.create(65536, 1);

  for ( int i = 0; i < 65536; ++i ) {
    lut[i][0] = cv::saturate_cast<uint16_t>(65535 * pow(i / 65535., g));
  }
}

static void create_log_lut16(double g, cv::Mat1w & lut)
{
  lut.create(65536, 1);

  const double L = 65535. / log(1. + g);

  for ( int i = 0; i < 65536; ++i ) {
    lut[i][0] = cv::saturate_cast<uint16_t>(L * log( 1 + i * g / 65535.));
  }
}

static bool apply_lut6(const cv::Mat & src, const cv::Mat1w & lut, cv::Mat & dst )
{
  if ( src.depth() != CV_16U ) {
    return false;
  }

  if ( dst.data != src.data ) {
    dst.create(src.size(), src.type());
  }

  const int cn = src.channels();

  for ( int y = 0; y < src.rows; ++y ) {

    const uint16_t * srcp = src.ptr<const uint16_t>(y);
    uint16_t * dstp = dst.ptr<uint16_t>(y);

    for ( int x = 0, nx = src.cols * cn; x < nx; ++x ) {
      dstp[x] = lut[srcp[x]][0];
    }
  }

  return true;
}


void histogram_white_balance(cv::InputArray src, cv::OutputArray dst, double clo, double chi)
{
  double omin = 0, omax = 1;

  switch (src.depth()) {
    case CV_8U:
      omin = 0, omax = UINT8_MAX;
      break;
    case CV_8S:
      omin = INT8_MIN, omax = INT8_MAX;
      break;
    case CV_16U:
      omin = 0, omax = UINT16_MAX;
      break;
    case CV_16S:
      omin = INT16_MIN, omax = INT16_MAX;
      break;
    case CV_32S:
      omin = INT32_MIN, omax = INT32_MAX;
      break;
    case CV_32F:
      omin = 0, omax = 1;
      break;
    case CV_64F:
      omin = 0, omax = 1;
      break;
  }

  if ( src.channels() < 2 ) {

    autoclip(src,
        cv::noArray(),
        dst,
        clo, chi,
        omin, omax);
  }
  else {
    std::vector<cv::Mat> channels;

    cv::split(src, channels);

    for ( int i = 0, cn = channels.size(); i < cn; ++i ) {
      autoclip(channels[i],
          cv::noArray(),
          channels[i],
          clo, chi,
          omin, omax);
    }

    cv::merge(channels, dst);
  }
}


static bool color_saturation(const cv::Mat & src, cv::Mat & dst, double a, double b)
{
  if ( src.depth() != CV_16U ) {
    return false;
  }

  if ( src.channels() != 3 ) {
    src.copyTo(dst);
  }
  else {
    cv::Mat3f hsv;
    src.convertTo(hsv, CV_32F, 1. / 65535);

    cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);

    for ( int y = 0; y < hsv.rows; ++y ) {
      for ( int x = 0; x < hsv.cols; ++x ) {

        cv::Vec3f & v = hsv[y][x];

        v[1] = std::max(0., std::min(1., a + v[1] * b));
      }
    }

    cv::cvtColor(hsv, hsv, cv::COLOR_HSV2BGR);
    hsv.convertTo(dst, CV_16U, 65535);
  }

  return true;
}

int main(int argc, char *argv[])
{


  std::vector<std::string> input_file_names;
  std::string output_path;
  bool downscale = false;

  double gamma = 1;

  double clog = 0;


  bool awb = false;
  double awbl = 0.1, awbh = 99.9;

  bool cs  = false;
  double csa = 0, csb = 1.1;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   raw2png [OPTIONS] input_file1 [input_file2 ...] \n"
          "\n"
          "OPTIONS:\n"
          "  -o <output-directory>\n"
          "\n"
          "  -d \n"
          "    2x downscale using pyrDown()\n"
          "\n"
          "  -g <gamma>\n"
          "    apply gamma correction. \n"
          "     Symbolic values: adobe, srgb\n"
          "\n"
          "  -l <gamma>\n"
          "    apply log correction. \n"
          "\n"
          "  -cs <a:b>\n"
          "    color saturation. \n"
          "\n"
          "  -awb <clo:chi>\n"
          "\n"

          "\n");

      return 0;
    }

    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Output directory name expected: %s\n", argv[i - 1]);
        return 1;
      }
      output_path = argv[i];

      if ( is_regular_file(output_path) ) {
        fprintf(stderr, "Requested output path is already existing regular file: %s\n", argv[i]);
        return 1;
      }
    }
    else if ( strcmp(argv[i], "-d") == 0 ) {
      downscale = true;
    }
    else if ( strcmp(argv[i], "-g") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Missing gamma value after %s\n", argv[i - 1]);
        return 1;
      }

      if ( strcasecmp(argv[i], "adobe")  == 0 ) {
        gamma = 0.45470692717584369449;
      }
      else if ( strcasecmp(argv[i], "srgb")  == 0 ) {
        gamma = 1./2.4;
      }
      else if ( sscanf(argv[i], "%lf", &gamma) != 1 ) {
        fprintf(stderr, "Invalid gamma value specified: %s\n", argv[i]);
        return 1;
      }
    }

    else if ( strcmp(argv[i], "-l") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Missing l value after %s\n", argv[i - 1]);
        return 1;
      }

      if ( sscanf(argv[i], "%lf", &clog) != 1 ) {
        fprintf(stderr, "Invalid l value specified: %s\n", argv[i]);
        return 1;
      }
    }

    else if ( strcmp(argv[i], "-awb") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Missing awb clips after %s\n", argv[i - 1]);
        return 1;
      }

      if ( sscanf(argv[i], "%lf:%lf", &awbl, &awbh) != 2 ) {
        fprintf(stderr, "Invalid awb clips specified: %s\n", argv[i]);
        return 1;
      }

      awb = true;
    }

    else if ( strcmp(argv[i], "-cs") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Missing cs params after %s\n", argv[i - 1]);
        return 1;
      }

      if ( sscanf(argv[i], "%lf:%lf", &csa, &csb) != 2 ) {
        fprintf(stderr, "Invalid cs params specified: %s\n", argv[i]);
        return 1;
      }

      cs = true;
    }

    else if ( is_regular_file(argv[i]) ) {
      input_file_names.emplace_back(argv[i]);
    }
    else {
      fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
      return 1;
    }
  }

  if ( input_file_names.empty() ) {
    fprintf(stderr, "No input FITS file specified\n");
    return 1;
  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_raw_file_reader raw_file;
  cv::Mat current_image;
  cv::Mat1w glut16, llut16;

  std::vector<int> jpeg_params;
  jpeg_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
  jpeg_params.emplace_back(100);


  raw_file.set_auto_apply_channel_multipliers(true);

  enum COLORID colorid = COLORID_UNKNOWN;
  int bpc = -1;

  for ( const std::string & current_path_file_name : input_file_names ) {

    std::string current_file_name =
        get_file_name(current_path_file_name);

    CF_DEBUG("[%s] ...", current_file_name.c_str());

    if ( !raw_file.read(current_path_file_name, current_image, &colorid, &bpc) ) {
      CF_ERROR("raw_file.read(%s) fails", current_file_name.c_str());
      continue;
    }

    if ( is_bayer_pattern(colorid) && !debayer(current_image, current_image, colorid, DEBAYER_NN2)  ) {
      CF_ERROR("debayer(%s) fails", current_file_name.c_str());
      continue;
    }

    if ( raw_file.has_color_matrix() ) {
      cv::transform(current_image, current_image,
          raw_file.color_matrix());
    }

    if ( downscale ) {
      cv::pyrDown(current_image, current_image);
    }

    if ( awb ) {
      histogram_white_balance(current_image, current_image,
          awbl, awbh);
    }

    if ( gamma != 1 && gamma != 0 ) {

      if ( current_image.depth() == CV_16U ) {
        if ( glut16.empty() ) {
          create_gamma_lut16(gamma, glut16);
        }
        apply_lut6(current_image, glut16, current_image);
      }
      else if ( current_image.depth() == CV_32F ) {
        cv::pow(current_image, gamma, current_image);
      }
    }

    if ( clog > 0  ) {
      if ( current_image.depth() == CV_16U ) {
        if ( llut16.empty() ) {
          create_log_lut16(clog, llut16);
        }
        apply_lut6(current_image, llut16, current_image);
      }
    }

    if ( cs ) {
      color_saturation(current_image, current_image, csa, csb);
    }


    if ( output_path.empty() ) {
      output_path = "raw2png";
    }

    if ( !create_path(output_path) ) {
      CF_ERROR("create_path(%s) fails : %s", output_path.c_str(), strerror(errno));
      return 1;
    }

    set_file_suffix(current_file_name, ".jpg");
    current_file_name = ssprintf("%s/%s", output_path.c_str(), current_file_name.c_str());

    cv::normalize(current_image, current_image, 0, 255, cv::NORM_MINMAX, CV_8U);

    if ( !save_image(current_image, current_file_name, jpeg_params) ) {
      CF_ERROR("save_image(%s) fails : %s", output_path.c_str(), strerror(errno));
      return 1;
    }

    CF_DEBUG("OK");
  }


  return 0;
}

#endif // HAVE_LIBRAW

