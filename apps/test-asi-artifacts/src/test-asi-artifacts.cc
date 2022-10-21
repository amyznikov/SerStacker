/*
 * test-asi-artifacts.cc
 *
 *  Created on: Aug 20, 2022
 *      Author: amyznikov
 */

#include <core/io/c_ser_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/io/save_image.h>
#include <core/debug.h>



int main(int argc, char *argv[])
{
  std::string input_file_name;
  int input_frame_index = 0;

  cv::Mat image;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   test-asi-artifacts [OPTIONS] input_file.ser -f input_frame_index\n"
          "\n"
          "OPTIONS:\n"
          " \n"
      );

      return 0;
    }

    if ( strcmp(argv[i], "-f") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Input frame index expected: %s\n", argv[i - 1]);
        return 1;
      }

      if ( sscanf(argv[i], "%d", &input_frame_index) != 1 || input_frame_index < 0 ) {
        fprintf(stderr, "Invalid frame index specified: %s\n", argv[i]);
        return 1;
      }
    }

    else if ( input_file_name.empty() && is_regular_file(argv[i]) ) {
      input_file_name = argv[i];
    }

    else {
      fprintf(stderr, "Invalid argument or not existing input file specified: %s\n", argv[i]);
      return 1;
    }
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input SER or AVI file specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);



  if ( strcasecmp(get_file_suffix(input_file_name).c_str(), ".ser") == 0 ) {

    c_ser_reader ser_reader;

    if( !ser_reader.open(input_file_name) ) {
      fprintf(stderr, "ser_reader.open(%s) fails: %s\n",
          input_file_name.c_str(),
          strerror(errno));
      return 1;
    }

    if( !ser_reader.seek(input_frame_index) ) {
      fprintf(stderr, "ser_reader.seek(input_frame_index=%d) fails: %s\n",
          input_frame_index,
          strerror(errno));
      return 1;
    }


    if ( !ser_reader.read(image) ) {
      fprintf(stderr, "ser_reader.read(input_frame_index=%d) fails: %s\n",
          input_frame_index,
          strerror(errno));
      return 1;
    }

    CF_DEBUG("input: %dx%d channels=%d depth=%d", image.cols, image.rows, image.channels(), image.depth());

    extract_bayer_planes(image, image, ser_reader.color_id());

    CF_DEBUG("planes: %dx%d channels=%d depth=%d", image.cols, image.rows, image.channels(), image.depth());


    std::vector<cv::Mat> planes;
    cv::split(image, planes);

    for ( int i = 0; i < image.channels(); ++i ) {

      static float K[2] = {
          -1, 1
      };

      save_image(planes[i], ssprintf("debug/planes/plane.%d.tiff", i));


      cv::filter2D(planes[i], planes[i], CV_32F, cv::Mat1f(2,1, K), cv::Point(0,0), 0, cv::BORDER_REFLECT);

      save_image(planes[i], ssprintf("debug/planes/diff.%d.tiff", i));

      cv::reduce(planes[i], planes[i], cv::REDUCE_AVG, 1, CV_32F);

      save_image(planes[i], ssprintf("debug/planes/avg.%d.tiff", i));

      double minVal, maxVal;
      cv::Scalar avgVal, stdVal;
      bool is_bad_frame;

      cv::minMaxLoc(planes[i], &minVal, &maxVal);
      cv::meanStdDev(planes[i], avgVal, stdVal);

      is_bad_frame = std::max(fabs(minVal), fabs(maxVal)) / stdVal[0] > 10;


      CF_DEBUG("plane[%d]: minVal=%g maxVal=%g avgVal=%g stdVal=%g BAD=%d", i, minVal, maxVal, avgVal[0], stdVal[0], is_bad_frame);


    }

  }
  else {
    c_ffmpeg_reader ffmpeg_reader;

    if( !ffmpeg_reader.open(input_file_name) ) {
      fprintf(stderr, "ffmpeg_reader.open(%s) fails: %s\n",
          input_file_name.c_str(),
          strerror(errno));
      return 1;
    }

    if( !ffmpeg_reader.seek_frame(input_frame_index) ) {
      fprintf(stderr, "ffmpeg_reader.seek_frame(input_frame_index=%d) fails: %s\n",
          input_frame_index,
          strerror(errno));
      return 1;
    }


    if ( !ffmpeg_reader.read(image) ) {
      fprintf(stderr, "ser_reader.read(input_frame_index=%d) fails: %s\n",
          input_frame_index,
          strerror(errno));
      return 1;
    }

    std::vector<cv::Mat> planes;

    cv::split(image, planes);

    for ( int i = 0; i < image.channels(); ++i ) {

      static float K[2] = {
          -1, 1
      };

      save_image(planes[i], ssprintf("debug/planes/plane.%d.tiff", i));


      cv::filter2D(planes[i], planes[i], CV_32F, cv::Mat1f(2,1, K), cv::Point(0,0), 0, cv::BORDER_REFLECT);

      save_image(planes[i], ssprintf("debug/planes/diff.%d.tiff", i));

      cv::reduce(planes[i], planes[i], cv::REDUCE_AVG, 1, CV_32F);

      save_image(planes[i], ssprintf("debug/planes/avg.%d.tiff", i));

      double minVal, maxVal;
      cv::Scalar avgVal, stdVal;
      bool is_bad_frame;

      cv::minMaxLoc(planes[i], &minVal, &maxVal);
      cv::meanStdDev(planes[i], avgVal, stdVal);

      is_bad_frame = std::max(fabs(minVal), fabs(maxVal)) / stdVal[0] > 10;


      CF_DEBUG("plane[%d]: minVal=%g maxVal=%g avgVal=%g stdVal=%g BAD=%d", i, minVal, maxVal, avgVal[0], stdVal[0], is_bad_frame);
    }
  }


  return 0;
}
