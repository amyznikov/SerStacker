/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/io/save_image.h>
#include <core/debug.h>

int main(int argc, char *argv[])
{

  cv::Mat2f optflow(256, 256);

  for ( int y = 0; y < optflow.rows; ++y ) {
    for ( int x = 0; x < optflow.cols; ++x ) {
      optflow[y][x][0] = (x - 0.5 * optflow.cols) / optflow.cols;
      optflow[y][x][1] = (y - 0.5 * optflow.rows) / optflow.rows;
    }
  }

  cv::writeOpticalFlow("test.flo", optflow);

  optflow.release();
  optflow = cv::readOpticalFlow("test.flo");

  fprintf(stdout, "optflow: %dx%d channels=%d depth=%d\n",
      optflow.cols, optflow.rows,
      optflow.channels(),
      optflow.depth());


  return 0;
}
