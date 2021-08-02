/*
 * small-planet-disk-detector.cc
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */
#include "small-planetary-disk-detector.h"
#include <core/proc/threshold.h>
#include <core/debug.h>

using namespace cv;

static bool get_maximal_connected_component(const Mat1b & src, cv::Rect * rc)
{
  Mat1i labels, stats;
  Mat1d centroids;
  int N;

  if ( (N = connectedComponentsWithStats(src, labels, stats, centroids, 8, labels.type())) < 2 ) {
    return false;
  }

  // CF_DEBUG("N=%d", N);

  struct ss {
    int label, area;
  };
  std::vector<ss> cstats;

  for ( int i = 1; i < N; ++i ) {
    cstats.emplace_back();
    cstats.back().label = i;
    cstats.back().area = stats[i][CC_STAT_AREA];
  }

  if ( cstats.size() > 1 ) {
    std::sort(cstats.begin(), cstats.end(),
        [](const ss & p, const ss & n) {
          return p.area > n.area;
        });

//    if ( cstats[0].area < 3 * cstats[1].area  ) {
//      CF_DEBUG("Bad second component");
//      return false;
//    }
  }

  if ( cstats[0].area < 4 ) {
    CF_DEBUG("Small area: %d", cstats[0].area);
    return false;
  }

  rc->x = stats[cstats[0].label][CC_STAT_LEFT];
  rc->y = stats[cstats[0].label][CC_STAT_TOP];
  rc->width = stats[cstats[0].label][CC_STAT_WIDTH];
  rc->height = stats[cstats[0].label][CC_STAT_HEIGHT];

  return true;
}


bool simple_small_planetary_disk_detector( cv::InputArray frame,
    cv::Point2f * out_centrold,
    double gbsigma,
    cv::Rect * optional_output_component_rect)
{
  Mat src, gray;
  Mat1b comp;
  //double gbsigma = 3.0;

  Rect rc, rcc;
  Point2f center;
  Moments m;

  if ( (src = frame.getMat()).empty() ) {
    return false;
  }

  if ( src.channels() == 1 ) {
    src.copyTo(gray);
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if ( gbsigma > 0 ) {
    GaussianBlur(gray, gray, cv::Size(0, 0), gbsigma);
  }

  cv::compare(gray, get_otsu_threshold(gray), comp, cv::CMP_GT);
  cv::morphologyEx(comp, comp, cv::MORPH_CLOSE, cv::Mat1b(5, 5, 255));

  if ( !get_maximal_connected_component(comp, &rc) ) {
    return false;
  }

  if ( optional_output_component_rect ) {
    *optional_output_component_rect = rc;
  }

  const int w = rc.width, h = rc.height;

  rc.x -= w / 4, rc.y -= h / 4, rc.width += w / 2, rc.height += h / 2;

  if ( rc.x < 0 ) {
    rc.x = 0;
  }
  if ( rc.y < 0 ) {
    rc.y = 0;
  }
  if ( rc.x + rc.width >= gray.cols ) {
    rc.width = gray.cols - rc.x -1;
  }
  if ( rc.y + rc.height >= gray.rows ) {
    rc.height = gray.rows - rc.y -1;
  }

  gray = gray(rc);

  subtract(gray, 0.5 * mean(gray), gray);
  gray.setTo(0, gray < 0);

  m = moments(gray, false);
  center = Point2f(m.m10 / m.m00, m.m01 / m.m00);

  if ( out_centrold ) {
    out_centrold->x = center.x + rc.x;
    out_centrold->y = center.y + rc.y;
  }

  return true;
}

