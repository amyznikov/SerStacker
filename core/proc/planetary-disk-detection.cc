/*
 * planetary-disk-detection.cc
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */
#include "autoclip.h"
#include "threshold.h"
#include "morphology.h"
#include "estimate_noise.h"
#include "geo-reconstruction.h"
#include "planetary-disk-detection.h"
#include <core/debug.h>

using namespace cv;

static bool get_maximal_connected_component(const Mat1b & src, cv::Rect * rc, cv::Mat * cmponent_mask, cv::Point2f * geometrical_center)
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
  }

  if ( cstats[0].area < 4 ) {
    CF_DEBUG("Small area: %d", cstats[0].area);
    return false;
  }

  rc->x = stats[cstats[0].label][CC_STAT_LEFT];
  rc->y = stats[cstats[0].label][CC_STAT_TOP];
  rc->width = stats[cstats[0].label][CC_STAT_WIDTH];
  rc->height = stats[cstats[0].label][CC_STAT_HEIGHT];

  if( cmponent_mask ) {
    cv::compare(labels, cstats[0].label, *cmponent_mask, cv::CMP_EQ);
  }

  if ( geometrical_center ) {

    geometrical_center->x = 0;
    geometrical_center->y = 0;
    int n = 0;

    for ( int y = 0; y < rc->height; ++y ) {
      for ( int x = 0; x < rc->width; ++x ) {
        if ( labels[rc->y + y][rc->x + x] == cstats[0].label ) {

          geometrical_center->x += (rc->x + x);
          geometrical_center->y = (rc->y + y);
          ++n;
        }
      }
    }

    geometrical_center->x /= n;
    geometrical_center->y /= n;

  }

  return true;
}


bool simple_planetary_disk_detector(cv::InputArray frame,
    cv::InputArray mask,
    cv::Point2f * out_centrold,
    double gbsigma,
    cv::Rect * optional_output_component_rect,
    cv::Mat * optional_output_cmponent_mask,
    cv::Point2f * optional_output_geometrical_center,
    cv::Mat * optional_output_debug_image)
{
  Mat src, gray;
  Mat1b comp;
  double noise;
  double threshold;

  Rect rc, rcc;
  Point2f center;
  Moments m;

  if ( (src = frame.getMat()).empty() ) {
    return false;
  }

  if ( src.channels() == 1 ) {
    src.copyTo(gray);
  }
  else if ( src.channels() == 4 ) {
    cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if ( gbsigma > 0 ) {
    GaussianBlur(gray, gray, cv::Size(0, 0), gbsigma);
  }

  autoclip(gray, mask, gray, 1, 99, 0, 255);
  gray.convertTo(gray, CV_8U);
  if ( optional_output_debug_image ) {
    gray.copyTo(*optional_output_debug_image);
  }


  cv::threshold(gray, comp, 0, 255, cv::THRESH_TRIANGLE);
  morphological_smooth_close(comp, comp, cv::Mat1b(5, 5, 255), cv::BORDER_CONSTANT);
  if ( !mask.empty() ) {
    comp.setTo(0, ~mask.getMat());
  }

  if ( !get_maximal_connected_component(comp, &rc, optional_output_cmponent_mask, optional_output_geometrical_center) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
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

