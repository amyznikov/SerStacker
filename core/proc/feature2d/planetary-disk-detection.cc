/*
 * planetary-disk-detection.cc
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */
//#include <core/proc/autoclip.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/threshold.h>
#include <core/proc/morphology.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/geo-reconstruction.h>
#include "planetary-disk-detection.h"
#include <core/debug.h>

/**
 * Given binary mask src this routine extracts the list of connected components,
 *  selects maximal one by area and returns its bounding rectangle and mask
 *  */
static bool get_maximal_connected_component(const cv::Mat1b & src,
    cv::Rect * bounding_rect,
    cv::Mat * component_mask = nullptr,
    cv::Point2f * geometrical_center = nullptr)
{
  cv::Mat1i labels, stats;
  cv::Mat1d centroids;
  int N;

  if( (N = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, labels.type())) < 2 ) {
    return false;
  }

  struct ss
  {
    int label, area;
  };

  std::vector<ss> cstats;

  for( int i = 1; i < N; ++i ) {
    cstats.emplace_back();
    cstats.back().label = i;
    cstats.back().area = stats[i][cv::CC_STAT_AREA];
  }

  if( cstats.size() > 1 ) {
    std::sort(cstats.begin(), cstats.end(),
        [](const ss & p, const ss & n) {
          return p.area > n.area;
        });
  }

  if( cstats[0].area < 4 ) {
    CF_DEBUG("Small area: %d", cstats[0].area);
    return false;
  }

  if( component_mask ) {
    cv::compare(labels, cstats[0].label,
        *component_mask,
        cv::CMP_EQ);
  }

  if( bounding_rect || geometrical_center ) {

    const cv::Rect rc(stats[cstats[0].label][cv::CC_STAT_LEFT], stats[cstats[0].label][cv::CC_STAT_TOP],
        stats[cstats[0].label][cv::CC_STAT_WIDTH], stats[cstats[0].label][cv::CC_STAT_HEIGHT]);

    if( bounding_rect ) {
      *bounding_rect = rc;
    }

    if( geometrical_center ) {

      double cx = 0, cy = 0;
      int n = 0;

      for( int y = 0; y < rc.height; ++y ) {
        for( int x = 0; x < rc.width; ++x ) {
          if( labels[rc.y + y][rc.x + x] == cstats[0].label ) {
            cx += x, cy += y;
            ++n;
          }
        }
      }

      geometrical_center->x = rc.x + cx / n;
      geometrical_center->y = rc.y + cy / n;
    }
  }

  return true;
}

bool simple_planetary_disk_detector(cv::InputArray frame, cv::InputArray mask,
    double gsigma, int se_radius,
    cv::Point2f * output_centroid,
    cv::Rect * optional_output_component_rect,
    cv::Mat * optional_output_cmponent_mask,
    cv::Point2f * optional_output_geometrical_center)
{
  cv::Mat src, gray, mgrad;
  cv::Mat1b comp;

  cv::Rect rc, rcc;
  cv::Point2f center;
  cv::Moments m;

  if ( (src = frame.getMat()).empty() ) {
    return false;
  }

  if ( src.channels() == 1 ) {
    autoClip(src, mask, gray, 0.01, 0.9999, 0, 255, CV_8U);
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    autoClip(gray, mask, gray, 0.01, 0.9999, 0, 255, CV_8U);
  }

  if ( gsigma > 0 ) {
    GaussianBlur(gray, gray, cv::Size(0, 0), gsigma, gsigma);
  }

  const cv::Size se1_size(2 * se_radius + 1, 2 * se_radius + 1);
  static thread_local cv::Mat1b SE1;
  if ( SE1.size() != se1_size ) {
    SE1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, se1_size);
  }
  cv::morphologyEx(gray, mgrad, cv::MORPH_GRADIENT, SE1, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
  cv::compare(mgrad, get_otsu_threshold(mgrad, mask), comp, cv::CMP_GE);
  morphological_smooth_close(comp, comp, SE1, cv::BORDER_REPLICATE);
  //cv::morphologyEx(comp, comp, cv::MORPH_CLOSE, SE1, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);

  geo_fill_holes(comp, comp, 8);
  if ( !mask.empty() ) {
    comp.setTo(0, ~mask.getMat());
  }

  if( !get_maximal_connected_component(comp, &rc, &comp, optional_output_geometrical_center) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
    return false;
  }

  if( optional_output_component_rect ) {
    *optional_output_component_rect = rc;
  }

  if( output_centroid ) {

    const int w = rc.width;
    const int h = rc.height;

    rc.x -= w / 4;
    rc.y -= h / 4;
    rc.width += w / 2;
    rc.height += h / 2;

    if( rc.x < 0 ) {
      rc.x = 0;
    }
    if( rc.y < 0 ) {
      rc.y = 0;
    }
    if( rc.x + rc.width >= gray.cols ) {
      rc.width = gray.cols - rc.x - 1;
    }
    if( rc.y + rc.height >= gray.rows ) {
      rc.height = gray.rows - rc.y - 1;
    }

    gray.setTo(0, ~comp);
    gray = gray(rc);

    m = moments(gray, false);
    center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);

    output_centroid->x = center.x + rc.x;
    output_centroid->y = center.y + rc.y;
  }

  if ( optional_output_cmponent_mask ) {
    *optional_output_cmponent_mask = std::move(comp);
  }


  return true;
}

bool serialize_base_planetary_disk_detector_options(c_config_setting section, bool save,
    c_simple_planetary_disk_detector_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, gbsigma);
  //SERIALIZE_OPTION(section, save, opts, stdev_factor);
  SERIALIZE_OPTION(section, save, opts, se_radius);
  return true;
}
