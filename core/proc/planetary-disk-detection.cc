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

      geometrical_center->x = 0;
      geometrical_center->y = 0;
      int n = 0;

      for( int y = 0; y < rc.height; ++y ) {
        for( int x = 0; x < rc.width; ++x ) {
          if( labels[rc.y + y][rc.x + x] == cstats[0].label ) {

            geometrical_center->x += (rc.x + x);
            geometrical_center->y = (rc.y + y);
            ++n;
          }
        }
      }

      geometrical_center->x /= n;
      geometrical_center->y /= n;
    }
  }

  return true;
}



bool simple_planetary_disk_detector(cv::InputArray frame, cv::InputArray mask,
    double gbsigma,
    double stdev_factor,
    int close_radius,
    cv::Point2f * output_centroid,
    cv::Rect * optional_output_component_rect,
    cv::Mat * optional_output_cmponent_mask,
    cv::Point2f * optional_output_geometrical_center,
    cv::Mat * optional_output_debug_image)
{
  cv::Mat src, gray;
  cv::Mat1b comp;
  double noise;
  double threshold;

  cv::Rect rc, rcc;
  cv::Point2f center;
  cv::Moments m;

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

  if ( close_radius > 0 ) {

    const int close_size =
        2 * close_radius + 1;

    morphological_smooth_close(comp, comp, cv::Mat1b(close_size, close_size, 255),
        cv::BORDER_CONSTANT);
  }

  if ( !mask.empty() ) {
    comp.setTo(0, ~mask.getMat());
  }

  if ( !get_maximal_connected_component(comp, &rc, &comp, nullptr) ) {
    CF_DEBUG("get_maximal_connected_component() fails");
    return false;
  }

  if( stdev_factor > 0 ) {

    cv::Scalar m, s;
    double min = 0, max = 0;

    cv::meanStdDev(gray, m, s, mask);
    cv::minMaxLoc(gray, &min, &max, nullptr, nullptr, mask);

    const double threshold = s[0] * stdev_factor;

    // CF_DEBUG("s[0]=%g threshold=%g min=%g max=%g", s[0], threshold,  min, max);

    cv::bitwise_and(comp, gray > threshold, comp);
  }

  morphological_smooth_close(comp, comp, cv::Mat1b(3, 3, 255));
  geo_fill_holes(comp, comp, 8);

  if ( !get_maximal_connected_component(comp, &rc, optional_output_cmponent_mask, optional_output_geometrical_center) ) {
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

    gray = gray(rc);

    subtract(gray, 0.5 * mean(gray), gray);
    gray.setTo(0, gray < 0);

    m = moments(gray, false);
    center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);

    output_centroid->x = center.x + rc.x;
    output_centroid->y = center.y + rc.y;
  }

  return true;
}
//
//
//bool detect_planetary_disk(cv::InputArray input_image, cv::InputArray input_mask,
//    double gbsigma,
//    double stdev_factor,
//    cv::Mat * output_component_mask,
//    cv::Rect * output_component_rect = nullptr)
//{
//  cv::Mat src, gray;
//  cv::Mat1b comp;
//  double noise;
//  double threshold;
//
//  cv::Rect rc, rcc;
//
//  if( (src = input_image.getMat()).empty() ) {
//    return false;
//  }
//
//  if( src.channels() == 1 ) {
//    src.copyTo(gray);
//  }
//  else if( src.channels() == 4 ) {
//    cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
//  }
//  else {
//    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
//  }
//
//  if( gbsigma > 0 ) {
//    GaussianBlur(gray, gray, cv::Size(0, 0), gbsigma);
//  }
//
//  autoclip(gray, input_mask, gray, 1, 99, 0, 255);
//  gray.convertTo(gray, CV_8U);
//
//  cv::threshold(gray, comp, 0, 255, cv::THRESH_TRIANGLE);
//
//  morphological_smooth_close(comp, comp,
//      cv::Mat1b(5, 5, 255),
//      cv::BORDER_CONSTANT);
//
//  if( !input_mask.empty() ) {
//    comp.setTo(0, ~input_mask.getMat());
//  }
//
//  if( !get_maximal_connected_component(comp, &rc, &comp) ) {
//    CF_DEBUG("get_maximal_connected_component() fails");
//    return false;
//  }
//
//  if( stdev_factor > 0 ) {
//
//    cv::Scalar m, s;
//    double min = 0, max = 0;
//
//    cv::meanStdDev(gray, m, s, input_mask);
//    cv::minMaxLoc(gray, &min, &max, nullptr, nullptr, input_mask);
//
//    const double threshold = s[0] * stdev_factor;
//
//    // CF_DEBUG("s[0]=%g threshold=%g min=%g max=%g", s[0], threshold,  min, max);
//
//    cv::bitwise_and(comp, gray > threshold, comp);
//  }
//
//  morphological_smooth_close(comp, comp, cv::Mat1b(3, 3, 255));
//  geo_fill_holes(comp, comp, 8);
//
//  if( !get_maximal_connected_component(comp, &rc, output_component_mask) ) {
//    CF_DEBUG("get_maximal_connected_component() fails");
//    return false;
//  }
//
//  if( output_component_rect ) {
//    *output_component_rect = rc;
//  }
//
//  return true;
//}
//

bool detect_saturn(cv::InputArray _image, int se_close_radius, cv::RotatedRect & output_bbox, cv::OutputArray output_mask,
    double gbsigma, double stdev_factor)
{

  cv::Point2f centroid;
  cv::Point2f geometrical_center;

  cv::Mat1b component_mask;
  cv::Mat1b rotated_component_mask;
  cv::Mat pts;

  bool fOk =
      simple_planetary_disk_detector(_image, cv::noArray(),
          gbsigma,
          stdev_factor,
          se_close_radius,
          &centroid,
          nullptr,
          &component_mask,
          &geometrical_center);

  if ( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  cv::findNonZero(component_mask, pts);
  pts = pts.reshape(1, pts.rows);
  pts.convertTo(pts, CV_32F);

  cv::PCA pca(pts, cv::noArray(), cv::PCA::DATA_AS_ROW);

  const cv::Mat1f mean = pca.mean;
  const cv::Point2f center(mean(0, 0), mean(0, 1));
  const cv::Matx22f eigenvectors = pca.eigenvectors;

//  CF_DEBUG("cntr=(x=%g y=%g)\n"
//      " eigenvectors= {\n"
//      "  %+20g %+20g\n"
//      "  %+20g %+20g\n"
//      "}\n"
//      "\n",
//      center.x, center.y,
//      eigenvectors(0, 0), eigenvectors(0, 1),
//      eigenvectors(1, 0), eigenvectors(1, 1));

  const double angle =
      std::atan2(eigenvectors(0, 1),
          eigenvectors(0, 0)) * 180 / CV_PI;

  const cv::Matx23f M =
      getRotationMatrix2D(center, angle, 1);

  cv::warpAffine(component_mask, rotated_component_mask, M,
      component_mask.size(),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT);

  const cv::Rect rc =
      cv::boundingRect(rotated_component_mask);

  output_bbox.center = center;
  output_bbox.size.width = rc.width - 2;
  output_bbox.size.height = rc.height - 2;
  output_bbox.angle = angle;

  if ( output_mask.needed() ) {
    output_mask.move(component_mask);
  }

  return true;
}


#if 0


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
#endif
