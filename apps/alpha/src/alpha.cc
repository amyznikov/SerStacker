/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/proc/eccalign.h>
#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/small-planetary-disk-detector.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/settings.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  cv::Mat image, mask;
  std::string filename;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 ) {
      printf("Usage: alpha <input-file-name.tiff>\n");
      return 0;
    }

    if ( filename.empty() ) {
      filename = argv[i];
    }
    else {
      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
      return 1;
    }
  }

  if ( filename.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !load_image(filename, image) ) {
    CF_ERROR("load_tiff_image() fails");
  }

  return 0;
}



//static bool fit_ellipse(const cv::Mat & image, cv::RotatedRect * rc)
//{
//  cv::Point2f centrold;
//  cv::Rect component_rect;
//  cv::Mat1b component_mask;
//  cv::Mat component_edge;
//  std::vector<cv::Point2f> edge_points;
//  cv::Mat draw;
//
//  if( !simple_small_planetary_disk_detector(image, cv::noArray(), &centrold, 1, &component_rect, &component_mask) ) {
//    CF_ERROR("simple_small_planetary_disk_detector() fails");
//    return false;
//  }
//
//  //save_image(component_mask, "raw_component_mask.png");
//  //geo_fill_holes(component_mask, component_mask, 8);
//  save_image(component_mask, "closed_component_mask.png");
//
//  morphological_gradient(component_mask, component_edge, cv::Mat1b(3, 3, 255), cv::BORDER_CONSTANT);
//  cv::findNonZero(component_edge, edge_points);
//
//  save_image(component_edge, "closed_component_edge.png");
//
//  //*rc = cv::fitEllipse(edge_points);
//  *rc = cv::fitEllipseAMS(edge_points);
//
//  CF_DEBUG("ellipse: center=(%g %g) angle=%g", rc->center.x, rc->center.y, rc->angle);
//
//  normalize_minmax(image, draw, 0, 255);
//  draw.convertTo(draw, CV_8U);
//  if( draw.channels() != 3 ) {
//    cv::cvtColor(draw, draw, cv::COLOR_GRAY2BGR);
//  }
//
//  draw.setTo(cv::Scalar(0, 0, 255), component_edge);
//  cv::ellipse(draw, *rc, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
//
//  double measured_polar_semi_axis = 0.5 * std::min(rc->size.width, rc->size.height);
//  double measured_equatorial_semi_axis = 0.5 * std::max(rc->size.width, rc->size.height);
//
//  if( true ) {
//    double angle1 = rc->angle * CV_PI / 180;
//    double angle2 = rc->angle * CV_PI / 180 - CV_PI / 2;
//
//    cv::Point2f pt1(-measured_polar_semi_axis * cos(angle1), -measured_polar_semi_axis * sin(angle1));
//    cv::Point2f pt2(measured_polar_semi_axis * cos(angle1), measured_polar_semi_axis * sin(angle1));
//
//    cv::Point2f pt3(-measured_polar_semi_axis * cos(angle2), -measured_polar_semi_axis * sin(angle2));
//    cv::Point2f pt4(measured_polar_semi_axis * cos(angle2), measured_polar_semi_axis * sin(angle2));
//
//    cv::line(draw, rc->center + pt1, rc->center + pt2, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
//    cv::line(draw, rc->center + pt3, rc->center + pt4, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
//  }
//
//  save_image(draw, "fitEllipse.png");
//
//  //  double jovian_axis_ratio = 71.492 / 66.854;
//  //  double masured_axis_ratio = measured_equatorial_semi_axis / measured_polar_semi_axis;
//  //  CF_DEBUG("jovian_axis_ratio=%g masured_axis_ratio=%g", jovian_axis_ratio, masured_axis_ratio);
//  //  return 0;
//
//  return true;
//}
//
//static bool fit_jovian_ellipse_ecc(const cv::Mat & image, cv::RotatedRect * rc, int index)
//{
//  cv::Point2f centrold;
//  cv::Rect component_rect;
//  cv::Mat1b component_mask;
//  cv::Mat component_edge;
//  std::vector<cv::Point2f> edge_points;
//  cv::Mat draw;
//
//  if( !simple_small_planetary_disk_detector(image, cv::noArray(), &centrold, 1, &component_rect, &component_mask) ) {
//    CF_ERROR("simple_small_planetary_disk_detector() fails");
//    return false;
//  }
//
//  geo_fill_holes(component_mask, component_mask, 8);
//  save_image(component_mask, ssprintf("closed_component_mask.%d.png", index));
//
//  morphological_gradient(component_mask, component_edge, cv::Mat1b(3, 3, 255), cv::BORDER_CONSTANT);
//  cv::findNonZero(component_edge, edge_points);
//
//  save_image(component_edge, ssprintf("closed_component_edge.%d.png", index));
//
//  //*rc = cv::fitEllipse(edge_points);
//  *rc = cv::fitEllipseAMS(edge_points);
//
//
//  static constexpr double jovian_polar_to_equatorial_axis_ratio = 66.854 / 71.492;
//
//  double rcA = std::max(rc->size.width, rc->size.height);
//  double rcB = std::min(rc->size.width, rc->size.height);
//  double rcR = 0.5 * (rcA + rcB);
//
////  rc->size.width = rcR;
////  rc->size.height = rcR * jovian_polar_to_equatorial_axis_ratio;
//
//  cv::Mat1f artifical_ellipse(component_rect.size(), 0);
//
//  cv::ellipse(artifical_ellipse, *rc, 255, -1, cv::LINE_AA);
//  save_image(artifical_ellipse, ssprintf("artifical_ellipse.%d.png", index));
//
//
//  return true;
//}
//
//static bool createEllipseRotationRemap(cv::Mat2f & remap, cv::Mat1f & mask, const cv::Size & size,
//    double Cx, double Cy, double a, double b, double l)
//{
//  remap.create(size);
//  mask.create(size);
//  mask.setTo(0);
//
//  const double c = a / b;
//
//
//  for ( int y = 0; y < size.height; ++y ) {
//    double yy = y - Cy;
//
//    for ( int x = 0; x < size.width; ++x ) {
//      double xx = x - Cx;
//
//      if ( (xx / a) * (xx / a) + (yy / b) * (yy / b) >= 0.95 ) {
//        remap[y][x][0] = -1;
//        remap[y][x][1] = -1;
//      }
//      else {
//        double lambda = asin(xx / (c * sqrt(b * b - yy * yy))) + l;
//        if ( (lambda <= - 0.8 * CV_PI / 2) || (lambda >= CV_PI / 2) ) {
//          remap[y][x][0] = -1;
//          remap[y][x][1] = -1;
//        }
//        else {
//          remap[y][x][0] = sin(lambda) * c * sqrt(b * b - yy * yy) + Cx;
//          remap[y][x][1] = y;
//          mask[y][x] = cos(0.5 * CV_PI * xx / ((c * sqrt(b * b - yy * yy)))) * cos(0.5 * CV_PI * yy / b) ;
//          //mask[y][x] *= mask[y][x];
//        }
//
//      }
//    }
//  }
//
//  return true;
//}
//
//
//
//static void ecc_normalize(cv::InputArray _src, cv::InputArray mask, cv::OutputArray dst, double sigma)
//{
////  cv::Scalar mv, sv;
////  cv::Mat src, mean, stdev;
////
////  src = _src.getMat();
////  cv::meanStdDev(src, mv, sv, mask);
////
////  cv::GaussianBlur(src, mean, cv::Size(), sigma, sigma);
////  cv::GaussianBlur(src.mul(src), stdev, cv::Size(), sigma, sigma);
////  cv::absdiff(stdev, mean.mul(mean), stdev);
////  cv::sqrt(stdev, stdev);
////  cv::add(stdev, 0.01 * sv[0], stdev);
////  cv::subtract(src, mean, mean);
////  cv::divide(mean, stdev, dst);
//
//  cv::Scalar mv, sv;
//  cv::Mat src, mean;
//
//  src = _src.getMat();
//  cv::meanStdDev(src, mv, sv, mask);
//
//  cv::GaussianBlur(src, mean, cv::Size(), sigma, sigma);
//  cv::subtract(src, mean, mean);
//  cv::multiply(mean, 1./sv[0], dst);
//
//
//}
//
//int main(int argc, char *argv[])
//{
//  std::string filenames[2];
//
//  cv::Mat images[2];
//  cv::Mat masks[2];
//
//  cv::Point2f centrolds[2];
//  cv::Rect component_rects[2];
//  cv::Mat1b component_masks[2];
//
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name1> <input-file-name2>\n");
//      return 0;
//    }
//
//    if ( filenames[0].empty() ) {
//      filenames[0] = argv[i];
//    }
//    else if ( filenames[1].empty() ) {
//      filenames[1] = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filenames[0].empty() || filenames[1].empty()) {
//    fprintf(stderr, "Two input file name required\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  for ( int i = 0; i < 2; ++i ) {
//
//    if ( !load_image(images[i], filenames[i]) ) {
//      CF_ERROR("load_image(%s) fails", filenames[i].c_str());
//      return 1;
//    }
//
//
//    if( images[i].channels() == 4 || images[i].channels() == 2 ) {
//      splitbgra(images[i], images[i], &masks[i]);
//    }
//
//    if( images[i].channels() == 3  ) {
//      cv::cvtColor(images[i], images[i], cv::COLOR_BGR2GRAY);
//    }
//
//    save_image(images[i], ssprintf("gray_image.%d.tiff", i));
//
//    if( !simple_small_planetary_disk_detector(images[i], masks[i], &centrolds[i], 1, &component_rects[i], &component_masks[i]) ) {
//      CF_ERROR("simple_small_planetary_disk_detector(%s) fails", filenames[i].c_str());
//      return 1;
//    }
//
//    save_image(component_masks[i], ssprintf("raw_component_mask.%d.png", i));
//
//    geo_fill_holes(component_masks[i], component_masks[i], 8);
//    cv::dilate(component_masks[i], component_masks[i], cv::Mat1b(3, 3, 255));
//
//    save_image(component_masks[i], ssprintf("closed_component_mask.%d.png", i));
//    images[i].setTo(0, ~component_masks[i]);
//  }
//
//  cv::Size maxsize(std::max(component_rects[0].width, component_rects[1].width) + 32, std::max(component_rects[0].height, component_rects[1].height) + 32);
//
//  cv::Mat comps[2];
//  cv::RotatedRect erc[2];
//  for ( int i = 0; i < 2; ++i ) {
//    comps[i] = cv::Mat::zeros(maxsize, images[i].type());
//
//    images[i](component_rects[i]).copyTo(
//        comps[i](cv::Rect(32 / 2, 32 / 2, component_rects[i].width, component_rects[i].height)));
//
//    comps[i].convertTo(comps[i], CV_32F);
//    save_image(comps[i], ssprintf("comps.%d.tiff", i));
//
//
//    fit_jovian_ellipse_ecc(comps[i], &erc[i], i);
//  }
//
//
////
////  c_ecc_forward_additive ecc;
////  c_ecc_pyramide_align ecch(&ecc);
////  cv::Mat avgimg, diff;
////  cv::Mat T;
////  cv::RotatedRect erc;
////
////  ecch.align(comps[0], comps[1], T = createEyeTransform(ECC_MOTION_EUCLIDEAN));
////  cv::remap(comps[0], comps[0], ecch.method()->current_remap(), cv::noArray(), cv::INTER_LINEAR);
////  save_image(comps[0], ssprintf("comps.%d.aligned.tiff", 0));
////
////
////  cv::add(comps[0], comps[1], avgimg);
////  if ( !fit_ellipse(avgimg, &erc) ) {
////    CF_ERROR("fit_ellipse() fails");
////  }
////
////  cv::Mat1f E;
////  cv::Mat emap;
////  E = createEuclideanTransform(erc.center.x, erc.center.y, erc.center.x, erc.center.y, 1.0, -erc.angle * CV_PI / 180 + CV_PI / 2);
////  createRemap(ECC_MOTION_EUCLIDEAN, E, emap, avgimg.size());
////  for ( int i = 0; i < 2; ++i ) {
////    cv::remap(comps[i], comps[i], emap, cv::noArray(), cv::INTER_LINEAR);
////    save_image(comps[i], ssprintf("comps.remap.%d.tiff", i));
////  }
////
////  cv::subtract(comps[1], comps[0], diff);
////  save_image(diff, "diff.initial.tiff");
////
////
////  double A = std::max(erc.size.width, erc.size.height);
////  double B = std::min(erc.size.width, erc.size.height);
////  double normalize_sigma = 3 * A / 500;
////
////  CF_DEBUG("ELLIPSE: A=%g B=%g normalize_sigma=%g", A, B, normalize_sigma);
////
////
////  cv::Mat2f rmap;
////  cv::Mat1f visibility_mask;
////  cv::Mat rotated_image;
////
////
////  ecc_normalize(comps[0], cv::noArray(), comps[0], normalize_sigma );
////  ecc_normalize(comps[1], cv::noArray(), comps[1], normalize_sigma );
////
////  for ( int i = 0; i < 50; ++i ) {
////
////    double angle = i * CV_PI / 180;
////
////
////    createEllipseRotationRemap(rmap, visibility_mask, comps[0].size(),
////        erc.center.x, erc.center.y, A / 2, B / 2, -angle);
////
////    cv::remap(comps[0], rotated_image, rmap, cv::noArray(), cv::INTER_LINEAR);
////    save_image(rotated_image, ssprintf("rotations/comps0.rotated.%02d.tiff", i));
////
////    cv::subtract(comps[1], rotated_image, diff);
////    cv::multiply(diff, visibility_mask, diff);
////
////    save_image(diff, ssprintf("rotations/diff.%02d.tiff", i));
////
////    double cost = cv::norm(diff, cv::NORM_L2);
////    CF_DEBUG("rotation %d: angle=%g cost=%g", i, angle * 180 / CV_PI,  cost);
////  }
//
////  cv::Mat gx, gy;
////  save_image(diff, "diff.rotated.tiff");
////  ecc_differentiate(comps[1], gx, gy);
//
////  for ( int i = 0; i < 2; ++i ) {
////    ecc_normalize(comps[i], cv::noArray(), comps[i], 5);
////    save_image(comps[i], ssprintf("comps.normalized.%d.tiff", i));
////  }
//
////  std::vector<cv::Mat> pyramids[2];
////  for(int i = 0; i < 2; ++i ) {
////
////    cv::buildPyramid(comps[i], pyramids[i], 7 );
////
////    for(int j = 0, nj = pyramids[i].size(); j < nj; ++j ) {
////      save_image(pyramids[i][j], ssprintf("pyr.%d.%d.tiff", i, j));
////    }
////  }
//
//
//
//  return 0;
//}

//
//
//int main(int argc, char *argv[])
//{
//  cv::Mat image, mask;
//  std::string filename;
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name.tiff>\n");
//      return 0;
//    }
//
//    if ( filename.empty() ) {
//      filename = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filename.empty() ) {
//    fprintf(stderr, "No input file name specified\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  if ( !load_image(image, filename) ) {
//    CF_ERROR("load_image(%s) fails", filename.c_str());
//    return 1;
//  }
//
//
//  if( image.channels() == 4 || image.channels() == 2 ) {
//    splitbgra(image, image, &mask);
//  }
//
//  if( image.channels() == 3  ) {
//    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
//  }
//  save_image(image, "gray_image.tiff");
//
//  cv::Point2f centrold;
//  cv::Rect component_rect;
//  cv::Mat1b component_mask;
//
//  if( !simple_small_planetary_disk_detector(image, &centrold, 1, &component_rect, &component_mask) ) {
//    CF_ERROR("simple_small_planetary_disk_detector(%s) fails", filename.c_str());
//    return 1;
//  }
//
//  save_image(component_mask, "raw_component_mask.png");
//
//  morphological_smooth_close(component_mask, component_mask, cv::Mat1b(5,5, 255));
//  geo_fill_holes(component_mask, component_mask, 8);
//
//  save_image(component_mask, "closed_component_mask.png");
//
//  cv::Mat component_edge;
//  std::vector<cv::Point2f> edge_points;
//  cv::RotatedRect ellipse_rect;
//
//  morphological_gradient(component_mask, component_edge, cv::Mat1b(3,3, 255), cv::BORDER_CONSTANT);
//  cv::findNonZero(component_edge, edge_points);
//
//  save_image(component_edge, "closed_component_edge.png");
//
//  ellipse_rect = cv::fitEllipse(edge_points);
//
//  CF_DEBUG("ellipse: center=(%g %g) angle=%g", ellipse_rect.center.x, ellipse_rect.center.y, ellipse_rect.angle);
//
//  normalize_minmax(image, image, 0, 255);
//  image.convertTo(image, CV_8U);
//  if( image.channels() != 3 ) {
//    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
//  }
//
//  image.setTo(cv::Scalar(0, 0, 255), component_edge);
//  cv::ellipse(image, ellipse_rect, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
//
//  double measured_polar_semi_axis = 0.5 * std::min(ellipse_rect.size.width, ellipse_rect.size.height);
//  double measured_equatorial_semi_axis = 0.5 * std::max(ellipse_rect.size.width, ellipse_rect.size.height);
//
//  if ( true ) {
//    double angle1 = ellipse_rect.angle * CV_PI / 180;
//    double angle2 = ellipse_rect.angle * CV_PI / 180 - CV_PI / 2;
//
//    cv::Point2f pt1(-measured_polar_semi_axis * cos(angle1), -measured_polar_semi_axis * sin(angle1));
//    cv::Point2f pt2(measured_polar_semi_axis * cos(angle1), measured_polar_semi_axis * sin(angle1));
//
//    cv::Point2f pt3(-measured_polar_semi_axis * cos(angle2), -measured_polar_semi_axis * sin(angle2));
//    cv::Point2f pt4(measured_polar_semi_axis * cos(angle2), measured_polar_semi_axis * sin(angle2));
//
//    cv::line(image, ellipse_rect.center + pt1, ellipse_rect.center + pt2, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
//    cv::line(image, ellipse_rect.center + pt3, ellipse_rect.center + pt4, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
//  }
//
//  save_image(image, "fitEllipse.png");
//
//
//
//  double jovian_axis_ratio = 71.492 / 66.854;
//  double masured_axis_ratio = measured_equatorial_semi_axis / measured_polar_semi_axis;
//
//  CF_DEBUG("jovian_axis_ratio=%g masured_axis_ratio=%g", jovian_axis_ratio, masured_axis_ratio);
//
//
//  return 0;
//}
//



//
//
//int main(int argc, char *argv[])
//{
//  cv::Mat image, mask;
//  std::string filename;
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name.tiff>\n");
//      return 0;
//    }
//
//    if ( filename.empty() ) {
//      filename = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filename.empty() ) {
//    fprintf(stderr, "No input file name specified\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  if ( !load_image(image, filename) ) {
//    CF_ERROR("load_tiff_image() fails");
//  }
//
//
//  cv::Mat sharp, sharpx, sharpy, gx, gy, g, smask;
//
//  if ( image.channels() == 4 || image.channels() == 2 ) {
//    CF_DEBUG("HAVE MASK");
//    splitbgra(image, image, &mask);
//    cv::erode(mask, smask, cv::Mat1b(55, 55, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//
//  }
//  else {
//    cv::erode(cv::Mat1b(image.size(), 255), smask, cv::Mat1b(55, 55, 255), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
//  }
//
//
//
////  {
////  double l1, l2, l2s;
////    cv::Mat1f m(100,100, 2.f);
////    l1 = cv::norm(m, cv::NORM_L1);
////    l2 = cv::norm(m, cv::NORM_L2);
////    l2s = cv::norm(m, cv::NORM_L2SQR);
////
////
////    printf("\n"
////        "M: L1=%g L2=%g L2SQR=%g"
////        "\n",
////        l1, l2, l2s);
////
////    return 0;
////  }
//
//
//  if ( !smask.empty() ) {
//    cv::compare(image, 0.01, smask, cv::CMP_GT);
//    cv::cvtColor(smask, smask, cv::COLOR_BGR2GRAY);
//    save_image(smask, ssprintf("sharp/smask.png"));
//  }
//
//  double sigma = 0.5;
//  double l1, l2;
//
//  static const double alpha[] = {
//      0,
//      0.5, 0.75, 0.9,
//      0.95,
//      0.96,
//      0.970, 0.971, 0.972, 0.973, 0.974, 0.975, 0.976, 0.977, 0.978, 0.979,
//      0.980, 0.981, 0.982, 0.983, 0.984, 0.985, 0.986, 0.987, 0.988, 0.989,
//      0.990, 0.991, 0.992, 0.993, 0.994, 0.995, 0.996, 0.997, 0.998, 0.999,
//      0.9991, 0.9991, 0.9993, 0.9994, 0.9995, 0.9996, 0.9997, 0.9998, 0.9999,
////      0.99995, 0.99999,
////      0.999995, 0.999999,
////      0.9999995, 0.9999999,
//  };
//
//
//  rmfiles("sharp/", "*");
//  rmfiles("g/", "*");
//
////  cv::absdiff(image, 0, image);
////  cv::sqrt(image, image);
//
////  differentiate(image, gx, gy);
////  save_image(gx, ssprintf("sharp/gx.tiff"));
////  save_image(gy, ssprintf("sharp/gy.tiff"));
//
////  cv::GaussianBlur(image, image, cv::Size(0,0), 0.9);
////  differentiate(image, gx, gy);
//
//  //smask.release();
//  fprintf(stdout, "I\talpha\tm\ts\te\tim\tis\tie\n");
//
//  cv::Scalar ims, iss, ies;
//  double imv, isv, iev;
//  cv::meanStdDev(image, ims, iss, smask);
//  ies = estimate_noise(image, cv::noArray(), smask);
//  imv = (ims[0] + ims[1] + ims[2]);
//  isv = (iss[0] + iss[1] + iss[2]);
//  iev = (ies[0] + ies[1] + ies[2]);
//
//  for ( int i = 0, n = sizeof(alpha)/sizeof(alpha[0]); i < n; ++i ) {
//
//    cv::Scalar ms, ss, es;
//    double mv, sv, ev;
//
//    unsharp_mask(image, sharp, sigma, alpha[i]);
//
//    cv::meanStdDev(sharp, ms, ss, smask);
//    es = estimate_noise(sharp, cv::noArray(), smask);
//
//    mv = (ms[0] + ms[1] + ms[2]);
//    sv = (ss[0] + ss[1] + ss[2]);
//    ev = (es[0] + es[1] + es[2]);
//
//    fprintf(stdout, "%6d\t%12.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\n",
//        i, alpha[i], mv, sv, ev, imv, isv, iev);
//
//    save_image(sharp, ssprintf("sharp/sharp.%05d.tiff", i));
//   }
//
//  return 0;
//}
//


//static bool convertTofp32(const cv::Mat & src, cv::Mat & dst)
//{
//  constexpr int ddepth = CV_32F;
//  switch (src.depth()) {
//    case CV_8S:
//      src.convertTo(dst, ddepth, 1./INT8_MAX, 0.5);
//      break;
//    case CV_8U:
//      src.convertTo(dst, ddepth, 1./UINT8_MAX, 0);
//      break;
//    case CV_16S:
//      src.convertTo(dst, ddepth, 1./INT16_MAX, 0.5);
//      break;
//    case CV_16U:
//      src.convertTo(dst, ddepth, 1./UINT16_MAX, 0);
//      break;
//    case CV_32S:
//      src.convertTo(dst, ddepth, 1./INT32_MAX, 0.5);
//      break;
//    case CV_32F:
//      if ( src.data != dst.data ) {
//        src.copyTo(dst);
//      }
//      break;
//    case CV_64F:
//      src.convertTo(dst, ddepth);
//      break;
//    default:
//      return  false;;
//  }
//
//  return true;
//}
//
//
//static void build_laplacian_pyramid(int max_levels, const cv::Mat & src, std::vector<cv::Mat> & laps, cv::Mat & res )
//{
//  laps.clear();
//  laps.reserve(max_levels);
//
//
//  cv::Mat s, gb;
//
//  cv::GaussianBlur(src, gb, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
//  laps.emplace_back(), cv::subtract(src, gb, laps.back());
//
////  for ( int i = 1; i < max_levels; ++i ) {
////    laps.emplace_back();
////    downstrike_uneven(gb, laps.back());
////    cv::GaussianBlur(laps.back(), gb, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
////    cv::subtract(laps.back(), gb, laps.back());
////  }
////
////  downstrike_uneven(gb, res);
//
//  for ( int i = 1; i < max_levels; ++i ) {
//    gb.copyTo(s);
//    cv::GaussianBlur(s, gb, cv::Size(3, 3), 0.5, 0.5, cv::BORDER_DEFAULT);
//    laps.emplace_back(), cv::subtract(s, gb, laps.back());
//  }
//
//  gb.copyTo(res);
//}
//
//static double squale(double x)
//{
//  return x * x * x / (100);
//}
//
//static void merge_laplacian_pyramid(const std::vector<cv::Mat> & laps, const cv::Mat & res, cv::Mat & dst )
//{
//  cv::Mat s;
//
//  cv::scaleAdd(laps[laps.size() - 1], squale(laps.size() - 1), res, s);
//
//  for ( int i = laps.size() - 2; i >= 0; --i ) {
//    cv::scaleAdd(laps[i], squale(laps.size() - i), s, s);
//  }
//
//  dst = std::move(s);
//}
////
////int main(int argc, char *argv[])
////{
////  std::string input_file_name;
////  std::string output_path;
////  cv::Mat image;
////  cv::Mat mask;
////  std::vector<cv::Mat> laps;
////  cv::Mat res;
////  double min, max;
////
////  for ( int i = 1; i < argc; ++i ) {
////
////    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
////      fprintf(stdout,
////          "Usage:\n"
////          "   alpha [OPTIONS] input_image -o output_path\n"
////          "\n"
////      );
////
////      return 0;
////    }
////
////    if ( strcmp(argv[i], "-o") == 0 ) {
////      if ( (++i >= argc) ) {
////        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
////        return 1;
////      }
////      output_path = argv[i];
////    }
////
////
////
////    else if ( input_file_name.empty() && is_regular_file(argv[i]) ) {
////      input_file_name = argv[i];
////    }
////
////
////    else {
////      fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
////      return 1;
////    }
////  }
////
////  if ( input_file_name.empty() ) {
////    fprintf(stderr, "No input SER file specified\n");
////    return 1;
////  }
////
////  cf_set_logfile(stderr);
////  cf_set_loglevel(CF_LOG_DEBUG);
////
////
////  if ( !load_image(image, input_file_name) ) {
////    CF_ERROR("load_image(%s) fails", input_file_name.c_str());
////    return 1;
////  }
////
////
////  switch ( image.channels() ) {
////  case 1 :
////    break;
////  case 2 : {
////    cv::Mat channels[2];
////    cv::split(image, channels);
////    image = channels[0];
////    cv::compare(channels[1], 0, mask, cv::CMP_GT);
////    CF_DEBUG("HAVE MASK");
////    break;
////  }
////  case 3 :
////    break;
////  case 4 : // Assuming BGRA
////    cv::extractChannel(image, mask, 3);
////    cv::compare(mask, 0, mask, cv::CMP_GT);
////    cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
////    CF_DEBUG("HAVE MASK");
////    break;
////  default :
////    CF_FATAL("Invalid input: RGB or grayscale image expected, image.channels()=%d rgb_image.depth()=%d",
////        image.channels(), image.depth());
////    return 1;
////    break;
////  }
////
////  convertTofp32(image,
////      image);
////
////  cv::minMaxLoc(image, &min, &max);
////
////  CF_DEBUG("input: %dx%d channels=%d depth=%d min=%g max=%g",
////        image.cols, image.rows,
////        image.channels(), image.depth(),
////        min, max);
////
////
////  build_laplacian_pyramid(15, image, laps, res );
////
////  for ( uint i = 0, n = laps.size(); i < n; ++i ) {
////    save_image(laps[i], ssprintf("pyramid/lap.%03u.tiff", i));
////  }
////
////  save_image(res, ssprintf("pyramid/res.tiff"));
////
////  merge_laplacian_pyramid(laps, res, image );
////
////  save_image(image, ssprintf("pyramid/merged-back.tiff"));
////
////  return 0;
////}
//
//
//int main(int argc, char *argv[])
//{
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//  c_image_processor::ptr processor =
//      c_image_processor::create("processor1");
//
//  processor->emplace_back(c_unsharp_mask_routine::create());
//  processor->emplace_back(c_align_color_channels_routine::create());
//  processor->emplace_back(c_rangeclip_routine::create());
//  processor->save("processor1.cfg");
//
//  processor = c_image_processor::load("processor1.cfg");
//  if ( !processor ) {
//    CF_ERROR("c_image_processor::load(\"processor1.cfg\") fails");
//    return 1;
//  }
//
//  processor->save("processor2.cfg");
//
//  return 0;
//}
