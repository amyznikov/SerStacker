/*
 * ellipsoid_bbox.h
 *
 *  Created on: Jul 9, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __ellipsoid_bbox_h__
#define __ellipsoid_bbox_h__

#include <opencv2/opencv.hpp>


/**
 * Given 3D ellipsoid with sem-axes A, B, C and rotated by matrix R
 * compute its outline (shadow) bounding box, appropriate for drawing
 * with cv::ellipse()
 *
 * https://math.stackexchange.com/questions/573055/projection-of-ellipsoid
 * https://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf
 * Hartley & Zisserman’s Multiple View Geometry In Computer Vision. Result 8.9 on page 201
 *
 * Example :
 *   const cv::Size image_size = image.size();
 *   const cv::Vec3d & rotation = orientation() * CV_PI / 180;
 *   const cv::Matx33d R = build_rotation2(rotation);
 *   const double A = equatorial_radius1();
 *   const double B = equatorial_radius2();
 *   const double C = polar_radius();
 *   const cv::Point2f center = cv::Point2f(image_size.width / 2, image_size.height / 2);
 *   const cv::RotatedRect bbox = ellipsoid_bbox(center, A, B, C, R.t());
 *   cv::ellipse(image, bbox, cv::Scalar::all(255), 1, cv::LINE_AA);
 */

cv::RotatedRect ellipsoid_bbox(const cv::Point2f & center,
    double A, double B, double C,
    const cv::Matx33d & R);


#endif /* __ellipsoid_bbox_h__ */
