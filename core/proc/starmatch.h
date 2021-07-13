/*
 * starmatch.h
 *
 *  Created on: Dec 1, 2019
 *      Author: amyznikov
 */

#ifndef __starmatch_h__
#define __starmatch_h__

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>


typedef cv::flann::L2_Simple<float>
  StarMathingDistanceType;


bool build_triangles(const std::vector<cv::KeyPoint> & keypoints,
    std::vector<cv::Vec3w> & triangles,
    std::vector<cv::Vec2f> & descriptors,
    float min_side_size = 10);

bool build_triangles(const std::vector<cv::Point2f> & keypoints,
    std::vector<cv::Vec3w> & triangles,
    std::vector<cv::Vec2f> & descriptors,
    float min_side_size = 10);


cv::Ptr<cvflann::KDTreeIndex<StarMathingDistanceType>>
  index_triangles(std::vector<cv::Vec2f> & triangle_descriptors);



/* TODO: extract matches as std::vector<cv::DMatch> */
void match_triangles(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<cv::Vec3w> & input_triangles, const std::vector<cv::Vec3w> & reference_triangles,
    const std::vector<cv::Vec2f> & input_descriptors, const std::vector<cv::Vec2f> & reference_descriptors,
    cvflann::KDTreeIndex<StarMathingDistanceType> & reference_index,
    std::vector<std::pair<uint16_t, uint16_t> > & matches,
    float eps);

void match_triangles(const std::vector<cv::Point2f> & input_stars, const std::vector<cv::Point2f> & reference_stars,
    const std::vector<cv::Vec3w> & input_triangles, const std::vector<cv::Vec3w> & reference_triangles,
    const std::vector<cv::Vec2f> & input_descriptors, const std::vector<cv::Vec2f> & reference_descriptors,
    cvflann::KDTreeIndex<StarMathingDistanceType> & reference_index,
    std::vector<std::pair<uint16_t, uint16_t> > & matches,
    float eps);




void extract_matches(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::Point2f> * input_positions,
    std::vector<cv::Point2f> * reference_positions );

void extract_matches(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::KeyPoint> * input_positions,
    std::vector<cv::KeyPoint> * reference_positions );

void extract_matches(const std::vector<cv::Point2f> & input_stars, const std::vector<cv::Point2f> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::Point2f> * input_positions,
    std::vector<cv::Point2f> * reference_positions );



#endif /* __starmatch_h__ */
