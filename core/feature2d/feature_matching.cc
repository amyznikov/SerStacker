/*
 * feature_matching.cc
 *
 *  Created on: Dec 26, 2021
 *      Author: amyznikov
 */

#include "feature_matching.h"
#include <core/debug.h>

template<>
const c_enum_member * members_of<FEATURE2D_MATCHER_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { FEATURE2D_MATCHER_HAMMING, "hamming", "" },
      { FEATURE2D_MATCHER_FLANN, "flann", "" },
      { FEATURE2D_MATCHER_TRIANGLES, "triangle_matcher", "" },
      { FEATURE2D_MATCHER_SNORM, "snorm", "" },
      { FEATURE2D_MATCHER_UNKNOWN, nullptr, "" },
  };

  return members;
}

c_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_feature2d_matcher_options & options)
{
  switch ( options.type ) {
  case FEATURE2D_MATCHER_HAMMING :
    return create_sparse_feature_matcher(options.hamming);
  case FEATURE2D_MATCHER_FLANN :
    return create_sparse_feature_matcher(options.flann);
  case FEATURE2D_MATCHER_SNORM :
    return create_sparse_feature_matcher(options.snorm);
  case FEATURE2D_MATCHER_TRIANGLES :
    return create_sparse_feature_matcher(options.triangles);
  case FEATURE2D_MATCHER_UNKNOWN :
    CF_ERROR("ERROR: c_feature2d_matcher type not specified");
    return nullptr;
  default :
    break;
  }

  CF_ERROR("ERROR: Unknown or not supported sparse feature matcher type=%d requested",
      options.type);
  return nullptr;
}



void dump_supported_feature2d_matchers(FILE * fp /*= stdout*/)
{
  fprintf(fp, "SUPPORTED FEATURE MATCHERS:\n"
      "\n");

  fprintf(fp, "%s: GLDDM matcher based on hamming distance\n"
      "double max_acceptable_distance = -1;\n"
      "\n", toString(FEATURE2D_MATCHER_HAMMING));

  fprintf(fp, "%s: GLDDM matcher based on sorded L1 norm (NOT recommended, prefer FLANN_INDEX_KDTREE for SIFT/SURF matching) \n"
      "double max_acceptable_distance = -1;\n"
      "double lowe_ratio = -1;\n"
      "\n", toString(FEATURE2D_MATCHER_SNORM));

  fprintf(fp, "%s:index=%s Based on cv::flann::LinearIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_linear));

  fprintf(fp, "%s:index=%s Based on  cv::flann::KDTreeIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "int trees = 1;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_kdtree));

  fprintf(fp, "%s:index=%s Based on  cv::flann::KMeansIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "int branching = 32;\n"
      "int iterations = 11;\n"
      "cvflann::flann_centers_init_t centers_init = RANDOM (RANDOM, GONZALES, KMEANSPP, GROUPWISE);\n"
      "float cb_index = 0.2f;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_kmeans));

  fprintf(fp, "%s:index=%s Based on cv::flann::CompositeIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "int trees = 1;\n"
      "int branching = 32;\n"
      "int iterations = 11;\n"
      "cvflann::flann_centers_init_t centers_init = RANDOM (RANDOM, GONZALES, KMEANSPP, GROUPWISE);\n"
      "float cb_index = 0.2f;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_composite));

  fprintf(fp, "%s:index=%s Based on cv::flann::HierarchicalClusteringIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "int branching = 32;\n"
      "cvflann::flann_centers_init_t centers_init = RANDOM (RANDOM, GONZALES, KMEANSPP, GROUPWISE);\n"
      "int trees = 4;\n"
      "int leaf_size = 100;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_hierarchical));

  fprintf(fp, "%s:index=%s Based on cv::flann::LshIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "int table_number = 8;\n"
      "int key_size = 12;\n"
      "int multi_probe_level = 1;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_lsh));

  fprintf(fp, "%s:index=%s Based on cv::flann::AutotunedIndexParams\n"
      "cvflann::flann_distance_t distance_type = L2 (L1, L2, MINKOWSKI, MAX, INTERSECT, HELLINGER, CS, KL, HAMMING, DNAMMING);\n"
      "double lowe_ratio = -1;\n"
      "float target_precision = 0.8f;\n"
      "float build_weight = 0.01f;\n"
      "float memory_weight = 0;\n"
      "float sample_fraction = 0.1f;\n"
      "\n", toString(FEATURE2D_MATCHER_FLANN), toString(FlannIndex_autotuned));

  fprintf(fp, "\n");
}


size_t match_keypoints(const cv::Ptr<c_feature2d_matcher> & keypoints_matcher,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const cv::Mat & current_descriptors,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const cv::Mat & reference_descriptors,

    std::vector<cv::DMatch> * output_matches /*= nullptr*/,
    std::vector<cv::Point2f> * output_matched_current_positions /*= nullptr*/,
    std::vector<cv::Point2f> * output_matched_reference_positions /*= nullptr*/)

{
  INSTRUMENT_REGION("");

  // Check input args
  if ( !keypoints_matcher ) {
    CF_ERROR("ERROR: keypoints_matcher is NULL");
    return 0;
  }

  cv::theRNG().state = 1234567890;

  // Train the matcher on reference descriptors
  if ( !keypoints_matcher->train(reference_descriptors) ) {
    CF_ERROR("keypoints_matcher->train() fails");
    return 0;
  }


  // Match the descriptors
  std::vector<cv::DMatch> matches;

  if ( !keypoints_matcher->match(current_descriptors, matches) ) {
    CF_ERROR("keypoints_matcher->match() fails");
    return 0;
  }


  const size_t num_matches_found =
      matches.size();


  // Send outputs to caller

  if ( output_matched_current_positions ) {

    output_matched_current_positions->reserve(
        output_matched_current_positions->size() +
            matches.size());

    for ( const cv::DMatch & m : matches ) {
      output_matched_current_positions->
          emplace_back(current_keypoints[m.queryIdx].pt);
    }
  }

  if ( output_matched_reference_positions ) {

    output_matched_reference_positions->reserve(
        output_matched_reference_positions->size() +
            matches.size());

    for ( const cv::DMatch & m : matches ) {
      output_matched_reference_positions->
          emplace_back(reference_keypoints[m.trainIdx].pt);
    }
  }

  if ( output_matches ) {

    *output_matches =
        std::move(matches);
  }

  return num_matches_found;
}




