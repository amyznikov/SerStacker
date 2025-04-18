/*
 * c_flann_based_feature2d_matcher.cc
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#include "c_flann_based_feature2d_matcher.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<cvflann::flann_centers_init_t>()
{
  static const c_enum_member members[] = {
      { cvflann::FLANN_CENTERS_RANDOM, "RANDOM", "" },
      { cvflann::FLANN_CENTERS_GONZALES, "GONZALES", "" },
      { cvflann::FLANN_CENTERS_KMEANSPP, "KMEANSPP", "" },
      { cvflann::FLANN_CENTERS_GROUPWISE, "GROUPWISE", "" },
      { cvflann::FLANN_CENTERS_RANDOM, },
  };

  return members;
}

template<>
const c_enum_member * members_of<cvflann::flann_distance_t>()
{
  static const c_enum_member members[] = {
      { cvflann::FLANN_DIST_L1, "L1", "" },
      { cvflann::FLANN_DIST_L2, "L2", "" },
      { cvflann::FLANN_DIST_MINKOWSKI, "MINKOWSKI", "" },
      { cvflann::FLANN_DIST_MAX, "MAX", "" },
      { cvflann::FLANN_DIST_HIST_INTERSECT, "INTERSECT", "" },
      { cvflann::FLANN_DIST_HELLINGER, "HELLINGER", "" },
      { cvflann::FLANN_DIST_CS, "CS", "" },
      { cvflann::FLANN_DIST_KL, "KL", "" },
      { cvflann::FLANN_DIST_HAMMING, "HAMMING", "" },
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,4,0) )
      { cvflann::FLANN_DIST_DNAMMING, "DNAMMING", "" },
#endif
      { cvflann::FLANN_DIST_L2, },
  };

  return members;
}


template<>
const c_enum_member * members_of<FlannIndexType>()
{
  static const c_enum_member members[] = {
      { FlannIndex_linear, "linear", "" },
      { FlannIndex_kdtree, "kdtree", "" },
      { FlannIndex_kmeans, "kmeans", "" },
      { FlannIndex_composite, "composite", "" },
      { FlannIndex_hierarchical, "hierarchical", "" },
      { FlannIndex_lsh, "lsh", "" },
      { FlannIndex_autotuned, "autotuned", "" },
      { FlannIndex_unknown}
  };

  return members;
}

c_flann_based_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_flann_based_feature2d_matcher_options & options)
{

  cv::Ptr<cv::flann::IndexParams> index_args;

  switch ( options.index.type ) {

  case FlannIndex_linear : {
    index_args.reset(new cv::flann::LinearIndexParams());
    break;
  }

  case FlannIndex_kdtree : {

    const c_flann_kdtree_index_options & opts =
        options.index.kdtree;

    index_args.reset(new cv::flann::KDTreeIndexParams(
        opts.trees));

    break;
  }

  case FlannIndex_kmeans : {

    const c_flann_kmeans_index_options & opts =
        options.index.kmeans;

    index_args.reset(new cv::flann::KMeansIndexParams(
        opts.branching,
        opts.iterations,
        opts.centers_init,
        opts.cb_index));

    break;
  }

  case FlannIndex_composite : {

    const c_flann_composite_index_options & opts =
        options.index.composite;

    index_args.reset(new cv::flann::CompositeIndexParams(
        opts.trees,
        opts.branching,
        opts.iterations,
        opts.centers_init,
        opts.cb_index));

    break;
  }

  case FlannIndex_hierarchical : {
    const c_flann_hierarchical_index_options & opts =
        options.index.hierarchical;

    index_args.reset(new cv::flann::HierarchicalClusteringIndexParams(
        opts.branching,
        opts.centers_init,
        opts.trees,
        opts.leaf_size));

    break;
  }

  case FlannIndex_lsh : {
    const c_flann_lsh_index_options & opts =
        options.index.lsh;

    index_args.reset(new cv::flann::LshIndexParams(
        opts.table_number,
        opts.key_size,
        opts.multi_probe_level));

    break;
  }

  case FlannIndex_autotuned : {

    const c_flann_autotuned_index_options & opts =
        options.index.autotuned;

    index_args.reset(new cv::flann::AutotunedIndexParams(
        opts.target_precision,
        opts.build_weight,
        opts.memory_weight,
        opts.sample_fraction));

    break;
  }

  default :
    CF_ERROR("ERROR: "
        "Unknown or not suppirted flann index type requesterd: '%d'",
        options.index.type);

    return nullptr;
  }

  c_flann_based_feature2d_matcher::ptr obj(new
      c_flann_based_feature2d_matcher(index_args));

  obj->set_distance_type(options.distance_type);
  obj->set_lowe_ratio(options.lowe_ratio);

  return obj;
}



c_flann_based_feature2d_matcher::c_flann_based_feature2d_matcher(const cv::Ptr<cv::flann::IndexParams> & index_args)
  : _index_params(index_args)
{
}

const cv::Ptr<cv::flann::IndexParams> & c_flann_based_feature2d_matcher::index_params() const
{
  return _index_params;
}


void c_flann_based_feature2d_matcher::set_distance_type(cvflann::flann_distance_t v)
{
  _distance_type = v;
}

cvflann::flann_distance_t c_flann_based_feature2d_matcher::distance_type() const
{
  return _distance_type;
}

void c_flann_based_feature2d_matcher::set_lowe_ratio(double v)
{
  _lowe_ratio = v;
  _search_params.reset();
}

double c_flann_based_feature2d_matcher::lowe_ratio() const
{
  return _lowe_ratio;
}


bool c_flann_based_feature2d_matcher::train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors)
{
  try {

    _index.build(train_descriptors,
        *_index_params,
        _distance_type);

    return true;
  }

  catch (const cv::Exception & e) {
    CF_FATAL("cv::Exception : index_.build() fails: %s",
        e.err.c_str());
  }
  catch (const std::exception & e) {
    CF_FATAL("std::exception: index_.build() fails: %s",
        e.what());
  }
  catch (...) {
    CF_FATAL("Unknown exception: index_.build() fails");
  }

  return false;
}

bool c_flann_based_feature2d_matcher::match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
    /* out */ std::vector<cv::DMatch> & matches)
{
  try {

    cv::Mat1i indices;
    cv::Mat dists;

    const int knn =
        _lowe_ratio > 0 ? 2 : 1;

    matches.clear();
    matches.reserve( _lowe_ratio > 0 ? query_descriptors.rows() : knn * query_descriptors.rows());

    if ( !_search_params ) {
      _search_params.reset(new cv::flann::SearchParams(
            cvflann::FLANN_CHECKS_UNLIMITED,
            0,
            true));
    }

    _index.knnSearch(query_descriptors,
        indices,
        dists,
        knn,
        *_search_params);

    static const auto dist =
        [](const cv::Mat & m, int r, int c) -> float {
          return m.depth() == CV_32F ? m.at<float>(r,c) : m.at<int32_t>(r,c);
        };

    const int cr = indices.rows;
    const int cc = indices.cols;
    const int ddepth = dists.depth();

    // CF_DEBUG("cr=%d cc=%d", cr, cc);
    for ( int i = 0; i < cr; ++i ) {

      if ( indices[i][0] < 0 ) {
        continue;
      }

      const float dist0 =
          dist(dists, i, 0);

      if ( _lowe_ratio > 0 && indices[i][1] >= 0 ) {

        if ( dist0 < _lowe_ratio * dist(dists, i, 1) ) {
          matches.emplace_back(i, indices[i][0], dist0);
        }

      }
      else {  // if ( lowe_ratio_ <= 0 || indices_[i][1] < 0 )

        matches.emplace_back(i, indices[i][0], dist0);

        for ( int j = 1; j < cc && indices[i][j] >= 0; ++j ) {

          const double distj = dist(dists, i, j);
          if ( dist0 - distj > FLT_EPSILON ) {
            break;
          }

          matches.emplace_back(i, indices[i][j], distj);
        }
      }
    }

    return true;
  }
  catch (const cv::Exception & e) {
    CF_FATAL("cv::Exception : index_.build() fails: %s",
        e.err.c_str());
  }
  catch (const std::exception & e) {
    CF_FATAL("std::exception: index_.build() fails: %s",
        e.what());
  }
  catch (...) {
    CF_FATAL("Unknown exception: index_.build() fails");
  }

  return false;
}
