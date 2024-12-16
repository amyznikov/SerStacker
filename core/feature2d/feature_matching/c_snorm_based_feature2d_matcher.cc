/*
 * c_sorted_norm_based_feature2d_matcher.cc
 *
 *  Created on: Jan 6, 2022
 *      Author: amyznikov
 *
 *  This VERY experiental code is NOT reccomended to use in practice.
 *  For SIFT/SURF descriptors prefer the FLANN_INDEX_KDTREE instead.
 */

#include "c_snorm_based_feature2d_matcher.h"
#include <core/debug.h>

c_snorm_based_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_snorm_based_feature2d_matcher_options & options)
{

  c_snorm_based_feature2d_matcher::ptr obj(new
      c_snorm_based_feature2d_matcher());

  obj->set_max_acceptable_distance(
      options.max_acceptable_distance);

  obj->set_lowe_ratio(
      options.lowe_ratio);

  return obj;
}


static float compute_norm(const float a[], int n)
{
  float s = std::abs(*a++);
  while ( --n ) {
    s += std::abs(*a++);
  }
  return s;
}

static float compute_distance(const float a[], const float b[], int n, float last_known_min_distance)
{
  float s;

  if ( n <= 64 ) {
    s = std::abs(*a++ - *b++);
    while ( --n ) {
      s += std::abs(*a++ - *b++);
    }
  }
  else {
    s = std::abs(*a++ - *b++);
    while ( --n && s <= last_known_min_distance ) {
      s += std::abs(*a++ - *b++);
    }
  }

  return s;
}


c_snorm_based_feature2d_matcher::c_snorm_based_feature2d_matcher()
{
}


void c_snorm_based_feature2d_matcher::set_max_acceptable_distance(double v)
{
  max_acceptable_distance_ = v;
}

double c_snorm_based_feature2d_matcher::max_acceptable_distance() const
{
  return max_acceptable_distance_;
}

void c_snorm_based_feature2d_matcher::set_lowe_ratio(double v)
{
  lowe_ratio_ = v;
}

double c_snorm_based_feature2d_matcher::lowe_ratio() const
{
  return lowe_ratio_;
}

bool c_snorm_based_feature2d_matcher::train(const std::vector<cv::KeyPoint> * train_keypoints, cv::InputArray train_descriptors)
{
  INSTRUMENT_REGION("");

  index_.clear();

  if ( train_descriptors.channels() != 1 ) {
    CF_ERROR("Invalid argument: multi-channel descriptors not supported");
    return false;
  }

  if ( train_descriptors.depth() == CV_32F ) {

    train_descriptors.copyTo(train_descriptors_);

  }
  else {

    train_descriptors.getMat().convertTo(train_descriptors_,
        train_descriptors_.depth());
  }

  if ( !train_descriptors.empty() ) {

    const int nrows =
        train_descriptors_.rows;

    const int ncols =
        train_descriptors_.cols;

    for ( int i = 0; i < nrows; ++i ) {

      index_.emplace_back(i,
          compute_norm(train_descriptors_[i],
              ncols));

    }

    std::sort(index_.begin(), index_.end(),
        [](const index_entry & prev, const index_entry & next) {
          return prev.norm < next.norm;
        });


    if ( false ) {

      FILE * fp = fopen("train.txt", "w");
      if ( fp ) {

        for ( uint i = 0, n = index_.size(); i < n; ++i ) {

          const index_entry & e =
              index_[i];

          const float * a =
              train_descriptors_[e.row];

          fprintf(fp, "%6u\t%12g    ", i, e.norm);
          for ( int j = 0; j < ncols; ++j ) {
            fprintf(fp, " %+9.3g", a[j]);
          }
          fprintf(fp, "\n");
        }

        fclose(fp);
      }

      // exit(0);
    }
  }


  return true;
}

bool c_snorm_based_feature2d_matcher::match(const std::vector<cv::KeyPoint> * query_keypoints, cv::InputArray _query_descriptors,
    /* out */ std::vector<cv::DMatch> & matches)
{
  INSTRUMENT_REGION("");

  if ( _query_descriptors.cols() != train_descriptors_.cols ) {
    CF_ERROR("train and query descriptor sizes not match: "
        "train.cols=%d query.cols=%d",
        train_descriptors_.cols,
        _query_descriptors.cols());
    return false;
  }

  if ( !train_descriptors_.empty() ) {

    const int ncols =
        train_descriptors_.cols;

    const float max_acceptable_distance =
        this->max_acceptable_distance_ >= 0 ?
            this->max_acceptable_distance_ :
            0.5f;

    cv::Mat1f query_descriptors;

    if ( _query_descriptors.depth() == CV_32F ) {
      query_descriptors = _query_descriptors.getMat();
      //_query_descriptors.copyTo(query_descriptors);
    }
    else {
      _query_descriptors.getMat().convertTo(query_descriptors,
          query_descriptors.depth());
    }


    matches.clear();
    matches.reserve(2 * query_descriptors.rows);

    using match_candidate =
        std::pair< float /*distance*/, std::vector<index_entry>::const_iterator /*pos*/>;

    std::vector<match_candidate> other_candidates;

    for ( int i = 0; i < query_descriptors.rows; ++i ) {

      const float * query_descriptor =
          query_descriptors[i];

      const float query_norm =
          compute_norm(query_descriptor,
              ncols);

      const float search_range_min =
          std::max(0.f, query_norm - max_acceptable_distance);

      const float search_range_max =
          query_norm + max_acceptable_distance;

      //CF_DEBUG("[%6d] query_norm=%g search_range_min=%g search_range_max=%g", i, query_norm, search_range_min, search_range_max);

      std::vector<index_entry>::const_iterator curpos;

      {
        INSTRUMENT_REGION("lower_bound");
        curpos =
            std::lower_bound(index_.begin(), index_.end(),
                search_range_min,
                [](const index_entry & e, float value) {
                  return e.norm < value;
                });
      }

      if ( curpos == index_.end() || curpos->norm > search_range_max ) {
        continue;
      }

      std::vector<index_entry>::const_iterator best_pos =
          curpos;

      float best_distance =
          compute_distance(query_descriptor,
              train_descriptors_[curpos->row],
              ncols,
              FLT_MAX);

      other_candidates.clear();

      {
        INSTRUMENT_REGION("range_search");

        while ( ++curpos != index_.end() && curpos->norm <= search_range_max ) {

          const float distance =
              compute_distance(query_descriptor,
                  train_descriptors_[curpos->row],
                  ncols,
                  best_distance);

          if ( distance <= best_distance ) {
            other_candidates.emplace_back(std::make_pair(best_distance, best_pos));
            best_pos = curpos;
            best_distance = distance;
          }
        }
      }

      if ( best_distance <= max_acceptable_distance ) {

        matches.emplace_back(cv::DMatch(i,
            best_pos->row,
            best_distance));

        if ( !other_candidates.empty() ) {

          if ( other_candidates.size() > 1 ) {

            std::sort(other_candidates.begin(), other_candidates.end(),
                [](const match_candidate & prev, const match_candidate & next) {
                  return prev.first < next.first;
                });

          }

          for ( std::vector<match_candidate>::const_iterator ii = other_candidates.begin();
              ii != other_candidates.end() && fabsf(ii->first - best_distance) <= FLT_EPSILON; ++ii ) {

            matches.emplace_back(cv::DMatch(i,
                ii->second->row,
                ii->first));

          }
        }

      }
    }

    /////////////////////
  }

  return true;
}
