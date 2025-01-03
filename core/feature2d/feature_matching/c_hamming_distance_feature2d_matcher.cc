/*
 * c_hamming_distance_feature2d_matcher.cc
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#include "c_hamming_distance_feature2d_matcher.h"
#include <core/debug.h>

#ifdef _MSC_VER
#  include <intrin.h>
#  define __builtin_popcount __popcnt
#endif


c_hamming_distance_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_hamming_distance_feature2d_matcher_options & options)
{
  c_hamming_distance_feature2d_matcher::ptr obj(
      new c_hamming_distance_feature2d_matcher());

  obj->set_max_acceptable_distance(options.max_acceptable_distance);
  obj->set_octavedif(options.octavedif);

  return obj;
}


static inline void copy_descriptor_rows(const cv::Mat1b & src, cv::Mat1i & dst)
{
  for ( int i = 0; i < src.rows; ++i ) {
    memcpy(dst[i], src[i], src.cols);
  }
}

static inline void convert_descriptors(const cv::Mat1b & src, cv::Mat1i & dst)
{
  dst.create(src.rows, (src.cols + 3) / 4);
  dst.setTo(0);
  copy_descriptor_rows(src, dst);
}



// Set add_compile_options(-march=native) in CMakeLists.txt
// DON'T ALLOW n = 0.
static inline unsigned int hamming_norm(const int32_t a[], unsigned int n)
{
  unsigned int s = __builtin_popcount(*a++);
  while ( --n  ) {
    s += __builtin_popcount(*a++);
  }
  return s;
}

// Set add_compile_options(-march=native) in CMakeLists.txt
// DON'T ALLOW n = 0.
static inline unsigned int hamming_distance(const int32_t a[], const int32_t b[], unsigned int n)
{
  unsigned int s = __builtin_popcount(*a++ ^ *b++);
  while ( --n ) {
    s += __builtin_popcount(*a++ ^ *b++);
  }
  return s;
}


void c_hamming_distance_feature2d_matcher::set_max_acceptable_distance(int v)
{
  _max_acceptable_distance = v;
}

int c_hamming_distance_feature2d_matcher::max_acceptable_distance() const
{
  return _max_acceptable_distance;
}

bool c_hamming_distance_feature2d_matcher::train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors)
{
  _index.clear();

  if ( train_descriptors.type() != CV_8U ) {
    CF_ERROR("train_descriptors.type()=%d not match to expected descriptor type CV_8U",
        train_descriptors.type());
    return false;
  }

  if( train_keypoints.size() != train_descriptors.rows() ) {
    CF_ERROR("train_keypoints.size=%zu not equal to train_descriptors.rows()=%d",
        train_keypoints.size(), train_descriptors.rows());
    return false;
  }

  if ( !train_descriptors.empty() ) {

    convert_descriptors(train_descriptors.getMat(),
        _train_descriptors);

    const int nrows =
        _train_descriptors.rows;

    const int ncols =
        _train_descriptors.cols;

    _index.reserve(nrows);

    for ( int i = 0; i < nrows; ++i ) {

      _index.emplace_back(i,
          hamming_norm(_train_descriptors[i], ncols),
          train_keypoints[i].octave,
          train_keypoints[i].response);
    }

    std::sort(_index.begin(), _index.end(),
        [](const index_entry & prev, const index_entry & next) {
          return prev.norm < next.norm;
        });

#if 0
    if ( false ) {

      FILE * fp = fopen("train.txt", "w");
      if ( fp ) {

        static const auto tobinary =
            [](int32_t x) -> const char * {
              static char buf[16] = "";
              for ( int i = 8 * sizeof(x) -1; i >= 0; --i ) {
                buf[7-i] = (x & (1<<i)) ? '1' : '0';
              }
              return buf;
            };

        for ( uint i = 0, n = _index.size(); i < n; ++i ) {

          const index_entry & e =
              _index[i];

          const int32_t * a =
              _train_descriptors[e.row];

          fprintf(fp, "%9u   ", e.norm);
          for ( int j = 0; j < ncols; ++j ) {
            fprintf(fp, " %s", tobinary(a[j]));
          }
          fprintf(fp, "\n");
        }

        fclose(fp);
      }
    }
#endif
  }


  return true;
}

bool c_hamming_distance_feature2d_matcher::match(const std::vector<cv::KeyPoint> & _query_keypoints, cv::InputArray _query_descriptors,
    std::vector<cv::DMatch> & matches)
{
  const int query_descriptor_size_in_bytes =
      _query_descriptors.cols();

  if ( ((query_descriptor_size_in_bytes + 3) / 4) != _train_descriptors.cols ) {
    CF_ERROR("train and query descriptor sizes not match: "
        "train.cols=%d (query.cols+3)/4=%d",
        _train_descriptors.cols,
        (query_descriptor_size_in_bytes + 3) / 4);
    return false;
  }

  if( _query_keypoints.size() != _query_descriptors.rows() ) {
    CF_ERROR("_query_keypoints.size=%zu not equal to query_descriptors.rows()=%d",
        _query_keypoints.size(), _query_descriptors.rows());
    return false;
  }

  if ( !_train_descriptors.empty() ) {

    const unsigned int nc =
        _train_descriptors.cols;

    const unsigned int max_acceptable_distance =
        this->_max_acceptable_distance >= 0 ? (unsigned int) this->_max_acceptable_distance :
            query_descriptor_size_in_bytes / 2;

    cv::Mat1i query_descriptors(_query_descriptors.rows(),
        (_query_descriptors.cols() + 3) / 4,
        (int)(0));

    copy_descriptor_rows(_query_descriptors.getMat(),
        query_descriptors);

    matches.clear();
    matches.reserve(query_descriptors.rows);

    using match_candidate =
        std::pair< int /*distance*/, std::vector<index_entry>::const_iterator /*pos*/>;

    //std::vector<match_candidate> candidates;

    for ( int i = 0; i < query_descriptors.rows; ++i ) {

      const cv::KeyPoint & keypoint =
          _query_keypoints[i];

      const int32_t * query_descriptor =
          query_descriptors[i];

      const unsigned int query_norm =
          hamming_norm(query_descriptor,
              nc);

      const unsigned int search_range_min =
          query_norm <= max_acceptable_distance ? 0 :
              query_norm - max_acceptable_distance;

      const unsigned int search_range_max =
          query_norm >= UINT_MAX - max_acceptable_distance ? UINT_MAX :
              query_norm + max_acceptable_distance;

      const int octave =
          keypoint.octave;

      const float & response =
          keypoint.response;

      unsigned int best_distance =
          max_acceptable_distance + 1;

      std::vector<index_entry>::const_iterator best_pos =
          _index.end();

      std::vector<index_entry>::const_iterator curpos =
          std::lower_bound(_index.begin(), _index.end(), search_range_min,
              [](const index_entry & e, unsigned int value) {
                return e.norm < value;
              });

      for( ; curpos != _index.end() && curpos->norm <= search_range_max; ++curpos ) {
        if( _octavedif < 0 || std::abs(octave - curpos->octave) <= _octavedif ) {

          const unsigned int distance =
              hamming_distance(query_descriptor,
                  _train_descriptors[curpos->row],
                  nc);

          if( distance < best_distance ) {
            best_distance = distance;
            best_pos = curpos;
          }
          else if( distance == best_distance && best_pos != _index.end() ) {
            if( std::abs(octave - curpos->octave) < std::abs(octave - best_pos->octave) ) {
              best_pos = curpos;
            }
            else if( std::abs(response - curpos->responce) < std::abs(response - best_pos->responce) ) {
              best_pos = curpos;
            }
          }
        }
      }

      if( best_pos != _index.end() ) {
        matches.emplace_back(cv::DMatch(i, best_pos->row,
            best_distance));
      }
    }

    /////////////////////
  }

  return true;
}
