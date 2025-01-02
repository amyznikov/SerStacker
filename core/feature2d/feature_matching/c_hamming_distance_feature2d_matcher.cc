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

    for ( int i = 0; i < nrows; ++i ) {

      _index.emplace_back(i,
          hamming_norm(_train_descriptors[i], ncols),
          train_keypoints[i].octave);
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

    const int max_acceptable_distance =
        this->_max_acceptable_distance >= 0 ?
            this->_max_acceptable_distance :
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

    std::vector<match_candidate> other_candidates;

    for ( int i = 0; i < query_descriptors.rows; ++i ) {

      const cv::KeyPoint & query_keypoint =
          _query_keypoints[i];

      const int32_t * query_descriptor =
          query_descriptors[i];

      const int query_norm =
          hamming_norm(query_descriptor,
              nc);

      const int search_range_min =
          std::max(0, query_norm - max_acceptable_distance);

      const int search_range_max =
          query_norm + max_acceptable_distance;

      std::vector<index_entry>::const_iterator curpos =
          std::lower_bound(_index.begin(), _index.end(),
              search_range_min,
              [](const index_entry & e, int value) {
                return e.norm < value;
              });

      if ( curpos == _index.end() || curpos->norm > search_range_max ) {
        continue;
      }

      std::vector<index_entry>::const_iterator best_pos =
          curpos;

      int best_distance =
          hamming_distance(query_descriptor,
              _train_descriptors[curpos->row],
              nc);

      other_candidates.clear();

      while ( ++curpos != _index.end() && curpos->norm <= search_range_max ) {

        if( _octavedif >= 0 && std::abs((int) curpos->octave - (int) query_keypoint.octave) > _octavedif ) {
          continue;
        }

        const int32_t * train_descriptor =
            _train_descriptors[curpos->row];

        const int distance =
            hamming_distance(query_descriptor,
                train_descriptor,
                nc);

        if ( distance > best_distance ) {
          continue;
        }

        other_candidates.emplace_back(std::make_pair(best_distance, best_pos));
        best_pos = curpos;
        best_distance = distance;
      }

      if ( best_distance <= max_acceptable_distance ) {

        matches.emplace_back(cv::DMatch(i,
            best_pos->row,
            best_distance));

        if( !other_candidates.empty() ) {

          if ( other_candidates.size() > 1 ) {
            std::sort(other_candidates.begin(), other_candidates.end(),
                [](const match_candidate & prev, const match_candidate & next) {
                  return prev.first < next.first;
                });
          }

          for ( std::vector<match_candidate>::const_iterator ii = other_candidates.begin();
              ii != other_candidates.end() && ii->first == best_distance; ++ii ) {

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
