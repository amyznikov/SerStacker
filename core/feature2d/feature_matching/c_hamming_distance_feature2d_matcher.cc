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

  obj->set_max_acceptable_distance(
      options.max_acceptable_distance);

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
  max_acceptable_distance_ = v;
}

int c_hamming_distance_feature2d_matcher::max_acceptable_distance() const
{
  return max_acceptable_distance_;
}

bool c_hamming_distance_feature2d_matcher::train(const std::vector<cv::KeyPoint> * train_keypoints, cv::InputArray train_descriptors)
{
  index_.clear();

  if ( train_descriptors.type() != CV_8U ) {
    CF_ERROR("train_descriptors.type()=%d not match to expected descriptor type CV_8U",
        train_descriptors.type());
    return false;
  }

  if ( !train_descriptors.empty() ) {

    convert_descriptors(train_descriptors.getMat(),
        train_descriptors_);

    const int nrows =
        train_descriptors_.rows;

    const int ncols =
        train_descriptors_.cols;

    for ( int i = 0; i < nrows; ++i ) {

      index_.emplace_back(i,
          hamming_norm(train_descriptors_[i],
              ncols));

    }

    std::sort(index_.begin(), index_.end(),
        [](const index_entry & prev, const index_entry & next) {
          return prev.norm < next.norm;
        });



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

        for ( uint i = 0, n = index_.size(); i < n; ++i ) {

          const index_entry & e =
              index_[i];

          const int32_t * a =
              train_descriptors_[e.row];

          fprintf(fp, "%9u   ", e.norm);
          for ( int j = 0; j < ncols; ++j ) {
            fprintf(fp, " %s", tobinary(a[j]));
          }
          fprintf(fp, "\n");
        }

        fclose(fp);
      }
    }
  }


  return true;
}

bool c_hamming_distance_feature2d_matcher::match(const std::vector<cv::KeyPoint> * _query_keypoints, cv::InputArray _query_descriptors,
    std::vector<cv::DMatch> & matches)
{
  const int query_descriptor_size_in_bytes =
      _query_descriptors.cols();

  if ( ((query_descriptor_size_in_bytes + 3) / 4) != train_descriptors_.cols ) {
    CF_ERROR("train and query descriptor sizes not match: "
        "train.cols=%d (query.cols+3)/4=%d",
        train_descriptors_.cols,
        (query_descriptor_size_in_bytes + 3) / 4);
    return false;
  }


  if ( !train_descriptors_.empty() ) {

    const unsigned int nc =
        train_descriptors_.cols;

    const int max_acceptable_distance =
        this->max_acceptable_distance_ >= 0 ?
            this->max_acceptable_distance_ :
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
          std::lower_bound(index_.begin(), index_.end(),
              search_range_min,
              [](const index_entry & e, int value) {
                return e.norm < value;
              });

      if ( curpos == index_.end() || curpos->norm > search_range_max ) {
        continue;
      }

      std::vector<index_entry>::const_iterator best_pos =
          curpos;

      int best_distance =
          hamming_distance(query_descriptor,
              train_descriptors_[curpos->row],
              nc);

      other_candidates.clear();

      while ( ++curpos != index_.end() && curpos->norm <= search_range_max ) {

        const int32_t * train_descriptor =
            train_descriptors_[curpos->row];

        const int distance =
            hamming_distance(query_descriptor,
                train_descriptor,
                nc);

        if ( distance <= best_distance ) {
          other_candidates.emplace_back(std::make_pair(best_distance, best_pos));
          best_pos = curpos;
          best_distance = distance;
        }
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
