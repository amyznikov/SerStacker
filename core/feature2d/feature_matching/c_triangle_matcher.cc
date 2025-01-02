/*
 * c_triangle_matcher.cc
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 *
 * References:
 *   "Star Stacker: Astrophotography with C++11"
 *    <https://benedikt-bitterli.me/astro>
 *
 *   Juan Domingo Velasquez, Francisco Gabriel Valdes,
 *   "FOCAS Automatic Catalog Matching Algorithms"
 *    <https://www.researchgate.net/publication/2828904_FOCAS_Automatic_Catalog_Matching_Algorithms>
 *
 */

#include "c_triangle_matcher.h"
#include <core/debug.h>

namespace {

// the sizeof for this struct MUST be multiple of sizeof(float)

#pragma pack(push, 4)
struct c_triangle_descriptor
{
  cv::Vec2f descriptor;
  cv::Vec3w triangle;
};
#pragma pack(pop)


template<class T>
static inline T hyp(T x, T y)
{
  return sqrt(x * x + y * y);
}


static void build_distances(const std::vector<cv::KeyPoint> & keypoints, cv::Mat1f & distances)
{
  const int n = keypoints.size();

  distances.create(n, n), distances.setTo(0);

  for ( int i = 0; i < n - 1; ++i ) {
    for ( int j = i + 1; j < n; ++j ) {
      distances[i][j] = distances[j][i] = hyp(keypoints[j].pt.x - keypoints[i].pt.x,
          keypoints[j].pt.y - keypoints[i].pt.y);
    }
  }
}

static void build_distances(const std::vector<cv::Point2f> & keypoints, cv::Mat1f & distances)
{
  const int n = keypoints.size();

  distances.create(n, n), distances.setTo(0);

  for ( int i = 0; i < n - 1; ++i ) {
    for ( int j = i + 1; j < n; ++j ) {
      distances[i][j] = distances[j][i] = hyp(keypoints[j].x - keypoints[i].x, keypoints[j].y - keypoints[i].y);
    }
  }
}

template<class PointType>
static bool build_triangles_(const std::vector<PointType> & keypoints,
    std::vector<c_triangle_descriptor> & descriptors,
    float min_side_size)
{
  cv::Mat1f distances;

  const int n = keypoints.size();


  build_distances(keypoints, distances);

  descriptors.clear(), descriptors.reserve((n * (n - 1) * (n - 2)) / 6);
  //descriptors.clear(), descriptors.reserve(triangles.size());


  for ( int s1 = 0; s1 < n - 2; ++s1 ) {
    for ( int s2 = s1 + 1; s2 < n - 1; ++s2 ) {
      for ( int s3 = s2 + 1; s3 < n; ++s3 ) {

        const float d12 = distances[s1][s2];
        const float d23 = distances[s2][s3];
        const float d13 = distances[s1][s3];

        if ( d12 < min_side_size || d23 < min_side_size || d13 < min_side_size ) {
          continue;
        }

//        cv::Vec3w triangle;
//        cv::Vec2f descripror;
        c_triangle_descriptor descriptor;
        float a, b, c;


        if ( (d12 >= d23) && (d12 >= d13) ) { /* longest side connects stars 1 and 2 */
          a = d12;
          descriptor.triangle[0] = s3;
          if ( d23 >= d13 ) {
            b = d23, c = d13;
            descriptor.triangle[1] = s1;
            descriptor.triangle[2] = s2;
          }
          else {
            b = d13, c = d23;
            descriptor.triangle[1] = s2;
            descriptor.triangle[2] = s1;
          }
        }
        else if ( (d23 > d12) && (d23 >= d13) ) { /* longest side connects stars 2 and 3 */
          a = d23;
          descriptor.triangle[0] = s1;
          if ( d12 > d13 ) {
            b = d12, c = d13;
            descriptor.triangle[1] = s3;
            descriptor.triangle[2] = s2;
          }
          else {
            b = d13, c = d12;
            descriptor.triangle[1] = s2;
            descriptor.triangle[2] = s3;
          }
        }
        else if ( (d13 > d12) && (d13 > d23) ) { /* longest side connects stars 1 and 3 */
          a = d13;
          descriptor.triangle[0] = s2;
          if ( d12 > d23 ) {
            b = d12, c = d23;
            descriptor.triangle[1] = s3;
            descriptor.triangle[2] = s1;
          }
          else {
            b = d23, c = d12;
            descriptor.triangle[1] = s1;
            descriptor.triangle[2] = s3;
          }
        }
        else {
          continue;
        }

        descriptor.descriptor[0] = b / a;
        descriptor.descriptor[1] = c / a;
        descriptors.emplace_back(descriptor);
      }
    }
  }

  return true;
}

static bool build_triangles(const std::vector<cv::KeyPoint> & keypoints,
    cv::OutputArray output_descriptors,
    float min_side_size)
{
  std::vector<c_triangle_descriptor> triangles;

  if ( !build_triangles_(keypoints, triangles, min_side_size) ) {
    CF_DEBUG("build_triangles_() fails");
    return false;
  }

  cv::Mat1b((int) triangles.size(), (int) sizeof(c_triangle_descriptor),
      (uint8_t*) triangles.data()).copyTo(output_descriptors);

  return true;
}
//
//static bool build_triangles(const std::vector<cv::Point2f> & keypoints,
//    cv::OutputArray output_descriptors,
//    float min_side_size)
//{
//  std::vector<c_triangle_descriptor> triangles;
//
//  if ( !build_triangles_(keypoints, triangles, min_side_size) ) {
//    CF_DEBUG("build_triangles_() fails");
//    return false;
//  }
//
//  cv::Mat1b((int) triangles.size(), (int) sizeof(triangles[0]),
//      (uint8_t*) triangles.data()).copyTo(output_descriptors);
//
//  return true;
//}

} /* namespace */


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_triangle_extractor::c_triangle_extractor(int min_side_size) :
    min_side_size_(min_side_size)
{
}

cv::Ptr<c_triangle_extractor> c_triangle_extractor::create(int min_side_size)
{
  return cv::Ptr<this_class>(new this_class(min_side_size));
}

void c_triangle_extractor::compute( cv::InputArray, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors)
{
  if ( !build_triangles(keypoints, descriptors, min_side_size_) ) {
    CF_ERROR("build_triangles() fails");
    descriptors.release();
  }

  CF_DEBUG("keypoints.size=%zu descriptors.rows=%d", keypoints.size(), descriptors.rows());
}

int c_triangle_extractor::descriptorSize() const
{
  return sizeof(c_triangle_descriptor);
}

int c_triangle_extractor::descriptorType() const
{
  return CV_8U;
}

int c_triangle_extractor::defaultNorm() const
{
  return cv::NORM_L2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_triangle_matcher::ptr create_sparse_feature_matcher(const c_triangle_matcher_options & options)
{
  c_triangle_matcher::ptr obj(new
      c_triangle_matcher(options.eps));

  return obj;
}

c_triangle_matcher::c_triangle_matcher(double eps) :
    eps_(eps)
{
}

c_triangle_matcher::ptr c_triangle_matcher::create(double eps)
{
  return ptr(new this_class(eps));
}

bool c_triangle_matcher::train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray reference_triangles)
{
  reference_triangles.getMat().copyTo(_reference_triangles);

  if( _reference_triangles.cols != sizeof(c_triangle_descriptor) ) {
    CF_ERROR("c_triangle_matcher: Invalid number of cols (%d) in triangle descriptor."
        "Must be %d",
        _reference_triangles.cols,
        (int )sizeof(c_triangle_descriptor));
    return false;
  }

  float * data =
      reinterpret_cast<float*>(_reference_triangles.data);

  const size_t rows = _reference_triangles.rows;
  const size_t cols = 2;
  const size_t stride = _reference_triangles.cols / sizeof(float);

  _reference_features = cvflann::Matrix<float>(data, rows, cols, stride);
  _index.reset(new cvflann::KDTreeIndex<DistanceType>(_reference_features, cvflann::KDTreeIndexParams(1)));
  _index->buildIndex();

  return !_index.empty();
}

bool c_triangle_matcher::match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
    /* out */ std::vector<cv::DMatch> & matches)
{
  const cv::Mat1b q =
      query_descriptors.getMat();

  if( q.cols != sizeof(c_triangle_descriptor) ) {
    CF_ERROR("c_triangle_matcher: Invalid number of cols (%d) in triangle descriptor."
        "Must be %d",
        q.cols,
        (int )sizeof(c_triangle_descriptor));
    return false;
  }

  int max_input_star_index = 0;
  for ( int i = 0, n = q.rows; i < n; ++i) {
    const c_triangle_descriptor * data =
        reinterpret_cast<const c_triangle_descriptor*>(q[i]);

    for ( int k = 0; k < 3; ++k ) {
      if ( data->triangle[k] > max_input_star_index ) {
        max_input_star_index = data->triangle[k];
      }
    }
  }

  int max_reference_star_index = 0;
  for ( int i = 0, n = _reference_triangles.rows; i < n; ++i) {

    const c_triangle_descriptor * data =
        reinterpret_cast<const c_triangle_descriptor*>(_reference_triangles[i]);

    for ( int k = 0; k < 3; ++k ) {
      if ( data->triangle[k] > max_reference_star_index ) {
        max_reference_star_index = data->triangle[k];
      }
    }
  }

  const int n1 = max_input_star_index + 1;
  const int n2 = max_reference_star_index + 1;

  cv::Mat1i votes(n1, n2, 0);

  // Use exact search
  cvflann::RadiusUniqueResultSet<DistanceType::ResultType> searchResult(eps_ * eps_);
  cvflann::SearchParams searchParams(cvflann::FLANN_CHECKS_UNLIMITED, 0, false);

  std::vector<int> indices;
  std::vector<DistanceType::ResultType> dists;

  for ( int i = 0, n = q.rows; i < n; ++i) {

    const c_triangle_descriptor * query =
        reinterpret_cast<const c_triangle_descriptor*>(q[i]);

    searchResult.clear();
    _index->findNeighbors(searchResult, (float*)query, searchParams);
    const size_t nnsize = searchResult.size();

    if ( nnsize > 0 ) {

      indices.resize(nnsize);
      dists.resize(nnsize);
      searchResult.copy(&indices[0], &dists[0], -1);

      for ( size_t j = 0; j < nnsize; ++j ) {

        const c_triangle_descriptor *reference =
            reinterpret_cast<const c_triangle_descriptor*>(
                _reference_triangles[indices[j]]);

        for ( int k = 0; k < 3; ++k ) {
          ++votes[query->triangle[k]][reference->triangle[k]];
        }
      }
    }
  }

  matches.clear();
  matches.reserve(n1);

  for ( int i = 0; i < n1; ++i ) {

    int best_vote_i2j = 0;
    int best_j = -1;

    for ( int j = 0; j < n2; ++j ) {
      if ( votes[i][j] > best_vote_i2j ) {
        best_vote_i2j = votes[i][j];
        best_j = j;
      }
    }

    if ( best_j < 0 ) {
      continue;
    }

    for ( int k = 0; k < n1; ++k ) {
      if ( k != i && votes[k][best_j] >= best_vote_i2j ) {
        best_vote_i2j = 0;
        break;
      }
    }

    if ( best_vote_i2j < 1 ) {
      continue;
    }


    //matches.emplace_back(std::make_pair((uint16_t)i, (uint16_t)best_j));

    matches.emplace_back(cv::DMatch(i, best_j, 1.f / best_vote_i2j));

  }

  return true;
}
