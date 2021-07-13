/*
 * starmatch.cc
 *
 *  Created on: Dec 1, 2019
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

#include "starmatch.h"


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
    std::vector<cv::Vec3w> & triangles,
    std::vector<cv::Vec2f> & descriptors,
    float min_side_size)
{
  cv::Mat1f distances;

  const int n = keypoints.size();


  build_distances(keypoints, distances);

  triangles.clear(), triangles.reserve((n * (n - 1) * (n - 2)) / 6);
  descriptors.clear(), descriptors.reserve(triangles.size());


  for ( int s1 = 0; s1 < n - 2; ++s1 ) {
    for ( int s2 = s1 + 1; s2 < n - 1; ++s2 ) {
      for ( int s3 = s2 + 1; s3 < n; ++s3 ) {

        const float d12 = distances[s1][s2];
        const float d23 = distances[s2][s3];
        const float d13 = distances[s1][s3];

        if ( d12 < min_side_size || d23 < min_side_size || d13 < min_side_size ) {
          continue;
        }

        cv::Vec3w triangle;
        cv::Vec2f descripror;
        float a, b, c;


        if ( (d12 >= d23) && (d12 >= d13) ) { /* longest side connects stars 1 and 2 */
          a = d12;
          triangle[0] = s3;
          if ( d23 >= d13 ) {
            b = d23, c = d13;
            triangle[1] = s1;
            triangle[2] = s2;
          }
          else {
            b = d13, c = d23;
            triangle[1] = s2;
            triangle[2] = s1;
          }
        }
        else if ( (d23 > d12) && (d23 >= d13) ) { /* longest side connects stars 2 and 3 */
          a = d23;
          triangle[0] = s1;
          if ( d12 > d13 ) {
            b = d12, c = d13;
            triangle[1] = s3;
            triangle[2] = s2;
          }
          else {
            b = d13, c = d12;
            triangle[1] = s2;
            triangle[2] = s3;
          }
        }
        else if ( (d13 > d12) && (d13 > d23) ) { /* longest side connects stars 1 and 3 */
          a = d13;
          triangle[0] = s2;
          if ( d12 > d23 ) {
            b = d12, c = d23;
            triangle[1] = s3;
            triangle[2] = s1;
          }
          else {
            b = d23, c = d12;
            triangle[1] = s1;
            triangle[2] = s3;
          }
        }
        else {
          continue;
        }

        descripror[0] = b / a;
        descripror[1] = c / a;

        triangles.emplace_back(triangle);
        descriptors.emplace_back(descripror);
      }
    }
  }

  return true;
}




template<class PointType>
static void match_triangles_(
    const std::vector<PointType> & input_stars, const std::vector<PointType> & reference_stars,
    const std::vector<cv::Vec3w> & input_triangles, const std::vector<cv::Vec3w> & reference_triangles,
    const std::vector<cv::Vec2f> & input_descriptors, const std::vector<cv::Vec2f> & reference_descriptors,
    cvflann::KDTreeIndex<StarMathingDistanceType> & index,
    /*out */ std::vector<std::pair<uint16_t, uint16_t> > & matches,
    float eps)
{
  const int n1 = input_stars.size();
  const int n2 = reference_stars.size();

  cv::Mat1i votes(n1, n2, 0);

  // Use exact search
  cvflann::RadiusUniqueResultSet<StarMathingDistanceType::ResultType> searchResult(eps * eps);
  cvflann::SearchParams searchParams(cvflann::FLANN_CHECKS_UNLIMITED, 0, false);

  std::vector<int> indices;
  std::vector<StarMathingDistanceType::ResultType> dists;

  for ( int i = 0, n = input_descriptors.size(); i < n; ++i) {

    searchResult.clear();
    index.findNeighbors(searchResult, (const float*) &input_descriptors[i], searchParams);
    const size_t nnsize = searchResult.size();

    if ( nnsize > 0 ) {

      indices.resize(nnsize);
      dists.resize(nnsize);
      searchResult.copy(&indices[0], &dists[0], -1);

      for ( size_t j = 0; j < nnsize; ++j ) {
        for ( int k = 0; k < 3; ++k ) {
          ++votes[input_triangles[i][k]][reference_triangles[indices[j]][k]];
        }
      }
    }
  }

  matches.clear(), matches.reserve(input_stars.size());

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


    matches.emplace_back(std::make_pair((uint16_t)i, (uint16_t)best_j));
  }
}



bool build_triangles(const std::vector<cv::KeyPoint> & keypoints,
    std::vector<cv::Vec3w> & triangles,
    std::vector<cv::Vec2f> & descriptors,
    float min_side_size)
{
  return build_triangles_(keypoints, triangles, descriptors, min_side_size);
}

bool build_triangles(const std::vector<cv::Point2f> & keypoints,
    std::vector<cv::Vec3w> & triangles,
    std::vector<cv::Vec2f> & descriptors,
    float min_side_size)
{
  return build_triangles_(keypoints, triangles, descriptors, min_side_size);
}



cv::Ptr<cvflann::KDTreeIndex<StarMathingDistanceType>> index_triangles(std::vector<cv::Vec2f> & descriptors)
{
  cvflann::Matrix<StarMathingDistanceType::ElementType> features((float*) &descriptors.front(), descriptors.size(), 2);
  cv::Ptr<cvflann::KDTreeIndex<StarMathingDistanceType>> index(new cvflann::KDTreeIndex<StarMathingDistanceType>(features, cvflann::KDTreeIndexParams(1)));
  index->buildIndex();
  return index;
}

void match_triangles(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<cv::Vec3w> & input_triangles, const std::vector<cv::Vec3w> & reference_triangles,
    const std::vector<cv::Vec2f> & input_descriptors, const std::vector<cv::Vec2f> & reference_descriptors,
    cvflann::KDTreeIndex<StarMathingDistanceType> & reference_index,
    std::vector<std::pair<uint16_t, uint16_t> > & matches,
    float eps)
{
  return match_triangles_(input_stars, reference_stars,
      input_triangles, reference_triangles,
      input_descriptors, reference_descriptors,
      reference_index,
      matches,
      eps);
}

void match_triangles(const std::vector<cv::Point2f> & input_stars, const std::vector<cv::Point2f> & reference_stars,
    const std::vector<cv::Vec3w> & input_triangles, const std::vector<cv::Vec3w> & reference_triangles,
    const std::vector<cv::Vec2f> & input_descriptors, const std::vector<cv::Vec2f> & reference_descriptors,
    cvflann::KDTreeIndex<StarMathingDistanceType> & reference_index,
    std::vector<std::pair<uint16_t, uint16_t> > & matches,
    float eps)
{
  return match_triangles_(input_stars, reference_stars,
      input_triangles, reference_triangles,
      input_descriptors, reference_descriptors,
      reference_index,
      matches,
      eps);
}


void extract_matches(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::Point2f> * input_positions,
    std::vector<cv::Point2f> * reference_positions )
{
  input_positions->clear(), input_positions->reserve(matches.size());
  reference_positions->clear(), reference_positions->reserve(matches.size());
  for ( const auto & m : matches ) {
    input_positions->emplace_back(input_stars[m.first].pt);
    reference_positions->emplace_back(reference_stars[m.second].pt);
  }
}


void extract_matches(const std::vector<cv::KeyPoint> & input_stars, const std::vector<cv::KeyPoint> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::KeyPoint> * input_positions,
    std::vector<cv::KeyPoint> * reference_positions )
{
  input_positions->clear(), input_positions->reserve(matches.size());
  reference_positions->clear(), reference_positions->reserve(matches.size());
  for ( const auto & m : matches ) {
    input_positions->emplace_back(input_stars[m.first]);
    reference_positions->emplace_back(reference_stars[m.second]);
  }
}



void extract_matches(const std::vector<cv::Point2f> & input_stars, const std::vector<cv::Point2f> & reference_stars,
    const std::vector<std::pair<uint16_t, uint16_t> > & matches,
    std::vector<cv::Point2f> * input_positions,
    std::vector<cv::Point2f> * reference_positions )
{
  input_positions->clear(), input_positions->reserve(matches.size());
  reference_positions->clear(), reference_positions->reserve(matches.size());
  for ( const auto & m : matches ) {
    input_positions->emplace_back(input_stars[m.first]);
    reference_positions->emplace_back(reference_stars[m.second]);
  }

}
