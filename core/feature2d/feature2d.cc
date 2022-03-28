/*
 * feature2d.cc
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#include "feature2d.h"
#include <core/readdir.h>
#include <core/debug.h>




/**
 * save_matched_points_in_octave_format()
 *
 * Dump matched keypoints (inliers only) into specified text file in octave format.
 * If image_size is not empty then all keypoimnts will shifted relative to image center as asked iuspeniev.
 */
bool save_matched_points_in_octave_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & matched_reference_keypoints,
    const std::vector<cv::Point2f> & matched_current_keypoints,
    const cv::InputArray inliers_mask,
    const cv::Size & image_size)
{

  // check input arguments
  if ( matched_reference_keypoints.size() != matched_current_keypoints.size() ) {
    CF_ERROR("Invalid args: matched_reference_keypoints.size()=%zu not equal to matched_current_keypoints.size()=%zu",
        matched_reference_keypoints.size(), matched_current_keypoints.size());
    return false;
  }

  if ( !inliers_mask.empty() )  {

    if ( inliers_mask.rows() != matched_reference_keypoints.size() ) {
      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%zu",
          inliers_mask.rows(), matched_reference_keypoints.size());
      return false;
    }

    if ( inliers_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid args: incorrect inliers_mask.type()=%d. Must be CV_8UC1",
          inliers_mask.type());
      return false;
    }
  }

  const cv::Mat1b M =
      inliers_mask.getMat();

  const cv::Point C = image_size.empty() ?
      cv::Point(0, 0) :
      cv::Point(image_size.width / 2, image_size.height / 2);

  const int N = M.empty() ?
      matched_reference_keypoints.size() :
      cv::countNonZero(M);

  CF_DEBUG("Num inliers = %d / %d", N, M.rows);

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen(%s) fails: %s", output_file_name.c_str(), strerror(errno));
    return false;
  }

  fprintf(fp, "reference_keypoints=[\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      fprintf(fp, "%+15.3f", r.x);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      if ( image_size.empty() ) {
        fprintf(fp, "%+15.3f", r.y);
      }
      else {
        fprintf(fp, "%+15.3f", C.y - r.y);
      }
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      fprintf(fp, "%+15.3f", 1.);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, "\n];\n");

  fprintf(fp, "current_keypoints=[\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      fprintf(fp, "%+15.3f", c.x);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      if ( image_size.empty() ) {
        fprintf(fp, "%+15.3f", c.y);
      }
      else {
        fprintf(fp, "%+15.3f", C.y - c.y);
      }
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      fprintf(fp, "%+15.3f", 1.);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, "\n];\n");

  fclose(fp);

  return true;
}


/**
 * save_matches_in_csv_format()
 *
 * Dump matched keypoints into specified text file in csv format.
 */
bool save_matches_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::DMatch> & matches,
    const std::vector<cv::KeyPoint> & query_keypoints,
    const std::vector<cv::KeyPoint> & train_keypoints,
    cv::InputArray mask)
{
  cv::Mat1b M;

  if ( !mask.empty() ) {

    if ( mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type: %d. Must be CV_8UC1",
          mask.type());
      return false;
    }

    if ( mask.rows() != matches.size() ) {
      CF_ERROR("Invalid mask size: %dx1. Must be %zux1",
          mask.rows(), matches.size());
      return false;
    }

    M = mask.getMat();
  }


  const std::string filepath =
      get_parent_directory(output_file_name);

  if ( !filepath.empty() && !create_path(filepath) ) {

    CF_ERROR("create_path('%s') fails: %s",
        filepath.c_str(),
        strerror(errno));

    return false;
  }

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {

    CF_ERROR("fopen(%s) fails: %s",
        output_file_name.c_str(),
        strerror(errno));

    return false;
  }


  fprintf(fp, "I\ttx\tty\ttr\ttl\tqx\tqy\tqr\tql\tdistance\n");


  for ( uint i = 0, n = matches.size(); i < n; ++i ) {

    if ( M.empty() || M[i][0] ) {

      const cv::DMatch & m =
          matches[i];

      const cv::KeyPoint & pt =
          train_keypoints[m.trainIdx];

      const cv::KeyPoint & pq =
          query_keypoints[m.queryIdx];

      fprintf(fp, "%6u"
          "\t%12.3f\t%12.3f\t%g\t%d"
          "\t%12.3f\t%12.3f\t%g\t%d"
          "\t%g"
          "\n",
          i,
          pt.pt.x, pt.pt.y, pt.response, pt.octave,
          pq.pt.x, pq.pt.y, pq.response, pq.octave,
          m.distance
          );
    }
  }


  fclose(fp);

  return true;
}

/**
 * save_matched_positions_in_csv_format()
 *
 * Utility routine to dump matched positions into text file in CSV format.
 *
 */
bool save_matched_positions_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & query_keypoints,
    const std::vector<cv::Point2f> & train_keypoints,
    cv::InputArray mask)
{
  cv::Mat1b M;

  if ( query_keypoints.size() != train_keypoints.size() ) {
    CF_ERROR("query_keypoints.size=%zu not equal to train_keypoints.size=%zu",
        query_keypoints.size(), train_keypoints.size());
    return false;
  }

  if ( !mask.empty() ) {

    if ( mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type: %d. Must be CV_8UC1. Mask ignored",
          mask.type());
    }
    else if ( mask.rows() != query_keypoints.size() ) {
      CF_ERROR("Invalid mask size: %dx%d. Must be %zux1. Mask ignored",
          mask.rows(), mask.cols(),
          query_keypoints.size());
    }
    else {
      M = mask.getMat();
    }
  }

  const std::string filepath =
      get_parent_directory(output_file_name);

  if ( !filepath.empty() && !create_path(filepath) ) {
    CF_ERROR("create_path('%s') fails: %s", filepath.c_str(), strerror(errno));
    return false;
  }

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen(%s) fails: %s", output_file_name.c_str(),
        strerror(errno));
    return false;
  }

  fprintf(fp, "I\ttx\tty\tqx\tqy\n");



  for ( uint i = 0, n = query_keypoints.size(); i < n; ++i ) {

    if ( M.empty() || M[i][0] ) {

      const cv::Point2f & pt =
          train_keypoints[i];

      const cv::Point2f & pq =
          query_keypoints[i];

      fprintf(fp, "%6u"
          "\t%12.3f\t%12.3f"
          "\t%12.3f\t%12.3f"
          "\n",
          i,
          pt.x, pt.y,
          pq.x, pq.y
          );
    }
  }


  fclose(fp);

  return true;
}

bool draw_matched_positions(cv::OutputArray _all_matches_image,
    cv::OutputArray _selected_matches_image,
    cv::InputArray _current_image,
    cv::InputArray _reference_image,
    const std::vector<cv::Point2f> & current_positions,
    const std::vector<cv::Point2f> & reference_positions,
    cv::InputArray _mask)
{
  INSTRUMENT_REGION("");


  if ( !_all_matches_image.needed() && !_selected_matches_image.needed() ) {
    return true;
  }

  if ( current_positions.size() != reference_positions.size() ) {
    CF_ERROR("current_positions.size()=%zu not match to reference_positions.size()=%zu",
        current_positions.size(), reference_positions.size());
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("Empty current image specified");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("Empty reference image specified");
    return false;
  }

  if ( !_mask.empty() ) {

    if ( _mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type %d: must be CV_8UC1",
          _mask.type());
      return false;
    }

    if ( _mask.rows() != (int) current_positions.size() ) {
      CF_ERROR("Invalid mask size %dx%d: mask must be of CV_8UC1 type and %dx%d of the size",
          _mask.rows(), _mask.cols(), (int )current_positions.size(), 1);
      return false;
    }
  }



  cv::Mat current_image, reference_image;

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, current_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    current_image =
        _current_image.getMat();
  }

  if ( _reference_image.channels() == 1 ) {
    cv::cvtColor(_reference_image, reference_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    reference_image =
        _reference_image.getMat();
  }

  const cv::Mat1b mask =
      _mask.getMat();


  cv::Size current_image_size =
      current_image.size();

  cv::Size reference_image_size =
      reference_image.size();

  cv::Size dst_size;

  enum {
    LAYOUT_HORZ,
    LAYOUT_VERT,
  } layout_type;


  if ( current_image_size.width + reference_image_size.width >= current_image_size.height + reference_image_size.height ) {
    dst_size.width = std::max(current_image_size.width, reference_image_size.width);
    dst_size.height = current_image_size.height + reference_image_size.height;
    layout_type = LAYOUT_VERT;
  }
  else {
    dst_size.width = current_image_size.width + reference_image_size.width;
    dst_size.height = std::max(current_image_size.height, reference_image_size.height);
    layout_type = LAYOUT_HORZ;
  }


  cv::Mat all_matches_image;
  cv::Mat selected_matches_image;

  if ( _all_matches_image.needed() ) {

    _all_matches_image.create(
        dst_size,
        CV_8UC3);

    all_matches_image =
        _all_matches_image.getMatRef();

    current_image.copyTo(all_matches_image(cv::Rect(0, 0,
        current_image_size.width,
        current_image_size.height)));
  }

  if ( _selected_matches_image.needed() ) {

    _selected_matches_image.create(
        dst_size,
        CV_8UC3);

    selected_matches_image =
        _selected_matches_image.getMatRef();

    current_image.copyTo(selected_matches_image(cv::Rect(0, 0,
            current_image_size.width,
            current_image_size.height)));
  }


  switch ( layout_type ) {
  case LAYOUT_VERT :


    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = current_positions.size(); i < n; ++i ) {

      const cv::Point2f & cp =
          current_positions[i];

      const cv::Point2f rp =
          cv::Point2f(reference_positions[i].x,
              reference_positions[i].y + current_image_size.height);

      const cv::Scalar c(
          rand() % 255,
          rand() % 255,
          rand() % 255);

      if ( !all_matches_image.empty() ) {
        cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
      }

      if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
        cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
      }
    }

    break;

  case LAYOUT_HORZ :

    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = current_positions.size(); i < n; ++i ) {

      const cv::Point2f & cp =
          current_positions[i];

      const cv::Point2f rp =
          cv::Point2f(reference_positions[i].x + current_image_size.width,
              reference_positions[i].y);

      const cv::Scalar c(
          rand() % 255,
          rand() % 255,
          rand() % 255);

      if ( !all_matches_image.empty() ) {
        cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
      }

      if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
        cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
      }
    }
    break;
  }


  return true;
}


/**
 * draw_matched_keyppints()
 *
 * Utility routine to draw sparse feature matches.
 *
 * Similar to cv::drawMatches()
 *
 */
bool draw_matched_keyppints(cv::OutputArray _all_matches_image,
    cv::OutputArray _selected_matches_image,
    cv::InputArray _current_image,
    cv::InputArray _reference_image,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch> & matches,
    cv::InputArray _mask)
{
  INSTRUMENT_REGION("");


  if ( !_all_matches_image.needed() && !_selected_matches_image.needed() ) {
    return true;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("Empty current image specified");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("Empty reference image specified");
    return false;
  }

  if ( !_mask.empty() ) {

    if ( _mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type %d: must be CV_8UC1",
          _mask.type());
      return false;
    }

    if ( _mask.rows() != (int) matches.size() ) {
      CF_ERROR("Invalid mask size %dx%d: mask must be of CV_8UC1 type and %dx%d of the size",
          _mask.rows(), _mask.cols(), (int )matches.size(), 1);
      return false;
    }
  }



  cv::Mat current_image, reference_image;

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, current_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    current_image =
        _current_image.getMat();
  }

  if ( _reference_image.channels() == 1 ) {
    cv::cvtColor(_reference_image, reference_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    reference_image =
        _reference_image.getMat();
  }

  const cv::Mat1b mask =
      _mask.getMat();


  cv::Size current_image_size =
      current_image.size();

  cv::Size reference_image_size =
      reference_image.size();

  cv::Size dst_size;

  enum {
    LAYOUT_HORZ,
    LAYOUT_VERT,
  } layout_type;


  if ( current_image_size.width + reference_image_size.width >= current_image_size.height + reference_image_size.height ) {
    dst_size.width = std::max(current_image_size.width, reference_image_size.width);
    dst_size.height = current_image_size.height + reference_image_size.height;
    layout_type = LAYOUT_VERT;
  }
  else {
    dst_size.width = current_image_size.width + reference_image_size.width;
    dst_size.height = std::max(current_image_size.height, reference_image_size.height);
    layout_type = LAYOUT_HORZ;
  }


  cv::Mat all_matches_image;
  cv::Mat selected_matches_image;

  if ( _all_matches_image.needed() ) {

    _all_matches_image.create(
        dst_size,
        CV_8UC3);

    all_matches_image =
        _all_matches_image.getMatRef();

    current_image.copyTo(all_matches_image(cv::Rect(0, 0,
        current_image_size.width,
        current_image_size.height)));
  }

  if ( _selected_matches_image.needed() ) {

    _selected_matches_image.create(
        dst_size,
        CV_8UC3);

    selected_matches_image =
        _selected_matches_image.getMatRef();

    current_image.copyTo(selected_matches_image(cv::Rect(0, 0,
            current_image_size.width,
            current_image_size.height)));
  }


  switch ( layout_type ) {
  case LAYOUT_VERT :


    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = matches.size(); i < n; ++i ) {
      if ( mask.empty() || mask[i][0] ) {

        const cv::DMatch & m =
            matches[i];

        const cv::Point2f & cp =
            current_keypoints[m. queryIdx].pt;

        const cv::Point2f rp =
            cv::Point2f(reference_keypoints[m.trainIdx].pt.x,
                reference_keypoints[m.trainIdx].pt.y + current_image_size.height);

        const cv::Scalar c(
            rand() % 255,
            rand() % 255,
            rand() % 255);

        if ( !all_matches_image.empty() ) {
          cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
        }

        if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
          cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
        }
      }
    }

    break;

  case LAYOUT_HORZ :

    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = matches.size(); i < n; ++i ) {
      if ( mask.empty() || mask[i][0] ) {

        const cv::DMatch & m =
            matches[i];

        const cv::Point2f & cp =
            current_keypoints[m. queryIdx].pt;

        const cv::Point2f rp =
            cv::Point2f(reference_keypoints[m.trainIdx].pt.x + current_image_size.width,
                reference_keypoints[m.trainIdx].pt.y);

        const cv::Scalar c(
            rand() % 255,
            rand() % 255,
            rand() % 255);

        if ( !all_matches_image.empty() ) {
          cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
        }

        if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
          cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
        }
      }
    }
    break;
  }


  return true;

}
