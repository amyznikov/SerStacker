/*
 * c_edgebox_routine.cc
 *
 *  Created on: Apr 15, 2023
 *      Author: amyznikov
 *
 *  See /opencv_contrib/modules/ximgproc/samples/edgeboxes_demo.cpp
 */

#include "c_edgebox_routine.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_edgebox_routine::DisplayType>()
{
  static constexpr c_enum_member members[] = {

  { c_edgebox_routine::DisplayEdgeMap, "EdgeMap",
      "The output from cv::ximgproc::StructuredEdgeDetection::detectEdges()" },

  { c_edgebox_routine::DisplayEdgeOrientation, "EdgeOrientation",
      "The output from cv::ximgproc::StructuredEdgeDetection::computeOrientation()" },

  { c_edgebox_routine::DisplayEdgeNMS, "EdgeNMS",
      "The output from cv::ximgproc::StructuredEdgeDetection::edgesNms()" },

  { c_edgebox_routine::DisplayBoxes, "Boxes",
      "The output from cv::ximgproc::EdgeBoxes::getBoundingBoxes(EdgeNMS)" },

  { c_edgebox_routine::DisplayEdgeMap }
  };

  return members;
}

bool c_edgebox_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !dollar_ ) {

    if (model_.empty() ) {
      CF_ERROR("Moel file not specified");
      return false;
    }

    try {
      dollar_ = cv::ximgproc::createStructuredEdgeDetection(model_);
    }
    catch( const cv::Exception &e ) {
      CF_ERROR("cv::ximgproc::createStructuredEdgeDetection('%s') fails:\n%s",
          model_.c_str(), e.msg.c_str());
      return false;
    }
  }

  cv::Mat img;

  switch (image.depth()) {
    case CV_8U:
      image.getMat().convertTo(img, CV_32F, 1.0 / UINT8_MAX);
      break;
    case CV_8S:
      image.getMat().convertTo(img, CV_32F, 1.0 / INT8_MAX);
      break;
    case CV_16U:
      image.getMat().convertTo(img, CV_32F, 1.0 / UINT16_MAX);
      break;
    case CV_16S:
      image.getMat().convertTo(img, CV_32F, 1.0 / INT16_MAX);
      break;
    case CV_32S:
      image.getMat().convertTo(img, CV_32F, 1.0 / INT32_MAX);
      break;
    case CV_32F:
      img = image.getMat();
      break;
    case CV_64F:
      image.getMat().convertTo(img, CV_32F);
      break;
  }


  // compute edge map
  if( display_ == DisplayEdgeMap ) {
    dollar_->detectEdges(img, image);
    return true;
  }

  dollar_->detectEdges(img, edges_);

  // compute orientation from edge map
  if( display_ == DisplayEdgeOrientation ) {
    dollar_->computeOrientation(edges_, image);
    return true;
  }

  dollar_->computeOrientation(edges_, orientations_);

  // apply edge nms
  if( display_ == DisplayEdgeNMS ) {
    dollar_->edgesNms(edges_, orientations_, image, 2, 0, 1, true);
    return true;
  }

  dollar_->edgesNms(edges_, orientations_, edgeNms_, 2, 0, 1, true);

  edgeboxes_->getBoundingBoxes(edgeNms_, orientations_, boxes_, scores_);

//  CF_DEBUG("\n");


  for( int i = 0, n = (int) boxes_.size(); i < n; ++i ) {

    const cv::Point p1(boxes_[i].x, boxes_[i].y);

    const cv::Point p2(boxes_[i].x + boxes_[i].width, boxes_[i].y + boxes_[i].height);

    const cv::Scalar color(std::max(32, rand() % 255),
        std::max(32, rand() % 255),
        std::max(32, rand() % 255));

    cv::rectangle(image, p1, p2, color, 1);

    //    cv::putText(image,
    //        ssprintf("%g", scores_[i]),
    //        p1,
    //        cv::FONT_HERSHEY_PLAIN,
    //        2,
    //        color,
    //        1,
    //        cv::LINE_AA,
    //        false);

    //  CF_DEBUG("box[%d]: %g", i, scores_[i]);

  }

//  CF_DEBUG("\n");

  return true;
}
