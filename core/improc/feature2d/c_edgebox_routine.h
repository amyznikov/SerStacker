/*
 * c_edgebox_routine.h
 *
 *  Created on: Apr 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_edgebox_routine_h__
#define __c_edgebox_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/threshold.h>
#include <opencv2/ximgproc.hpp>

class c_edgebox_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_edgebox_routine,
      "edgebox", "cv::ximgproc::EdgeBoxes");

  enum DisplayType
  {
    DisplayEdgeMap,
    DisplayEdgeOrientation,
    DisplayEdgeNMS,
    DisplayBoxes,
  };

  enum GradientType {
    GradientMagnitude,
    StructuredEdgeDetection
  };


  void set_gradient_type(GradientType v)
  {
    gradient_type_ = v;
  }

  GradientType gradient_type() const
  {
    return gradient_type_;
  }

  void set_gradient_pscale(int v)
  {
    gradient_pscale_ = v;
  }

  int gradient_pscale() const
  {
    return gradient_pscale_;
  }

  void set_gradient_threshold(THRESHOLD_TYPE v)
  {
    gradient_threshold_ = v;
  }

  THRESHOLD_TYPE gradient_threshold() const
  {
    return gradient_threshold_;
  }

  void set_model(const std::string & v)
  {
    model_ = v;
    dollar_.reset();
  }

  const std::string& model() const
  {
    return model_;
  }

  void set_display(DisplayType v)
  {
    display_ = v;
  }

  DisplayType display() const
  {
    return display_;
  }

  // Options for cv::ximgproc::EdgeBoxes

  /** @brief Returns the step size of sliding window search.
   */
  float Alpha() const
  {
    return edgeboxes_->getAlpha();
  }

  /** @brief Sets the step size of sliding window search.
   */
  void set_Alpha(float value)
  {
    edgeboxes_->setAlpha(value);
  }

  /** @brief Returns the nms threshold for object proposals.
   */
  float Beta() const
  {
    return edgeboxes_->getBeta();
  }

  /** @brief Sets the nms threshold for object proposals.
   */
  void set_Beta(float value)
  {
    edgeboxes_->setBeta(value);
  }

  /** @brief Returns adaptation rate for nms threshold.
   */
  float Eta() const
  {
    return edgeboxes_->getEta();
  }

  /** @brief Sets the adaptation rate for nms threshold.
   */
  void set_Eta(float value)
  {
    edgeboxes_->setEta(value);
  }

  /** @brief Returns the min score of boxes to detect.
   */
  float MinScore() const
  {
    return edgeboxes_->getMinScore();
  }

  /** @brief Sets the min score of boxes to detect.
   */
  void set_MinScore(float value)
  {
    edgeboxes_->setMinScore(value);
  }

  /** @brief Returns the max number of boxes to detect.
   */
  int MaxBoxes() const
  {
    return edgeboxes_->getMaxBoxes();
  }
  /** @brief Sets max number of boxes to detect.
   */
  void set_MaxBoxes(int value)
  {
    edgeboxes_->setMaxBoxes(value);
  }

  /** @brief Returns the edge min magnitude.
   */
  float EdgeMinMag() const
  {
    return edgeboxes_->getEdgeMinMag();
  }

  /** @brief Sets the edge min magnitude.
   */
  void set_EdgeMinMag(float value)
  {
    edgeboxes_->setEdgeMinMag(value);
  }

  /** @brief Returns the edge merge threshold.
   */
  float EdgeMergeThr() const
  {
    return edgeboxes_->getEdgeMergeThr();
  }

  /** @brief Sets the edge merge threshold.
   */
  void set_EdgeMergeThr(float value)
  {
    edgeboxes_->setEdgeMergeThr(value);
  }

  /** @brief Returns the cluster min magnitude.
   */
  float ClusterMinMag() const
  {
    return edgeboxes_->getClusterMinMag();
  }

  /** @brief Sets the cluster min magnitude.
   */
  void set_ClusterMinMag(float value)
  {
    edgeboxes_->setClusterMinMag(value);
  }

  /** @brief Returns the max aspect ratio of boxes.
   */
  float MaxAspectRatio() const
  {
    return edgeboxes_->getMaxAspectRatio();
  }

  /** @brief Sets the max aspect ratio of boxes.
   */
  void set_MaxAspectRatio(float value)
  {
    edgeboxes_->setMaxAspectRatio(value);
  }

  /** @brief Returns the minimum area of boxes.
   */
  float MinBoxArea() const
  {
    return edgeboxes_->getMinBoxArea();
  }

  /** @brief Sets the minimum area of boxes.
   */
  void set_MinBoxArea(float value)
  {
    edgeboxes_->setMinBoxArea(value);
  }

  /** @brief Returns the affinity sensitivity.
   */
  float Gamma() const
  {
    return edgeboxes_->getGamma();
  }

  /** @brief Sets the affinity sensitivity
   */
  void set_Gamma(float value)
  {
    edgeboxes_->setGamma(value);
  }

  /** @brief Returns the scale sensitivity.
   */
  float Kappa() const
  {
    return edgeboxes_->getKappa();
  }

  /** @brief Sets the scale sensitivity.
   */
  void set_Kappa(float value)
  {
    edgeboxes_->setKappa(value);
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, gradient_type, "Method for computing image gradients");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, gradient_pscale, "Gradient pyramid scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, gradient_threshold, "Gradient threshold method");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display, "Display image");

    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, model,
        "Model file for createStructuredEdgeDetection()\n"
            "https://github.com/opencv/opencv_extra/blob/master/testdata/cv/ximgproc/model.yml.gz\n");

    ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "EdgeBoxes", "Options for cv::ximgproc::EdgeBoxes");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, MaxBoxes, "max number of boxes to detect");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, EdgeMinMag, "the edge min magnitude");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, EdgeMergeThr, "Sets the edge merge threshold");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, ClusterMinMag, "the cluster min magnitude");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, MaxAspectRatio, "the max aspect ratio of boxes");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, MinBoxArea, "the minimum area of boxes");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, Gamma, "the affinity sensitivity");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, Kappa, "the scale sensitivity");
    END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, gradient_type);
      SERIALIZE_PROPERTY(settings, save, *this, gradient_pscale);
      SERIALIZE_PROPERTY(settings, save, *this, gradient_threshold);
      SERIALIZE_PROPERTY(settings, save, *this, model);
      SERIALIZE_PROPERTY(settings, save, *this, display);
      SERIALIZE_PROPERTY(settings, save, *this, MaxBoxes);
      SERIALIZE_PROPERTY(settings, save, *this, EdgeMinMag);
      SERIALIZE_PROPERTY(settings, save, *this, EdgeMergeThr);
      SERIALIZE_PROPERTY(settings, save, *this, ClusterMinMag);
      SERIALIZE_PROPERTY(settings, save, *this, MaxAspectRatio);
      SERIALIZE_PROPERTY(settings, save, *this, MinBoxArea);
      SERIALIZE_PROPERTY(settings, save, *this, Gamma);
      SERIALIZE_PROPERTY(settings, save, *this, Kappa);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::string model_;
  GradientType gradient_type_ = GradientMagnitude;
  DisplayType display_ = DisplayEdgeMap;
  THRESHOLD_TYPE gradient_threshold_ = THRESHOLD_TYPE_VALUE;
  int gradient_pscale_ = 0;
  cv::Ptr<cv::ximgproc::StructuredEdgeDetection> dollar_;
  cv::Ptr<cv::ximgproc::EdgeBoxes> edgeboxes_ = cv::ximgproc::createEdgeBoxes();
  cv::Mat edges_;
  cv::Mat orientations_;
  cv::Mat edgeNms_;
  std::vector<cv::Rect> boxes_;
  std::vector<float> scores_;

};

#endif /* __c_edgebox_routine_h__ */
