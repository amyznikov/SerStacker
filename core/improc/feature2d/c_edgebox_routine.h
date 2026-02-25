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
    _gradient_type = v;
  }

  GradientType gradient_type() const
  {
    return _gradient_type;
  }

  void set_gradient_pscale(int v)
  {
    _gradient_pscale = v;
  }

  int gradient_pscale() const
  {
    return _gradient_pscale;
  }

  void set_gradient_threshold(THRESHOLD_TYPE v)
  {
    _gradient_threshold = v;
  }

  THRESHOLD_TYPE gradient_threshold() const
  {
    return _gradient_threshold;
  }

  void set_model(const std::string & v)
  {
    _model = v;
    _dollar.reset();
  }

  const std::string& model() const
  {
    return _model;
  }

  void set_display(DisplayType v)
  {
    _display = v;
  }

  DisplayType display() const
  {
    return _display;
  }

  // Options for cv::ximgproc::EdgeBoxes

  /** @brief Returns the step size of sliding window search.
   */
  float Alpha() const
  {
    return _edgeboxes->getAlpha();
  }

  /** @brief Sets the step size of sliding window search.
   */
  void set_Alpha(float value)
  {
    _edgeboxes->setAlpha(value);
  }

  /** @brief Returns the nms threshold for object proposals.
   */
  float Beta() const
  {
    return _edgeboxes->getBeta();
  }

  /** @brief Sets the nms threshold for object proposals.
   */
  void set_Beta(float value)
  {
    _edgeboxes->setBeta(value);
  }

  /** @brief Returns adaptation rate for nms threshold.
   */
  float Eta() const
  {
    return _edgeboxes->getEta();
  }

  /** @brief Sets the adaptation rate for nms threshold.
   */
  void set_Eta(float value)
  {
    _edgeboxes->setEta(value);
  }

  /** @brief Returns the min score of boxes to detect.
   */
  float MinScore() const
  {
    return _edgeboxes->getMinScore();
  }

  /** @brief Sets the min score of boxes to detect.
   */
  void set_MinScore(float value)
  {
    _edgeboxes->setMinScore(value);
  }

  /** @brief Returns the max number of boxes to detect.
   */
  int MaxBoxes() const
  {
    return _edgeboxes->getMaxBoxes();
  }
  /** @brief Sets max number of boxes to detect.
   */
  void set_MaxBoxes(int value)
  {
    _edgeboxes->setMaxBoxes(value);
  }

  /** @brief Returns the edge min magnitude.
   */
  float EdgeMinMag() const
  {
    return _edgeboxes->getEdgeMinMag();
  }

  /** @brief Sets the edge min magnitude.
   */
  void set_EdgeMinMag(float value)
  {
    _edgeboxes->setEdgeMinMag(value);
  }

  /** @brief Returns the edge merge threshold.
   */
  float EdgeMergeThr() const
  {
    return _edgeboxes->getEdgeMergeThr();
  }

  /** @brief Sets the edge merge threshold.
   */
  void set_EdgeMergeThr(float value)
  {
    _edgeboxes->setEdgeMergeThr(value);
  }

  /** @brief Returns the cluster min magnitude.
   */
  float ClusterMinMag() const
  {
    return _edgeboxes->getClusterMinMag();
  }

  /** @brief Sets the cluster min magnitude.
   */
  void set_ClusterMinMag(float value)
  {
    _edgeboxes->setClusterMinMag(value);
  }

  /** @brief Returns the max aspect ratio of boxes.
   */
  float MaxAspectRatio() const
  {
    return _edgeboxes->getMaxAspectRatio();
  }

  /** @brief Sets the max aspect ratio of boxes.
   */
  void set_MaxAspectRatio(float value)
  {
    _edgeboxes->setMaxAspectRatio(value);
  }

  /** @brief Returns the minimum area of boxes.
   */
  float MinBoxArea() const
  {
    return _edgeboxes->getMinBoxArea();
  }

  /** @brief Sets the minimum area of boxes.
   */
  void set_MinBoxArea(float value)
  {
    _edgeboxes->setMinBoxArea(value);
  }

  /** @brief Returns the affinity sensitivity.
   */
  float Gamma() const
  {
    return _edgeboxes->getGamma();
  }

  /** @brief Sets the affinity sensitivity
   */
  void set_Gamma(float value)
  {
    _edgeboxes->setGamma(value);
  }

  /** @brief Returns the scale sensitivity.
   */
  float Kappa() const
  {
    return _edgeboxes->getKappa();
  }

  /** @brief Sets the scale sensitivity.
   */
  void set_Kappa(float value)
  {
    _edgeboxes->setKappa(value);
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _model;
  GradientType _gradient_type = GradientMagnitude;
  DisplayType _display = DisplayEdgeMap;
  THRESHOLD_TYPE _gradient_threshold = THRESHOLD_TYPE_VALUE;
  int _gradient_pscale = 0;
  cv::Ptr<cv::ximgproc::StructuredEdgeDetection> _dollar;
  cv::Ptr<cv::ximgproc::EdgeBoxes> _edgeboxes = cv::ximgproc::createEdgeBoxes();
  cv::Mat _edges;
  cv::Mat _orientations;
  cv::Mat _edgeNms;
  std::vector<cv::Rect> _boxes;
  std::vector<float> _scores;

};

#endif /* __c_edgebox_routine_h__ */
