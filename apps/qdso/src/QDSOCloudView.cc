/*
 * QDSOCloudView.cc
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#include "QDSOCloudView.h"
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/debug.h>

namespace qdso {

QDSOCloudView::QDSOCloudView(QWidget * parent) :
    Base(parent)
{
  setDisplayFunction(this);

  connect(this, &ThisClass::redrawCloud,
      this, &ThisClass::updateDisplayPoints,
      Qt::QueuedConnection);


  addDisplay("default", 0, 255);
  setDisplayChannel("default");
}

void QDSOCloudView::loadParameters()
{
  Base::loadParameters();
}

void QDSOCloudView::saveParameters()
{
  Base::saveParameters();
}

void QDSOCloudView::showPoints(cv::InputArray points)
{
  display_lock_.lock();

  points.getMat().copyTo(currentPoints_);
  currentColors_.create(currentPoints_.size(), CV_8UC3);
  currentColors_.setTo(cv::Scalar::all(255));

  display_lock_.unlock();

  Q_EMIT redrawCloud();
}



void QDSOCloudView::createDisplayPoints(cv::InputArray currentPoints,
    cv::InputArray currentColors,
    cv::InputArray currentMask,
    cv::OutputArray displayPoints,
    cv::OutputArray mtfColors,
    cv::OutputArray displayColors)
{
  if ( currentPoints.empty() ) {

    displayPoints.release();
    mtfColors.release();
    displayColors.release();

    Q_EMIT displayImageChanged();

    return;
  }

  DisplayParams & opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  cv::Mat mtfcolors, displaycolors;

  const bool needColormap =
      opts.colormap != COLORMAP_NONE;// &&
          //currentColors.channels() == 1;


  currentPoints.getMat().convertTo(displayPoints, CV_32F);


  adjustMtfRange(mtf, needColormap ? currentColors : cv::noArray(), currentMask, &a);
  mtf->apply(currentColors, mtfcolors, CV_8U);
  restoreMtfRange(mtf, a);

  if ( !mtfColors.fixedType() || mtfColors.type() == mtfcolors.type() ) {
    mtfcolors.copyTo(mtfColors);
  }
  else {
    mtfcolors.convertTo(mtfColors, mtfColors.type());
  }

  if ( needColormap ) {

    if( mtfcolors.channels() != 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_RGB2GRAY);
    }

    cv::applyColorMap(mtfcolors,
        displaycolors,
        opts.lut);

    if( currentMask.size() == displaycolors.size() ) {
      displaycolors.setTo(0, ~currentMask.getMat());
    }

    cv::cvtColor(displaycolors, displaycolors, cv::COLOR_BGR2RGB);

    displaycolors.copyTo(displayColors);

  }

  else {

    if( mtfcolors.channels() == 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_GRAY2BGR);
    }

    mtfcolors.copyTo(displayColors);
  }

  Q_EMIT displayImageChanged();
}

void QDSOCloudView::getInputDataRange(double * minval, double * maxval) const
{
  getminmax(currentColors(), minval, maxval, currentMask());
}

void QDSOCloudView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(currentColors(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}

void QDSOCloudView::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(mtfColors(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}

} /* namespace qdso */
