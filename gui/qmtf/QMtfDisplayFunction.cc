/*
 * QMtfDisplayFunction.cc
 *
 *  Created on: Mar 30, 2022
 *      Author: amyznikov
 */

#include "QMtfDisplayFunction.h"
#include <core/proc/histogram.h>
#include <core/debug.h>

QMtfDisplayFunction::QMtfDisplayFunction(QObject * parent) :
  Base(parent)
{
}

void QMtfDisplayFunction::set_mtf(c_pixinsight_mtf * mtf)
{
  this->mtf_ = mtf;
}

c_pixinsight_mtf * QMtfDisplayFunction::mtf() const
{
  return this->mtf_;
}

void QMtfDisplayFunction::set_colormap(COLORMAP cmap)
{
  this->colormap_ = cmap;
  emit update();
}

COLORMAP QMtfDisplayFunction::colormap() const
{
  return colormap_;
}

bool QMtfDisplayFunction::createDisplayImage(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst, int ddepth)
{
  if ( mtf_ ) {
    mtf_->apply(src, dst, ddepth);
  }

  if( colormap_ != COLORMAP_NONE && !dst.fixedType() && dst.type() == CV_8UC1 ) {
    apply_colormap(dst.getMat(), dst, colormap_);
  }

  return true;
}

bool QMtfDisplayFunction::createInputHistogram(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray H, double * hmin, double * hmax)
{
  return create_histogram(src, srcmask, H, hmin, hmax);
}

bool QMtfDisplayFunction::createOutputHistogram(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray H, double * hmin, double * hmax)
{
  bool fOk;

  if( !mtf_ || (mtf_->shadows() == 0 && mtf_->highlights() == 1 && mtf_->midtones() == 0.5) ) {
    fOk = create_histogram(src, srcmask, H, hmin, hmax, 256);
  }
  else {
    cv::Mat M;
    mtf_->apply(src, M);
    fOk = create_histogram(M, srcmask, H, hmin, hmax, 256);
  }

  return fOk;
}
