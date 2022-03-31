/*
 * QMtfDisplayFunction.h
 *
 *  Created on: Mar 30, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfDisplayFunction_h__
#define __QMtfDisplayFunction_h__

#include <gui/qimageview/QImageViewer.h>
#include <core/mtf/c_pixinsight_mtf.h>
#include <core/proc/colormap.h>

class QMtfDisplayFunction:
    public QImageDisplayFunction
{
  Q_OBJECT;
public:
  typedef QMtfDisplayFunction ThisClass;
  typedef QImageDisplayFunction Base;

  QMtfDisplayFunction(QObject * parent = Q_NULLPTR);

  void set_mtf(c_pixinsight_mtf * mtf);
  c_pixinsight_mtf * mtf() const;

  void set_colormap(COLORMAP cmap);
  COLORMAP colormap() const;

  bool createDisplayImage(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst, int ddepth = -1) override;

  bool createInputHistogram(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray H, double * hmin, double * hmax) override;

  bool createOutputHistogram(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray H, double * hmin, double * hmax) override;


protected:
  c_pixinsight_mtf * mtf_ = Q_NULLPTR;
  COLORMAP colormap_ = COLORMAP_NONE;
};

#endif /* __QMtfDisplayFunction_h__ */
