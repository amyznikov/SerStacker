/*
 * QImageFocusMeasure.h
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageFocusMeasure_h__
#define __QImageFocusMeasure_h__

#include <gui/qfocus/QFocusMeasureProvider.h>

namespace qserstacker {

class QImageFocusMeasure:
    public QFocusMeasureProvider
{
public:
  typedef QImageFocusMeasure ThisClass;
  typedef QFocusMeasureProvider Base;

  QImageFocusMeasure(QObject * parent = nullptr);

  void measure(const cv::Mat & image, COLORID colorid,
      int bpp, const QRect & roi);

};

} /* namespace qserstacker */

#endif /* __QImageFocusMeasure_h__ */
