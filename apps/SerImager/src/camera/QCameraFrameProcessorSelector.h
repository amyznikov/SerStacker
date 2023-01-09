/*
 * QCameraFrameProcessorSelector.h
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFrameProcessorSelector__h__
#define __QCameraFrameProcessorSelector__h__

#include <gui/qimproc/QImageProcessorSelector.h>

class QCameraFrameProcessorSelector :
    public QImageProcessorSelector
{
public:
  typedef QCameraFrameProcessorSelector ThisClass;
  typedef QImageProcessorSelector Base;

  QCameraFrameProcessorSelector(QWidget * parent = nullptr);

protected:
};

#endif /* __QCameraFrameProcessorSelector__h__ */
