/*
 * QDSOCloudView.h
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDSOCloudView_h__
#define __QDSOCloudView_h__

#include <gui/qglview/QGLPointCloudView.h>

namespace qdso {

class QDSOCloudView :
    public QGLPointCloudView
{
  Q_OBJECT;
public:
  typedef QDSOCloudView ThisClass;
  typedef QGLPointCloudView Base;

  QDSOCloudView(QWidget * parent = nullptr);

protected:
};



} /* namespace qdso */

#endif /* __QDSOCloudView_h__ */
