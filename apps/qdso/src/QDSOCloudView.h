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
#include <gui/qmtf/QMtfDisplay.h>

namespace qdso {

class QDSOCloudView :
    public QGLPointCloudView,
    public IMtfDisplay,
    public QCloudViewDisplayFunction
{
  Q_OBJECT;
  Q_INTERFACES(IMtfDisplay);
public:
  typedef QDSOCloudView ThisClass;
  typedef QGLPointCloudView Base;

  QDSOCloudView(QWidget * parent = nullptr);

  void loadParameters();
  void saveParameters();

  void showPoints(cv::InputArray pts);

Q_SIGNALS: // IMtfDisplay
  void displayChannelsChanged();
  void parameterChanged();
  void displayImageChanged();

protected: // IMtfDisplay
  //QStringList displayChannels() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

protected: // QCloudViewDisplayFunction
  void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMask,
      cv::OutputArray displayPoints,
      cv::OutputArray mtfColors,
      cv::OutputArray displayColors) override;

Q_SIGNALS:
  void redrawCloud(QPrivateSignal sig = QPrivateSignal());

protected:
};



} /* namespace qdso */

#endif /* __QDSOCloudView_h__ */
