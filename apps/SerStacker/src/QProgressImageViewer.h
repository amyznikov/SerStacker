/*
 * QProgressImageViewer.h
 *
 *  Created on: Dec 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QProgressImageViewer_h__
#define __QProgressImageViewer_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qmtf/QMtfDisplay.h>


namespace serstacker {

class QProgressImageViewer :
    public QImageEditor,
    public IMtfDisplay,
    public QImageDisplayFunction

{
  Q_OBJECT;
  Q_INTERFACES(IMtfDisplay)
public:
  typedef QProgressImageViewer ThisClass;
  typedef QImageEditor Base;

  QProgressImageViewer(QWidget * parent = nullptr);

  IMtfDisplay * mtfDisplay();
  const IMtfDisplay * mtfDisplay() const;

Q_SIGNALS:
  void displayTypeChanged();
  void parameterChanged();
  void displayImageChanged();


protected: // QImageDisplayFunction
  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::Mat & mtfImage, cv::Mat & displayImage, int ddepth = CV_8U);
  bool applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U);
  bool applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
      cv::OutputArray colormapImage);

protected: // MTF
  const c_enum_member * displayChannels() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

protected:
};

} /* namespace serstacker */

#endif /* __QProgressImageViewer_h__ */
