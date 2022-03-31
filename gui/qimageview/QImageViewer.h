/*
 * QImageViewer.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageViewer_h__
#define __QImageViewer_h__

#include "QImageSceneView.h"
#include <opencv2/opencv.hpp>


class QImageDisplayFunction :
    public QObject
{
  Q_OBJECT;
public:
  typedef QImageDisplayFunction ThisClass;
  typedef QObject Base;

  QImageDisplayFunction(QObject * parent = Q_NULLPTR) :
      Base(parent)
  {
  }

  virtual bool createDisplayImage(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst, int ddepth = -1) = 0;

  virtual bool createInputHistogram(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray H, double * hmin, double * hmax) = 0;

  virtual bool createOutputHistogram(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray H, double * hmin, double * hmax) = 0;

signals:
  void update();
};

class QImageViewer
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QImageViewer ThisClass;
  typedef QWidget Base;

  QImageViewer(QWidget * parent = Q_NULLPTR);

  QImageSceneView * sceneView() const;

  QToolBar * embedToolbar(QToolBar * toolbar = Q_NULLPTR);
  QToolBar * toolbar() const;

  QStatusBar * embedStatusbar(QStatusBar * statusBar = Q_NULLPTR);
  QStatusBar * statusbar() const;

  void setDisplayFunction(QImageDisplayFunction * func);
  QImageDisplayFunction * displayFunction() const;

  void setViewScale(int scale, const QPoint * centerPos = Q_NULLPTR);
  int viewScale() const;

  virtual void setImage(cv::InputArray image, cv::InputArray mask,
      cv::InputArray imageData = cv::noArray(),
      bool make_copy = true);

  const cv::Mat & currentImage() const;
  const cv::Mat & currentMask() const;
  const cv::Mat & currentImageData() const;
  const cv::Mat & displayImage() const;

  QString currentFileName() const;
  void setCurrentFileName(const QString & newFileName);

  QString statusStringForPixel(const QPoint & viewpos);


signals:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onScaleChanged(int currentScale);
  void onFocusInEvent(QFocusEvent *e);
  void onFocusOutEvent(QFocusEvent *e);
  void visibilityChanged(bool visible);
  void currentImageChanged();
  void currentImageFileNameChanged();
  void currentDisplayImageChanged();
  void onLineShapeChanged(QGraphicsLineItem * item);
  void onRectShapeChanged(QGraphicsRectItem * item);

public slots:
  virtual void updateDisplay();
  void copyDisplayImageToClipboard();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;
  virtual void createDisplayImage();
  virtual void showCurrentDisplayImage();

protected:
  QVBoxLayout * layout_  = Q_NULLPTR;
  QImageSceneView * view_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStatusBar * statusbar_ = Q_NULLPTR;
  QImageDisplayFunction * displayFunction_ = Q_NULLPTR;

  QString currentFileName_;
  cv::Mat currentImage_, currentMask_;
  cv::Mat currentImageData_;
  cv::Mat displayImage_;
  QImage qimage_;


};

#endif /* __QImageViewer_h__ */
