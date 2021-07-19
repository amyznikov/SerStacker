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

class QImageViewer
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QImageViewer ThisClass;
  typedef QWidget Base;
  typedef std::function<void(const cv::Mat & src, cv::Mat & dst, int ddepth)> DisplayFunction;

  QImageViewer(QWidget * parent = Q_NULLPTR);

  QToolBar * embedToolbar(QToolBar * toolbar = Q_NULLPTR);
  QToolBar * toolbar() const;

  QStatusBar * embedStatusbar(QStatusBar * statusBar = Q_NULLPTR);
  QStatusBar * statusbar() const;

  void setDisplayFunction(const DisplayFunction & func);
  const DisplayFunction & displayFunction() const;

  virtual void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/);
  const cv::Mat & image() const;
  const cv::Mat & mask() const;
  const cv::Mat & imageData() const;

  QString statusStringForPixel(const QPoint & viewpos);

  void setSelectionRectVisible(bool v);
  bool selectionRectIsVisible() const;

  void setSelectionRect(const QRectF & rc);
  QRectF selectionRect() const;

  void selectionRectToCenterOfView();

signals:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onScaleChanged(int currentScale);
  void onShowEvent(QShowEvent *e);
  void onHideEvent(QHideEvent *e);
  void onFocusInEvent(QFocusEvent *e);
  void onFocusOutEvent(QFocusEvent *e);

public slots:
  virtual void updateDisplay();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;
  void createSelectionRect(const QRectF & rc);

protected:
  QVBoxLayout * vbox_  = Q_NULLPTR;
  QImageSceneView * view_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStatusBar * statusbar_ = Q_NULLPTR;
  QGraphicsRectItem * selectionRect_ = Q_NULLPTR;

  cv::Mat currentImage_, currentMask_;
  cv::Mat currentImageData_;
  QImage qimage_;
  DisplayFunction displayFunction_;


};

#endif /* __QImageViewer_h__ */
