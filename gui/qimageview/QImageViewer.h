/*
 * QImageViewer.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageViewer_h__
#define __QImageViewer_h__

#include "QImageDisplayFunction.h"
#include "QImageSceneView.h"

class QImageViewer
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QImageViewer ThisClass;
  typedef QWidget Base;

  enum DisplayType {
    DisplayImage,
    DisplayMask,
    DisplayBlend
  };

  enum PenShape {
    PenShape_square,
    PenShape_circle,
  };

  QImageViewer(QWidget * parent = Q_NULLPTR);

  QImageSceneView * sceneView() const;

  QToolBar * embedToolbar(QToolBar * toolbar = Q_NULLPTR);
  QToolBar * toolbar() const;

  QStatusBar * embedStatusbar(QStatusBar * statusBar = Q_NULLPTR);
  QStatusBar * statusbar() const;

  void setDisplayFunction(QImageDisplayFunction * displayfunc);
  QImageDisplayFunction * displayFunction() const;

  void setViewScale(int scale, const QPoint * centerPos = Q_NULLPTR);
  int viewScale() const;

  void setDisplayType(DisplayType v);
  DisplayType displayType() const;

  virtual void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/);
  virtual void setMask(cv::InputArray mask, bool make_copy /*= true*/);

  const cv::Mat & currentImage() const;
  const cv::Mat & currentMask() const;
  const cv::Mat & currentImageData() const;
  const cv::Mat & displayImage() const;

  QString currentFileName() const;
  void setCurrentFileName(const QString & newFileName);

  QString statusStringForPixel(const QPoint & viewpos);

  void setEnableEditMask(bool enable);
  bool enableEditMask() const;

  void setEditMaskPenRadius(int v);
  int editMaskPenRadius() const;

  void setEditMaskPenShape(PenShape v);
  PenShape editMaskPenShape() const;

signals:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onScaleChanged(int currentScale);
  //  void onShowEvent(QShowEvent *e);
  //  void onHideEvent(QHideEvent *e);
  void onFocusInEvent(QFocusEvent *e);
  void onFocusOutEvent(QFocusEvent *e);
  void visibilityChanged(bool visible);
  void currentImageChanged();
  void currentDisplayImageChanged();
  void currentDisplayTypeChanged();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;
  virtual void setCurrentImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/);
  virtual void createDisplayImage();
  virtual void showCurrentDisplayImage();
  void updateCursor();

public slots:
  virtual void updateDisplay();
  void copyDisplayImageToClipboard();
  void handleMousePressEvent(QMouseEvent * e);
  void handleMouseMoveEvent(QMouseEvent * e);
  void editMask(QMouseEvent * e);
  void undoEditMask();

protected:
  QVBoxLayout * layout_  = Q_NULLPTR;
  QImageSceneView * view_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStatusBar * statusbar_ = Q_NULLPTR;

  QImageDisplayFunction * displayFunction_ = Q_NULLPTR;
  DisplayType currentDisplayType_ = DisplayImage;
  QString currentFileName_;
  cv::Mat currentImage_, currentMask_;
  cv::Mat currentImageData_;
  cv::Mat displayImage_;
  QImage qimage_;

  bool enableEditMask_ = false;
  int editMaskPenRadius_ = 5;
  PenShape editMaskPenShape_ = PenShape_square;
  // QAction * undoEditMaskAction_ = Q_NULLPTR;
  QShortcut * undoEditMaskActionShortcut_ = Q_NULLPTR;
  QStack<cv::Mat> editMaskUndoQueue_;
};

#endif /* __QImageViewer_h__ */
