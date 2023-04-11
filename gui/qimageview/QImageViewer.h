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

class QImageViewer:
    public QWidget
{
Q_OBJECT;
public:
  typedef QImageViewer ThisClass;
  typedef QWidget Base;

  class current_image_lock:
      public std::lock_guard<std::mutex>
  {
  public:
    typedef current_image_lock this_class;
    typedef std::lock_guard<std::mutex> base;

    current_image_lock(QImageViewer * view) :
        base(view->currentImageLock_)
    {
    }
  };

  enum DisplayType
  {
    DisplayImage,
    DisplayMask,
    DisplayBlend
  };

  enum PenShape
  {
    PenShape_square,
    PenShape_circle,
  };

  QImageViewer(QWidget * parent = nullptr);
  QImageViewer(QImageScene * scene, QWidget * parent = nullptr);

  QImageSceneView* sceneView() const;
  QImageScene* scene() const;

  QToolBar* embedToolbar(QToolBar * toolbar = nullptr);
  QToolBar* toolbar() const;

  QStatusBar* embedStatusbar(QStatusBar * statusBar = nullptr);
  QStatusBar* statusbar() const;

  void setDisplayFunction(QImageDisplayFunction * displayfunc);
  QImageDisplayFunction* displayFunction() const;

  void setViewScale(int scale, const QPoint * centerPos = nullptr);
  int viewScale() const;

  void setDisplayType(DisplayType v);
  DisplayType displayType() const;

  void setTransparentMask(bool v);
  bool transparentMask() const;

  virtual void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/,
      bool make_copy /*= true*/);
  virtual void setMask(cv::InputArray mask, bool make_copy /*= true*/);
  virtual void setCurrentImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/,
      bool make_copy /*= true*/);

  const cv::Mat& currentImage() const;
  const cv::Mat& currentMask() const;
  const cv::Mat& currentImageData() const;
  const cv::Mat& displayImage() const;

  QString currentFileName() const;
  void setCurrentFileName(const QString & newFileName);

  QString statusStringForPixel(const QPoint & viewpos);

  void setEnableEditMask(bool enable);
  bool enableEditMask() const;

  void setEditMaskPenRadius(int v);
  int editMaskPenRadius() const;

  void setEditMaskPenShape(PenShape v);
  PenShape editMaskPenShape() const;

Q_SIGNALS:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onMouseEnterEvent(QEvent * e);
  void onMouseLeaveEvent(QEvent * e);
  void onScaleChanged(int currentScale);
  void onFocusInEvent(QFocusEvent * e);
  void onFocusOutEvent(QFocusEvent * e);
  void visibilityChanged(bool visible);
  void currentImageChanged();
  void displayImageChanged();
  void displayTypeChanged();

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;
  void focusInEvent(QFocusEvent * event) override;
  void focusOutEvent(QFocusEvent * event) override;
  virtual void createDisplayImage();
  virtual void showCurrentDisplayImage();
  void updateCursor();

public Q_SLOTS:
  virtual void updateDisplay();
  void copyDisplayImageToClipboard();
  void handleMousePressEvent(QMouseEvent * e);
  void handleMouseMoveEvent(QMouseEvent * e);
  void editMask(QMouseEvent * e);
  void undoEditMask();

protected:
  QVBoxLayout *layout_ = nullptr;
  QImageScene *scene_ = nullptr;
  QImageSceneView *view_ = nullptr;
  QToolBar *toolbar_ = nullptr;
  QStatusBar *statusbar_ = nullptr;

  QImageDisplayFunction *displayFunction_ = nullptr;
  DisplayType currentDisplayType_ = DisplayImage;
  QString currentFileName_;
  cv::Mat currentImage_, currentImageData_, currentMask_;
  cv::Mat displayImage_;
  QImage qimage_;


  bool transparentMask_ = true;
  bool enableEditMask_ = false;
  int editMaskPenRadius_ = 5;
  PenShape editMaskPenShape_ = PenShape_square;
  // QAction * undoEditMaskAction_ = nullptr;
  QShortcut *undoEditMaskActionShortcut_ = nullptr;
  QStack<cv::Mat> editMaskUndoQueue_;

  std::mutex currentImageLock_;

};

#endif /* __QImageViewer_h__ */
