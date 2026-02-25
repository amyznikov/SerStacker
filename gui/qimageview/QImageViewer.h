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
        base(view->_currentImageLock)
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

  void setMaskBlendAlpha(double v);
  double maskBlendAlpha() const;

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
  const cv::Mat& mtfImage() const;
  const cv::Mat& displayImage() const;
  QPixmap grabViewportPixmap();

  QString currentFileName() const;
  void setCurrentFileName(const QString & newFileName);

  QString statusStringForPixel(const QPoint & viewpos);


  void setEnableEditMask(bool enable);
  bool enableEditMask() const;

  void setEditMaskPenRadius(int v);
  int editMaskPenRadius() const;

  void setEditMaskPenShape(PenShape v);
  PenShape editMaskPenShape() const;

  void setKeepMaskOnMaskEditMode(bool v);
  bool keepMaskOnMaskEditMode() const;

  static bool adjustRoi(const cv::Rect & srcRoi, const cv::Rect & imageRect, cv::Rect * dstRoi);
  static bool adjustRoi(const QRect & srcRoi, const cv::Rect & imageRect, cv::Rect * dstRoi);

  virtual void populateContextMenu(QMenu & menu, const QPoint & mpos);

  void loadSettings(const QString & prefix = "");
  void loadSettings(const QSettings & settings, const QString & prefix = "");
  void saveSettings(const QString & prefix = "");
  void saveSettings(QSettings & settings, const QString & prefix = "");

Q_SIGNALS:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onMouseEnterEvent(QEvent * e);
  void onMouseLeaveEvent(QEvent * e);
  void onScaleChanged(int currentScale);
  void onViewScrolled();
  void onFocusInEvent(QFocusEvent * e);
  void onFocusOutEvent(QFocusEvent * e);
  void onPopulateContextMenu(QMenu & menu, const QPoint & mpos);
  void visibilityChanged(bool visible);
  void currentImageChanged();
  void currentFileNameChanged();
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
  virtual void onload(const QSettings & settings, const QString & prefix);
  virtual void onsave(QSettings & settings, const QString & prefix);

public Q_SLOTS:
  virtual void updateDisplay();
  void copyDisplayImageToClipboard();
  void copyDisplayImageROIToClipboard(const QRect & roi);
  void handleMousePressEvent(QMouseEvent * e);
  void handleMouseMoveEvent(QMouseEvent * e);
  void editMask(QMouseEvent * e);
  void undoEditMask();

protected:
  QVBoxLayout *_layout = nullptr;
  QImageScene *_scene = nullptr;
  QImageSceneView *_view = nullptr;
  QToolBar *_toolbar = nullptr;
  QStatusBar *_statusbar = nullptr;

  QImageDisplayFunction *_displayFunction = nullptr;
  DisplayType _currentDisplayType = DisplayImage;
  double _maskBlendAlpha  = 0.9;

  QString _currentFileName;
  cv::Mat _currentImage;
  cv::Mat _mtfImage;
  cv::Mat _currentImageData;
  cv::Mat _currentMask;
  cv::Mat _displayImage;
  QImage _qimage;


  bool _transparentMask = true;
  bool _enableEditMask = false;
  bool _keepMaskOnMaskEditMode = true;
  int _editMaskPenRadius = 15;
  PenShape _editMaskPenShape = PenShape_circle;
  QShortcut *_undoEditMaskActionShortcut = nullptr;
  QStack<cv::Mat> _editMaskUndoQueue;

  std::mutex _currentImageLock;

};

#endif /* __QImageViewer_h__ */
