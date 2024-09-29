/*
 * QImageViewer.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageViewer.h"
#include "cv2qt.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<QImageViewer::DisplayType>()
{
  static const c_enum_member members[] = {
      { QImageViewer::DisplayImage, "Image", ""},
      { QImageViewer::DisplayMask, "Mask", "" },
      { QImageViewer::DisplayBlend, "Blend", "" },
      { QImageViewer::DisplayImage}, // must be last
      };

  return members;
}


QImageViewer::QImageViewer(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

QImageViewer::QImageViewer(QImageScene * scene, QWidget * parent) :
    Base(parent),
    _scene(scene)
{
  if ( !_scene ) {
    _scene = new QImageScene(this);
  }

  _layout = new QVBoxLayout(this);
  _layout->setAlignment(Qt::AlignTop);
  _layout->setContentsMargins(0,0,0,0);
  //layout_->setMargin(0);
  _layout->setSpacing(0);
  _layout->addWidget(_view = new QImageSceneView(this), 1000);
  _view->setScene(_scene);

  connect(_view, &QImageSceneView::onMouseMove,
      this, &ThisClass::handleMouseMoveEvent);
  connect(_view, &QImageSceneView::onMousePressEvent,
      this, &ThisClass::handleMousePressEvent);
  connect(_view, &QImageSceneView::onMouseReleaseEvent,
      this, &ThisClass::onMouseReleaseEvent);
  connect(_view, &QImageSceneView::onMouseDoubleClick,
      this, &ThisClass::onMouseDoubleClick);
  connect(_view, &QImageSceneView::onMouseEnterEvent,
      this, &ThisClass::onMouseEnterEvent);
  connect(_view, &QImageSceneView::onMouseLeaveEvent,
      this, &ThisClass::onMouseLeaveEvent);
  connect(_view, &QImageSceneView::scaleChanged,
      this, &ThisClass::onScaleChanged);
  connect(_view, &QImageSceneView::viewScrolled,
      this, &ThisClass::onViewScrolled);

  _undoEditMaskActionShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Z), this);
  connect(_undoEditMaskActionShortcut, &QShortcut::activated,
      this, &ThisClass::undoEditMask);
}

QImageSceneView * QImageViewer::sceneView() const
{
  return _view;
}

QImageScene * QImageViewer::scene() const
{
  return _scene;
}

QToolBar * QImageViewer::embedToolbar(QToolBar * toolbar)
{
  if ( this->_toolbar ) {
    if ( !toolbar ) {
      return this->_toolbar; // already embedded
    }
    _layout->removeWidget(this->_toolbar);
  }


  if ( !(this->_toolbar = toolbar) ) {
    this->_toolbar = new QToolBar(this);
    this->_toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    this->_toolbar->setOrientation(Qt::Horizontal);
    this->_toolbar->setIconSize(QSize(16,16));
  }

  _layout->insertWidget(0, this->_toolbar, 0);

  return this->_toolbar;
}

QToolBar * QImageViewer::toolbar() const
{
  return _toolbar;
}

QStatusBar * QImageViewer::embedStatusbar(QStatusBar * statusBar)
{
  if ( this->_statusbar ) {
    if ( !statusBar ) {
      return this->_statusbar; // already embedded
    }
    _layout->removeWidget(this->_statusbar);
  }

  if ( !(this->_statusbar = statusBar) ) {
    this->_statusbar = new QStatusBar(this);
  }

  _layout->addWidget(this->_statusbar, 0);

  return this->_statusbar;
}

QStatusBar * QImageViewer::statusbar() const
{
  return _statusbar;
}


void QImageViewer::setDisplayFunction(QImageDisplayFunction *  displayfunction)
{
  if( _displayFunction ) {
//    disconnect(displayFunction_, &QImageDisplayFunction::update,
//        this, &ThisClass::updateDisplay);
  }

  if( (_displayFunction = displayfunction) ) {
//    connect(displayFunction_, &QImageDisplayFunction::update,
//        this, &ThisClass::updateDisplay);
  }
}

QImageDisplayFunction * QImageViewer::displayFunction() const
{
  return this->_displayFunction;
}

void QImageViewer::setViewScale(int scale, const QPoint * centerPos)
{
  _view->setViewScale(scale, centerPos);
}

int QImageViewer::viewScale() const
{
  return _view->viewScale();
}

void QImageViewer::setDisplayType(DisplayType v)
{
  _currentDisplayType = v;
  displayTypeChanged();
  updateDisplay();
}

QImageViewer::DisplayType QImageViewer::displayType() const
{
  return _currentDisplayType;
}

void QImageViewer::setMaskBlendAlpha(double v)
{
  _maskBlendAlpha = v;
  updateDisplay();
}

double QImageViewer::maskBlendAlpha() const
{
  return _maskBlendAlpha;
}

void QImageViewer::setTransparentMask(bool v)
{
  _transparentMask = v;
  updateDisplay();
}

bool QImageViewer::transparentMask() const
{
  return _transparentMask;
}

const cv::Mat & QImageViewer::currentImage() const
{
  return _currentImage;
}

const cv::Mat & QImageViewer::currentMask() const
{
  return _currentMask;
}

const cv::Mat & QImageViewer::currentImageData() const
{
  return _currentImageData;
}

const cv::Mat& QImageViewer::mtfImage() const
{
  return _mtfImage;
}

const cv::Mat & QImageViewer::displayImage() const
{
  return _displayImage;
}

QPixmap QImageViewer::grabViewportPixmap()
{
  QWidget * w = _view->viewport();
  return w ? w->grab(w->rect()) : _view->grab(_view->rect());
}

QString QImageViewer::currentFileName() const
{
  return this->_currentFileName;
}

void QImageViewer::setCurrentFileName(const QString & newFileName)
{
  if( _currentFileName != newFileName ) {
    _currentFileName = newFileName;
    Q_EMIT currentFileNameChanged();
  }
}

void QImageViewer::setCurrentImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  // c_current_image_lock lock(this);

  _editMaskUndoQueue.clear();

  if( image.empty() ) {
    current_image_lock lock(this);
    _currentImage.release();
    _currentMask.release();
    _currentImageData.release();
  }
  else if( make_copy ) {
    current_image_lock lock(this);
    image.getMat().copyTo(_currentImage);
    mask.getMat().copyTo(_currentMask);
    imageData.getMat().copyTo(_currentImageData);
  }
  else {
    current_image_lock lock(this);
    _currentImage = image.getMat();
    _currentMask = mask.getMat();
    _currentImageData = imageData.getMat();
  }

//  CF_DEBUG("Q_EMIT currentImageChanged()");
//  Q_EMIT currentImageChanged();
}

void QImageViewer::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData, bool make_copy)
{
  setCurrentImage(image, mask, imageData, make_copy);
  updateDisplay();
}

void QImageViewer::setMask(cv::InputArray mask, bool make_copy /*= true*/)
{
  if( !mask.empty() && (mask.size() != _currentImage.size() || mask.type() != CV_8UC1) ) {
    CF_ERROR("Invalid mask specifed: %dx%d channes=%d depth=%d. Must be  %dx%d CV_8UC1",
        mask.cols(), mask.rows(), mask.channels(), mask.depth(),
        _currentImage.cols, _currentImage.rows);
    return;
  }

  _editMaskUndoQueue.clear();

  if ( make_copy ) {
    mask.getMat().copyTo(_currentMask);
  }
  else {
    _currentMask = mask.getMat();
  }

  Q_EMIT currentImageChanged();
  updateDisplay();
}


void QImageViewer::updateDisplay()
{
  if ( isVisible() ) {
    createDisplayImage();
    showCurrentDisplayImage();
  }
}

void QImageViewer::createDisplayImage()
{
  if ( _currentDisplayType == DisplayMask ) {
    _currentMask.copyTo(_displayImage);
  }
  else {

    if( true ) {
      current_image_lock lock(this);

      if( _currentImage.empty() ) {
        _displayImage.release();
      }
      else {

        if( !_displayFunction ) {
          _currentImage.copyTo(_displayImage);
        }
        else if( _currentImage.channels() == 2 ) { // assume this is optical flow image

          _displayFunction->createDisplayImage(_currentImage,
              _transparentMask ? cv::noArray() : _currentMask,
              _mtfImage,
              _displayImage,
              _currentImage.depth());
        }
        else {
          // displayImage_.release();

          _displayFunction->createDisplayImage(_currentImage,
              _transparentMask ? cv::noArray() : _currentMask,
              _mtfImage,
              _displayImage,
              CV_8U);
        }
      }
    }

    if ( _currentDisplayType == DisplayBlend && !_displayImage.empty() && !_currentMask.empty() ) {

      cv::Mat mask;
      if ( _displayImage.channels() == _currentMask.channels() ) {
        mask = _currentMask;
      }
      else {
        const int cn = _displayImage.channels();
        cv::Mat channels[cn];
        for ( int i = 0; i < cn; ++i ) {
          channels[i] = _currentMask;
        }
        cv::merge(channels, cn, mask);
      }

      cv::addWeighted(_displayImage, _maskBlendAlpha,
          mask, 1 - _maskBlendAlpha,
          0,
          _displayImage,
          _displayImage.depth());
    }

  }

  Q_EMIT displayImageChanged();
}

//////
//static QPixmap QPixmap_fromImage(const QImage &image, Qt::ImageConversionFlags flags)
//{
//    if (image.isNull())
//        return QPixmap();
//    if (Q_UNLIKELY(!qobject_cast<QGuiApplication *>(QCoreApplication::instance()))) {
//        qWarning("QPixmap::fromImage: QPixmap cannot be created without a QGuiApplication");
//        return QPixmap();
//    }
//    QScopedPointer<QPlatformPixmap> data(QGuiApplicationPrivate::platformIntegration()->createPlatformPixmap(QPlatformPixmap::PixmapType));
//    data->fromImage(image, flags);
//    return QPixmap(data.take());
//}
//////

void QImageViewer::showCurrentDisplayImage()
{
  if( _displayImage.empty() ) {
    scene()->setImage(QPixmap());
  }
  else {

    QPixmap pixmap =
        createPixmap(_displayImage, true,
            Qt::AutoColor |
                Qt::ThresholdDither |
                Qt::ThresholdAlphaDither |
                Qt::NoOpaqueDetection);

    scene()->setImage(pixmap);
  }
}


void QImageViewer::copyDisplayImageToClipboard()
{
  QClipboard * clipboard = QApplication::clipboard();
  if ( !clipboard ) {
    QMessageBox::critical(this, "ERROR", "No application clipboard available");
  }
  else {
    cv2qt(displayImage(), &_qimage, true);
    clipboard->setImage(_qimage);
  }
}

void QImageViewer::copyDisplayImageROIToClipboard(const QRect & roi)
{
  QClipboard *clipboard = QApplication::clipboard();
  if( !clipboard ) {
    QMessageBox::critical(this, "ERROR", "No application clipboard available");
  }
  else {
    cv::Rect rc;
    if( adjustRoi(roi, cv::Rect(0, 0, _displayImage.cols, _displayImage.rows), &rc) ) {
      cv2qt(_displayImage(rc), &_qimage, true);
      clipboard->setImage(_qimage);
    }
  }
}


template<class T>
static int sdump_(const cv::Mat & image, int x, int y, char buf[], int bufsz, int n, const char * dtype, const char * fmt)
{
  const int nc = image.channels();
  const T * m = image.ptr<const T>(y) + x * nc;

  n += snprintf(buf + n, std::max(0, bufsz - n - 1), "%s: ", dtype);
  for ( int c = 0; c < nc; ++c ) {
    n += snprintf(buf + n, std::max(0, bufsz - n - 1), fmt, m[c]);
    if ( c < nc - 1 && n < bufsz - 1 ) {
      *(buf + n++) = ' ';
    }
  }
  return n;
}

QString QImageViewer::statusStringForPixel(const QPoint & viewpos)
{
  static const auto sdump =
      [](const cv::Mat & image, int x, int y, char buf[], int bufsz, int n) -> int
  {
    if ( x >= 0 && y >= 0 && x < image.cols && y < image.rows ) {

      n += snprintf(buf + n, bufsz - 1 - n, "  ");

      switch ( image.depth() ) {
      case CV_8U :
        n += sdump_<uint8_t>(image, x, y, buf, bufsz, n, "uint8", "%hhu");
        break;

      case CV_8S :
        n += sdump_<int8_t>(image, x, y, buf, bufsz, n, "int8", "%+hhd");
        break;

      case CV_16U:
        n += sdump_<uint16_t>(image, x, y, buf, bufsz, n, "uint16", "%hu");
        break;

      case CV_16S:
        n += sdump_<int16_t>(image, x, y, buf, bufsz, n, "int16", "%+hd");
        break;

      case CV_32S:
        n += sdump_<int32_t>(image, x, y, buf, bufsz, n, "int32", "%+d");
        break;

      case CV_32F:
        n += sdump_<float>(image, x, y, buf, bufsz, n, "float", "%+.8g");
        break;

      case CV_64F:
        n += sdump_<double>(image, x, y, buf, bufsz, n, "double", "%+.16g");
        break;
      }
    }

    return n;
  };


  char buf[2048] = "";
  int n = 0;

  QPointF pos =
      _view->mapToScene(viewpos);

  const int x =
      pos.x();

  const int y =
      pos.y();

  n += snprintf(buf + n, sizeof(buf) - 1 - n,
      "| x=%g y=%g",
      pos.x(),
      pos.y());

  current_image_lock lock(this);

  if ( !_currentImageData.empty() ) {
    n = sdump(_currentImageData, x, y, buf, sizeof(buf) - 1 - n, n);
  }
  else if ( !_currentImage.empty() ) {
    n = sdump(_currentImage, x, y, buf, sizeof(buf) - 1 - n, n);
  }
  else if ( !_qimage.isNull() && x >= 0 && x < _qimage.width() && y >= 0 && y < _qimage.height() ) {
    const QRgb px = _qimage.pixel(x, y);
    n += snprintf(buf + n, sizeof(buf) - 1 - n, " RGB %d %d %d", qRed(px), qGreen(px), qBlue(px));
  }

  return buf;
}

void QImageViewer::showEvent(QShowEvent *e)
{
  updateDisplay();
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QImageViewer::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QImageViewer::focusInEvent(QFocusEvent *e)
{
  Q_EMIT onFocusInEvent(e);
  Base::focusInEvent(e);
}

void QImageViewer::focusOutEvent(QFocusEvent *e)
{
  Q_EMIT onFocusOutEvent(e);
  Base::focusOutEvent(e);
}

void QImageViewer::setEnableEditMask(bool enable)
{
  _enableEditMask = enable;
  _view->setMouseScrollEnabled(!enable);
  updateCursor();
}

bool QImageViewer::enableEditMask() const
{
  return _enableEditMask;
}

void QImageViewer::setEditMaskPenRadius(int v)
{
  _editMaskPenRadius = v;
  updateCursor();
}

int QImageViewer::editMaskPenRadius() const
{
  return _editMaskPenRadius;
}

void QImageViewer::setEditMaskPenShape(PenShape v)
{
  _editMaskPenShape = v;
  updateCursor();
}

QImageViewer::PenShape QImageViewer::editMaskPenShape() const
{
  return _editMaskPenShape;
}

void QImageViewer::setKeepMaskOnMaskEditMode(bool v)
{
  _keepMaskOnMaskEditMode = v;
}

bool QImageViewer::keepMaskOnMaskEditMode() const
{
  return _keepMaskOnMaskEditMode;
}

void QImageViewer::updateCursor()
{
  static const QCursor defaultCursor =
      _view->cursor();

  _view->setCursor(_enableEditMask ?
      Qt::CursorShape::CrossCursor :
      defaultCursor);
}

void QImageViewer::editMask(QMouseEvent * e)
{
  //c_current_image_lock lock(this);

  if( !_currentImage.empty() ) {

    if( _currentMask.empty() ) {
      _currentMask.create(_currentImage.size(), CV_8UC1);
      _currentMask.setTo(255);
    }

    if ( _editMaskUndoQueue.size() > 100 ) {
      _editMaskUndoQueue.erase(_editMaskUndoQueue.begin());
    }

    _editMaskUndoQueue.push(_currentMask.clone());

    const QPointF pos = _view->mapToScene(e->pos());
    const int x = pos.x(), y = pos.y();

    switch (_editMaskPenShape) {
    case PenShape_circle:

      cv::circle(_currentMask, cv::Point(x, y),
          _editMaskPenRadius,
          0,
          -1,
          cv::LINE_8,
          0);

      break;

    default:
      cv::rectangle(_currentMask,
          cv::Point(x - _editMaskPenRadius, y - _editMaskPenRadius),
          cv::Point(x + _editMaskPenRadius, y + _editMaskPenRadius),
          0,
          -1,
          cv::LINE_8,
          0);
      break;
    }

    updateDisplay();
    Q_EMIT currentImageChanged();
  }
}

void QImageViewer::undoEditMask()
{
  //c_current_image_lock lock(this);

  if( !_editMaskUndoQueue.empty() ) {

    cv::Mat mask = _editMaskUndoQueue.pop();
    if( mask.size() == _currentImage.size() ) {
      _currentMask = mask;
      Q_EMIT currentImageChanged();
      updateDisplay();
    }
  }
}


bool QImageViewer::adjustRoi(const cv::Rect & srcRoi, const cv::Rect & imageRect, cv::Rect * dstRoi)
{
  const int l =
      (std::min)(imageRect.x + imageRect.width - 1,
          (std::max)(srcRoi.x, imageRect.x));

  const int t =
      (std::min)(imageRect.y + imageRect.height - 1,
          (std::max)(srcRoi.y, imageRect.y));

  const int r =
      std::max(imageRect.x,
          std::min(srcRoi.x + srcRoi.width - 1, imageRect.x + imageRect.width - 1));

  const int b =
      std::max(imageRect.y,
          std::min(srcRoi.y + srcRoi.height - 1, imageRect.y + imageRect.height - 1));

  return !(*dstRoi = cv::Rect(l, t, r - l + 1, b - t + 1)).empty();
}

bool QImageViewer::adjustRoi(const QRect & srcRoi, const cv::Rect & imageRect, cv::Rect * dstRoi)
{
  return adjustRoi(cv::Rect(srcRoi.x(), srcRoi.y(), srcRoi.width(), srcRoi.height()),
      imageRect,
      dstRoi);
}

void QImageViewer::handleMousePressEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && _enableEditMask ) {
    editMask(e);
  }

  Q_EMIT onMousePressEvent(e);
}

void QImageViewer::handleMouseMoveEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && _enableEditMask ) {
    editMask(e);
  }
  Q_EMIT onMouseMove(e);
}


