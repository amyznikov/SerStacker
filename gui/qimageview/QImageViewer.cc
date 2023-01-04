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
  static constexpr c_enum_member members[] = {
      { QImageViewer::DisplayImage, "Image", "" },
      { QImageViewer::DisplayMask, "Mask", "" },
      { QImageViewer::DisplayBlend, "Blend", "" },
      { QImageViewer::DisplayImage }, // must be last
      };

  return members;
}

QImageViewer::QImageViewer(QWidget * parent)
  : Base(parent)
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0,0,0,0);
  layout_->addWidget(view_ = new QImageSceneView(this), 100);

  connect(view_, &QImageSceneView::onMouseMove,
      this, &ThisClass::handleMouseMoveEvent);
  connect(view_, &QImageSceneView::onMousePressEvent,
      this, &ThisClass::handleMousePressEvent);
  connect(view_, &QImageSceneView::onMouseReleaseEvent,
      this, &ThisClass::onMouseReleaseEvent);
  connect(view_, &QImageSceneView::onMouseDoubleClick,
      this, &ThisClass::onMouseDoubleClick);
  connect(view_, &QImageSceneView::scaleChanged,
      this, &ThisClass::onScaleChanged);
//  connect(view_, &QImageSceneView::graphicsShapeChanged,
//      this, &ThisClass::graphicsShapeChanged);

  //QShortcut * shortcut = new QShortcut();

//  undoEditMaskAction_ = new QAction(this);
//  connect(undoEditMaskAction_, &QAction::triggered,
//      this, &ThisClass::undoEditMask);

  undoEditMaskActionShortcut_ = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Z), this);
  //connect(undoEditMaskActionShortcut_, &QShortcut::activated,
  //  undoEditMaskAction_, &QAction::trigger);
  connect(undoEditMaskActionShortcut_, &QShortcut::activated,
      this, &ThisClass::undoEditMask);

}

QImageSceneView * QImageViewer::sceneView() const
{
  return view_;
}

QImageScene * QImageViewer::scene() const
{
  return view_->scene();
}

QToolBar * QImageViewer::embedToolbar(QToolBar * toolbar)
{
  if ( this->toolbar_ ) {
    if ( !toolbar ) {
      return this->toolbar_; // already embedded
    }
    layout_->removeWidget(this->toolbar_);
  }


  if ( !(this->toolbar_ = toolbar) ) {
    this->toolbar_ = new QToolBar(this);
    this->toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    this->toolbar_->setOrientation(Qt::Horizontal);
    this->toolbar_->setIconSize(QSize(16,16));
  }

  layout_->insertWidget(0, this->toolbar_, 1);

  return this->toolbar_;
}

QToolBar * QImageViewer::toolbar() const
{
  return toolbar_;
}

QStatusBar * QImageViewer::embedStatusbar(QStatusBar * statusBar)
{
  if ( this->statusbar_ != Q_NULLPTR ) {
    if ( statusBar == Q_NULLPTR ) {
      return this->statusbar_; // already embedded
    }
    layout_->removeWidget(this->statusbar_);
  }

  if ( (this->statusbar_ = statusBar) == Q_NULLPTR ) {
    this->statusbar_ = new QStatusBar(this);
  }

  layout_->addWidget(this->statusbar_, 0);

  return this->statusbar_;
}

QStatusBar * QImageViewer::statusbar() const
{
  return statusbar_;
}


void QImageViewer::setDisplayFunction(QImageDisplayFunction *  displayfunction)
{
  if( displayFunction_ ) {
    disconnect(displayFunction_, &QImageDisplayFunction::update,
        this, &ThisClass::updateDisplay);
  }

  if( (displayFunction_ = displayfunction) ) {
    connect(displayFunction_, &QImageDisplayFunction::update,
        this, &ThisClass::updateDisplay);
  }
}

QImageDisplayFunction * QImageViewer::displayFunction() const
{
  return this->displayFunction_;
}

void QImageViewer::setViewScale(int scale, const QPoint * centerPos)
{
  view_->setViewScale(scale, centerPos);
}

int QImageViewer::viewScale() const
{
  return view_->viewScale();
}

void QImageViewer::setDisplayType(DisplayType v)
{
  currentDisplayType_ = v;
  currentDisplayTypeChanged();
  updateDisplay();
}

QImageViewer::DisplayType QImageViewer::displayType() const
{
  return currentDisplayType_;
}

const cv::Mat & QImageViewer::currentImage() const
{
  return currentImage_;
}

const cv::Mat & QImageViewer::currentMask() const
{
  return currentMask_;
}

const cv::Mat & QImageViewer::currentImageData() const
{
  return currentImageData_;
}

const cv::Mat & QImageViewer::displayImage() const
{
  return displayImage_;
}

QString QImageViewer::currentFileName() const
{
  return this->currentFileName_;
}

void QImageViewer::setCurrentFileName(const QString & newFileName)
{
  this->currentFileName_ = newFileName;
  Q_EMIT currentImageChanged();
}

void QImageViewer::setCurrentImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  INSTRUMENT_REGION("");
  editMaskUndoQueue_.clear();

  if( image.empty() ) {
    currentImage_.release();
    currentMask_.release();
    currentImageData_.release();
  }
  else if( make_copy ) {
    image.getMat().copyTo(currentImage_);
    mask.getMat().copyTo(currentMask_);
    imageData.getMat().copyTo(currentImageData_);
  }
  else {
    currentImage_ = image.getMat();
    currentMask_ = mask.getMat();
    currentImageData_ = imageData.getMat();
  }

  if ( displayFunction_ ) {
    displayFunction_->setCurrentImage(currentImage_,
        currentMask_);
  }

  Q_EMIT currentImageChanged();
}

void QImageViewer::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData, bool make_copy)
{
  setCurrentImage(image, mask, imageData, make_copy);
  updateDisplay();
}

void QImageViewer::setMask(cv::InputArray mask, bool make_copy /*= true*/)
{
  if( !mask.empty() && (mask.size() != currentImage_.size() || mask.type() != CV_8UC1) ) {
    CF_ERROR("Invalid mask specifed: %dx%d channes=%d depth=%d. Must be  %dx%d CV_8UC1",
        mask.cols(), mask.rows(), mask.channels(), mask.depth(),
        currentImage_.cols, currentImage_.rows);
    return;
  }

  editMaskUndoQueue_.clear();

  if ( make_copy ) {
    mask.getMat().copyTo(currentMask_);
  }
  else {
    currentMask_ = mask.getMat();
  }

  Q_EMIT currentImageChanged();
  updateDisplay();
}


void QImageViewer::updateDisplay()
{
  INSTRUMENT_REGION("");
  if ( isVisible() ) {
    createDisplayImage();
    showCurrentDisplayImage();
  }
}

void QImageViewer::createDisplayImage()
{
  if ( currentDisplayType_ == DisplayMask ) {
    currentMask_.copyTo(displayImage_);
  }
  else {
    if ( currentImage_.empty() ) {
      displayImage_.release();
    }
    else {
      if ( !displayFunction_ ) {
        currentImage_.copyTo(displayImage_);
      }
      else if ( currentImage_.channels() == 2  ) { // assume this is optical flow image
        displayImage_.release();
        displayFunction_->getDisplayImage(displayImage_, currentImage_.depth());
      }
      else  {
        displayImage_.release();
        displayFunction_->getDisplayImage(displayImage_, CV_8U);
      }
    }

    if ( currentDisplayType_ == DisplayBlend && !displayImage_.empty() && !currentMask_.empty() ) {

      cv::Mat mask;
      if ( displayImage_.channels() == currentMask_.channels() ) {
        mask = currentMask_;
      }
      else {
        const int cn = displayImage_.channels();
        cv::Mat channels[cn];
        for ( int i = 0; i < cn; ++i ) {
          channels[i] = currentMask_;
        }
        cv::merge(channels, cn, mask);
      }

      cv::addWeighted(displayImage_, 0.5, mask, 0.5, 0, displayImage_, displayImage_.depth());
    }

  }
  emit currentDisplayImageChanged();
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
  INSTRUMENT_REGION("");
  if( displayImage_.empty() ) {
    scene()->setBackground(QPixmap());
  }
  else {

    QPixmap pixmap;

    if( displayImage_.type() == CV_8UC3 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_BGR888 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }
    else if( displayImage_.type() == CV_8UC1 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_Grayscale8 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }
    else if( displayImage_.type() == CV_16UC1 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_Grayscale16 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }
    else {

      cv2qt(displayImage_, &qimage_);

      pixmap =
          QPixmap::fromImage(qimage_,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);
    }


    scene()->setBackground(pixmap);
  }
}


void QImageViewer::copyDisplayImageToClipboard()
{
  QClipboard * clipboard = QApplication::clipboard();
  if ( !clipboard ) {
    QMessageBox::critical(this, "ERROR", "No application clipboard available");
  }
  else {
    cv2qt(displayImage(), &qimage_, true);
    clipboard->setImage(qimage_);
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

  QPointF pos = view_->mapToScene(viewpos);
  const int x = pos.x(), y = pos.y();

  n += snprintf(buf + n, sizeof(buf) - 1 - n, "x=%g y=%g", pos.x(), pos.y());

  if ( !currentImageData_.empty() ) {
    n = sdump(currentImageData_, x, y, buf, sizeof(buf) - 1 - n, n);
  }
  else if ( !currentImage_.empty() ) {
    n = sdump(currentImage_, x, y, buf, sizeof(buf) - 1 - n, n);
  }
  else if ( !qimage_.isNull() && x >= 0 && x < qimage_.width() && y >= 0 && y < qimage_.height() ) {
    const QRgb px = qimage_.pixel(x, y);
    n += snprintf(buf + n, sizeof(buf) - 1 - n, " RGB %d %d %d", qRed(px), qGreen(px), qBlue(px));
  }

  return buf;
}

void QImageViewer::showEvent(QShowEvent *e)
{
  updateDisplay();
  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QImageViewer::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}

void QImageViewer::focusInEvent(QFocusEvent *e)
{
  emit onFocusInEvent(e);
  Base::focusInEvent(e);
}

void QImageViewer::focusOutEvent(QFocusEvent *e)
{
  emit onFocusOutEvent(e);
  Base::focusOutEvent(e);
}

void QImageViewer::setEnableEditMask(bool enable)
{
  enableEditMask_ = enable;
  view_->setMouseScrollEnabled(!enable);
  updateCursor();
}

bool QImageViewer::enableEditMask() const
{
  return enableEditMask_;
}

void QImageViewer::setEditMaskPenRadius(int v)
{
  editMaskPenRadius_ = v;
  updateCursor();
}

int QImageViewer::editMaskPenRadius() const
{
  return editMaskPenRadius_;
}

void QImageViewer::setEditMaskPenShape(PenShape v)
{
  editMaskPenShape_ = v;
  updateCursor();
}

QImageViewer::PenShape QImageViewer::editMaskPenShape() const
{
  return editMaskPenShape_;
}

void QImageViewer::updateCursor()
{
  static const QCursor defaultCursor =
      view_->cursor();

  if ( !enableEditMask_ ) {
    view_->setCursor(defaultCursor);
  }
  else {
    view_->setCursor(Qt::CursorShape::CrossCursor);

//    const int imageSize =
//        std::min(65, std::max(17, 2 * editMaskPenRadius_ + 1));
//
//    QImage image(imageSize, imageSize, QImage::Format_Mono);
//    QPainter image_painter(&image);
//    QPen imagePen(Qt::black);
//
//    imagePen.setWidth(1);
//    image_painter.setPen(imagePen);
//    image_painter.fillRect(0, 0, imageSize, imageSize, QBrush(Qt::white));
//
//
//
//    QImage mask(imageSize, imageSize, QImage::Format_Mono);
//    QPainter mask_painter(&mask);
//    QPen maskPen(Qt::black);
//
//    maskPen.setWidth(2);
//    mask_painter.setPen(maskPen);
//    mask_painter.fillRect(0, 0, imageSize, imageSize, QBrush(Qt::white));
//
//
//    switch (editMaskPenShape_) {
//    case PenShape_circle:
//      image_painter.drawEllipse(0, 0, imageSize - 1, imageSize - 1);
//      mask_painter.drawEllipse(0, 0, imageSize - 1, imageSize - 1);
//      break;
//    default:
//      image_painter.drawRect(0, 0, imageSize - 1, imageSize - 1);
//      mask_painter.drawRect(0, 0, imageSize - 1, imageSize - 1);
//      break;
//    }
//
//    image_painter.drawRect(imageSize / 2 - 2, imageSize / 2 - 2, 4, 4);
//    mask_painter.fillRect(imageSize / 2 - 2, imageSize / 2 - 2, 5, 5, Qt::black);
//
//    setCursor(QCursor(QBitmap::fromImage(image), QBitmap::fromImage(mask),
//        imageSize / 2, imageSize / 2));
  }
}

void QImageViewer::editMask(QMouseEvent * e)
{
  if( !currentImage_.empty() ) {

    if( currentMask_.empty() ) {
      currentMask_.create(currentImage_.size(), CV_8UC1);
      currentMask_.setTo(255);
    }

    if ( editMaskUndoQueue_.size() > 100 ) {
      editMaskUndoQueue_.erase(editMaskUndoQueue_.begin());
    }

    editMaskUndoQueue_.push(currentMask_.clone());

    const QPointF pos = view_->mapToScene(e->pos());
    const int x = pos.x(), y = pos.y();

    switch (editMaskPenShape_) {
    case PenShape_circle:

      cv::circle(currentMask_, cv::Point(x, y),
          editMaskPenRadius_,
          0,
          -1,
          cv::LINE_8,
          0);

      break;

    default:
      cv::rectangle(currentMask_,
          cv::Point(x - editMaskPenRadius_, y - editMaskPenRadius_),
          cv::Point(x + editMaskPenRadius_, y + editMaskPenRadius_),
          0,
          -1,
          cv::LINE_8,
          0);
      break;
    }

    Q_EMIT currentImageChanged();
    updateDisplay();
  }
}

void QImageViewer::undoEditMask()
{
  if ( !editMaskUndoQueue_.empty() ) {

    cv::Mat mask = editMaskUndoQueue_.pop();
    if ( mask.size() == currentImage_.size() ) {
      currentMask_ = mask;
      Q_EMIT currentImageChanged();
      updateDisplay();
    }
  }
}

void QImageViewer::handleMousePressEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && enableEditMask_ ) {
    editMask(e);
  }

  emit onMousePressEvent(e);
}

void QImageViewer::handleMouseMoveEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && enableEditMask_ ) {
    editMask(e);
  }
  emit onMouseMove(e);
}


