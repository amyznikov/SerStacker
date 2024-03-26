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
    scene_(scene)
{
  if ( !scene_ ) {
    scene_ = new QImageScene(this);
  }

  layout_ = new QVBoxLayout(this);
  layout_->setAlignment(Qt::AlignTop);
  layout_->setContentsMargins(0,0,0,0);
  //layout_->setMargin(0);
  layout_->setSpacing(0);
  layout_->addWidget(view_ = new QImageSceneView(this), 1000);
  view_->setScene(scene_);

  connect(view_, &QImageSceneView::onMouseMove,
      this, &ThisClass::handleMouseMoveEvent);
  connect(view_, &QImageSceneView::onMousePressEvent,
      this, &ThisClass::handleMousePressEvent);
  connect(view_, &QImageSceneView::onMouseReleaseEvent,
      this, &ThisClass::onMouseReleaseEvent);
  connect(view_, &QImageSceneView::onMouseDoubleClick,
      this, &ThisClass::onMouseDoubleClick);
  connect(view_, &QImageSceneView::onMouseEnterEvent,
      this, &ThisClass::onMouseEnterEvent);
  connect(view_, &QImageSceneView::onMouseLeaveEvent,
      this, &ThisClass::onMouseLeaveEvent);
  connect(view_, &QImageSceneView::scaleChanged,
      this, &ThisClass::onScaleChanged);
  connect(view_, &QImageSceneView::viewScrolled,
      this, &ThisClass::onViewScrolled);

  undoEditMaskActionShortcut_ = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Z), this);
  connect(undoEditMaskActionShortcut_, &QShortcut::activated,
      this, &ThisClass::undoEditMask);
}

QImageSceneView * QImageViewer::sceneView() const
{
  return view_;
}

QImageScene * QImageViewer::scene() const
{
  return scene_;
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

  layout_->insertWidget(0, this->toolbar_, 0);

  return this->toolbar_;
}

QToolBar * QImageViewer::toolbar() const
{
  return toolbar_;
}

QStatusBar * QImageViewer::embedStatusbar(QStatusBar * statusBar)
{
  if ( this->statusbar_ ) {
    if ( !statusBar ) {
      return this->statusbar_; // already embedded
    }
    layout_->removeWidget(this->statusbar_);
  }

  if ( !(this->statusbar_ = statusBar) ) {
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
//    disconnect(displayFunction_, &QImageDisplayFunction::update,
//        this, &ThisClass::updateDisplay);
  }

  if( (displayFunction_ = displayfunction) ) {
//    connect(displayFunction_, &QImageDisplayFunction::update,
//        this, &ThisClass::updateDisplay);
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
  displayTypeChanged();
  updateDisplay();
}

QImageViewer::DisplayType QImageViewer::displayType() const
{
  return currentDisplayType_;
}

void QImageViewer::setMaskBlendAlpha(double v)
{
  maskBlendAlpha_ = v;
  updateDisplay();
}

double QImageViewer::maskBlendAlpha() const
{
  return maskBlendAlpha_;
}

void QImageViewer::setTransparentMask(bool v)
{
  transparentMask_ = v;
  updateDisplay();
}

bool QImageViewer::transparentMask() const
{
  return transparentMask_;
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

const cv::Mat& QImageViewer::mtfImage() const
{
  return mtfImage_;
}

const cv::Mat & QImageViewer::displayImage() const
{
  return displayImage_;
}

QPixmap QImageViewer::grabViewportPixmap()
{
  return view_->grab();
}

QString QImageViewer::currentFileName() const
{
  return this->currentFileName_;
}

void QImageViewer::setCurrentFileName(const QString & newFileName)
{
  if( currentFileName_ != newFileName ) {
    currentFileName_ = newFileName;
    Q_EMIT currentFileNameChanged();
  }
}

void QImageViewer::setCurrentImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  // c_current_image_lock lock(this);

  editMaskUndoQueue_.clear();

  if( image.empty() ) {
    current_image_lock lock(this);
    currentImage_.release();
    currentMask_.release();
    currentImageData_.release();
  }
  else if( make_copy ) {
    current_image_lock lock(this);
    image.getMat().copyTo(currentImage_);
    mask.getMat().copyTo(currentMask_);
    imageData.getMat().copyTo(currentImageData_);
  }
  else {
    current_image_lock lock(this);
    currentImage_ = image.getMat();
    currentMask_ = mask.getMat();
    currentImageData_ = imageData.getMat();
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

    if( true ) {
      current_image_lock lock(this);

      if( currentImage_.empty() ) {
        displayImage_.release();
      }
      else {

        if( !displayFunction_ ) {
          currentImage_.copyTo(displayImage_);
        }
        else if( currentImage_.channels() == 2 ) { // assume this is optical flow image

          displayFunction_->createDisplayImage(currentImage_,
              transparentMask_ ? cv::noArray() : currentMask_,
              mtfImage_,
              displayImage_,
              currentImage_.depth());
        }
        else {
          // displayImage_.release();

          displayFunction_->createDisplayImage(currentImage_,
              transparentMask_ ? cv::noArray() : currentMask_,
              mtfImage_,
              displayImage_,
              CV_8U);
        }
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

      cv::addWeighted(displayImage_, maskBlendAlpha_,
          mask, 1 - maskBlendAlpha_,
          0,
          displayImage_,
          displayImage_.depth());
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
  if( displayImage_.empty() ) {
    scene()->setImage(QPixmap());
  }
  else {

    QPixmap pixmap =
        createPixmap(displayImage_, true,
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
    cv2qt(displayImage(), &qimage_, true);
    clipboard->setImage(qimage_);
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
    if( adjustRoi(roi, cv::Rect(0, 0, displayImage_.cols, displayImage_.rows), &rc) ) {
      cv2qt(displayImage_(rc), &qimage_, true);
      clipboard->setImage(qimage_);
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
      view_->mapToScene(viewpos);

  const int x =
      pos.x();

  const int y =
      pos.y();

  n += snprintf(buf + n, sizeof(buf) - 1 - n,
      "| x=%g y=%g",
      pos.x(),
      pos.y());

  current_image_lock lock(this);

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

  view_->setCursor(enableEditMask_ ?
      Qt::CursorShape::CrossCursor :
      defaultCursor);
}

void QImageViewer::editMask(QMouseEvent * e)
{
  //c_current_image_lock lock(this);

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

    updateDisplay();
    Q_EMIT currentImageChanged();
  }
}

void QImageViewer::undoEditMask()
{
  //c_current_image_lock lock(this);

  if( !editMaskUndoQueue_.empty() ) {

    cv::Mat mask = editMaskUndoQueue_.pop();
    if( mask.size() == currentImage_.size() ) {
      currentMask_ = mask;
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
  if( e->buttons() == Qt::LeftButton && enableEditMask_ ) {
    editMask(e);
  }

  Q_EMIT onMousePressEvent(e);
}

void QImageViewer::handleMouseMoveEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && enableEditMask_ ) {
    editMask(e);
  }
  Q_EMIT onMouseMove(e);
}


