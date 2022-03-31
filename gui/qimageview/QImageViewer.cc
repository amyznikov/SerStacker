/*
 * QImageViewer.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageViewer.h"
#include <gui/widgets/QWaitCursor.h>
#include "cv2qt.h"
#include <core/debug.h>

QImageViewer::QImageViewer(QWidget * parent)
  : Base(parent)
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0,0,0,0);
  layout_->addWidget(view_ = new QImageSceneView(this), 100);

  connect(view_, &QImageSceneView::onMouseMove,
      this, &ThisClass::onMouseMove);
  connect(view_, &QImageSceneView::onMousePressEvent,
      this, &ThisClass::onMousePressEvent);
  connect(view_, &QImageSceneView::onMouseReleaseEvent,
      this, &ThisClass::onMouseReleaseEvent);
  connect(view_, &QImageSceneView::onMouseDoubleClick,
      this, &ThisClass::onMouseDoubleClick);
  connect(view_, &QImageSceneView::scaleChanged,
      this, &ThisClass::onScaleChanged);
  connect(view_, &QImageSceneView::onLineShapeChanged,
      this, &ThisClass::onLineShapeChanged);
  connect(view_, &QImageSceneView::onRectShapeChanged,
      this, &ThisClass::onRectShapeChanged);
}

QImageSceneView * QImageViewer::sceneView() const
{
  return view_;
}

QToolBar * QImageViewer::embedToolbar(QToolBar * toolbar)
{
  if ( this->toolbar_ != Q_NULLPTR ) {
    if ( toolbar == Q_NULLPTR ) {
      return this->toolbar_; // already embedded
    }
    layout_->removeWidget(this->toolbar_);
  }


  if ( (this->toolbar_ = toolbar) == Q_NULLPTR ) {
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


void QImageViewer::setDisplayFunction(QImageDisplayFunction * displayFunction)
{
  if( displayFunction_ ) {
    QObject::disconnect(displayFunction_, &QImageDisplayFunction::update,
        this, &ThisClass::updateDisplay);
  }

  displayFunction_ = displayFunction;

  if( displayFunction_ ) {
    QObject::connect(displayFunction_, &QImageDisplayFunction::update,
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
//  if ( input_sequence_->is_open() ) {
//
//    c_input_source::ptr source =
//        input_sequence_->current_source();
//
//    if ( source ) {
//      return source->filename().c_str();
//    }
//  }

  return this->currentFileName_;
}

void QImageViewer::setCurrentFileName(const QString & newFileName)
{
  this->currentFileName_ = newFileName;
  emit currentImageFileNameChanged();
}

void QImageViewer::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData, bool make_copy)
{
  if ( image.empty() ) {
    currentImage_.release();
    currentMask_.release();
    currentImageData_.release();
    qimage_ = QImage();
  }
  else {

    if ( make_copy ) {
      image.getMat().copyTo(currentImage_);
      mask.getMat().copyTo(currentMask_);
      imageData.getMat().copyTo(currentImageData_);
    }
    else {
      currentImage_ = image.getMat();
      currentMask_ = mask.getMat();
      currentImageData_ = imageData.getMat();
    }
  }

  updateDisplay();
}

void QImageViewer::updateDisplay()
{
  createDisplayImage();
  showCurrentDisplayImage();
}

void QImageViewer::createDisplayImage()
{
  if ( currentImage_.empty() ) {
    displayImage_.release();
  }
  else if ( !displayFunction_ ) {
    currentImage_.copyTo(displayImage_);
  }
  else {

    const int ddepth =
        currentImage_.channels() == 2 ?
            currentImage_.depth() : // asumme this is optical flow image
            CV_8U; // create regular BGR 8bit image

    displayImage_.release();
    displayFunction_->createDisplayImage(currentImage_, currentMask_,
        displayImage_, ddepth);
  }

  emit currentDisplayImageChanged();
}

void QImageViewer::showCurrentDisplayImage()
{
  if( displayImage_.empty() ) {
    view_->scene()->setBackground(QImage());
  }
  else {
    cv2qt(displayImage_, &qimage_);
    view_->scene()->setBackground(qimage_);
  }
}

void QImageViewer::copyDisplayImageToClipboard()
{
  QClipboard * clipboard = QApplication::clipboard();
  if ( !clipboard ) {
    QMessageBox::critical(this, "ERROR",
        "No application clipboard available");
  }
  else {
    QWaitCursor wait(this);

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
  //emit onShowEvent(e);
  emit visibilityChanged(true);
  Base::showEvent(e);
}

void QImageViewer::hideEvent(QHideEvent *e)
{
  //emit onHideEvent(e);
  Base::hideEvent(e);
  emit visibilityChanged(false);
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

