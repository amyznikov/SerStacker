/*
 * QImageViewer.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageViewer.h"
#include "cv2qt.h"
#include <core/debug.h>

QImageViewer::QImageViewer(QWidget * parent)
  : Base(parent)
{
  vbox_ = new QVBoxLayout(this);
  vbox_->setContentsMargins(0,0,0,0);
  vbox_->addWidget(view_ = new QImageSceneView(this), 100);

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

}

QToolBar * QImageViewer::embedToolbar(QToolBar * toolbar)
{
  if ( this->toolbar_ != Q_NULLPTR ) {
    if ( toolbar == Q_NULLPTR ) {
      return this->toolbar_; // already embedded
    }
    vbox_->removeWidget(this->toolbar_);
  }


  if ( (this->toolbar_ = toolbar) == Q_NULLPTR ) {
    this->toolbar_ = new QToolBar(this);
    this->toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    this->toolbar_->setOrientation(Qt::Horizontal);
    this->toolbar_->setIconSize(QSize(16,16));
  }

  vbox_->insertWidget(0, this->toolbar_, 1);

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
    vbox_->removeWidget(this->statusbar_);
  }

  if ( (this->statusbar_ = statusBar) == Q_NULLPTR ) {
    this->statusbar_ = new QStatusBar(this);
  }

  vbox_->addWidget(this->statusbar_, 0);

  return this->statusbar_;
}

QStatusBar * QImageViewer::statusbar() const
{
  return statusbar_;
}


void QImageViewer::setDisplayFunction(const DisplayFunction & func)
{
  this->displayFunction_ = func;
}

const QImageViewer::DisplayFunction & QImageViewer::displayFunction() const
{
  return this->displayFunction_;
}


const cv::Mat & QImageViewer::image() const
{
  return currentImage_;
}

const cv::Mat & QImageViewer::imageData() const
{
  return currentImageData_;
}

void QImageViewer::setImage(cv::InputArray image, cv::InputArray imageData, bool make_copy)
{
  if ( image.empty() ) {
    currentImage_.release();
    currentImageData_.release();
    qimage_ = QImage();
  }
  else {

    if ( make_copy ) {
      image.getMat().copyTo(currentImage_);
    }
    else {
      currentImage_ = image.getMat();
    }

    if ( imageData.empty() ) {
      currentImageData_.release();
    }
    else {
      if ( make_copy ) {
        imageData.getMat().copyTo(currentImageData_);
      }
      else {
        currentImageData_ = imageData.getMat();
      }
    }
  }

  updateDisplay();
}

void QImageViewer::updateDisplay()
{
  if ( currentImage_.empty() ) {
    view_->scene()->setBackground(QImage());
  }
  else {
    cv::Mat tmp;
    if ( displayFunction_ ) {
      displayFunction_(currentImage_, tmp, CV_8U);
    }
    else {
      tmp = currentImage_;
    }

    cv2qt(tmp, &qimage_);
    view_->scene()->setBackground(qimage_);

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
  emit onShowEvent(e);
  Base::showEvent(e);
}

void QImageViewer::hideEvent(QHideEvent *e)
{
  emit onHideEvent(e);
  Base::hideEvent(e);
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


void QImageViewer::createSelectionRect(const QRectF & rc)
{
  if ( !selectionRect_ ) {
    QPen pen(Qt::yellow);
    pen.setWidth(1);
    selectionRect_ =  view_->scene()->addRect(rc, pen);

    QGraphicsItem::GraphicsItemFlags flags = selectionRect_->flags();
    flags |= QGraphicsItem::ItemIsMovable;
    flags &= ~(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    selectionRect_->setFlags(flags);

    selectionRectToCenterOfView();
  }
}

void QImageViewer::setSelectionRectVisible(bool v)
{
  if ( v && !selectionRect_ ) {
    createSelectionRect(QRectF(0, 0, 11, 11));
  }
  if ( selectionRect_ ) {
    selectionRect_->setVisible(v);
  }
}

bool QImageViewer::selectionRectIsVisible() const
{
  return selectionRect_ && selectionRect_->isVisible();
}


void QImageViewer::setSelectionRect(const QRectF & rc)
{
  if ( !selectionRect_ ) {
    createSelectionRect(rc);
  }
  selectionRect_->setPos(selectionRect_->mapFromScene(rc.topLeft()));
}

QRectF QImageViewer::selectionRect() const
{
  if ( !selectionRect_ || !selectionRect_->isVisible() ) {
    return QRectF(-1, -1, -1, -1);
  }

  QPointF tl = selectionRect_->mapToScene(selectionRect_->rect().topLeft());
  QPointF br = selectionRect_->mapToScene(selectionRect_->rect().bottomRight());

  return  QRectF(tl, br);
}

void QImageViewer::selectionRectToCenterOfView()
{
  if ( !selectionRect_ ) {
    createSelectionRect(QRectF(0, 0, 11, 11));
  }
  else {
    QPointF viewCenterOnScene = view_->mapToScene(view_->rect().center());
    selectionRect_->setPos(viewCenterOnScene.x() - selectionRect_->rect().width() / 2,
        viewCenterOnScene.y() - selectionRect_->rect().height() / 2);
  }

  selectionRect_->setVisible(true);
}
