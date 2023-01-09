/*
 * QRadialPolySharpSettings.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "QRadialPolySharpSettings.h"

QRadialPolySharpSettings::QRadialPolySharpSettings(const c_radial_polysharp_routine::ptr & routine, QWidget * parent) :
    Base(routine, parent)
{
}

void QRadialPolySharpSettings::setupControls()
{
  const c_radial_polysharp_routine::ptr routine =
      std::dynamic_pointer_cast<c_radial_polysharp_routine>(processor_);

  profileView_ =
      add_widget<QRadialPolyProfileView>();

  profileView_->
      set_polysharp_routine(routine);

  coeffs_ctl =
      QSettingsWidget::add_textbox("coeffs:",
          [this, routine](const QString & s) {
            std::vector<double> values;
            if ( fromString(s, &values)) {
              routine->set_coeffs(values);
              emit parameterChanged();
            }
          });

  updateControls();
}

void QRadialPolySharpSettings::onupdatecontrols()
{
  const c_radial_polysharp_routine::ptr routine =
      std::dynamic_pointer_cast<c_radial_polysharp_routine>(processor_);

  if ( !routine ) {
    setEnabled(false);
  }
  else {
    routine->set_postprocess_notify_callback(
        [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {
          profileView_->update();
        });

    coeffs_ctl->setValue(routine->coeffs());
    profileView_->update();
    setEnabled(true);
  }

  Base::onupdatecontrols();
}


static constexpr int MARGIN = 2;

QRadialPolyProfileView::QRadialPolyProfileView(QWidget * parent)
  : Base(parent)
{
  setMinimumSize( 128 + 2 * MARGIN, 128 + 2 * MARGIN);
  resize(256, (int)(256 / 1.62));
}

void QRadialPolyProfileView::set_polysharp_routine(const c_radial_polysharp_routine::ptr & routine)
{
  routine_ = routine;
}

const c_radial_polysharp_routine::ptr & QRadialPolyProfileView::polysharp_routine() const
{
  return routine_;
}

QSize QRadialPolyProfileView::sizeHint() const
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  const int width = std::max(256, this->screen()->geometry().width() / 3);
#else
  const int width = std::max(256, QGuiApplication::primaryScreen()->geometry().width() / 3);
#endif

  return QSize(width, (int) (width / 2));
}

void QRadialPolyProfileView::paintEvent(QPaintEvent *e)
{
  const QColor backgroundColor_ =
      Qt::white;

  const QSize size =
      this->size();

  QPainter p(this);

  p.fillRect(0, 0, size.width(), size.height(),
      backgroundColor_);

  if ( size.width() <= 2 * MARGIN || size.height() <= 2 * MARGIN ) {
    return;  // too small wiindow
  }

  if ( !routine_ ) {
    return;
  }

  static const auto drawBorder =
      [](QPainter & p, const QSize & size)
      {
        p.setPen(Qt::black);
        p.drawRect(0, 0, size.width()-1, size.height()-1);
      };


  static const auto drawMajorGrid =
      [](QPainter & p, const QSize & size)
  {
    constexpr int xparts = 8;
    constexpr int yparts = 8;

    const int w = size.width() - 2 * MARGIN;
    const int h = size.height() - 2 * MARGIN;

    int xpix, ypix;

    p.setPen(Qt::darkGray);
    for ( int i = 1; i < xparts; ++i ) {
      if ( i != xparts / 2 ) {
        xpix = MARGIN + i * w / xparts;
        p.drawLine(xpix, MARGIN, xpix, size.height() - MARGIN - 1);
      }
    }

    for ( int i = 1; i < yparts; ++i ) {
      if ( i != yparts / 2 ) {
        ypix = MARGIN + i * h / yparts;
        p.drawLine(MARGIN, ypix, size.width() - MARGIN - 1, ypix);
      }
    }

    p.setPen(Qt::black);
    xpix = MARGIN + w / 2;
    p.drawLine(xpix, MARGIN, xpix, size.height() - MARGIN - 1);

    ypix = MARGIN + h / 2;
    p.drawLine(MARGIN, ypix, size.width() - MARGIN - 1, ypix);
  };


  static const auto drawLines =
      [](QPainter & p, const QColor & color, const QSize & size,
          const std::vector<double> & profile)
      {
        if ( !profile.empty() ) {

          const double maxval =
              *std::max_element(profile.begin(),
                  profile.end()) * 1.1;

          QPen pen;
          pen.setColor(color);
          pen.setWidth(2);
          p.setPen(pen);

          const int l = MARGIN;
          const int t = MARGIN;
          const int r = size.width() - MARGIN;
          const int b = size.height() - MARGIN;
          const int w = r - l;
          const int h = b - t;

          int xprev = (int)(l);
          int yprev = (int)(b - profile[0] * h / maxval );

          for ( uint i = 1, n = profile.size(); i < n; ++i ) {

            const int x = (int)(l + i * w / n);
            const int y = (int)(b - profile[i] * h / maxval );

            p.drawLine(xprev, yprev, x, y);

            xprev = x;
            yprev = y;
          }
        }
      };

  drawMajorGrid(p, size);
  drawBorder(p, size);
  drawLines(p, Qt::blue, size, routine_->profile_before());
  drawLines(p, Qt::red, size, routine_->profile_after());
  drawLines(p, Qt::darkGray, size, routine_->profile_poly());
}
