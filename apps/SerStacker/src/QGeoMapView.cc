/*
 * QGeoMapView.cc
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#include "QGeoMapView.h"
#include <gui/widgets/style.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

#define ICON_options  ":/gui/icons/options.png"
#define ICON_eye      ":/gui/icons/eye.png"
#define ICON_send     ":/gui/icons/send.png"
#define ICON_delete   ":/gui/icons/delete2.png"

namespace serstacker {

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackItem::QGpxTrackItem(QGraphicsItem *parent) :
    Base(parent)
{
  setEnableAddPoints(false);
  setEnableRemovePoints(false);
  setEnableMovePoints(true);
}

const c_gpx_track & QGpxTrackItem::track() const
{
  return _track;
}

void QGpxTrackItem::setPathFileName(const QString& filename)
{
  _pathFileName = filename;
}

const QString & QGpxTrackItem::pathFileName() const
{
  return _pathFileName;
}

void QGpxTrackItem::setAssociatedVideoFileName(const QString & fname)
{
  _associatedVideoFileName = fname;
}

const QString & QGpxTrackItem::associatedVideoFileName() const
{
  return _associatedVideoFileName;
}

bool QGpxTrackItem::loadTrack(const QString & filename)
{
  if ( !load_gpx_track_xml(filename.toStdString(), &_track) ) {
    return false;
  }

  return true;
}

int QGpxTrackItem::pointsCount() const
{
  return _track.pts.size();
}

void QGpxTrackItem::insertPoint(const QGeoPos &, int insert_pos)
{
  CF_DEBUG("APP BUG: this must be not called");
}

void QGpxTrackItem::removePoint(int remove_pos)
{
  CF_DEBUG("APP BUG: this must be not called");
}

void QGpxTrackItem::setGeoPoint(int index, const QGeoPos & )
{
  CF_DEBUG("APP BUG: this must be not called");
}

QGeoPos QGpxTrackItem::getGeoPoint(int index) const
{
  return QGeoPos(_track.pts[index]);
}

bool QGpxTrackItem::popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  return Base::popuateContextMenu(event, menu);
}

// Use keyboard modifier like CTRL, ALT or SHIFT in order to receive this event,
// because GeoView uses Mouse move event with left button for view dragging
void QGpxTrackItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
  if( event->buttons() == Qt::LeftButton  && !projPoints_.empty() ) {

    const QGraphicsView *view =
        getActiveView(event);

    if ( view ) {

      const int index =
          findPointByViewPos(view, view->mapFromScene(event->scenePos()), 9);

      if ( index >= 0 && index < (int)_track.pts.size() ) {

        const c_gps_position & gps =
            _track.pts[index];

        CF_DEBUG("\ngpx point %d: lat=%+.8f lon=%+.8f alt=%g ts=%.3f relts=%.3f", index,
            gps.latitude * 180 / CV_PI, gps.longitude * 180 / CV_PI, gps.altitude,
            gps.timestamp, gps.timestamp - _track.pts[0].timestamp);

      }

    }

  }

  Base::mousePressEvent(event);

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackSelectorWidget::QGpxTrackSelectorWidget(QWidget * parent) :
    Base(parent)
{
  _hbox = new QHBoxLayout(this);
  _hbox->setContentsMargins(0, 0, 0, 0);

  combobox_ctl = new QComboBox(this);
  combobox_ctl->setEditable(false);
  combobox_ctl->setToolTip("Select track for edit its view settings");

  trackVisibilty_ctl =
      createCheckableToolButton(getIcon(ICON_eye), "Show",
          "Show / Hide selected track",
          false,
          this,
          &ThisClass::toggleTrackVisibilityClicked);

  trackSend_ctl =
      createToolButton(getIcon(ICON_send), "FlyTo",
          "Show selected track on map",
          this,
          &ThisClass::showSelectedTrackOnMapClicked);

  trackDelete_ctl =
      createToolButton(getIcon(ICON_delete), "Delete",
          "Delete selected track",
          this, &ThisClass::deleteSelectedTrackClicked);

  _hbox->addWidget(combobox_ctl, 1000);
  _hbox->addWidget(trackVisibilty_ctl);
  _hbox->addWidget(trackSend_ctl);
  _hbox->addWidget(trackDelete_ctl);

}

QComboBox * QGpxTrackSelectorWidget::combo() const
{
  return combobox_ctl;
}

QToolButton * QGpxTrackSelectorWidget::trackVisibiltyControl() const
{
  return trackVisibilty_ctl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackViewSettings::QGpxTrackViewSettings(QWidget * parent) :
    Base(parent)
{
  gpxTrackSelector_ctl =
      add_widget<QGpxTrackSelectorWidget>("Select track:");


  showLines_ctl =
      add_checkbox("Show lines",
          "Show /Hide lines",
          [this](bool checked) {
            if ( options_ && options_->showLines() != checked ) {
              options_->setShowLines(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->showLines();
              return true;
            }
            return false;
          });


  lineWidth_ctl =
      add_spinbox("Line width", "Set GPS line width",
          [this](int value) {
            if ( options_ ) {
              options_->setLineWidth(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->lineWidth();
              return true;
            }
            return false;
          });

  lineWidth_ctl->setRange(0, 32);

  lineColor_ctl =
      add_color_picker_button("Line Color",
          "Select color for GPS lines",
          [this](const QColor & value) {
            if ( options_ ) {
              options_->setLineColor(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](QColor * value) {
            if ( options_ ) {
              * value = options_->lineColor();
              return true;
            }
            return false;
          });

  lineOpaqueness_ctl =
      add_sliderspinbox<int>("Line opaqueness",
          "Set line opaqueness",
          [this](int value) {
            if ( options_ && options_->lineOpaqueness() != value ) {
              options_->setLineOpaqueness(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->lineOpaqueness();
              return true;
            }
            return false;
          });

  lineOpaqueness_ctl->setRange(0, 255);


  showPoints_ctl =
      add_checkbox("Show points",
          "Show /Hide points",
          [this](bool checked) {
            if ( options_ && options_->showPoints() != checked ) {
              options_->setShowPoints(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->showPoints();
              return true;
            }
            return false;
          });

  pointSize_ctl =
      add_spinbox("Point size",
          "Set point radius",
          [this](int value) {
            if ( options_ && options_->pointSize() != value ) {
              options_->setPointSize(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->pointSize();
              return true;
            }
            return false;
          });

  pointSize_ctl->setRange(0, 32);

  pointPenWidth_ctl =
      add_spinbox("Point pen width", "Set pen width used to draw points",
          [this](int value) {
            if ( options_ && options_->pointPenWidth() != value ) {
              options_->setPointPenWidth(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->pointPenWidth();
              return true;
            }
            return false;
          });

  pointPenWidth_ctl->setRange(0, 32);

  pointColor_ctl =
      add_color_picker_button("Point Color",
          "Select color for GPS points",
          [this](const QColor & value) {
            if ( options_ ) {
              options_->setPointColor(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](QColor * value) {
            if ( options_ ) {
              * value = options_->pointColor();
              return true;
            }
            return false;
          });

  pointOpaqueness_ctl =
      add_sliderspinbox<int>("Point opaqueness", "Set point opaqueness",
          [this](int value) {
            if ( options_ && options_->pointOpaqueness() != value ) {
              options_->setPointOpaqueness(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->pointOpaqueness();
              return true;
            }
            return false;
          });

  pointOpaqueness_ctl->setRange(0, 255);

  associatedVideoFileName_ctl =
      add_browse_for_path("", "Associated video:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & value) {
            if ( options_ ) {
              options_ ->setAssociatedVideoFileName(value);
            }
          },
          [this](QString * v) {
            if ( options_ ) {
              *v = options_->associatedVideoFileName();
              return true;
            }
            return false;
          });


  connect(gpxTrackSelector_ctl->combo(),
      static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onGpxTrackSelected);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::toggleTrackVisibilityClicked,
      this, &ThisClass::onToggleTrackVisibilityClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::showSelectedTrackOnMapClicked,
      this, &ThisClass::onShowSelectedTrackOnMapClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::deleteSelectedTrackClicked,
      this, &ThisClass::onDeleteSelectedTrackClicked);


  updateControls();
}

QGpxTrackItem * QGpxTrackViewSettings::selectedTrack() const
{
  QGpxTrackItem * item = nullptr;

  if ( gpxTracks ) {

    const int cursel =
        gpxTrackSelector_ctl->combo()->currentIndex();

    if ( cursel >= 0 && cursel < (int)gpxTracks->size() ) {
      item = (*gpxTracks)[cursel];
    }
  }

  return item;
}

void QGpxTrackViewSettings::setGpxTracks(std::vector<QGpxTrackItem*> * gpxTracks)
{
  this->gpxTracks = gpxTracks;
  populateTrackSelectionCombo();
  updateControls();
}



void QGpxTrackViewSettings::onupdatecontrols()
{
  if ( !gpxTracks ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
    Base::onupdatecontrols();
  }
}

void QGpxTrackViewSettings::populateTrackSelectionCombo()
{
  c_update_controls_lock lock(this);

  QComboBox * combo =
      gpxTrackSelector_ctl->combo();

  combo->clear();

  if ( gpxTracks ) {
    for( const QGpxTrackItem * track : *gpxTracks ) {
      combo->addItem(track->name());
    }
  }
}

void QGpxTrackViewSettings::onGpxTrackSelected(int index)
{
  if( !gpxTracks || index < 0 || index >= (int) gpxTracks->size() ) {
    set_options(nullptr);
  }
  else {
    QGpxTrackItem * item = (*gpxTracks)[index];
    set_options(item);
    gpxTrackSelector_ctl->trackVisibiltyControl()->setChecked(item->isVisible());
  }
}

void QGpxTrackViewSettings::onToggleTrackVisibilityClicked(bool visible)
{
  QGpxTrackItem * item =
      selectedTrack();

  if( item ) {
    item->setVisible(visible);
  }

}

void QGpxTrackViewSettings::onShowSelectedTrackOnMapClicked()
{
  QGpxTrackItem * item =
      selectedTrack();

  if( item ) {

    const c_gpx_track & track =
        item->track();

    if ( !track.pts.empty() ) {

      const c_gps_position & gps =
          track.pts[track.pts.size()/2];

      Q_EMIT showPositionOnMap(gps.latitude, gps.longitude);
    }
  }
}

void QGpxTrackViewSettings::onDeleteSelectedTrackClicked()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackViewSettingsDialogBox::QGpxTrackViewSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("GPX track options");

  connect(settings_ctl, &QGpxTrackViewSettings::showPositionOnMap,
      this, &ThisClass::showPositionOnMap);

}

void QGpxTrackViewSettingsDialogBox::setGpxTracks(std::vector<QGpxTrackItem*> * gpxTracks)
{
  settings_ctl->setGpxTracks(gpxTracks);
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QGeoMapView::QGeoMapView(QWidget * parent) :
    Base(parent)
{
  // geoview_->setMouseTracking(true);
}


QGpxTrackItem * QGeoMapView::loadGpxTrack(const QString & filename)
{
  QGpxTrackItem * item =
      new QGpxTrackItem();

  if ( !item->loadTrack(filename) ) {
    delete item;
    return nullptr;
  }

  item->setZValue(1000);
  item->setVisible(true);
  item->setName(QFileInfo(filename).fileName());
  item->setDescription(item->track().name.c_str());
  item->setPathFileName(filename);

  return item;
}


bool QGeoMapView::addGpxTrack(const QString & filename)
{
  QGpxTrackItem * item =
      loadGpxTrack(filename);

  if ( !item ) {
    return false;
  }

  scene_->addItem(item);
  gpxTrackItems.emplace_back(item);

  if ( viewSettingsDialogBox ) {
    viewSettingsDialogBox->settingsWidget()->setGpxTracks(&gpxTrackItems);
  }

  return true;
}


void QGeoMapView::createToolbarActions()
{
  toolbarActions_.append(toggleOptionsDialogBoxAction =
      createCheckableAction(getIcon(ICON_options), "Options",
          "Show/Hide Options dialog box",
          viewSettingsDialogBox && viewSettingsDialogBox->isVisible(),
          this,
          &ThisClass::onToggleOptionsDialogBox));

  Base::createToolbarActions();
}

void QGeoMapView::onToggleOptionsDialogBox(bool checked)
{
  if ( !checked ) {

    if ( viewSettingsDialogBox && viewSettingsDialogBox->isVisible() ) {
      viewSettingsDialogBox->hide();
    }

  }
  else {

    if ( !viewSettingsDialogBox ) {

      viewSettingsDialogBox = new QGpxTrackViewSettingsDialogBox(this);
      viewSettingsDialogBox->settingsWidget()->setGpxTracks(&gpxTrackItems);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::visibilityChanged,
          toggleOptionsDialogBoxAction, &QAction::setChecked);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::showPositionOnMap,
        this, &ThisClass::flyToPosition);

    }


    viewSettingsDialogBox->show();
  }
}

void QGeoMapView::flyToPosition(double latitude, double longitude)
{
  Base::flyTo(QGeoPos(latitude * 180 / CV_PI, longitude * 180 / CV_PI));
}

void QGeoMapView::loadSettings()
{
  QSettings settings;
  loadSettings(settings);
}

void QGeoMapView::saveSettings()
{
  QSettings settings;
  saveSettings(settings);
}


void QGeoMapView::loadSettings(QSettings & settings)
{

  const int num_tracks =
      std::min(64, settings.value("QGeoMapView/num_tracks", 0).value<int>());

  for ( int i = 0; i < num_tracks; ++i ) {

    const QString prefix =
        qsprintf("QGeoMapView/track%d_", i);

    const QString pathFileName =
        settings.value(QString("%1_pathFileName").arg(prefix), "").toString();

    if ( !pathFileName.isEmpty() ) {

      QGpxTrackItem * trackItem =
          loadGpxTrack(pathFileName);

      trackItem->setAssociatedVideoFileName(settings.value(QString("%1_videoFileName").arg(prefix)).toString());
      trackItem->setVisible(settings.value(QString("%1_isVisible").arg(prefix)).toBool());
      trackItem->setShowLines(settings.value(QString("%1_showLines").arg(prefix)).toBool());
      trackItem->setLineWidth(settings.value(QString("%1_lineWidth").arg(prefix)).toInt());
      trackItem->setLineColor(settings.value(QString("%1_lineColor").arg(prefix)).value<QColor>());
      trackItem->setLineOpaqueness(settings.value(QString("%1_lineOpaqueness").arg(prefix)).toInt());
      trackItem->setShowPoints(settings.value(QString("%1_showPoints").arg(prefix)).toBool());
      trackItem->setPointSize(settings.value(QString("%1_pointSize").arg(prefix)).toInt());
      trackItem->setPointPenWidth(settings.value(QString("%1_pointPenWidth").arg(prefix)).toInt());
      trackItem->setPointColor(settings.value(QString("%1_pointColor").arg(prefix)).value<QColor>());
      trackItem->setPointOpaqueness(settings.value(QString("%1_pointOpaqueness").arg(prefix)).toInt());

      gpxTrackItems.emplace_back(trackItem);
      scene_->addItem(trackItem);
    }
  }

  if ( viewSettingsDialogBox ) {
    viewSettingsDialogBox->settingsWidget()->setGpxTracks(&gpxTrackItems);
  }

}

void QGeoMapView::saveSettings(QSettings & settings)
{
  const int num_tracks =
      (int)gpxTrackItems.size();

  settings.setValue("QGeoMapView/num_tracks", num_tracks);

  for ( int i = 0; i < num_tracks; ++i ) {

    const QGpxTrackItem * trackItem =
        gpxTrackItems[i];

    const QString prefix =
        qsprintf("QGeoMapView/track%d_", i);

    settings.setValue(QString("%1_pathFileName").arg(prefix), trackItem->pathFileName());
    settings.setValue(QString("%1_videoFileName").arg(prefix), trackItem->associatedVideoFileName());
    settings.setValue(QString("%1_isVisible").arg(prefix), trackItem->isVisible());
    settings.setValue(QString("%1_showLines").arg(prefix), trackItem->showLines());
    settings.setValue(QString("%1_lineWidth").arg(prefix), trackItem->lineWidth());
    settings.setValue(QString("%1_lineColor").arg(prefix), trackItem->lineColor());
    settings.setValue(QString("%1_lineOpaqueness").arg(prefix), trackItem->lineOpaqueness());
    settings.setValue(QString("%1_showPoints").arg(prefix), trackItem->showPoints());
    settings.setValue(QString("%1_pointSize").arg(prefix), trackItem->pointSize());
    settings.setValue(QString("%1_pointPenWidth").arg(prefix), trackItem->pointPenWidth());
    settings.setValue(QString("%1_pointColor").arg(prefix), trackItem->pointColor());
    settings.setValue(QString("%1_pointOpaqueness").arg(prefix), trackItem->pointOpaqueness());
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////


QGeoMapViewDock::QGeoMapViewDock(const QString &title, QWidget * parent) :
    Base(title, parent)
{
  Base::setWidget(_geoView = new QGeoMapView(this));

  const QList<QAction*> actions = _geoView->toolbarActions();
  for( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }
}

QGeoMapView * QGeoMapViewDock::geoView() const
{
  return _geoView;
}



QGeoMapViewDock * addGeoMapViewDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QGeoMapViewDock *dock = new QGeoMapViewDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serstacker */
