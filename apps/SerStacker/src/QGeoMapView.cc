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

#define ICON_options              ":/gui/icons/options.png"
#define ICON_eye                  ":/gui/icons/eye.png"
#define ICON_send                 ":/gui/icons/send.png"
#define ICON_delete               ":/gui/icons/delete2.png"
#define ICON_video                ":/gui/icons/video.png"
#define ICON_geo_landmark         ":/gui/icons/geo-landmark.png"
//":/gui/icons/show-geo-landmarks.png"
#define ICON_show_geo_landmarks   ICON_geo_landmark

#define ICON_car000               ":/serstacker/icons/car/car000.png"
#define ICON_car045               ":/serstacker/icons/car/car045.png"
#define ICON_car090               ":/serstacker/icons/car/car090.png"
#define ICON_car135               ":/serstacker/icons/car/car135.png"
#define ICON_car180               ":/serstacker/icons/car/car180.png"
#define ICON_car225               ":/serstacker/icons/car/car225.png"
#define ICON_car270               ":/serstacker/icons/car/car270.png"
#define ICON_car315               ":/serstacker/icons/car/car315.png"


#define TRACK_ZVALUE              1000
#define LANDMARK_ZVALUE           1001
#define CAR_ZVALUE                1002

namespace serstacker {

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxLandmarkItem::QGpxLandmarkItem(int gpxPointIndex, const QGeoPos & geoPos, QGraphicsItem * parent) :
    Base(geoPos, getPixmap(ICON_geo_landmark), QPoint(-1, -1), parent),
    _gpxPointIndex(gpxPointIndex)
{
  setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
  setFlag(QGraphicsItem::ItemIsMovable, false);
  setFlag(QGraphicsItem::ItemIsSelectable, false);
}

int QGpxLandmarkItem::gpxPointIndex() const
{
  return _gpxPointIndex;
}

void QGpxLandmarkItem::setAssociatedVideoFrameIndex(int v)
{
  _associatedVideoFrameIndex = v;
}

int QGpxLandmarkItem::associatedVideoFrameIndex() const
{
  return _associatedVideoFrameIndex;
}

bool QGpxLandmarkItem::populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  // CF_DEBUG("MENU");

  menu.addAction("Associate with video frame",
      [this]() {

        bool fOk = false;

        const int frameIndex = QInputDialog::getInt(QApplication::activeWindow(),
            "Associate with video frame",
            "Specify Target Video Frame Index to associate:",
            _associatedVideoFrameIndex,
            INT32_MIN, INT32_MAX, 1,
            &fOk,
            Qt::WindowFlags());

        if ( fOk && frameIndex != _associatedVideoFrameIndex ) {

          // remove_landmark

          setAssociatedVideoFrameIndex(frameIndex);
        }

      });

  menu.addAction("Open associated video frame",
      [this]() {
        Q_EMIT openAssociatedVideoFrameRequested(this);
      });

  menu.addSeparator();
  menu.addAction("Delete KeyPoint",
      [this]() {
        Q_EMIT deleteRequested(this);
      });

  return Base::populateContextMenu(event, menu) || true;
}

void QGpxLandmarkItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
  event->ignore();
  Q_EMIT openAssociatedVideoFrameRequested(this);
  // Base::mouseDoubleClickEvent(event);
}

void QGpxLandmarkItem::onGeoPosChanged(const QGeoPos & pos)
{
 //  CF_DEBUG("pos: lat=%+.8f lon=%+.8f", pos.latitude(), pos.longitude());
  Base::onGeoPosChanged(pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxCarItem::QGpxCarItem(QGraphicsItem *parent) :
    Base(parent)
{
  setZValue(CAR_ZVALUE);
  setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
  setFlag(QGraphicsItem::ItemIsMovable, false);
  setFlag(QGraphicsItem::ItemIsSelectable, false);
}

QGpxCarItem::QGpxCarItem(const QGeoPos & geopos, QGraphicsItem *parent) :
    Base(geopos, getPixmap(ICON_car000), QPoint(-1, -1), parent)
{
  setZValue(CAR_ZVALUE);
  setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
  setFlag(QGraphicsItem::ItemIsMovable, false);
  setFlag(QGraphicsItem::ItemIsSelectable, false);
}


bool QGpxCarItem::populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  return Base::populateContextMenu(event, menu);
}

void QGpxCarItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
  event->ignore();
}

void QGpxCarItem::onGeoPosChanged(const QGeoPos & pos)
{
  // CF_DEBUG("H");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackItem::QGpxTrackItem(QGeoMapView * geoMmapView, QGraphicsItem * parent) :
    Base(parent),
    _geoMmapView(geoMmapView)
{
  _gpx_interpolation.set_gpx_track(&_track);

  setFlag(QGraphicsItem::ItemIgnoresTransformations, false);
  setFlag(QGraphicsItem::ItemIsMovable, false);
  setFlag(QGraphicsItem::ItemIsSelectable, false);
  setEnableAddPoints(false);
  setEnableRemovePoints(false);
  setEnableMovePoints(false);
}

const c_gpx_track& QGpxTrackItem::track() const
{
  return _track;
}

void QGpxTrackItem::setPathFileName(const QString & filename)
{
  _pathFileName = filename;
}

const QString& QGpxTrackItem::pathFileName() const
{
  return _pathFileName;
}

void QGpxTrackItem::setAssociatedVideoFileName(const QString & fname)
{
  _associatedVideoFileName = fname;
}

const QString& QGpxTrackItem::associatedVideoFileName() const
{
  return _associatedVideoFileName;
}

const std::vector<QGpxLandmarkItem*> QGpxTrackItem::gpxLandmarks() const
{
  return _gpxLandmarks;
}

const QGpxLandmarkItem* QGpxTrackItem::gpxLandmarks(int index) const
{
  return _gpxLandmarks[index];
}

bool QGpxTrackItem::loadTrack(const QString & filename)
{
  if( !load_gpx_track_xml(filename.toStdString(), &_track) ) {
    return false;
  }

  return true;
}

//void QGpxTrackItem::addGpxKeypoint(QGpxKeypointItem * item)
//{
//  gpxKeypoints.emplace_back(item);
//}

int QGpxTrackItem::pointsCount() const
{
  return _track.pts.size();
}

void QGpxTrackItem::insertPoint(const QGeoPos&, int insert_pos)
{
  CF_DEBUG("APP BUG: this must be not called");
}

void QGpxTrackItem::removePoint(int remove_pos)
{
  CF_DEBUG("APP BUG: this must be not called");
}

void QGpxTrackItem::setGeoPoint(int index, const QGeoPos&)
{
  CF_DEBUG("APP BUG: this must be not called");
}

QGeoPos QGpxTrackItem::getGeoPoint(int index) const
{
  return QGeoPos(_track.pts[index]);
}

void QGpxTrackItem::setLandmarksVisible(bool v)
{
  _landmarksVisible = v;
  for ( auto landmark : _gpxLandmarks ) {
    landmark->setVisible(v);
  }
}

bool QGpxTrackItem::landmarksVisible() const
{
  return _landmarksVisible;
}

void QGpxTrackItem::setCarVisible(bool v)
{
  if ( !_carItem ) {
    scene()->addItem(_carItem = new QGpxCarItem(_track.pts[0]));
  }

  _carItem->setVisible(v);
}

bool QGpxTrackItem::carVisible() const
{
  return _carItem && _carItem->isVisible();
}

void QGpxTrackItem::computeCarPositon(int scrollpos)
{
  if ( carVisible() ) {

    c_gps_position gps;

    if ( !_gpx_interpolation.interpolate_for_frame(scrollpos, &gps) ) {
      CF_ERROR("interpolate_for_frame() fails");
    }
    else {
      _carItem->setGeoPos(gps);
    }
  }

}

QGpxLandmarkItem * QGpxTrackItem::findGpxLandmarkItem(int gpxPointIndex)
{
  for( QGpxLandmarkItem * item : _gpxLandmarks ) {
    if ( item->gpxPointIndex() == gpxPointIndex ) {
      return item;
    }
  }
  return nullptr;
}

void QGpxTrackItem::addGpxLandmarkItem(int gpxPointIndex, int associatedVideoFrameIndex, const QGeoPos & geopos)
{
  QGpxLandmarkItem * item = new QGpxLandmarkItem(gpxPointIndex, geopos);
  item->setVisible(_landmarksVisible);
  item->setZValue(LANDMARK_ZVALUE);
  item->setAssociatedVideoFrameIndex(associatedVideoFrameIndex);
  scene()->addItem(item);
  _gpxLandmarks.emplace_back(item);

  _gpx_interpolation.set_landmark(associatedVideoFrameIndex,
      gpxPointIndex);

  QObject::connect(item, &QGpxLandmarkItem::openAssociatedVideoFrameRequested,
      [this](QGpxLandmarkItem * item) {
        Q_EMIT openAssociatedVideoFrameRequested(this, item);
      });

  QObject::connect(item, &QGpxLandmarkItem::deleteRequested,
      [this](QGpxLandmarkItem * item) {

        _gpx_interpolation.remove_landmark(item->associatedVideoFrameIndex());

        scene()->removeItem(item);

        const auto pos =
            std::find(_gpxLandmarks.begin(), _gpxLandmarks.end(), item);

        if ( pos != _gpxLandmarks.end() ) {

          _gpxLandmarks.erase(pos);
        }

        delete item;
      });
}

bool QGpxTrackItem::populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  bool populated = false;

  QGraphicsView * view =
      getActiveView(event);

  const QGeoProjection * projection =
      this->projection();

  if( view ) {

    const QPointF scenePos =
        event->scenePos();

    const QPointF viewPos =
        view->mapFromScene(scenePos);

    const int gpxPointIndex =
        findPointByViewPos(view, viewPos, 25);

    if( gpxPointIndex >= 0 && gpxPointIndex < (int) (_track.pts.size()) ) {

      QGpxLandmarkItem * existingLandmark =
          findGpxLandmarkItem(gpxPointIndex);

      if ( !existingLandmark ) {

        menu.addAction("Add GPX Landmark...",
            [this, gpxPointIndex]() {

              bool fOk = false;

              const int frameIndex =
                  QInputDialog::getInt(QApplication::activeWindow(),
                  "Add keypoint",
                  "Specify Target Video Frame Index to associate:",
                  _geoMmapView->currentVideoScrollpos(),
                  INT32_MIN, INT32_MAX, 1,
                  &fOk,
                  Qt::WindowFlags());

              if ( fOk ) {
                addGpxLandmarkItem(gpxPointIndex, frameIndex, _track.pts[gpxPointIndex]);
              }
        });

        populated = true;
      }
    }


  }

  return Base::populateContextMenu(event, menu) || populated;
}

// Use keyboard modifier like CTRL, ALT or SHIFT in order to receive this event,
// because GeoView uses Mouse move event with left button for view dragging
void QGpxTrackItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
  if( event->buttons() == Qt::LeftButton && !projPoints_.empty() ) {

    const QGraphicsView * view =
        getActiveView(event);

    if( view ) {

      const int index =
          findPointByViewPos(view, view->mapFromScene(event->scenePos()), 9);

      if( index >= 0 && index < (int) _track.pts.size() ) {

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

  _hbox->addWidget(combobox_ctl, 1000);

  _hbox->addWidget(track_visibilty_ctl =
      createCheckableToolButton(getIcon(ICON_eye), "Show Track",
          "Show / Hide selected track",
          false,
          this,
          &ThisClass::toggleTrackVisibilityClicked));

  _hbox->addWidget(landmarks_visibilty_ctl =
    createCheckableToolButton(getIcon(ICON_show_geo_landmarks), "Show Landmarks",
        "Show / Hide landmarks associated with selected track",
        false,
        this,
        &ThisClass::toggleLandmarksVisibilityClicked));

  _hbox->addWidget(car_visibilty_ctl =
    createCheckableToolButton(getIcon(ICON_car000), "Show Car",
        "Show / Hide car associated with selected track",
        false,
        this,
        &ThisClass::toggleCarVisibilityClicked));

  _hbox->addWidget(show_track_on_geomap_ctl =
      createToolButton(getIcon(ICON_send), "FlyTo",
          "Show selected track on map",
          this,
          &ThisClass::showSelectedTrackOnMapClicked));


  _hbox->addWidget(open_video_ctl =
      createToolButton(getIcon(ICON_video), "Video",
          "Open Associated video file",
          this,
          &ThisClass::openAssociatedVideoFileClicked));

  _hbox->addWidget(delete_track_ctl =
      createToolButton(getIcon(ICON_delete), "Delete",
          "Delete selected track",
          this, &ThisClass::deleteSelectedTrackClicked));


}

QComboBox* QGpxTrackSelectorWidget::combo() const
{
  return combobox_ctl;
}

QToolButton* QGpxTrackSelectorWidget::trackVisibiltyControl() const
{
  return track_visibilty_ctl;
}

QToolButton * QGpxTrackSelectorWidget::landmarksVisibiltyControl() const
{
  return landmarks_visibilty_ctl;
}

QToolButton * QGpxTrackSelectorWidget::carVisibiltyControl() const
{
  return car_visibilty_ctl;
}

QToolButton* QGpxTrackSelectorWidget::openVideoControl() const
{
  return open_video_ctl;
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
              update_control_states();
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

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::toggleLandmarksVisibilityClicked,
      this, &ThisClass::onToggleLandmarksVisibilityClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::toggleCarVisibilityClicked,
      this, &ThisClass::onToggleCarVisibilityClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::showSelectedTrackOnMapClicked,
      this, &ThisClass::onShowSelectedTrackOnMapClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::openAssociatedVideoFileClicked,
      this, &ThisClass::openSelectedAssociatedVideoFileClicked);

  connect(gpxTrackSelector_ctl, &QGpxTrackSelectorWidget::deleteSelectedTrackClicked,
      this, &ThisClass::deleteSelectedTrackClicked);

  updateControls();
}

QGpxTrackItem* QGpxTrackViewSettings::selectedTrack() const
{
  QGpxTrackItem * item = nullptr;

  if( gpxTracks ) {

    const int cursel =
        gpxTrackSelector_ctl->combo()->currentIndex();

    if( cursel >= 0 && cursel < (int) gpxTracks->size() ) {
      item = (*gpxTracks)[cursel];
    }
  }

  return item;
}

void QGpxTrackViewSettings::setGpxTracks(const std::vector<QGpxTrackItem*> * gpxTracks)
{
  this->gpxTracks = gpxTracks;
  populateTrackSelectionCombo();
  updateControls();
}

void QGpxTrackViewSettings::onupdatecontrols()
{
  if( !gpxTracks ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
    Base::onupdatecontrols();
  }
}

void QGpxTrackViewSettings::update_control_states()
{
  QGpxTrackItem * item =
      selectedTrack();

  if( !item ) {
    gpxTrackSelector_ctl->openVideoControl()->setEnabled(false);
  }
  else {
    gpxTrackSelector_ctl->trackVisibiltyControl()->setChecked(item->isVisible());
    gpxTrackSelector_ctl->landmarksVisibiltyControl()->setChecked(item->landmarksVisible());
    gpxTrackSelector_ctl->carVisibiltyControl()->setChecked(item->carVisible());
    gpxTrackSelector_ctl->openVideoControl()->setEnabled(!item->associatedVideoFileName().isEmpty());
  }
}

void QGpxTrackViewSettings::populateTrackSelectionCombo()
{
  c_update_controls_lock lock(this);

  QComboBox * combo =
      gpxTrackSelector_ctl->combo();

  combo->clear();

  if( gpxTracks ) {
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

void QGpxTrackViewSettings::onToggleLandmarksVisibilityClicked(bool visible)
{
  QGpxTrackItem * item =
      selectedTrack();

  if( item ) {
    item->setLandmarksVisible(visible);
  }
}

void QGpxTrackViewSettings::onToggleCarVisibilityClicked(bool visible)
{
  QGpxTrackItem * item =
      selectedTrack();

  if( item ) {
    item->setCarVisible(visible);
  }
}

void QGpxTrackViewSettings::onShowSelectedTrackOnMapClicked()
{
  QGpxTrackItem * item =
      selectedTrack();

  if( item ) {

    const c_gpx_track & track =
        item->track();

    if( !track.pts.empty() ) {

      const c_gps_position & gps =
          track.pts[track.pts.size() / 2];

      Q_EMIT showPositionOnMap(gps.latitude, gps.longitude);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGpxTrackViewSettingsDialogBox::QGpxTrackViewSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("GPX track options");

  connect(settings_ctl, &QGpxTrackViewSettings::showPositionOnMap,
      this, &ThisClass::showPositionOnMap);

  connect(settings_ctl, &QGpxTrackViewSettings::deleteSelectedTrackClicked,
      this, &ThisClass::deleteSelectedTrackClicked);

  connect(settings_ctl, &QGpxTrackViewSettings::openSelectedAssociatedVideoFileClicked,
      this, &ThisClass::openSelectedAssociatedVideoFileClicked);

}

void QGpxTrackViewSettingsDialogBox::setGpxTracks(std::vector<QGpxTrackItem*> * gpxTracks)
{
  settings_ctl->setGpxTracks(gpxTracks);
}

QGpxTrackItem* QGpxTrackViewSettingsDialogBox::selectedTrack() const
{
  return settings_ctl->selectedTrack();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGeoMapView::QGeoMapView(QWidget * parent) :
    Base(parent)
{
  // geoview_->setMouseTracking(true);
}

QGpxTrackItem* QGeoMapView::loadGpxTrack(const QString & filename)
{
  QGpxTrackItem * item =
      new QGpxTrackItem(this);

  if( !item->loadTrack(filename) ) {
    delete item;
    return nullptr;
  }

  item->setZValue(TRACK_ZVALUE);
  item->setVisible(true);
  item->setName(QFileInfo(filename).fileName());
  item->setDescription(item->track().name.c_str());
  item->setPathFileName(filename);

  QObject::connect(item, &QGpxTrackItem::openAssociatedVideoFrameRequested,
      [this](QGpxTrackItem * trackItem, QGpxLandmarkItem * keyPointItem) {

        const QString& associatedVideoFileName =
            trackItem->associatedVideoFileName();

        if ( !associatedVideoFileName.isEmpty() ) {
          Q_EMIT openVideoFileRequested(associatedVideoFileName,
              keyPointItem->associatedVideoFrameIndex());
        }

      });

  return item;
}

bool QGeoMapView::addGpxTrack(const QString & filename)
{
  QGpxTrackItem * item =
      loadGpxTrack(filename);

  if( !item ) {
    return false;
  }

  scene_->addItem(item);
  gpxTrackItems.emplace_back(item);

  if( viewSettingsDialogBox ) {
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
  if( !checked ) {

    if( viewSettingsDialogBox && viewSettingsDialogBox->isVisible() ) {
      viewSettingsDialogBox->hide();
    }

  }
  else {

    if( !viewSettingsDialogBox ) {

      viewSettingsDialogBox = new QGpxTrackViewSettingsDialogBox(this);
      viewSettingsDialogBox->settingsWidget()->setGpxTracks(&gpxTrackItems);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::visibilityChanged,
          toggleOptionsDialogBoxAction, &QAction::setChecked);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::showPositionOnMap,
          this, &ThisClass::flyToPosition);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::openSelectedAssociatedVideoFileClicked,
          this, &ThisClass::onOpenSelectedAssociatedVideoFileClicked);

      connect(viewSettingsDialogBox, &QGpxTrackViewSettingsDialogBox::deleteSelectedTrackClicked,
          this, &ThisClass::onDeleteSelectedTrackClicked);
    }

    viewSettingsDialogBox->show();
  }
}

void QGeoMapView::flyToPosition(double latitude, double longitude)
{
  Base::flyTo(QGeoPos(latitude * 180 / CV_PI, longitude * 180 / CV_PI));
}

void QGeoMapView::setCurrentVideoScrollpos(const QString & currentFileName, int scrollpos)
{
  _currentVideoScrollpos = scrollpos;

  if ( !currentFileName.isEmpty() ) {

    for ( QGpxTrackItem * trackItem : gpxTrackItems ) {
      if ( trackItem->associatedVideoFileName() == currentFileName ) {

        trackItem->computeCarPositon(scrollpos);

        break;
      }

    }
  }

}

int QGeoMapView::currentVideoScrollpos() const
{
  return _currentVideoScrollpos;
}

void QGeoMapView::onOpenSelectedAssociatedVideoFileClicked()
{
  QGpxTrackItem * item =
      viewSettingsDialogBox->selectedTrack();

  if( item ) {

    const QString filename =
        item->associatedVideoFileName();

    if( !filename.isEmpty() ) {
      Q_EMIT openVideoFileRequested(filename, -1);
    }
  }
}

void QGeoMapView::onDeleteSelectedTrackClicked()
{
  if( viewSettingsDialogBox ) {

    QGpxTrackItem * item =
        viewSettingsDialogBox->selectedTrack();

    if( item ) {

      const int reply =
          QMessageBox::question(this, "Confirmation required",
              "Confirmation required:\n"
                  "Are you sure to delete selected GPX track from Geo Map View ?",
              QMessageBox::Yes | QMessageBox::No);

      if( reply == QMessageBox::Yes ) {

        scene_->removeItem(item);

        const auto pos =
            std::find(gpxTrackItems.begin(), gpxTrackItems.end(), item);

        if( pos != gpxTrackItems.end() ) {
          gpxTrackItems.erase(pos);
        }

        viewSettingsDialogBox->setGpxTracks(&gpxTrackItems);

        delete item;
      }
    }
  }
}

void QGeoMapView::showEvent(QShowEvent *event)
{
  Base::showEvent(event);

  CF_DEBUG("_firstShow=%d", _firstShow, _firstShow);

  if ( !Base::visibleRegion().isEmpty() ) {
    _firstShow = false;
    geoview_->cameraTo(QGeoPos(0, 0), 90, 120);
  }
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

  for( int i = 0; i < num_tracks; ++i ) {

    const QString prefix =
        qsprintf("QGeoMapView/track%d_", i);

    const QByteArray utf8prefix =
        prefix.toUtf8();

    const QString pathFileName =
        settings.value(QString("%1_pathFileName").arg(prefix), "").toString();

    if( !pathFileName.isEmpty() ) {

      QGpxTrackItem * item =
          loadGpxTrack(pathFileName);

      if ( !item ) {
        continue;
      }


      item->setAssociatedVideoFileName(settings.value(QString("%1_videoFileName").arg(prefix)).toString());
      item->setVisible(settings.value(QString("%1_isVisible").arg(prefix)).toBool());
      item->setShowLines(settings.value(QString("%1_showLines").arg(prefix)).toBool());
      item->setLineWidth(settings.value(QString("%1_lineWidth").arg(prefix)).toInt());
      item->setLineColor(settings.value(QString("%1_lineColor").arg(prefix)).value<QColor>());
      item->setLineOpaqueness(settings.value(QString("%1_lineOpaqueness").arg(prefix)).toInt());
      item->setShowPoints(settings.value(QString("%1_showPoints").arg(prefix)).toBool());
      item->setPointSize(settings.value(QString("%1_pointSize").arg(prefix)).toInt());
      item->setPointPenWidth(settings.value(QString("%1_pointPenWidth").arg(prefix)).toInt());
      item->setPointColor(settings.value(QString("%1_pointColor").arg(prefix)).value<QColor>());
      item->setPointOpaqueness(settings.value(QString("%1_pointOpaqueness").arg(prefix)).toInt());

      gpxTrackItems.emplace_back(item);
      scene_->addItem(item);

      const int num_gpx_keypoints =
          settings.value(QString("%1_keypoints").arg(prefix), 0).toInt();

      for( int j = 0; j < num_gpx_keypoints; ++j ) {

        const QString prefix2 =
            qsprintf("%skeypoint%d_", utf8prefix.constData(), j);

        const int associatedFrameIndex =
            settings.value(QString("%1_associatedVideoFrameIndex").arg(prefix2)).value<int>();

        const int gpxPointIndex =
            settings.value(QString("%1_gpxPointIndex").arg(prefix2), -1).toInt();

        if ( gpxPointIndex >= 0 && gpxPointIndex < (int)item->track().pts.size() ) {
          if ( !item->findGpxLandmarkItem(gpxPointIndex) )   {

            item->addGpxLandmarkItem(gpxPointIndex, associatedFrameIndex,
                item->track().pts[gpxPointIndex]);

          }
        }
      }

    }
  }

  if( viewSettingsDialogBox ) {
    viewSettingsDialogBox->settingsWidget()->setGpxTracks(&gpxTrackItems);
  }

}

void QGeoMapView::saveSettings(QSettings & settings)
{
  const int num_tracks =
      (int) gpxTrackItems.size();

  settings.setValue("QGeoMapView/num_tracks", num_tracks);

  for( int i = 0; i < num_tracks; ++i ) {

    const QGpxTrackItem * trackItem =
        gpxTrackItems[i];

    const QString prefix =
        qsprintf("QGeoMapView/track%d_", i);

    const QByteArray utf8prefix =
        prefix.toUtf8();


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

    const int num_gpx_keypoints =
        (int) trackItem->gpxLandmarks().size();

    settings.setValue(QString("%1_keypoints").arg(prefix), num_gpx_keypoints);


    for( int j = 0; j < num_gpx_keypoints; ++j ) {

      const QGpxLandmarkItem * kpItem =
          trackItem->gpxLandmarks(j);

      const QString prefix2 =
          qsprintf("%skeypoint%d_",utf8prefix.constData(), j);

      settings.setValue(QString("%1_gpxPointIndex").arg(prefix2),  kpItem->gpxPointIndex());
      settings.setValue(QString("%1_associatedVideoFrameIndex").arg(prefix2), kpItem->associatedVideoFrameIndex());
    }

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QGeoMapViewDock::QGeoMapViewDock(const QString & title, QWidget * parent) :
    Base(title, parent)
{
  Base::setWidget(_geoView = new QGeoMapView(this));

  const QList<QAction*> actions = _geoView->toolbarActions();
  for( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }
}

QGeoMapView* QGeoMapViewDock::geoView() const
{
  return _geoView;
}

QGeoMapViewDock* addGeoMapViewDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QGeoMapViewDock * dock = new QGeoMapViewDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serstacker */
