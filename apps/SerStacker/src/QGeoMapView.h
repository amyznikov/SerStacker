/*
 * QGeoMapView.h
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoMapView_h__
#define __QGeoMapView_h__

#include <gui/qgeoview/QGeoViewWidgetDock.h>
#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include <core/proc/gps/c_gpx_interpolation.h>


namespace serstacker {

class QGeoMapView;

class QGpxLandmarkItem :
    public QGeoPixmapItem
{
  Q_OBJECT;
public:
  typedef QGpxLandmarkItem ThisClass;
  typedef QGeoPixmapItem Base;

  QGpxLandmarkItem(int gpxPointIndex, const QGeoPos & geoPos,
      QGraphicsItem *parent = nullptr);

  int gpxPointIndex() const;

  void setAssociatedVideoFrameIndex(int v);
  int associatedVideoFrameIndex() const;


Q_SIGNALS:
  void deleteRequested(QGpxLandmarkItem * item);
  void openAssociatedVideoFrameRequested(QGpxLandmarkItem * item);

protected:
  bool populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) final;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)final;
  void onGeoPosChanged(const QGeoPos & pos) final;

protected:
  const int _gpxPointIndex;
  int _associatedVideoFrameIndex = 0;
};

class QGpxCarItem :
    public QGeoPixmapItem
{
  Q_OBJECT;
public:
  typedef QGpxLandmarkItem ThisClass;
  typedef QGeoPixmapItem Base;

  QGpxCarItem(QGraphicsItem *parent = nullptr);
  QGpxCarItem(const QGeoPos & pos, QGraphicsItem *parent = nullptr);

protected:
  bool populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) final;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)final;
  void onGeoPosChanged(const QGeoPos & pos) final;
};


class QGpxTrackItem :
    public QAbstractGeoPolyLineItem
{
  Q_OBJECT;
public:
  typedef QGpxTrackItem ThisClass;
  typedef QAbstractGeoPolyLineItem Base;

  QGpxTrackItem(QGeoMapView * geoMapView, QGraphicsItem *parent = nullptr);

  void deleteChildItems();

  bool loadGPXTrack(const QString & filename);

  const c_gpx_track & track() const;

  void setGPXPathFileName(const QString& filename);
  const QString & gpxPathFileName() const;

  void setAssociatedVideoFileName(const QString & fname);
  const QString & associatedVideoFileName() const;

  const std::vector<QGpxLandmarkItem*> gpxLandmarks() const;
  const QGpxLandmarkItem* gpxLandmarks(int index) const;

  bool addGpxLandmarkItem(int gpxPointIndex, int associatedVideoFrameIndex);

  void setTrackVisible(bool v);
  bool trackVisible() const;

  void setLandmarksVisible(bool v);
  bool landmarksVisible() const;

  void setCarVisible(bool v);
  bool carVisible() const;
  void computeCarPositon(int scrollpos);

  QGpxLandmarkItem * findGpxLandmarkItem(int gpxPointIndex);


Q_SIGNALS:
  void openAssociatedVideoFrameRequested(QGpxTrackItem * trackItem, QGpxLandmarkItem * keyPointItem);

protected: // QAbstractGeoPolygonItem
  void mousePressEvent(QGraphicsSceneMouseEvent * event) final;
  bool populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) final;
  int pointsCount() const final;
  void insertPoint(const QGeoPos &, int insert_pos) final;
  void removePoint(int remove_pos) final;
  void setGeoPoint(int index, const QGeoPos & ) final;
  QGeoPos getGeoPoint(int index) const final;

protected:
  QGeoMapView * _geoMmapView = nullptr;
  QString _gpxPathFileName;
  QString _associatedVideoFileName;
  c_gpx_track _track;
  c_gpx_interpolation _gpx_interpolation;
  std::vector<QGpxLandmarkItem*> _gpxLandmarks;
  QGpxCarItem * _carItem = nullptr;
  bool _trackVisible = true;
  bool _landmarksVisible = true;
  bool _carVisible = true;
};


class QGpxTrackSelectorWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QGpxTrackSelectorWidget ThisClass;
  typedef QWidget Base;

  QGpxTrackSelectorWidget(QWidget * parent = nullptr);

  QComboBox * combo() const;

  QToolButton * trackVisibiltyControl() const;
  QToolButton * landmarksVisibiltyControl() const;
  QToolButton * carVisibiltyControl() const;
  QToolButton * openVideoControl() const;

Q_SIGNALS:
  void toggleTrackVisibilityClicked(bool visible);
  void toggleLandmarksVisibilityClicked(bool visible);
  void toggleCarVisibilityClicked(bool visible);
  void showSelectedTrackOnMapClicked();
  void exportGPXTrackToConfigFileClicked();
  void deleteSelectedTrackClicked();
  void openAssociatedVideoFileClicked();

protected:
  QHBoxLayout * _hbox = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * track_visibilty_ctl = nullptr;
  QToolButton * landmarks_visibilty_ctl = nullptr;
  QToolButton * car_visibilty_ctl = nullptr;
  QToolButton * show_track_on_geomap_ctl = nullptr;
  QToolButton * open_video_ctl = nullptr;
  QToolButton * export_to_config_file_ctl = nullptr;
  QToolButton * delete_track_ctl = nullptr;
};

class QGpxTrackViewSettings :
  public QSettingsWidgetTemplate<QGpxTrackItem>
{
  Q_OBJECT;
public:
  typedef QGpxTrackViewSettings ThisClass;
  typedef QSettingsWidgetTemplate<QGpxTrackItem> Base;

  QGpxTrackViewSettings(QWidget * parent = nullptr);

  void setGpxTracks(const std::vector<QGpxTrackItem*> * gpxTracks);

  QGpxTrackItem * selectedTrack() const;

Q_SIGNALS:
  void openSelectedAssociatedVideoFileClicked();
  void showPositionOnMapClicked(double latitude, double longitude);
  void exportSelectedGPXTrackToConfigFileClicked();
  void deleteSelectedTrackClicked();

protected:
  void update_control_states();
  void populateTrackSelectionCombo();
  void onGpxTrackSelected(int index);
  void onToggleTrackVisibilityClicked(bool visible);
  void onToggleLandmarksVisibilityClicked(bool visible);
  void onToggleCarVisibilityClicked(bool visible);
  void onShowSelectedTrackOnMapClicked();

protected:
  const std::vector<QGpxTrackItem*> * gpxTracks = nullptr;

  QGpxTrackSelectorWidget * gpxTrackSelector_ctl = nullptr;

  QCheckBox * showLines_ctl = nullptr;
  QSpinBox * lineWidth_ctl = nullptr;
  QColorPickerButton * lineColor_ctl = nullptr;
  QIntegerSliderSpinBox * lineOpaqueness_ctl = nullptr;

  QCheckBox * showPoints_ctl = nullptr;
  QSpinBox * pointSize_ctl = nullptr;
  QSpinBox * pointPenWidth_ctl = nullptr;
  QColorPickerButton * pointColor_ctl = nullptr;
  QIntegerSliderSpinBox * pointOpaqueness_ctl = nullptr;

  QBrowsePathCombo * associatedVideoFileName_ctl = nullptr;

};


class QGpxTrackViewSettingsDialogBox :
    public QSettingsDialogBoxTemplate<QGpxTrackViewSettings>
{
  Q_OBJECT;
public:
  typedef QGpxTrackViewSettingsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QGpxTrackViewSettings> Base;

  QGpxTrackViewSettingsDialogBox(QWidget * parent = nullptr);

  void setGpxTracks(std::vector<QGpxTrackItem*> * gpxTracks);

  QGpxTrackItem * selectedTrack() const;

Q_SIGNALS:
  void showPositionOnMap(double latitude, double longitude);
  void openSelectedAssociatedVideoFileClicked();
  void exportSelectedGPXTrackToConfigFileClicked();
  void deleteSelectedTrackClicked();
};


class QGeoMapView :
    public QGeoViewWidget
{
  Q_OBJECT
public:
  typedef QGeoMapView ThisClass;
  typedef QGeoViewWidget Base;

  QGeoMapView(QWidget * parent = nullptr);

  void importGpxTrack();
  void exportSelectedGpxTrack();
  bool importGpxTrack(const QString & filename);

  void flyToPosition(double lat_deg, double lon_deg, double lat_size_deg, double lon_size_deg);

  void loadSettings(const QString & prefix = "");
  void saveSettings(const QString & prefix = "");
  void loadSettings(const QSettings & settings, const QString & prefix = "");
  void saveSettings(QSettings & settings, const QString & prefix = "");



  void setCurrentVideoScrollpos(const QString & currentFileName, int scrollpos);
  int currentVideoScrollpos() const;

Q_SIGNALS:
  void openVideoFileRequested(const QString & filename, int scrollToIndex = -1);

protected Q_SLOTS:
  void onToggleOptionsDialogBox(bool checked);
  void onOpenSelectedAssociatedVideoFileClicked();
  void onDeleteSelectedTrackClicked();

protected:
  bool exportGpxTrackToConfigFile(const QGpxTrackItem * item, const QString & filename) const;
  bool importGpxTrackFromConfigFile(const QString & filename);

protected:
  void createToolbarActions() final;
  QGpxTrackItem * loadGpxTrack(const QString & filename);

protected:
  std::vector<QGpxTrackItem*> gpxTrackItems;
  QGpxTrackViewSettingsDialogBox * viewSettingsDialogBox = nullptr;
  QAction * toggleOptionsDialogBoxAction = nullptr;
  int _currentVideoScrollpos = 0;
};



class QGeoMapViewDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QGeoMapViewDock ThisClass;
  typedef QCustomDockWidget Base;

  QGeoMapViewDock(const QString &title, QWidget * parent = nullptr);

  QGeoMapView * geoView() const;

protected:
  QGeoMapView * _geoView = nullptr;
};


QGeoMapViewDock * addGeoMapViewDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);

} /* namespace serstacker */

#endif /* __QGeoMapView_h__ */
