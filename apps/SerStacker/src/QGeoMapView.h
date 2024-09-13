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
#include <core/proc/gps/gpx.h>

namespace serstacker {

class QGpxTrackItem :
    public QAbstractGeoPolyLineItem
{
public:
  typedef QGpxTrackItem ThisClass;
  typedef QAbstractGeoPolyLineItem Base;

  QGpxTrackItem(QGraphicsItem *parent = nullptr);

  bool loadTrack(const QString & filename);

  const c_gpx_track & track() const;

  void setPathFileName(const QString& filename);
  const QString & pathFileName() const;

  void setAssociatedVideoFileName(const QString & fname);
  const QString & associatedVideoFileName() const;

protected: // QAbstractGeoPolygonItem
  void mousePressEvent(QGraphicsSceneMouseEvent * event) final;
  bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) final;
  int pointsCount() const final;
  void insertPoint(const QGeoPos &, int insert_pos) final;
  void removePoint(int remove_pos) final;
  void setGeoPoint(int index, const QGeoPos & ) final;
  QGeoPos getGeoPoint(int index) const final;



protected:
  c_gpx_track _track;
  QString _pathFileName;
  QString _associatedVideoFileName;
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

Q_SIGNALS:
  void toggleTrackVisibilityClicked(bool visible);
  void showSelectedTrackOnMapClicked();
  void deleteSelectedTrackClicked();

protected:
  QHBoxLayout * _hbox = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * trackVisibilty_ctl = nullptr;
  QToolButton * trackSend_ctl = nullptr;
  QToolButton * trackDelete_ctl = nullptr;
};

class QGpxTrackViewSettings :
  public QSettingsWidgetTemplate<QGpxTrackItem>
{
  Q_OBJECT;
public:
  typedef QGpxTrackViewSettings ThisClass;
  typedef QSettingsWidgetTemplate<QGpxTrackItem> Base;

  QGpxTrackViewSettings(QWidget * parent = nullptr);

  void setGpxTracks(std::vector<QGpxTrackItem*> * gpxTracks);

  QGpxTrackItem * selectedTrack() const;

Q_SIGNALS:
  void showPositionOnMap(double latitude, double longitude);

protected:
  void onupdatecontrols() final;
  void populateTrackSelectionCombo();
  void onGpxTrackSelected(int index);
  void onToggleTrackVisibilityClicked(bool visible);
  void onShowSelectedTrackOnMapClicked();
  void onDeleteSelectedTrackClicked();


protected:
  std::vector<QGpxTrackItem*> * gpxTracks = nullptr;

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

Q_SIGNALS:
  void showPositionOnMap(double latitude, double longitude);

protected:
};


class QGeoMapView :
    public QGeoViewWidget
{
  Q_OBJECT
public:
  typedef QGeoMapView ThisClass;
  typedef QGeoViewWidget Base;

  QGeoMapView(QWidget * parent = nullptr);

  bool addGpxTrack(const QString & filename);

  void flyToPosition(double latitude, double longitude);

  void loadSettings();
  void saveSettings();
  void loadSettings(QSettings & settings);
  void saveSettings(QSettings & settings);

protected Q_SLOTS:
  void onToggleOptionsDialogBox(bool checked);

protected:
  void createToolbarActions() final;
  static QGpxTrackItem * loadGpxTrack(const QString & filename);

protected:
  QGpxTrackViewSettingsDialogBox * viewSettingsDialogBox = nullptr;
  QAction * toggleOptionsDialogBoxAction = nullptr;

  std::vector<QGpxTrackItem*> gpxTrackItems;
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
