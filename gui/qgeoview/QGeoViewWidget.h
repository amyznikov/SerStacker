/*
 * QGeoMapWidget.h
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoMapWidget_h__
#define __QGeoMapWidget_h__

#include <gui/qgeomapview/qgeomapview.h>

class QGeoViewWidget :
    public QWidget
{
  Q_OBJECT;

public:
  typedef QGeoViewWidget ThisClass;
  typedef QWidget Base;

  QGeoViewWidget(QWidget * parent = nullptr);
  ~QGeoViewWidget();

  const QList<QAction*> toolbarActions();

  //  virtual QToolBar * createToolbar(bool attachToLayout = true);
  //  QToolBar * toolBar() const;


  QGeoView * view() const;
  QRect viewRect() const;
  QRectF sceneViewRect() const;
  QGeoRect geoViewRect() const;
  QPixmap grabGeoViewPixmap() const;


  QAction * addCopyImageToClipboardAction(const QString & text,
      const QKeySequence & keysequence = QKeySequence());


  void cameraTo(const c_gps_position & gps);
  void cameraTo(const QGeoPos & geopos );
  void flyTo(const c_gps_position & gps);
  void flyTo(const QGeoPos & geopos );

Q_SIGNALS:
  void visibilityChanged(bool visible);

public Q_SLOTS:
  void showReferecePosition();

protected Q_SLOTS:
  void onSceneContextMenuRequested(QGraphicsSceneContextMenuEvent *event);


protected:
  void onupdatecontrols() {};
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void setReferecePosition(double lat_degrees, double lon_degrees, double alt_meters);
  void setReferecePosition(const c_gps_position & gps);
  virtual void createToolbarActions();

protected:
  QGeoProjectionEPSG3857 projection_;
  QList<QAction * > toolbarActions_;

  QVBoxLayout * layout_ = nullptr;
  // QToolBar * toolBar_ctl = nullptr;
  QGeoView * geoview_ = nullptr;
  QGeoScene * scene_ = nullptr;

  QAction * layersToolButtonAction = nullptr;
  QAction * flyToAction = nullptr;
  QAction * selectGeoLayerAction = nullptr;
  QString savedFlyToinputText;

//  QAction * showHidePolygons_ctl = nullptr;
//  QAction * showHideLabels_ctl = nullptr;
//  QAction * showHideRings_ctl = nullptr;
//  QAction * showHideRingsOptions_ctl = nullptr;
  QMenu layersMenu_;

  QList<QPair<QAction*, QGeoTiles*>> geotiles_;
  QGeoPixmapItem * referecePositionItem_ = nullptr;
};

#endif /* __QGeoMapWidget_h__ */
