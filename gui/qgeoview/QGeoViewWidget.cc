/*
 * QGeoMapWidget.cc
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#include "QGeoViewWidget.h"
#include <gui/widgets/style.h>
#include <gui/widgets/createAction.h>
#include <core/debug.h>


#define ICON_cartesian    ":/qgeoview/icons/cartesian64_icon.png"
#define ICON_lidar        ":/qgeoview/icons/lidar_label_icon.png"
#define ICON_circles      ":/qgeoview/icons/circles1_icon.png"
#define ICON_pentagon     ":/qgeoview/icons/pentagon_icon.png"
#define ICON_polygon      ":/qgeoview/icons/polygon_icon.png"
#define ICON_layers       ":/qgeoview/icons/layers_icon.png"
#define ICON_send         ":/qgeoview/icons/send.png"


static const QPixmap & icon_cartesian()
{
  static QPixmap pixmap =
      getPixmap(ICON_cartesian);

  return pixmap;
}


QGeoViewWidget::QGeoViewWidget(QWidget * parent) :
    Base(parent)
{

  Q_INIT_RESOURCE(qgeoview_resources);

  layout_ = new QVBoxLayout(this);

  this->setContentsMargins(0,0,0,0);
  layout_->setContentsMargins(0,0,0,0);

  scene_ = new QGeoScene(this);
  scene_->setProjection(&projection_);

  geoview_ = new QGeoView(this);
  geoview_->setContentsMargins(0,0,0,0);
  geoview_->setScene(scene_);

  layout_->addWidget(geoview_);

  connect(scene_, &QGeoScene::contextMenuRequested,
      this, &ThisClass::onSceneContextMenuRequested);


  //
//  layersMenu_.addAction(showHidePolygons_ctl =
//      new QAction(getIcon(ICON_pentagon),
//          "Show ROI"));
//
//  showHidePolygons_ctl->setCheckable(true);
//  showHidePolygons_ctl->setChecked(false);
//  connect(showHidePolygons_ctl, &QAction::triggered,
//      this, &ThisClass::showPolygons);
//
//  //
//
////  layersMenu_.addAction(showHideLabels_ctl =
////      new QAction(getIcon(ICON_lidar),
////          "Show labels"));
//
//  showHideLabels_ctl->setCheckable(true);
//  showHideLabels_ctl->setChecked(true);
//  connect(showHideLabels_ctl, &QAction::triggered,
//      this, &ThisClass::updateLabelsForLidars);
//
//  //
//
////  layersMenu_.addAction(showHideRings_ctl =
////      new QAction(getIcon(ICON_circles),
////          "Show rings"));
//
//  showHideRings_ctl->setCheckable(true);
//  showHideRings_ctl->setChecked(false);
//  connect(showHideRings_ctl, &QAction::triggered,
//      this, &ThisClass::updateLabelsForLidars);


  //

//  layersMenu_.addAction(showHideRingsOptions_ctl =
//      new QAction(getIcon(ICON_circles),
//          "Rings options..."));

//  showHideRingsOptions_ctl->setCheckable(true);
//  showHideRingsOptions_ctl->setChecked(false);
//  connect(showHideRingsOptions_ctl, &QAction::triggered,
//      this, &ThisClass::showRingsOptions);

  //

  layersMenu_.addSeparator();

  //
  /// List of available tile maps.
  //  const QString customURI = "http://c.tile.stamen.com/watercolor/${z}/${x}/${y}.jpg";
  geotiles_ = {
      { new QAction("GOOGLE_SATELLITE"), new QGeoTilesGoogle(QGeoTilesGoogle::TilesType::Satellite) },
      { new QAction("GOOGLE_HYBRID"), new QGeoTilesGoogle(QGeoTilesGoogle::TilesType::Hybrid) },
      { new QAction("GOOGLE_SCHEMA"), new QGeoTilesGoogle(QGeoTilesGoogle::TilesType::Schema) },
      { new QAction("BING_SATELLITE"), new QGeoTilesBing(QGeoTilesBing::TilesType::Satellite) },
      { new QAction("BING_HYBRID"), new QGeoTilesBing(QGeoTilesBing::TilesType::Hybrid) },
      { new QAction("BING_SCHEMA"), new QGeoTilesBing(QGeoTilesBing::TilesType::Schema) },
      { new QAction("OSM"), new QGeoTilesOSM() },
      // { "CUSTOM_OSM", new QGVLayerOSM(customURI) },
      };

  for( int i = 0, n = geotiles_.size(); i < n; ++i ) {

    const auto &p =
        geotiles_[i];

    QAction *action =
        p.first;

    QGraphicsObject * layer =
        p.second;

    layer->setVisible(i == 0);

    scene_->addItem(layer);

    action->setCheckable(true);
    action->setChecked(layer->isVisible());
    layersMenu_.addAction(action);

    QObject::connect(action, &QAction::triggered,
        [this, action](bool checked) {
          for ( const auto & p : geotiles_ ) {
            if ( p.first == action ) {
              p.second->setVisible(checked);
            }
            else {
              p.second->setVisible(false);
              p.first->setChecked(false);
            }
          }
        });
  }

  //

}

QGeoViewWidget::~QGeoViewWidget()
{
}

void QGeoViewWidget::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QGeoViewWidget::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}



void QGeoViewWidget::cameraTo(const QGeoPos & geopos)
{
  geoview_->cameraTo(geopos, 0.001, 0.001);
}

void QGeoViewWidget::cameraTo(const c_gps_position & gps)
{
  cameraTo(QGeoPos(gps.latitude * 180 / M_PI, gps.longitude * 180 / M_PI));
}


void QGeoViewWidget::flyTo(const QGeoPos & geopos )
{
  geoview_->flyTo(geopos, 0.001, 0.001);
}


void QGeoViewWidget::flyTo(const c_gps_position & gps)
{
  flyTo(QGeoPos(gps.latitude * 180 / M_PI, gps.longitude * 180 / M_PI));
}

const QList<QAction*> QGeoViewWidget::toolbarActions()
{
  if ( toolbarActions_.empty() ) {
    createToolbarActions();
  }

  return toolbarActions_;
}

void QGeoViewWidget::createToolbarActions()
{

  toolbarActions_.append(layersToolButtonAction = new QAction(getIcon(ICON_layers), "Layers"));
  layersToolButtonAction->setToolTip("Select visible layers");
  layersToolButtonAction->setMenu(&layersMenu_);

  toolbarActions_.append(flyToAction = new QAction(getIcon(ICON_send), "FlyTo"));
  flyToAction->setToolTip("Fly to specific GPS position");

  connect(flyToAction, &QAction::triggered,
      [this]() {

        while ( 42 ) {

          QString inputText = QInputDialog::getText(this, "Enter target GPS position",
              "Target GPS position as lattude:longitude in degrees",
              QLineEdit::Normal,
              savedFlyToinputText);

          if ( !inputText.isEmpty() ) {

            savedFlyToinputText = inputText;

            double lat = 0, lon = 0;

            const QByteArray a = inputText.toUtf8();

            if ( sscanf(a.constData(), "%lf %lf", &lat, &lon) != 2 && sscanf(a.constData(), "%lf %*[:;\t ] %lf", &lat, &lon) != 2) {

              QMessageBox::critical(this, "INPUT ERROR",
                  "Syntax error in GPS position.\n"
                  "Input format is 'lat ; lon' with lat and lon units in degrees\n");
              continue;
            }

            flyTo(QGeoPos(lat, lon));
          }

          break;

        }
      });
}


//
//QToolBar * QGeoViewWidget::createToolbar(bool attachToLayout)
//{
//  if ( !toolBar_ctl ) {
//
//    toolBar_ctl = new QToolBar(this);
//    toolBar_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
//    toolBar_ctl->setIconSize(QSize(16,16));
//
//
//    layersToolButton_ctl = new QToolButton(this);
//    layersToolButton_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
//    layersToolButton_ctl->setText("Layers...");
//    layersToolButton_ctl->setIcon(getIcon(ICON_layers));
//    toolBar_ctl->addWidget(layersToolButton_ctl);
//
//    QObject::connect(layersToolButton_ctl, &QToolButton::clicked,
//        [this]() {
//          layersMenu_.exec(layersToolButton_ctl->
//              mapToGlobal(QPoint(layersToolButton_ctl->width()/2,
//                      layersToolButton_ctl->height()/2)));
//        });
//
//
//
////    selectLayers_ctl = new QToolButton(this);
////    selectLayers_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
////    selectLayers_ctl->setText("Layers...");
////    selectLayers_ctl->setIcon(getIcon(ICON_layers));
////    toolBar_ctl->addWidget(selectLayers_ctl);
////
////
////    showRings_ctl = new QToolButton(this);
////    showRings_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
////    showRings_ctl->setIcon(getIcon(ICON_circles));
////    showRings_ctl->setText("Rings");
////    showRings_ctl->setToolTip("Show/Hide LiDAR points");
////    showRings_ctl->setCheckable(true);
////    showRings_ctl->setPopupMode(QToolButton::MenuButtonPopup);
////    showRings_ctl->setMenu(&ringsMenu_);
////    toolBar_ctl->addWidget(showRings_ctl);
////
////
////    showPolygons_ctl = new QToolButton(this);
////    showPolygons_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
////    showPolygons_ctl->setIcon(getIcon(ICON_pentagon));
////    showPolygons_ctl->setText("ROI");
////    showPolygons_ctl->setToolTip("Show/Hide ROI polygons...");
////    showPolygons_ctl->setCheckable(true);
////    toolBar_ctl->addWidget(showPolygons_ctl);
////
////
////
////
////    connect(selectLayers_ctl, &QToolButton::clicked,
////        [this]() {
////
////          QMenu menu;
////          QAction * action;
////
////          for ( int i = 0, n = geotiles_.size(); i < n; ++i ) {
////
////            menu.addAction(action = new QAction(geotiles_[i].first, &menu));
////            action->setCheckable(true);
////            action->setChecked(geotiles_[i].second->isVisible());
////
////            connect(action, &QAction::triggered,
////                [this, i](bool checked) {
////
////                  QGeoTiles * visibleLayer = nullptr;
////
////                  for ( int j = 0, m = geotiles_.size(); j < m; ++j ) {
////                    geotiles_[j].second->setVisible(j == i ? checked : false);
////                    if ( geotiles_[j].second->isVisible() ) {
////                      visibleLayer = geotiles_[j].second;
////                    }
////                  }
////                });
////          }
////
////          menu.exec(selectLayers_ctl->mapToGlobal(QPoint(
////                      selectLayers_ctl->width()/2,
////                      selectLayers_ctl->height()/2)));
////        });
////
////
////    ringsMenu_.addAction(ringsOptionsAction = new QAction("Options..."));
////    ringsOptionsAction->setCheckable(true);
////    connect(ringsOptionsAction, &QAction::triggered,
////        [this](bool checked) {
////          if ( !checked ) {
////            if ( ringsOptions_ctl ) {
////              delete ringsOptions_ctl;
////              ringsOptions_ctl = Q_NULLPTR;
////            }
////          }
////          else {
////
////            if ( !ringsOptions_ctl ) {
////
////              ringsOptions_ctl = new QLidarRingsDisplayOptions(this);
////              ringsOptions_ctl->setGeoViewLidarRingsMtfSettings(&displaySettings_);
////
////              connect(ringsOptions_ctl, &QLidarRingsDisplayOptions::visibilityChanged,
////                  [this](bool visible) {
////                    ringsOptionsAction->setChecked(visible);
////                  });
////            }
////
////            ringsOptions_ctl->show();
////          }
////        });
////
////    connect(showRings_ctl, &QToolButton::clicked,
////        this, &ThisClass::updateLabelsForLidars);
////
////
////    connect(showPolygons_ctl, &QToolButton::toggled,
////        [this](bool checked) {
////
////          if ( multilidar_ ) {
////
////            QList<QGraphicsItem*> items =
////                scene_->items();
////
////            for( QGraphicsItem *item : items ) {
////
////              QROIPolygonItem *polygon =
////                  dynamic_cast<QROIPolygonItem*>(item);
////
////              if( polygon ) {
////                polygon->setVisible(checked);
////              }
////            }
////          }
////        });
//
//    if( attachToLayout ) {
//      layout_->insertWidget(0, toolBar_ctl, 0, Qt::AlignTop);
//    }
//  }
//
//  return toolBar_ctl;
//}
//
//QToolBar * QGeoViewWidget::toolBar() const
//{
//  return toolBar_ctl;
//}

QGeoView * QGeoViewWidget::view() const
{
  return geoview_;
}

QRect QGeoViewWidget::viewRect() const
{
  return geoview_->viewRect();
}

QRectF QGeoViewWidget::sceneViewRect() const
{
  return geoview_->sceneViewRect();
}

QGeoRect QGeoViewWidget::geoViewRect() const
{
  return geoview_->geoViewRect();
}

QPixmap QGeoViewWidget::grabGeoViewPixmap() const
{
  return geoview_->grabViewPixmap();
}

QAction * QGeoViewWidget::addCopyImageToClipboardAction(const QString & text, const QKeySequence & keysequence)
{
  return geoview_->addCopyImageToClipboardAction(text, keysequence);
}

void QGeoViewWidget::onSceneContextMenuRequested(QGraphicsSceneContextMenuEvent * event)
{
//  QMenu menu;
//
//  const QPointF scenePos =
//      event->scenePos();
//
//  menu.addAction("Add Polygon",
//      [this, scenePos]() {
//        addROIPolygonItem(scenePos);
//    });
//
//  event->accept();
//
//  menu.exec(event->screenPos());

}



void QGeoViewWidget::setReferecePosition(double lat_degrees, double lon_degrees, double alt_meters)
{
  if( referecePositionItem_ ) {
    referecePositionItem_->setGeoPos(QGeoPos(lat_degrees, lon_degrees));
  }
  else {

    referecePositionItem_ =
        new QGeoPixmapItem(QGeoPos(lat_degrees, lon_degrees),
            icon_cartesian());

    referecePositionItem_->setFlags(QGraphicsItem::ItemIsMovable| QGraphicsItem::ItemIgnoresTransformations);
    referecePositionItem_->setZValue(1100);
    //referecePositionItem_->setVisible(showHideLabels_ctl->isChecked());
    scene_->addItem(referecePositionItem_);

//    connect(referecePositionItem_, &QGeoPixmapItem::geoPosChanged,
//        [this](const QGeoPos & geopos) {
//
//          if ( multilidar_ ) {
//
//            c_guard_lock lock(multilidar_->mutex());
//
//            c_gps_position & gps =
//                multilidar_->reference_position();
//
//            gps.latitude =
//                geopos.latitude() * M_PI / 180;
//
//            gps.longitude =
//                geopos.longitude() * M_PI / 180;
//
//            multilidar_->reference_position_changed(this);
//          }
//        });

    cameraTo(QGeoPos(lat_degrees, lon_degrees));
  }
}

void QGeoViewWidget::setReferecePosition(const c_gps_position & gps)
{
  setReferecePosition(gps.latitude * 180 / M_PI, gps.longitude * 180 / M_PI, gps.altitude);
}

void QGeoViewWidget::showReferecePosition()
{
  if( referecePositionItem_ ) {
    if ( isVisible() ) {
      flyTo(referecePositionItem_->geoPos());
    }
    else {
      cameraTo(referecePositionItem_->geoPos());
    }
  }
}




