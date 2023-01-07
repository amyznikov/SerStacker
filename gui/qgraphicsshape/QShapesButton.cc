/*
 * QShapesButton.cc
 *
 *  Created on: Oct 14, 2021
 *      Author: amyznikov
 */

#include "QShapesButton.h"
#include "QGraphicsRectShape.h"
#include <gui/qimageview/QImageScene.h>
#include <core/debug.h>


#define ICON_shapes     "shapes"
#define ICON_line       "line"
#define ICON_rectangle  "rectangle"
#define ICON_delete     "delete"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qimageview/icons/%1").arg(name));
}

static QIcon shapes_icon;
static QIcon line_icon;
static QIcon rectangle_icon;
static QIcon delete_icon;

QShapesButton::QShapesButton(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(qimageview_resources);

  if ( shapes_icon.isNull() ) {
    shapes_icon = getIcon(ICON_shapes);
  }
  if ( line_icon.isNull() ) {
    line_icon = getIcon(ICON_line);
  }
  if ( rectangle_icon.isNull() ) {
    rectangle_icon = getIcon(ICON_rectangle);
  }
  if ( delete_icon.isNull() ) {
    delete_icon = getIcon(ICON_delete);
  }

  setToolButtonStyle(Qt::ToolButtonIconOnly);
  setIconSize(QSize(16, 16));
  setIcon(getIcon(ICON_shapes));
  setText("Shapes");
  setToolTip("Show/Hide shapes");
  setCheckable(true);

  pen_.setCosmetic(true);
  pen_.setColor(Qt::yellow);


  connect(this, &QToolButton::toggled,
      [this](bool checked) {
        if ( sceneView_ ) {
         // sceneView_->setShapesVisible(checked);
        }
      });


  popup_.addAction(line_icon,
      "Add line ...",
      [this]() {
//        if ( sceneView_ ) {
//          sceneView_->addLineShape();
//          if ( !isChecked() ) {
//            setChecked(true);
//          }
//          else {
//            sceneView_->shapesVisible();
//          }
//        }
      });

  popup_.addAction(rectangle_icon,
      "Add rectangle ...",
      [this]() {
        if ( sceneView_ ) {

          QGraphicsScene * scene =
              sceneView_->scene();

          if ( scene ) {

            QRect rc1 =
                sceneView_->rect();

            QPoint center =
                rc1.center();

            QRect rc2(center.x() - std::max(10, rc1.width()/4),
                center.y() - std::max(10, rc1.height()/4),
                rc1.width() / 2, rc1.height() / 2);

            QGraphicsRectShape *shape =
                new QGraphicsRectShape(sceneView_->mapToScene(rc2).boundingRect());

            shape->setVisible(true);
            shape->setFlag(QGraphicsItem::ItemIsMovable, true);
            shape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
            shape->setPen(pen_);
            scene->addItem(shape);

            QImageScene * imageScene =
                dynamic_cast<QImageScene * >(scene);

            if ( imageScene ) {
              connect(shape, &QGraphicsShape::itemChanged,
                  imageScene, &QImageScene::graphicsItemChanged,
                  Qt::QueuedConnection);
            }

          }
        }
  });

  popup_.addSeparator();
  popup_.addAction(delete_icon,
      "Delete all shapes...",
      [this]() {
        if ( sceneView_ ) {

          QGraphicsScene * scene =
              sceneView_->scene();

          if ( scene ) {

            QList<QGraphicsItem *> items =
                scene->items();

            for ( QGraphicsItem * item : items ) {

              QGraphicsShape * shape =
                  dynamic_cast<QGraphicsShape * >(item);
              if ( shape ) {
                scene->removeItem(shape);
              }
            }
          }
        }
      });

  setPopupMode(QToolButton::MenuButtonPopup);
  setMenu(&popup_);
}

void QShapesButton::setSceneView(QGraphicsView * sceneView)
{
  if ( !(sceneView_ = sceneView) ) {
    setEnabled(false);
  }
  else {
    //setChecked(sceneView_->shapesVisible());
    setEnabled(true);
  }
}

QGraphicsView * QShapesButton::sceneView() const
{
  return sceneView_;
}
