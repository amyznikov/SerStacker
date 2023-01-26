/*
 * QShapesButton.cc
 *
 *  Created on: Oct 14, 2021
 *      Author: amyznikov
 */

#include "QShapesButton.h"
#include "QGraphicsRectShape.h"
#include "QGraphicsLineShape.h"
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


QShapesButton::QShapesButton(QGraphicsView * view, QWidget * parent) :
    Base(parent)
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
        for ( QGraphicsShape * shape : shapes_ ) {
          shape->setVisible(checked);
        }
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

            const int w =
                std::max(10, rc1.width() / 2);

            const int h =
                std::max(10, rc1.height() / 2);

            QRect rc2(center.x() - w/2, center.y() - h/2, w, h);

            QGraphicsRectShape *shape =
                new QGraphicsRectShape(sceneView_->mapToScene(rc2).boundingRect());

            shape->setVisible(true);
            shape->setFlag(QGraphicsItem::ItemIsMovable, true);
            shape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
            shape->setPen(pen_);

            connect(shape, &QGraphicsShape::populateContextMenuReuested,
                this, &ThisClass::onPopulateGraphicsShapeContextMenu);

            scene->addItem(shape);
            shapes_.append(shape);

            QImageScene * imageScene =
                dynamic_cast<QImageScene * >(scene);

            if ( imageScene ) {
              connect(shape, &QGraphicsShape::itemChanged,
                  imageScene, &QImageScene::graphicsItemChanged,
                  Qt::QueuedConnection);
            }

            setChecked(true);
          }
        }
  });

  popup_.addAction(line_icon,
      "Add line ...",
      [this]() {
        if ( sceneView_ ) {

          QGraphicsScene * scene =
              sceneView_->scene();

          if ( scene ) {

            const QRect rc1 =
                sceneView_->rect();

            const QPoint center =
                rc1.center();


            const double L =
                std::max(20, std::max( rc1.width(), rc1.height()) / 6 );

            const QPointF p1 =
                sceneView_->mapToScene(QPoint(center.x() - L, center.y() - L));

            const QPointF p2 =
                sceneView_->mapToScene(QPoint(center.x() + L, center.y() + L));


            QGraphicsLineShape *shape =
                new QGraphicsLineShape(p1, p2);

            shape->setVisible(true);
            shape->setFlag(QGraphicsItem::ItemIsMovable, true);
            shape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
            shape->setPen(pen_);

            connect(shape, &QGraphicsShape::populateContextMenuReuested,
                this, &ThisClass::onPopulateGraphicsShapeContextMenu);

            scene->addItem(shape);
            shapes_.append(shape);

            QImageScene * imageScene =
                dynamic_cast<QImageScene * >(scene);

            if ( imageScene ) {
              connect(shape, &QGraphicsShape::itemChanged,
                  imageScene, &QImageScene::graphicsItemChanged,
                  Qt::QueuedConnection);
            }

            setChecked(true);
          }
        }
    });


  popup_.addSeparator();
  popup_.addAction(delete_icon,
      "Delete all shapes...",
      [this]() {
        if ( sceneView_ ) {

          QGraphicsScene * scene = sceneView_->scene();
          if ( scene ) {

            for ( QGraphicsShape * shape : shapes_ ) {
              scene->removeItem(shape);
              delete shape;
            }

            shapes_.clear();
          }
        }
      });

  setPopupMode(QToolButton::MenuButtonPopup);
  setMenu(&popup_);

  setSceneView(view);
}


QShapesButton::QShapesButton(QWidget * parent) :
    ThisClass(nullptr, parent)
{
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

void QShapesButton::onPopulateGraphicsShapeContextMenu(QGraphicsShape* shape, const QGraphicsSceneContextMenuEvent * event, QMenu * menu)
{
  menu->addAction("Delete this object",
      [this, shape]() {
        if ( sceneView_ ) {
          QGraphicsScene * scene = sceneView_->scene();
          if ( scene ) {
            shapes_.removeOne(shape);
            scene->removeItem(shape);
            delete shape;
          }
        }
      });

}

