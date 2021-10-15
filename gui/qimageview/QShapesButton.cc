/*
 * QShapesButton.cc
 *
 *  Created on: Oct 14, 2021
 *      Author: amyznikov
 */

#include "QShapesButton.h"
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


  connect(this, &QToolButton::toggled,
      [this](bool checked) {
        if ( sceneView_ ) {
          sceneView_->setShapesVisible(checked);
        }
      });


  popup_.addAction(line_icon,
      "Add line ...",
      [this]() {
        if ( sceneView_ ) {
          sceneView_->addLineShape();
          if ( !isChecked() ) {
            setChecked(true);
          }
          else {
            sceneView_->shapesVisible();
          }
        }
      });

  popup_.addAction(rectangle_icon,
      "Add rectangle ...",
      [this]() {
        if ( sceneView_ ) {
          sceneView_->addRectShape();
          if ( !isChecked() ) {
            setChecked(true);
          }
          else {
            sceneView_->shapesVisible();
          }
        }
      });

  popup_.addSeparator();
  popup_.addAction(delete_icon,
      "Delete all shapes...",
      [this]() {
        if ( sceneView_ ) {
          sceneView_->deleteAllShapes();
          if ( !isChecked() ) {
            setChecked(true);
          }
          else {
            sceneView_->shapesVisible();
          }
        }
      });

  setPopupMode(QToolButton::MenuButtonPopup);
  setMenu(&popup_);
}

void QShapesButton::setSceneView(QImageSceneView * sceneView)
{
  if ( !(sceneView_ = sceneView) ) {
    setEnabled(false);
  }
  else {
    setChecked(sceneView_->shapesVisible());
    setEnabled(true);
  }
}

QImageSceneView * QShapesButton::sceneView() const
{
  return sceneView_;
}
