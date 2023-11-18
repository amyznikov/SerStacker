/*
 * QGeoGraphicsItem.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoGraphicsItem.h"
#include "QGeoViewCameraState.h"
#include "QGeoView.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<QGraphicsItem::GraphicsItemChange>()
{
  static constexpr c_enum_member members[] = {
      { QGraphicsItem::ItemPositionChange, "ItemPositionChange", "ItemPositionChange" },
      #if !QT_DEPRECATED_SINCE(5, 14)
        {QGraphicsItem::ItemMatrixChange, "ItemMatrixChange", "ItemMatrixChange" },
#endif
      { QGraphicsItem::ItemVisibleChange, "ItemVisibleChange", "ItemVisibleChange" },
      { QGraphicsItem::ItemEnabledChange, "ItemEnabledChange", "ItemEnabledChange" },
      { QGraphicsItem::ItemSelectedChange, "ItemSelectedChange", "ItemSelectedChange" },
      { QGraphicsItem::ItemParentChange, "ItemParentChange", "ItemParentChange" },
      { QGraphicsItem::ItemChildAddedChange, "ItemChildAddedChange", "ItemChildAddedChange" },
      { QGraphicsItem::ItemChildRemovedChange, "ItemChildRemovedChange", "ItemChildRemovedChange" },
      { QGraphicsItem::ItemTransformChange, "ItemTransformChange", "ItemTransformChange" },
      { QGraphicsItem::ItemPositionHasChanged, "ItemPositionHasChanged", "ItemPositionHasChanged" },
      { QGraphicsItem::ItemTransformHasChanged, "ItemTransformHasChanged", "ItemTransformHasChanged" },
      { QGraphicsItem::ItemSceneChange, "ItemSceneChange", "ItemSceneChange" },
      { QGraphicsItem::ItemVisibleHasChanged, "ItemVisibleHasChanged", "ItemVisibleHasChanged" },
      { QGraphicsItem::ItemEnabledHasChanged, "ItemEnabledHasChanged", "ItemEnabledHasChanged" },
      { QGraphicsItem::ItemSelectedHasChanged, "ItemSelectedHasChanged", "ItemSelectedHasChanged" },
      { QGraphicsItem::ItemParentHasChanged, "ItemParentHasChanged", "ItemParentHasChanged" },
      { QGraphicsItem::ItemSceneHasChanged, "ItemSceneHasChanged", "ItemSceneHasChanged" },
      { QGraphicsItem::ItemCursorChange, "ItemCursorChange", "ItemCursorChange" },
      { QGraphicsItem::ItemCursorHasChanged, "ItemCursorHasChanged", "ItemCursorHasChanged" },
      { QGraphicsItem::ItemToolTipChange, "ItemToolTipChange", "ItemToolTipChange" },
      { QGraphicsItem::ItemToolTipHasChanged, "ItemToolTipHasChanged", "ItemToolTipHasChanged" },
      { QGraphicsItem::ItemFlagsChange, "ItemFlagsChange", "ItemFlagsChange" },
      { QGraphicsItem::ItemFlagsHaveChanged, "ItemFlagsHaveChanged", "ItemFlagsHaveChanged" },
      { QGraphicsItem::ItemZValueChange, "ItemZValueChange", "ItemZValueChange" },
      { QGraphicsItem::ItemZValueHasChanged, "ItemZValueHasChanged", "ItemZValueHasChanged" },
      { QGraphicsItem::ItemOpacityChange, "ItemOpacityChange", "ItemOpacityChange" },
      { QGraphicsItem::ItemOpacityHasChanged, "ItemOpacityHasChanged", "ItemOpacityHasChanged" },
      { QGraphicsItem::ItemScenePositionHasChanged, "ItemScenePositionHasChanged", "ItemScenePositionHasChanged" },
      { QGraphicsItem::ItemRotationChange, "ItemRotationChange", "ItemRotationChange" },
      { QGraphicsItem::ItemRotationHasChanged, "ItemRotationHasChanged", "ItemRotationHasChanged" },
      { QGraphicsItem::ItemScaleChange, "ItemScaleChange", "ItemScaleChange" },
      { QGraphicsItem::ItemScaleHasChanged, "ItemScaleHasChanged", "ItemScaleHasChanged" },
      { QGraphicsItem::ItemTransformOriginPointChange, "ItemTransformOriginPointChange",
          "ItemTransformOriginPointChange" },
      { QGraphicsItem::ItemTransformOriginPointHasChanged, "ItemTransformOriginPointChange",
          "ItemTransformOriginPointChange" },
      { (QGraphicsItem::GraphicsItemChange) (-1) }
  };

  return members;
}


QGeoGraphicsItem::QGeoGraphicsItem(QGraphicsItem *parent) :
  ThisClass("", "", parent)
{
}

QGeoGraphicsItem::QGeoGraphicsItem(const QString & name, const QString & description, QGraphicsItem *parent ) :
  Base(parent),
  name_(name),
  description_(description)
{
  // setFlag(ItemSendsGeometryChanges, true);
  setFlag(ItemSendsScenePositionChanges, true);
}

QGeoGraphicsItem::~QGeoGraphicsItem()
{
}

void QGeoGraphicsItem::setName(const QString & name)
{
  name_ = name;
}

const QString& QGeoGraphicsItem::name() const
{
  return name_;
}

void QGeoGraphicsItem::setDescription(const QString & description)
{
  description_ = description;
}

const QString& QGeoGraphicsItem::description() const
{
  return description_;
}

QGeoScene * QGeoGraphicsItem::geoScene() const
{
  return dynamic_cast<QGeoScene * >(this->scene());
}

QGeoProjection * QGeoGraphicsItem::projection() const
{
  QGeoScene * scene = geoScene();
  return scene ? scene->projection() : nullptr;
}

QGraphicsView* QGeoGraphicsItem::getActiveView(const QGraphicsSceneEvent * event)
{
  QWidget *widget = event ? event->widget() : nullptr;
  return widget ? qobject_cast<QGraphicsView*>(widget->parentWidget()) : nullptr;
}

void QGeoGraphicsItem::setRenderHints(QPainter::RenderHints hints, bool on)
{
  if( on ) {
    renderHintsOn_ |= hints;
  }
  else {
    renderHintsOff_ |= hints;
  }
}

QPainter::RenderHints QGeoGraphicsItem::renderHintsOn() const
{
  return renderHintsOn_;
}

QPainter::RenderHints QGeoGraphicsItem::renderHintsOff() const
{
  return renderHintsOff_;
}


QVariant QGeoGraphicsItem::itemChange(GraphicsItemChange change, const QVariant & value)
{
//  CF_DEBUG("%s: '%s' : change=%s inUpdatingPos_=%d",
//      this->metaObject()->className(),
//      this->name_.toUtf8().constData(),
//      toString(change),
//      inUpdatingPos_);

  QVariant v =
      Base::itemChange(change, value);

  switch (change) {
    case ItemFlagsChange:
      // ItemSendsGeometryChanges |
      v.setValue(v.value<quint32>() | ItemSendsScenePositionChanges);
      break;

    case ItemScenePositionHasChanged:
      if( !inUpdatingPos() ) {
        updateGeo();
      }
      break;

    case ItemSceneChange:
      if( ignoreTransformation_ ) {
        myPreviousScene_ = Base::scene();
      }
      break;

    case ItemSceneHasChanged: {
      if( ignoreTransformation_ ) {

        if( myPreviousScene_ ) {
          setParentItem(ignoreTransformation_->parentItem());
          myPreviousScene_->removeItem(ignoreTransformation_);
        }

        delete ignoreTransformation_;
        ignoreTransformation_ = nullptr;
      }

      QGraphicsScene *newScene =
          Base::scene();

      if( newScene ) {

        if( flags() & QGraphicsItem::ItemIgnoresTransformations ) {

          QGraphicsItem *myParentItem =
              this->parentItem();

          newScene->addItem(ignoreTransformation_ = new QIgnoreTransformationStub());
          setParentItem(ignoreTransformation_);
          ignoreTransformation_->setParentItem(myParentItem);
          ignoreTransformation_->setZValue(this->zValue());
        }

        updateProjected(dynamic_cast<QGeoScene*>(newScene));
      }

      break;
    }

    case ItemZValueHasChanged:
      if( ignoreTransformation_ ) {
        ignoreTransformation_->setZValue(this->zValue());
      }
      break;

    default:
      break;
  }

  return v;
}

void QGeoGraphicsItem::onCamera(const QGeoViewCameraState& oldState, const QGeoViewCameraState & newState)
{
  const bool neededUpdate =
      (flags() & QGraphicsItem::ItemIgnoresTransformations) && !qFuzzyCompare(oldState.scale(), newState.scale());

  if (neededUpdate) {
    update();
  }
}

void QGeoGraphicsItem::setUpdatingPos(bool v)
{
  if( v ) {
    ++inUpdatingPos_;
  }
  else if( inUpdatingPos_ && --inUpdatingPos_ < 0 ) {
    inUpdatingPos_ = 0;
  }
}

bool QGeoGraphicsItem::inUpdatingPos() const
{
  return inUpdatingPos_;
}

void QGeoGraphicsItem::updateProjected(const QGeoScene* geoScene)
{
}

void QGeoGraphicsItem::updateGeo(const QGeoScene* geoScene)
{
}

void QGeoGraphicsItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  if( renderHintsOn_ ) {
    painter->setRenderHints(renderHintsOn_, true);
  }
  if( renderHintsOff_ ) {
    painter->setRenderHints(renderHintsOff_, false);
  }
}

bool QGeoGraphicsItem::popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  const int n = menu.actions().count();

  Q_EMIT populateContextMenuReuested(event, &menu);

  return menu.actions().count() != n;
}


void QGeoGraphicsItem::contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
{
//    CF_DEBUG("ENTER modifiers=0x%0X isAccepted=%d",
//        (uint)event->modifiers(),
//        event->isAccepted());

  event->ignore();
  Base::contextMenuEvent(event);
  if ( event->isAccepted() ) {
    CF_DEBUG("Base::contextMenuEvent() Accepted");
    return;
  }

  QMenu menu;

  if( popuateContextMenu(event, menu) ) {
    menu.addSeparator();
  }

  QGeoScene * scene =
      geoScene();

  if ( scene && scene->popuateContextMenu(event, menu) ) {
    menu.addSeparator();
  }

  QGeoView *view =
      dynamic_cast<QGeoView*>(getActiveView(event));

  if ( view && view->popuateContextMenu(event, menu) ) {
    menu.addSeparator();
  }

  if ( !menu.isEmpty() ) {
    event->accept();
    menu.exec(event->screenPos());
  }

//  CF_DEBUG("LEAVE isAccepted=%d",
//      event->isAccepted());
}
