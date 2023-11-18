/*
 * QGeoTiles.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoTiles.h"
#include "QGeoView.h"
#include <core/ssprintf.h>
#include <core/debug.h>
#include "QGeoScene.h"

namespace {
constexpr int minMargin = 1;
constexpr int maxMargin = 3;
constexpr int msAnimationUpdateDelay = 250;
}

QGeoTiles::QGeoTiles(QGraphicsItem * parent) :
    Base(parent)
{
  // sendToBack();
}

QGeoTiles::QGeoTiles(const QString & name, const QString & description, QGraphicsItem * parent) :
    Base(name, description, parent)
{
  // sendToBack();
}

int QGeoTiles::scaleToZoom(double scale) const
{
  const double scaleChange = 1 / scale;
  const int newZoom = qRound((17.0 - qLn(scaleChange) * M_LOG2E));
  return newZoom;
}

QRectF QGeoTiles::boundingRect() const
{
  // empty
  return QRectF();
}

void QGeoTiles::paint(QPainter * /*painter*/, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/)
{
  // empty
}

QVariant QGeoTiles::itemChange(GraphicsItemChange change, const QVariant & value)
{
  const QVariant v =
      Base::itemChange(change, value);

  switch (change) {
    case ItemVisibleHasChanged:

      if( !isVisible() ) { // remove non-visible tiles

        for( int z = minZoomlevel(), zmax = maxZoomlevel(); z <= zmax; ++z ) {
          for( const QGeoTilePos &target : existingTiles(z) ) {
            removeTile(target);
          }
        }

      }
      else { // update visible tiles

        QTimer::singleShot(0, [this]() {

          QGeoScene * scene = geoScene();
          if ( scene ) {

            const QList <QGraphicsView *> views =
                scene->views();

            for ( QGraphicsView * view : views ) {

              QGeoView * geoview =
                  dynamic_cast<QGeoView * >(view);

              if ( geoview ) {
                processCamera(geoview->getCameraState(), true);
              }
            }
          }

        });
      }

      break;

    default:
      break;
  }

  return v;
}

void QGeoTiles::onCamera(const QGeoViewCameraState & oldState, const QGeoViewCameraState & newState)
{
  Base::onCamera(oldState, newState);

  if( oldState != newState ) {

    bool needUpdate =
        true;

    if( newState.animation() ) {
      if( !mLastAnimation.isValid() ) {
        mLastAnimation.start();
      }
      else if( mLastAnimation.elapsed() < msAnimationUpdateDelay ) {
        needUpdate = false;
      }
      else {
        mLastAnimation.restart();
      }
    }
    else {
      mLastAnimation.invalidate();
    }

    if( needUpdate ) {
      processCamera(newState);
    }
  }
}

void QGeoTiles::processCamera(const QGeoViewCameraState & newCameraState, bool force)
{
  QGeoScene *geoScene =
      dynamic_cast<QGeoScene*>(Base::scene());

  if( !geoScene ) {
    return;
  }

  const QGeoProjection *projection =
      geoScene->projection();

  const QRectF areaProjRect =
      newCameraState.projRect().intersected(
          projection->boundaryProjRect());

  const QGeoRect areaGeoRect =
      projection->projToGeo(areaProjRect);

  const int originZoom =
      scaleToZoom(newCameraState.scale());

  const int newZoom =
      qMin(maxZoomlevel(), qMax(minZoomlevel(), originZoom));

  if( newZoom != originZoom ) {
    return;
  }

  const bool zoomChanged =
      (currentZoom_ != newZoom);

  currentZoom_ = newZoom;

  const int margin =
      (zoomChanged) ? minMargin : maxMargin;

  const int sizePerZoom =
      static_cast<int>(qPow(2, currentZoom_));

  const QRect maxRect =
      QRect(QPoint(0, 0), QPoint(sizePerZoom, sizePerZoom));

  const QPoint topLeft =
      QGeoTilePos::geoToTilePos(currentZoom_, areaGeoRect.topLeft()).pos();

  const QPoint bottomRight =
      QGeoTilePos::geoToTilePos(currentZoom_, areaGeoRect.bottomRight()).pos();

  const QRect activeRect =
      QRect(topLeft, bottomRight)
          .adjusted(-margin, -margin, margin, margin)
          .intersected(maxRect);

  const bool rectChanged =
      (!zoomChanged && (currentRect_ != activeRect));

  currentRect_ = activeRect;

  if( !force && !zoomChanged && !rectChanged ) {
    return;
  }

  if( zoomChanged ) {

    const int fromZoom =
        minZoomlevel();

    const int toZoom =
        maxZoomlevel();

    for( int zoom = fromZoom; zoom <= toZoom; ++zoom ) {

      if( zoom == currentZoom_ ) {
        for( const QGeoTilePos &current : existingTiles(zoom) ) {
          removeAllAbove(current);
        }
        continue;
      }

      for( const QGeoTilePos &nonCurrent : existingTiles(zoom) ) {

        if( !isTileFinished(nonCurrent) ) {
          removeTile(nonCurrent);
          continue;
        }

        if( zoom < currentZoom_ ) {
          removeWhenCovered(nonCurrent);
          continue;
        }
      }
    }
  }

  if( rectChanged ) {

    for( const QGeoTilePos &tilePos : existingTiles(currentZoom_) ) {
      if( !currentRect_.contains(tilePos.pos()) ) {
        removeTile(tilePos);
      }
    }
  }

  if ( isVisible() ) {

    QList<QGeoTilePos> missing;

    for( int x = currentRect_.left(); x < currentRect_.right(); ++x ) {
      for( int y = currentRect_.top(); y < currentRect_.bottom(); ++y ) {

        const QGeoTilePos tilePos(currentZoom_,
            QPoint(x, y));

        if( !isTileExists(tilePos) ) {
          missing.append(tilePos);
        }
      }
    }

    for( const QGeoTilePos &tilePos : missing ) {
      addTile(tilePos, nullptr);
    }
  }

}

void QGeoTiles::removeAllAbove(const QGeoTilePos & tilePos)
{
  const int fromZoom =
      tilePos.zoom() + 1;

  const int toZoom =
      maxZoomlevel();

  for( int zoom = fromZoom; zoom <= toZoom; ++zoom ) {
    for( const QGeoTilePos &target : existingTiles(zoom) ) {

      if( tilePos.contains(target) ) {
        removeTile(target);
      }
    }
  }
}

void QGeoTiles::removeWhenCovered(const QGeoTilePos & tilePos)
{
  const int zoomDelta =
      currentZoom_ - tilePos.zoom() + 1;

  const int neededCount =
      static_cast<int>(qPow(2, zoomDelta));

  int count =
      neededCount;

  for( const QGeoTilePos &current : existingTiles(currentZoom_) ) {
    if( !tilePos.contains(current) ) {
      continue;
    }
    if( !isTileFinished(current) ) {
      break;
    }
    count--;
    if( count == 0 ) {
      break;
    }
  }

  if( !count ) {
    removeTile(tilePos);
  }
}

void QGeoTiles::addTile(const QGeoTilePos & tilePos, QGraphicsItem * tileObj)
{
  if( isTileFinished(tilePos) ) {
    delete tileObj;
    return;
  }

  mIndex[tilePos.zoom()][tilePos] = tileObj;

  if( !tileObj ) {
    request(tilePos);
  }
  else {
    tileObj->setZValue(static_cast<qint16>(tilePos.zoom()));
    scene()->addItem(tileObj);
  }
}

void QGeoTiles::removeTile(const QGeoTilePos & tilePos)
{
  QGraphicsItem * tile =
      mIndex[tilePos.zoom()].take(tilePos);

  if( !tile ) {
    cancel(tilePos);
  }
  else {
    scene()->removeItem(tile);
    delete tile;
  }
}

bool QGeoTiles::isTileExists(const QGeoTilePos & tilePos) const
{
  return mIndex[tilePos.zoom()].contains(tilePos);
}

bool QGeoTiles::isTileFinished(const QGeoTilePos & tilePos) const
{
  if( !isTileExists(tilePos) ) {
    return false;
  }
  return mIndex[tilePos.zoom()][tilePos] != nullptr;
}

QList<QGeoTilePos> QGeoTiles::existingTiles(int zoom) const
{
  return mIndex[zoom].keys();
}

void QGeoTiles::onTile(const QGeoTilePos & tilePos, QGraphicsItem * tileObj)
{
  if( tilePos.zoom() != currentZoom_ || !currentRect_.contains(tilePos.pos()) ) {
    delete tileObj;
    return;
  }

  addTile(tilePos, tileObj);

  removeAllAbove(tilePos);

  const int fromZoom =
      minZoomlevel();

  const int toZoom =
      tilePos.zoom() - 1;

  for( int zoom = fromZoom; zoom <= toZoom; ++zoom ) {
    for( const QGeoTilePos &below : existingTiles(zoom) ) {
      if( below.contains(tilePos) ) {
        removeWhenCovered(below);
      }
    }
  }
}
