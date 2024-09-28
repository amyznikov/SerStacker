/*
 * QGeoMapOnlineTilesLayer.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoMapOnlineTiles.h"
#include <core/debug.h>
#include "QGeoGraphicsImageItem.h"

QGeoMapOnlineTiles::QGeoMapOnlineTiles()
{
  connect(networkManager(), &QNetworkAccessManager::finished,
      this, &ThisClass::onReplyFinished);
}

QGeoMapOnlineTiles::~QGeoMapOnlineTiles()
{
  qDeleteAll(mRequest);
}

void QGeoMapOnlineTiles::request(const QGeoTilePos & tilePos)
{
  const QUrl url(tilePosToUrl(tilePos));

  QNetworkRequest request(url);

  request.setRawHeader("User-Agent",
      "Mozilla/5.0 (Windows; U; MSIE "
          "6.0; Windows NT 5.1; SV1; .NET "
          "CLR 2.0.50727)");

  request.setAttribute(QNetworkRequest::HttpPipeliningAllowedAttribute, true);
  request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);

  QNetworkReply *reply = networkManager()->get(request);
  reply->setProperty("TILE_OWNER", QVariant::fromValue(this));
  reply->setProperty("TILE_REQUEST", true);
  reply->setProperty("TILE_POS", QVariant::fromValue(tilePos));

  mRequest[tilePos] = reply;

  //CF_DEBUG("request %s", url.toString().toUtf8().constData());
}

void QGeoMapOnlineTiles::cancel(const QGeoTilePos & tilePos)
{
  removeReply(tilePos);
}

void QGeoMapOnlineTiles::onReplyFinished(QNetworkReply * reply)
{
  const bool isTileRequest =
      reply->property("TILE_REQUEST").toBool();

  if( !isTileRequest ) {
    return;
  }

  const ThisClass *tileOwner =
      reply->property("TILE_OWNER").value<ThisClass*>();

  if( tileOwner != this ) {
    return;
  }

  const QGeoTilePos tilePos =
      reply->property("TILE_POS").value<QGeoTilePos>();

  if( reply->error() != QNetworkReply::NoError ) {
    if( reply->error() != QNetworkReply::OperationCanceledError ) {
      CF_ERROR("ERROR : %s ", reply->errorString().toUtf8().constData());
    }
    removeReply(tilePos);
    return;
  }

  QGeoPixmapItem *tile =
      new QGeoPixmapItem(tilePos.toGeoRect(),
          QPixmap::fromImage(QImage::fromData(
              reply->readAll())));

  tile->setRenderHints(QPainter::SmoothPixmapTransform, true);

  removeReply(tilePos);
  onTile(tilePos, tile);
}

void QGeoMapOnlineTiles::removeReply(const QGeoTilePos & tilePos)
{
  QNetworkReply *reply =
      mRequest.value(tilePos, nullptr);

  if( reply ) {
    mRequest.remove(tilePos);
    reply->abort();
    reply->close();
    reply->deleteLater();
  }
}
