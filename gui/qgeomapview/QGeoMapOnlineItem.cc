/*
 * QGeoMapOnlineItem.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoMapOnlineItem.h"

QGeoMapOnlineItem::QGeoMapOnlineItem()
{
}


QNetworkAccessManager* QGeoMapOnlineItem::networkManager()
{
  static QNetworkAccessManager *networkAccessManager_;
  static QNetworkDiskCache *networkDiskCache_;

  if( !networkAccessManager_ ) {
    // QDir("cacheDir").removeRecursively();
    networkDiskCache_ = new QNetworkDiskCache(qApp);
    networkDiskCache_->setCacheDirectory("QGeoMap/NetworkCache");
    networkAccessManager_ = new QNetworkAccessManager(qApp);
    networkAccessManager_->setCache(networkDiskCache_);
  }

  return networkAccessManager_;
}
