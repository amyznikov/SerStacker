/*
 * QGeoMapOnlineItem.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoMapOnlineItem_h__
#define __QGeoMapOnlineItem_h__

#include <QtNetwork/QtNetwork>

class QGeoMapOnlineItem
{
public:
  QGeoMapOnlineItem();
  virtual ~QGeoMapOnlineItem() = default;

  QNetworkAccessManager* networkManager();
};

#endif /* __QGeoMapOnlineItem_h__ */
