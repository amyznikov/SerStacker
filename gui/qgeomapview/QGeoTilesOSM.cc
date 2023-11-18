/*
 * QGeoTilesOSM.cc
 *
 *  Created on: Nov 29, 2022
 *      Author: amyznikov
 *
 *  Copied from https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoTilesOSM.h"

namespace {
// clang-format off
const QStringList URLTemplates = {
    "http://a.tile.openstreetmap.org/${z}/${x}/${y}.png",
    "http://b.tile.openstreetmap.org/${z}/${x}/${y}.png",
    "http://c.tile.openstreetmap.org/${z}/${x}/${y}.png",
};
// clang-format on
}

QGeoTilesOSM::QGeoTilesOSM(int serverNumber) :
    url_(URLTemplates.value(serverNumber))
{
  setName("OpenStreetMap");
  setDescription("Copyrights ©OpenStreetMap");
}

QGeoTilesOSM::QGeoTilesOSM(const QString & url) :
    url_(url)
{
  setName("Custom");
  setDescription("OSM-like map");
}

void QGeoTilesOSM::setUrl(const QString & url)
{
  url_ = url;
}

const QString& QGeoTilesOSM::getUrl() const
{
  return url_;
}

int QGeoTilesOSM::minZoomlevel() const
{
  return 0;
}

int QGeoTilesOSM::maxZoomlevel() const
{
  return 20;
}

QString QGeoTilesOSM::tilePosToUrl(const QGeoTilePos & tilePos) const
{
  QString url =
      url_.toLower();

  url.replace("${z}", QString::number(tilePos.zoom()));
  url.replace("${x}", QString::number(tilePos.pos().x()));
  url.replace("${y}", QString::number(tilePos.pos().y()));

  return url;
}
