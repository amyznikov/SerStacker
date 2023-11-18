/*
 * QGeoTilesBing.cc
 *
 *  Created on: Nov 29, 2022
 *      Author: amyznikov
 *
 *  Copied from https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoTilesBing.h"

namespace {
// clang-format off
const QMap<QGeoTilesBing::TilesType, QStringList> URLTemplates = {
    { QGeoTilesBing::TilesType::Satellite, {
        "http://t0.tiles.virtualearth.net/tiles/a${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t1.tiles.virtualearth.net/tiles/a${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t2.tiles.virtualearth.net/tiles/a${qk}.jpeg?g=181&mkt=${lcl}", }
    },
    { QGeoTilesBing::TilesType::Schema, {
        "http://t0.tiles.virtualearth.net/tiles/r${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t1.tiles.virtualearth.net/tiles/r${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t2.tiles.virtualearth.net/tiles/r${qk}.jpeg?g=181&mkt=${lcl}", }
    },
    { QGeoTilesBing::TilesType::Hybrid, {
        "http://t0.tiles.virtualearth.net/tiles/h${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t1.tiles.virtualearth.net/tiles/h${qk}.jpeg?g=181&mkt=${lcl}",
        "http://t2.tiles.virtualearth.net/tiles/h${qk}.jpeg?g=181&mkt=${lcl}", }
    },
};
// clang-format on
}



QGeoTilesBing::QGeoTilesBing(TilesType type, const QLocale & locale, int serverNumber) :
    tilesType_(type),
    locale_(locale),
    serverNumber_(serverNumber)
{
  updateName();
  setDescription("Copyrights ©Microsoft");
}

void QGeoTilesBing::setTilesType(TilesType type)
{
  tilesType_ = type;
  updateName();
}

QGeoTilesBing::TilesType QGeoTilesBing::tilesType() const
{
  return tilesType_;
}

void QGeoTilesBing::setLocale(const QLocale& locale)
{
  locale_ = locale;
  updateName();
}

const QLocale & QGeoTilesBing::locale() const
{
  return locale_;
}

void QGeoTilesBing::updateName()
{
  // clang-format off
  // clang-format off
  const QMap<TilesType, QString> adapter = {
      { TilesType::Satellite, "Bing::Satellite" },
      { TilesType::Schema, "Bing::Schema" },
      { TilesType::Hybrid, "Bing::Hybrid" },
  };
  // clang-format on
  setName("Bing Maps (" + adapter[tilesType_] + " " + locale_.name() + ")");
}

int QGeoTilesBing::minZoomlevel() const
{
  return 0;
}

int QGeoTilesBing::maxZoomlevel() const
{
  return 19;
}

QString QGeoTilesBing::tilePosToUrl(const QGeoTilePos & tilePos) const
{

  const QStringList &list =
      URLTemplates[tilesType_];

  QString url =
      list.value(serverNumber_).toLower();

  url.replace("${lcl}", locale_.name());
  url.replace("${qk}", tilePos.toQuadKey());

  return url;
}
