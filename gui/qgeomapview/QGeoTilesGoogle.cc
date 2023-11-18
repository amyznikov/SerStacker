/*
 * QGeoTilesGoogle.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Copied from https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoTilesGoogle.h"


namespace {

// clang-format off
const QMap<QGeoTilesGoogle::TilesType, QStringList> URLTemplates = {
    { QGeoTilesGoogle::TilesType::Satellite, {
        "https://mts0.google.com/vt/lyrs=s@186112443&hl=${lcl}&x=${x}&y=${y}&z=${z}&s=Galile",
        "https://mts1.google.com/vt/lyrs=s@186112443&hl=${lcl}&x=${x}&y=${y}&z=${z}&s=Galile",
        "https://mts2.google.com/vt/lyrs=s@186112443&hl=${lcl}&x=${x}&y=${y}&z=${z}&s=Galile",
    }
    },
    { QGeoTilesGoogle::TilesType::Schema, {
        "http://mt1.google.com/vt/lyrs=m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
        "http://mt2.google.com/vt/lyrs=m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
        "http://mt3.google.com/vt/lyrs=m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
    }
    },
    { QGeoTilesGoogle::TilesType::Hybrid, {
        "http://mt1.google.com/vt/lyrs=s,m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
        "http://mt2.google.com/vt/lyrs=s,m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
        "http://mt3.google.com/vt/lyrs=s,m@110&hl=${lcl}&x=${x}&y=${y}&z=${z}",
    }
    },
};
// clang-format on
}

QGeoTilesGoogle::QGeoTilesGoogle(TilesType type, const QLocale & locale, int serverNumber) :
    tilesType_(type),
    locale_(locale),
    serverNumber_(serverNumber)
{
  updateName();
  setDescription("Copyrights ©Google");
}

void QGeoTilesGoogle::setTilesType(TilesType type)
{
  tilesType_ = type;
  updateName();
}

QGeoTilesGoogle::TilesType QGeoTilesGoogle::tilesType() const
{
  return tilesType_;
}

void QGeoTilesGoogle::setLocale(const QLocale& locale)
{
  locale_ = locale;
  updateName();
}

const QLocale & QGeoTilesGoogle::locale() const
{
  return locale_;
}

void QGeoTilesGoogle::updateName()
{
  // clang-format off
  const QMap<TilesType, QString> adapter = {
      { TilesType::Satellite, "Google::Satellite" },
      { TilesType::Schema, "Google::Schema" },
      { TilesType::Hybrid, "Google::Hybrid" },
  };
  // clang-format on
  setName("Google Maps (" + adapter[tilesType_] + " " + locale_.name() + ")");
}

int QGeoTilesGoogle::minZoomlevel() const
{
  return 0;
}

int QGeoTilesGoogle::maxZoomlevel() const
{
  return 21;
}

QString QGeoTilesGoogle::tilePosToUrl(const QGeoTilePos & tilePos) const
{
  const QStringList &list =
      URLTemplates[tilesType_];

  QString url =
      list.value(serverNumber_).toLower();

  url.replace("${lcl}", locale_.name());
  url.replace("${z}", QString::number(tilePos.zoom()));
  url.replace("${x}", QString::number(tilePos.pos().x()));
  url.replace("${y}", QString::number(tilePos.pos().y()));

  return url;
}


