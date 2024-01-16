/*
 * colormap.cc
 *
 *  Created on: Mar 18, 2022
 *      Author: amyznikov
 */
#include "colormap.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<COLORMAP>()
{
  static const c_enum_member members[] = {
      { COLORMAP_GRAYS, "GRAYS", "" },
      { COLORMAP_JET, "JET", "" },
      { COLORMAP_RAINBOW, "RAINBOW", "" },
      { COLORMAP_AUTUMN, "AUTUMN", "" },
      { COLORMAP_BONE, "BONE", "" },
      { COLORMAP_WINTER, "WINTER", "" },
      { COLORMAP_OCEAN, "OCEAN", "" },
      { COLORMAP_SUMMER, "SUMMER", "" },
      { COLORMAP_SPRING, "SPRING", "" },
      { COLORMAP_COOL, "COOL", "" },
      { COLORMAP_HSV, "HSV", "" },
      { COLORMAP_PINK, "PINK", "" },
      { COLORMAP_HOT, "HOT", "" },
      { COLORMAP_PARULA, "PARULA", "" },
      { COLORMAP_MAGMA, "MAGMA", "" },
      { COLORMAP_INFERNO, "INFERNO", "" },
      { COLORMAP_PLASMA, "PLASMA", "" },
      { COLORMAP_VIRIDIS, "VIRIDIS", "" },
      { COLORMAP_CIVIDIS, "CIVIDIS", "" },
      { COLORMAP_TWILIGHT, "TWILIGHT", "" },
      { COLORMAP_TWILIGHT_SHIFTED, "TWILIGHT_SHIFTED", "" },
      { COLORMAP_TURBO, "TURBO", "" },
#if HAVE_COLORMAP_DEEPGREEN
      { COLORMAP_DEEPGREEN, "DEEPGREEN", "" },
#endif // HAVE_COLORMAP_DEEPGREEN
      { COLORMAP_NONE, "NONE", "" },
      { COLORMAP_NONE},
  };

  return members;
}



bool apply_colormap(cv::InputArray src, cv::OutputArray dst, COLORMAP cmap)
{
  if( cmap != COLORMAP_NONE ) {

    switch (cmap) {
      case COLORMAP_GRAYS: {

        static std::mutex mtx;

        mtx.lock();

        static cv::Mat3b lut;

        if( lut.empty() ) {

          lut.create(256, 1);
          for( int i = 0; i < 256; ++i ) {
            lut[i][0] = cv::Vec3b(i, i, i);
          }
        }

        mtx.unlock();

        cv::applyColorMap(src, dst, lut);
        break;
      }

      default:
        cv::applyColorMap(src, dst, cmap);
        break;
    }

    return true;
  }

  return false;
}
