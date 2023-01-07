/*
 * colormap.h
 *
 *  Created on: Mar 18, 2022
 *      Author: amyznikov
 */

#ifndef __colormap_h__
#define __colormap_h__

#include <opencv2/opencv.hpp>


// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) ((a)<<16 | (b)<<8 | (c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,4,0) )
  #define HAVE_COLORMAP_DEEPGREEN 1
#endif

enum COLORMAP {
  COLORMAP_NONE = -1,

  COLORMAP_AUTUMN = cv::COLORMAP_AUTUMN, //!< ![autumn](pics/colormaps/colorscale_autumn.jpg)
  COLORMAP_BONE = cv::COLORMAP_BONE, //!< ![bone](pics/colormaps/colorscale_bone.jpg)
  COLORMAP_JET = cv::COLORMAP_JET, //!< ![jet](pics/colormaps/colorscale_jet.jpg)
  COLORMAP_WINTER = cv::COLORMAP_WINTER, //!< ![winter](pics/colormaps/colorscale_winter.jpg)
  COLORMAP_RAINBOW = cv::COLORMAP_RAINBOW, //!< ![rainbow](pics/colormaps/colorscale_rainbow.jpg)
  COLORMAP_OCEAN = cv::COLORMAP_OCEAN, //!< ![ocean](pics/colormaps/colorscale_ocean.jpg)
  COLORMAP_SUMMER = cv::COLORMAP_SUMMER, //!< ![summer](pics/colormaps/colorscale_summer.jpg)
  COLORMAP_SPRING = cv::COLORMAP_SPRING, //!< ![spring](pics/colormaps/colorscale_spring.jpg)
  COLORMAP_COOL = cv::COLORMAP_COOL, //!< ![cool](pics/colormaps/colorscale_cool.jpg)
  COLORMAP_HSV = cv::COLORMAP_HSV, //!< ![HSV](pics/colormaps/colorscale_hsv.jpg)
  COLORMAP_PINK = cv::COLORMAP_PINK, //!< ![pink](pics/colormaps/colorscale_pink.jpg)
  COLORMAP_HOT = cv::COLORMAP_HOT, //!< ![hot](pics/colormaps/colorscale_hot.jpg)
  COLORMAP_PARULA = cv::COLORMAP_PARULA, //!< ![parula](pics/colormaps/colorscale_parula.jpg)
  COLORMAP_MAGMA = cv::COLORMAP_MAGMA, //!< ![magma](pics/colormaps/colorscale_magma.jpg)
  COLORMAP_INFERNO = cv::COLORMAP_INFERNO, //!< ![inferno](pics/colormaps/colorscale_inferno.jpg)
  COLORMAP_PLASMA = cv::COLORMAP_PLASMA, //!< ![plasma](pics/colormaps/colorscale_plasma.jpg)
  COLORMAP_VIRIDIS = cv::COLORMAP_VIRIDIS, //!< ![viridis](pics/colormaps/colorscale_viridis.jpg)
  COLORMAP_CIVIDIS = cv::COLORMAP_CIVIDIS, //!< ![cividis](pics/colormaps/colorscale_cividis.jpg)
  COLORMAP_TWILIGHT = cv::COLORMAP_TWILIGHT, //!< ![twilight](pics/colormaps/colorscale_twilight.jpg)
  COLORMAP_TWILIGHT_SHIFTED = cv::COLORMAP_TWILIGHT_SHIFTED, //!< ![twilight shifted](pics/colormaps/colorscale_twilight_shifted.jpg)
  COLORMAP_TURBO = cv::COLORMAP_TURBO, //!< ![turbo](pics/colormaps/colorscale_turbo.jpg)
#if HAVE_COLORMAP_DEEPGREEN
  COLORMAP_DEEPGREEN = cv::COLORMAP_DEEPGREEN, //!< ![deepgreen](pics/colormaps/colorscale_deepgreen.jpg)
#endif // HAVE_COLORMAP_DEEPGREEN

  COLORMAP_GRAYS = 100,

};

bool apply_colormap(cv::InputArray src,
    cv::OutputArray dst,
    COLORMAP cmap);


#endif /* __colormap_h__ */
