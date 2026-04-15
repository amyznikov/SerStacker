/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <cmath>
#include <array>
#include <core/ssprintf.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/levmar.h>
#include <core/proc/levmar3.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>
//#include <gui/widgets/QSettingsWidget.h>

#include <core/debug.h>


int main(int argc, char *argv[])
{
  uint8_t a = 100, b = 200;
  uint8_t c = (a + b) / 2;

  printf("a=%u b=%u c=%u\n", a, b, c);

  uint8_t x1 = 150, x2 = 150, x3 = 150, x4 = 200;
  uint8_t y = (x1 + x2 + x3 + x4) / 4;

  printf("x1=%u x2=%u x3=%u x4=%u y=%u\n", x1, x2, x3, x4, y);

  return 0;
}
