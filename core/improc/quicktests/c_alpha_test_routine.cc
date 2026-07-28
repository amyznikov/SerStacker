/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>


template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_FILL_SRC_VOIDS, "FILL_SRC_VOIDS", "" },
      { c_alpha_test_routine::DISPLAY_SRC_SPECTRUM, "SRC_SPECTRUM", "" },
      { c_alpha_test_routine::DISPLAY_RADIAL_PROFILE, "RADIAL_PROFILE", "" },
      { c_alpha_test_routine::DISPLAY_FILTER, "FILTER", "" },
      { c_alpha_test_routine::DISPLAY_SRC_SPECTRUM}
  };
  return members;
}


namespace {

struct VariogramPoint {
  double h; // Distance between pixels (in pixels)
  double gamma; // Variogram value (half the variance of the difference)
  double log_h; // ln(h) for the graph axes
  double log_gamma; // ln(gamma) for the graph axes
};

/**
* @brief Calculate the experimental variogram of the image texture
* @param image Input raw image (preferably CV_32FC1, but the method converts it automatically)
* @param outputPoints Vector of points for plotting the graph
* @param maxH Maximum analysis step (default is 64 pixels; more is not needed for the midrange structure)
* @param samplesPerH Number of random pixel pairs for evaluating each distance
*/
static void computeTextureVariogram(cv::InputArray image,
    std::vector<VariogramPoint>& outputPoints,
    int maxH = 64,
    int samplesPerH = 50000)
{
    outputPoints.clear();

    // Преобразуем во float для точности и непрерывности вычислений
    cv::Mat1f img;
    if (image.depth() == CV_32F) {
        img = image.getMat();
    }
    else {
        image.getMat().convertTo(img, CV_32F);
    }

    const int width = img.cols;
    const int height = img.rows;

    // Генерируем массив шагов h по логарифмической (экспоненциальной) шкале,
    // чтобы получить равномерный шаг точек на лог-графике.
    std::vector<double> h_steps;
    double current_h = 1.0;
    const double step_multiplier = 1.15; // Шаг геометрической прогрессии
    while (current_h <= maxH) {
        h_steps.push_back(current_h);
        current_h *= step_multiplier;
        // Страховка от зависания
        if (std::abs(step_multiplier - 1.0) < 0.001) {
          break;
        }
    }

    // Настройка быстрого генератора случайных чисел
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_angle(0.0, 2.0 * M_PI);

    outputPoints.reserve(h_steps.size());

    // Основной цикл по всем дистанциям h
    for (double h : h_steps) {
        double sum_sq_diff = 0.0;
        int valid_pairs = 0;

        // Чтобы не привязываться к осям X и Y (изотропия), для каждой пары
        // мы выбираем случайную точку и случайный угол направления вектора h
        for (int s = 0; s < samplesPerH; ++s) {
            // Случайный угол направления вектора
            double angle = dist_angle(gen);
            double dx = h * std::cos(angle);
            double dy = h * std::sin(angle);

            // Определяем безопасные границы для выбора первой точки
            double min_x = std::max(0.0, -dx);
            double max_x = std::min(double(width - 1), double(width - 1) - dx);
            double min_y = std::max(0.0, -dy);
            double max_y = std::min(double(height - 1), double(height - 1) - dy);

            if (max_x <= min_x || max_y <= min_y) {
              continue;
            }

            std::uniform_real_distribution<double> dist_x(min_x, max_x);
            std::uniform_real_distribution<double> dist_y(min_y, max_y);

            // Координаты первой точки
            double x1 = dist_x(gen);
            double y1 = dist_y(gen);

            // Координаты второй точки, удаленной ровно на расстояние h под углом angle
            double x2 = x1 + dx;
            double y2 = y1 + dy;

            // Билинейная интерполяция яркости (важно для нецелочисленных h)
            // Реализация инлайном для обеспечения максимальной скорости рендеринга
            const auto getInterpolatedPix = [&](double x, double y) -> float {
                int px = static_cast<int>(x);
                int py = static_cast<int>(y);
                int ax = std::clamp(px, 0, width - 2);
                int ay = std::clamp(py, 0, height - 2);
                double fx = x - ax;
                double fy = y - ay;

                return (1.0 - fx) * (1.0 - fy) * img(ay, ax) +
                       fx * (1.0 - fy) * img(ay, ax + 1) +
                       (1.0 - fx) * fy * img(ay + 1, ax) +
                       fx * fy * img(ay + 1, ax + 1);
            };

            float val1 = getInterpolatedPix(x1, y1);
            float val2 = getInterpolatedPix(x2, y2);

            double diff = val1 - val2;
            sum_sq_diff += diff * diff;
            valid_pairs++;
        }

        if (valid_pairs > 100) {
            VariogramPoint pt;
            pt.h = h;
            // Вариаграмма по определению — это половина среднего квадрата разностей
            pt.gamma = sum_sq_diff / (2.0 * valid_pairs);
            pt.log_h = std::log(pt.h);
            // Защита логарифма от нулевой дисперсии на абсолютно пустых кадрах
            pt.log_gamma = std::log(pt.gamma + 1e-12);

            outputPoints.push_back(pt);
        }
    }
}

} // namespace

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "maxH: ", CTL_CONTEXT(ctx, _maxH), "");
  ctlbind(ctls, "samplesPerH", CTL_CONTEXT(ctx, _samplesPerH), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, _debug_file_name), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _maxH);
    SERIALIZE_OPTION(settings, save, *this, _samplesPerH);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);
    return true;
  }
  return false;
}


bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  CF_DEBUG("c_alpha_test_routine: ENTER");

  std::vector<VariogramPoint> outputPoints;

  cv::Mat src;
  extract_channel(image, src, cv::noArray(), cv::noArray(), _intensity_channel, CV_32F, false);

  computeTextureVariogram(src,
      outputPoints,
      _maxH,
      _samplesPerH);

  CF_DEBUG("outputPoints.size=%zu", outputPoints.size());

  if ( !_debug_file_name.empty() ) {

    const std::string outpath = get_parent_directory(_debug_file_name);
    if ( !create_path(outpath) ) {
      CF_ERROR("create_path(outpath='%s') fails: %s", outpath.c_str(), strerror(errno));
      return false;
    }

    c_stdio_file fp;
    if ( !fp.open(_debug_file_name, "wb") ) {
      CF_ERROR("fp.open(_debug_file_name='%s') fails: %s", _debug_file_name.c_str(), strerror(errno));
      return false;
    }

    fprintf(fp, "I\th\tgamma\tlog_h\tlog_gamma\n");
    for ( size_t i = 0, n = outputPoints.size(); i < n; ++i ) {
      const auto & p = outputPoints[i];
      fprintf(fp, "%4zu\t%12f\t%12f\t%12f\t%12f\n", i, p.h, p.gamma, p.log_h, p.log_gamma);
    }

    fp.close();
    CF_DEBUG("Saved %s", _debug_file_name.c_str());
  }

  CF_DEBUG("c_alpha_test_routine: LEAVE");
  return true;
}

