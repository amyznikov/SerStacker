/*
 * c_mtf_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_mtf_routine.h"

//float mapToParameter(double guiValue) {
//    if (guiValue >= 0) {
//        // Линейно отображаем [0, 100] -> [1.0, 5.0]
//        return 1.0f + (static_cast<float>(guiValue) / 100.0f) * 4.0f;
//    } else {
//        // Экспоненциально отображаем [-100, 0) -> [0.01, 1.0)
//        // Это даст ту самую "плавность" в районе малых значений
//        return std::pow(10.0f, static_cast<float>(guiValue) / 50.0f);
//    }
//}

//class AdaptiveSpinBox : public QDoubleSpinBox {
//protected:
//    void stepBy(int steps) override {
//        double cur = value();
//        // Если значение маленькое, делаем микро-шаг, если большое - обычный
//        double dynamicStep = (cur < 1.0) ? (cur * 0.1) : 0.1;
//        setValue(cur + steps * dynamicStep);
//    }
//};

//3. Совет по формуле (1e-9f)
//В вашей функции eval есть защита от деления на ноль 1e-9f. При очень малых shadow (близких к 0.01) и малых t, значение ts может стать сопоставимым с этой константой, что вызовет резкий "скачок" яркости в тенях.
//
//    Рекомендация: Используйте std::clamp(t, 1e-6f, 1.0f) перед расчетами или уменьшите константу до 1e-12f, если работаете с float.


//#include <cstdint>
//
//// Быстрый логарифм по основанию 2
//inline float fast_log2(float val) {
//    union { float f; uint32_t i; } vx = { val };
//    float y = (float)vx.i;
//    y *= 1.1920928955078125e-7f; // 1 / 2^23
//    return y - 126.94269504f;
//}
//
//// Быстрая экспонента по основанию 2
//inline float fast_exp2(float val) {
//    union { uint32_t i; float f; } vx;
//    vx.i = (uint32_t)((1 << 23) * (val + 126.94269504f));
//    return vx.f;
//}
//
//// Итоговая функция x^y
//inline float epow_fast(float x, float y) {
//    // x^y = 2^(y * log2(x))
//    return fast_exp2(y * fast_log2(x));
//}
//

//#include <cstdint>
//
///**
// * Быстрое возведение x^y для x в диапазоне (0..1].
// * Точность: ~0.001 (0.1%).
// * Скорость: в 5-10 раз быстрее std::powf.
// */
//inline float epow_fast(float x, float y) {
//    // Шаг 1: Быстрый log2(x) через работу с экспонентой float
//    union { float f; uint32_t i; } vx = { x };
//    // Превращаем биты float в логарифмическую шкалу
//    float log2_x = (float)vx.i * 1.1920928955e-7f - 126.942695f;
//
//    // Шаг 2: Умножаем на показатель степени
//    float intermediate = y * log2_x;
//
//    // Шаг 3: Быстрая exp2(intermediate)
//    union { uint32_t i; float f; } vy;
//    vy.i = (uint32_t)((intermediate + 126.942695f) * 8388608.0f);
//
//    return vy.f;
//}

bool c_mtf_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, inputRange);
    SERIALIZE_OPTION(settings, save, *this, outputRange);
    SERIALIZE_OPTION(settings, save, *this, lclip);
    SERIALIZE_OPTION(settings, save, *this, hclip);
    SERIALIZE_OPTION(settings, save, *this, shadows);
    SERIALIZE_OPTION(settings, save, *this, highlights);
    SERIALIZE_OPTION(settings, save, *this, midtones);
    return true;
  }
  return false;
}

void c_mtf_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "clip input range", ctx(&this_class::inputRange), "");
  ctlbind(ctls, "stretch output range", ctx(&this_class::outputRange), "");
  ctlbind_slider_spinbox(ctls, "lclip", ctx(&this_class::lclip), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "shadows", ctx(&this_class::shadows), 0.0, 5.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "midtones", ctx(&this_class::midtones), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "highlights", ctx(&this_class::highlights), 0.0, 5.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "hclip", ctx(&this_class::hclip), 0.0, 1.0, 0.001, "");
}


bool c_mtf_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_mtf_options opts;

  opts.lclip = lclip;
  opts.hclip = hclip;
  opts.shadows = shadows;
  opts.highlights = highlights;
  opts.midtones = midtones;

  if( !(inputRange[1] > inputRange[0]) ) {
    double minv = 0, maxv = 1;
    cv::minMaxLoc(image, &minv, &maxv);
    inputRange[0] = minv, inputRange[1] = maxv;
  }

  if( !(outputRange[1] > outputRange[0]) ) {
    c_mtf::suggest_levels_range(image.depth(), &outputRange[0], &outputRange[1]);
  }

  mtf.set_input_range(inputRange[0], inputRange[1]);
  mtf.set_output_range(outputRange[0], outputRange[1]);
  mtf.set_opts(opts);

  return mtf.apply(image.getMat(), image);
}

