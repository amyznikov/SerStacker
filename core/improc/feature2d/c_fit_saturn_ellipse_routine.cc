/*
 * c_fit_saturn_ellipse_routine.cc
 *
 *  Created on: Jun 4, 2026
 *      Author: amyznikov
 */

#include "c_fit_saturn_ellipse_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_fit_saturn_ellipse_routine::display_type>()
{
  static const c_enum_member members[] = {
      { c_fit_saturn_ellipse_routine::display_final_fit, "final_fit", },
      { c_fit_saturn_ellipse_routine::display_gray_image, "gray_image", },
      { c_fit_saturn_ellipse_routine::display_initial_mask, "initial_mask", },
      { c_fit_saturn_ellipse_routine::display_pca_mask, "pca_mask", },
      { c_fit_saturn_ellipse_routine::display_pca_fit, "pca_fit", },
      { c_fit_saturn_ellipse_routine::display_gradient_image, "gradient_image", },
      { c_fit_saturn_ellipse_routine::display_gx, "gx", },
      { c_fit_saturn_ellipse_routine::display_gy, "gy", },
      { c_fit_saturn_ellipse_routine::display_gr, "gr", },
      { c_fit_saturn_ellipse_routine::display_grth, "grth", },
      { c_fit_saturn_ellipse_routine::display_grth_fit, "grth_fit", },
      { c_fit_saturn_ellipse_routine::display_grid, "grid", },
      { c_fit_saturn_ellipse_routine::display_debug_image, "debug_image", },

      { c_fit_saturn_ellipse_routine::display_final_fit, },
  };

  return members;
}



void c_fit_saturn_ellipse_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display_type), "Select image to display");
  ctlbind(ctls, ctx(&this_class::_opts));
  ctlbind_expandable_group(ctls, "grid options", [&, ctx = CTL_CONTEXT(ctx, _grid)]() {
      ctlbind(ctls, "lat_step [deg]", CTL_CONTEXT(ctx, lat_step_deg), "" );
      ctlbind(ctls, "lon_step [deg]", CTL_CONTEXT(ctx, lon_step_deg), "" );
  });

  ctlbind_menu_button(ctls, "Options >>", ctx);
  ctlbind_item(ctls, "Copy pose", ctx, [](const this_class * ths) {
    if ( const auto & cb = get_ctlbind_copy_to_clipboard_callback() ) {
      cb(serialize_ellipsoid_to_string(ths->_detector.center(), ths->_detector.axes(), ths->_detector.pose() * 180 / CV_PI));
    }
    return false;
  });
  ctlbind_item(ctls, "Copy parameters", ctx, [](const auto * ths) {
    return ctlbind_copy_config_to_clipboard("c_saturn_ellipse_detector_options", ths->_opts), false;
  });
  ctlbind_item(ctls, "Paste parameters", ctx, [](auto * ths) {
    return ctlbind_paste_config_from_clipboard("c_saturn_ellipse_detector_options", &ths->_opts);
  });
}

bool c_fit_saturn_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, _grid, lat_step_deg);
    SERIALIZE_OPTION(settings, save, _grid, lon_step_deg);
    serialize_base_saturn_ellipse_detector_options(settings, save, _opts);
    return true;
  }
  return false;
}

static void drawRotatedRect(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{
  if ( 0 ) {
    cv::rectangle(image, ellipse_bounding_box(rc),
        cv::Scalar(0, 0, 1),
        1);
  }

  cv::Point2f pts[4];
  rc.points(pts);


  for( int i = 0; i < 4; i++ ) {
    cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
  }

  cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
  cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
}

bool c_fit_saturn_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  static const auto genrgb =
      [](cv::InputArray img, double r, double g, double b) -> cv::Scalar {
        double minv = 0, maxv = 1;
        cv::minMaxLoc(img, &minv, &maxv);
        return CV_RGB(1.05 * r * maxv, 1.05 * g * maxv, 1.05 * b * maxv);
      };


  _detector.set_options(_opts);
  _detector.detect(image, mask);

  switch (_display_type) {
    case display_gray_image:
      _detector.grayscale_image().copyTo(image);
      //_detector.pca_mask().copyTo(mask);
      break;

    case display_initial_mask:
      _detector.initial_mask().copyTo(image);
      mask.release();
      break;

    case display_pca_mask:
      _detector.pca_mask().copyTo(image);
      mask.release();
      break;

    case display_pca_fit: {
      drawRotatedRect(image, _detector.pca_rect(), genrgb(image, 1, 0, 0), 1);
      cv::ellipse(image, _detector.pca_rect(), genrgb(image, 0, 0, 1), 1);
      mask.release();
      break;
    }

    case display_gradient_image:
      _detector.gradient_image().copyTo(image);
      _detector.skirt_mask().copyTo(mask);
      break;

    case display_gx: {
      _detector.gx_image().copyTo(image);
      _detector.skirt_mask().copyTo(mask);
      break;
    }
    case display_gy: {
      _detector.gy_image().copyTo(image);
      _detector.skirt_mask().copyTo(mask);
      break;
    }

    case display_gr: {
      _detector.gr_image().copyTo(image);
      _detector.skirt_mask().copyTo(mask);
      break;
    }

    case display_grth: {
      cv::cvtColor(_detector.grth_image(), image, cv::COLOR_GRAY2BGR);
      _detector.skirt_mask().copyTo(mask);
      break;
    }

    case display_grth_fit: {
      cv::cvtColor(_detector.grth_image(), image, cv::COLOR_GRAY2BGR);
      _detector.skirt_mask().copyTo(mask);
      draw_ellipse(image, _detector.final_planetary_disk_ellipse(), genrgb(image, 0.5, 0, 0), 1, cv::LINE_8);
      break;
    }

    case display_grid: {
      draw_ellipsoid(image, _detector.center(), _detector.axes(), _detector.pose(),
          _grid.lat_step_deg * CV_PI / 180, _grid.lon_step_deg * CV_PI / 180,
          genrgb(image, 1, 1, 1), 1, cv::LINE_AA);
      break;
    }

    case display_final_fit: {
      if( image.channels() == 1 ) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      }
      drawRotatedRect(image, _detector.pca_rect(), genrgb(image, 1, 0, 0), 1);
      cv::ellipse(image, _detector.final_planetary_disk_ellipse(), genrgb(image, 0, 0, 1), 1);
      mask.release();
      break;
    }

    case display_debug_image: {
      _detector.debugImage().copyTo(image);
      //mask.release();
      break;
    }

  }
  return true;
}
