/*
 * c_keypoins_detector_routine.cc
 *
 *  Created on: Aug 9, 2023
 *      Author: amyznikov
 */

#include "c_keypoins_detector_routine.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/pose.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<c_keypoins_detector_routine::DisplayType>()
{
  static const c_enum_member members[] = {
    {c_keypoins_detector_routine::DisplayRichKeypoints, "RichKeypoints", "Display Rich Keypoints"},
    {c_keypoins_detector_routine::DisplayTestHomography, "TestHomography", "Display Test Homography"},
    {c_keypoins_detector_routine::DisplayRichKeypoints}
  };

  return members;
}

static cv::Matx34f create_perspective_transform(const cv::Vec3f & A, const cv::Vec3f & T, float F, const cv::Point2f & cameraCenter)
{
  const cv::Matx44f cameraMatrix(
      F, 0, cameraCenter.x, 0,
      0, F, cameraCenter.y, 0,
      0, 0, 1,              0,
      0, 0, 0,              1
      );

  const cv::Matx44f cameraMatrixInv =
      cameraMatrix.inv();

  const cv::Matx33f R =
      build_rotation(A);

  const cv::Matx44f RT(
      R(0, 0), R(0, 1), R(0, 2), -T(0),
      R(1, 0), R(1, 1), R(1, 2), -T(1),
      R(2, 0), R(2, 1), R(2, 2), -T(2),
      0,        0,        0,       1);

  cv::Matx44f HH =
      cameraMatrix * RT * cameraMatrixInv;


  CF_DEBUG("HH: {\n"
      "%+15.6f %+15.6f %+15.6f %+15.6f\n"
      "%+15.6f %+15.6f %+15.6f %+15.6f\n"
      "%+15.6f %+15.6f %+15.6f %+15.6f\n"
      "%+15.6f %+15.6f %+15.6f %+15.6f\n"
      "\n",
      HH(0,0), HH(0,1), HH(0,2), HH(0,3),
      HH(1,0), HH(1,1), HH(1,2), HH(1,3),
      HH(2,0), HH(2,1), HH(2,2), HH(2,3),
      HH(3,0), HH(3,1), HH(3,2), HH(3,3)
  );

  return cv::Matx34f(
      HH(0,0), HH(0,1), HH(0,2), HH(0,3),
      HH(1,0), HH(1,1), HH(1,2), HH(1,3),
      HH(2,0), HH(2,1), HH(2,2), HH(2,3)
    );
}

static cv::Point2f warp(const cv::Point2f & p, const cv::Matx33f & H)
{
  const cv::Vec3f v = H * cv::Vec3f(p.x, p.y, 1);
  return v[2] ? cv::Point2f(v[0] / v[2], v[1] / v[2]) : cv::Point2f(v[0], v[1]);
}

static cv::Point2f warp(const cv::Point2f & p, const cv::Matx34f & H)
{
  const cv::Vec3f v = H * cv::Vec4f(p.x, p.y, 1, 1);
  return v[2] ? cv::Point2f(v[0] / v[2], v[1] / v[2]) : cv::Point2f(v[0], v[1]);
}

static cv::Point2f warp(int x, int y, const cv::Matx34f & H)
{
  const cv::Vec3f v = H * cv::Vec4f(x, y, 1, 1);
  return v[2] ? cv::Point2f(v[0] / v[2], v[1] / v[2]) : cv::Point2f(v[0], v[1]);
}

static cv::Mat2f create_perspective_remap(const cv::Matx34f & H, const cv::Size & output_size)
{
  cv::Mat2f m(output_size);

  for( int y = 0; y < output_size.height; ++y ) {
    for( int x = 0; x < output_size.width; ++x ) {

      const cv::Point2f wp =
          warp(x, y, H);

      m[y][x][0] = wp.x;
      m[y][x][1] = wp.y;
    }
  }

  return m;
}


static void perspective_remap(cv::InputArray src, cv::OutputArray dst, const cv::Matx34f & H,
    const cv::Size & output_size,
    int interpolation,
    int borderMode = cv::BORDER_CONSTANT,
    const cv::Scalar & borderValue = cv::Scalar())
{
  const cv::Mat2f m =
      create_perspective_remap((interpolation & cv::WARP_INVERSE_MAP) ? H :
          invert_pose(H),
          output_size);

  cv::remap(src, dst, m, cv::noArray(),
      interpolation,
      borderMode,
      borderValue);
}

static inline void draw_epipole(cv::Mat & image, const cv::Point2d & E, const cv::Scalar & color)
{
  if( E.x >= 0 && E.y >= 0 && E.x < image.cols && E.y < image.rows ) {
    cv::ellipse(image, E, cv::Size(11, 11), 0, 0, 360, color, 1, cv::LINE_8);
    cv::line(image, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }
}

void c_keypoins_detector_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Options", ctx(&this_class:: _opts), "Options for feature2D detector");
  ctlbind(ctls, "octave", ctx, &this_class::octave, &this_class::set_octave, "Draw key points from selected octave only");
  ctlbind(ctls, "", ctx, &this_class::black_background, &this_class::set_black_background, "");
  ctlbind(ctls, "display", ctx, &this_class::display_type, &this_class::set_display_type, "");
  ctlbind(ctls, "rotation", ctx, &this_class::rotation, &this_class::set_rotation, "");
  ctlbind(ctls, "translation", ctx, &this_class::translation, &this_class::set_translation, "");
  ctlbind(ctls, "focus", ctx, &this_class::focus, &this_class::set_focus, "");
}

bool c_keypoins_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_OPTION(settings, save, *this, _opts);
    SERIALIZE_PROPERTY(settings, save, *this, octave);
    SERIALIZE_PROPERTY(settings, save, *this, black_background);
    SERIALIZE_PROPERTY(settings, save, *this, display_type);
    SERIALIZE_PROPERTY(settings, save, *this, rotation);
    SERIALIZE_PROPERTY(settings, save, *this, translation);
    SERIALIZE_PROPERTY(settings, save, *this, focus);
    return true;
  }
  return false;
}

void c_keypoins_detector_routine::parameter_changed()
{
  _keypoints_detector.reset();
}

bool c_keypoins_detector_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  try {

    if( image.needed() && !image.empty() ) {

      if( !_keypoints_detector && !(_keypoints_detector = create_sparse_feature_detector(_opts)) ) {
        CF_ERROR("create_sparse_feature_detector() fails");
        return false;
      }

      _keypoints.clear();

      _keypoints_detector->detect(image, _keypoints, mask);

      if( _opts.max_keypoints > 0 && _keypoints.size() > _opts.max_keypoints ) {

        std::sort(_keypoints.begin(), _keypoints.end(),
            [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
              return prev.response > next.response;
            });

        _keypoints.erase(_keypoints.begin() + _opts.max_keypoints,
            _keypoints.end());
      }

      if( _octave >= 0 ) {

        std::vector<cv::KeyPoint> tmp;

        const int octave =
            this->_octave;

        std::copy_if(_keypoints.begin(), _keypoints.end(),  std::back_inserter(tmp),
            [octave](const cv::KeyPoint & p) {
              return p.octave == octave;
            });

        _keypoints = std::move(tmp);
      }



      if( _black_background ) {
        _display.create(image.size(), CV_MAKETYPE(image.depth(), 3));
        _display.setTo(cv::Scalar::all(0));
      }
      else if( image.channels() == 3 ) {
        image.copyTo(_display);
      }
      else {
        cv::cvtColor(image, _display, cv::COLOR_GRAY2BGR);
      }

      switch (_display_type) {
        case DisplayTestHomography: {

          const cv::Matx34f H =
              create_perspective_transform(A * CV_PI / 180, T, F,
                  cv::Point2f(_display.cols / 2.f, _display.rows / 2.f));

          const cv::Matx34f Hi =
              invert_pose(H);

          const cv::Mat2f m =
              create_perspective_remap(Hi,
                  _display.size());

          if ( !_black_background ) {

            cv::remap(_display, _display,
                m, cv::noArray(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
          }

          const cv::Scalar color1 =
              CV_RGB(24, 200, 24);

          const cv::Scalar color2 =
              CV_RGB(250, 250, 32);

          for ( const cv::KeyPoint & kp : _keypoints ) {

            const cv::Point2f & p1 =
                kp.pt;

            const cv::Point2f p2 =
                warp(kp.pt, H);

            cv::rectangle(_display,
                cv::Point2f(p1.x - 1, p1.y - 1),
                cv::Point2f(p1.x + 1, p1.y + 1),
                color1,
                1,
                cv::LINE_8);

            cv::line(_display, p1, p2,
                color2,
                1,
                cv::LINE_8);
          }

          if( cv::norm(T) ) {

            const cv::Matx33f cameraMatrix(
                F, 0, _display.cols / 2.f,
                0, F, _display.rows / 2.f,
                0, 0, 1);

            const cv::Vec3f EV1 =
                cameraMatrix * T;

            const cv::Point2f E1 =
                cv::Point2f(EV1[0] / EV1[2],
                    EV1[1] / EV1[2]);

            const cv::Point2f E2 =
                warp(E1, H);

            CF_DEBUG("E1: (%+g %+g) E2: (%+g %+g)", E1.x, E1.y, E2.x, E2.y);

            draw_epipole(_display, E1, CV_RGB(255, 60, 60));
            draw_epipole(_display, E2, CV_RGB(60, 60, 255));
          }


          if( mask.needed() ) {
            if( mask.empty() ) {
              cv::remap(cv::Mat1b(_display.size(), 255), mask,
                  m, cv::noArray(),
                  cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
            }
            else {
              cv::remap(mask.getMat(), mask,
                  m, cv::noArray(),
                  cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
            }

            cv::compare(mask.getMat(), 254, mask,
                cv::CMP_GE);
          }

          break;
        }

        default: {
          if( _display.depth() != CV_8U ) {

            double scale = 1, offset = 0;

            get_scale_offset(_display.depth(), CV_8U,
                &scale, &offset);

            _display.convertTo(_display, CV_8U,
                scale, offset);
          }

          cv::drawKeypoints(_display, _keypoints, _display,
              cv::Scalar::all(-1),
              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          break;
        }
      }

      _display.copyTo(image);
    }

    return true;
  }

  catch( const std::exception & e ) {
    CF_ERROR("keypoints_detector_->detect() fails: %s", e.what());
  }
  catch( ... ) {
    CF_ERROR("UNknown exception in c_keypoins_detector_routine::process()");
  }

  return false;
}
