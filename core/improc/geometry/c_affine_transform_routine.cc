/*
 * c_affine_transform_routine.cc
 *
 *  Created on: Jun 12, 2022
 *      Author: amyznikov
 */

#include "c_affine_transform_routine.h"

template<>
const c_enum_member * members_of<c_affine_transform_routine::image_resize_mode>()
{
  static const c_enum_member members[] = {
      {c_affine_transform_routine::resize_keep, "KEEP", ""},
      {c_affine_transform_routine::resize_adjust, "ADJUST", ""},
      {c_affine_transform_routine::resize_scale, "SCALE", ""},
      {c_affine_transform_routine::resize_keep},
  };

  return members;
}

void c_affine_transform_routine::set_resize_mode(image_resize_mode v)
{
  _resize_mode = v;
  _remap.release();
}

c_affine_transform_routine::image_resize_mode c_affine_transform_routine::resize_mode() const
{
  return _resize_mode;
}

void c_affine_transform_routine::set_interpolation(cv::InterpolationFlags v)
{
  _interpolation = v;
  _remap.release();
}

cv::InterpolationFlags c_affine_transform_routine::interpolation() const
{
  return _interpolation;
}

void c_affine_transform_routine::set_border_type(cv::BorderTypes v)
{
  _border_type = v;
  _remap.release();
}

cv::BorderTypes c_affine_transform_routine::border_type() const
{
  return _border_type;
}

void c_affine_transform_routine::set_border_value(const cv::Scalar & v)
{
  _border_value = v;
  _remap.release();
}

const cv::Scalar & c_affine_transform_routine::border_value() const
{
  return _border_value;
}

void c_affine_transform_routine::set_rotation(double v)
{
  _rotation = v;
  _remap.release();
}

double c_affine_transform_routine::rotation() const
{
  return _rotation;
}

void c_affine_transform_routine::set_translation(const cv::Point2f & v)
{
  _translation = v;
  _remap.release();
}

const cv::Point2f & c_affine_transform_routine::translation() const
{
  return _translation;
}

void c_affine_transform_routine::set_scale(const cv::Size2f & v)
{
  _scale = v;
  _remap.release();
}

const cv::Size2f & c_affine_transform_routine::scale() const
{
  return _scale;
}

void c_affine_transform_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "translation", ctx, &this_class::translation, &this_class::set_translation, "image translation in pixels before rotation/scale");
   ctlbind_slider_spinbox(ctls, "rotation", ctx, &this_class::rotation, &this_class::set_rotation, -180, 180, 0.05, "rotation angle in degrees");
   ctlbind(ctls, "scale", ctx, &this_class::scale, &this_class::set_scale, "image scale");
   ctlbind(ctls, "resize_mode", ctx, &this_class::resize_mode, &this_class::set_resize_mode, "resize mode");
   ctlbind(ctls, "interpolation", ctx, &this_class::interpolation, &this_class::set_interpolation, "interpolation");
   ctlbind(ctls, "border_type", ctx, &this_class::border_type, &this_class::set_border_type, "border_type");
   ctlbind(ctls, "border_value", ctx, &this_class::border_value, &this_class::set_border_value, "border_value");
}

bool c_affine_transform_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, rotation);
    SERIALIZE_PROPERTY(settings, save, *this, translation);
    SERIALIZE_PROPERTY(settings, save, *this, scale);
    SERIALIZE_PROPERTY(settings, save, *this, interpolation);
    SERIALIZE_PROPERTY(settings, save, *this, border_type);
    SERIALIZE_PROPERTY(settings, save, *this, border_value);
    SERIALIZE_PROPERTY(settings, save, *this, resize_mode);
    return true;
  }
  return false;
}

bool c_affine_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() || !mask.empty() ) {

    if( _rotation || _translation.x || _translation.y || _scale.width != 1 || _scale.height != 1 ) {

      if( _remap.empty() || _previous_image_size != image.size() ) {

        _previous_image_size = image.size();

        if ( !create_transformation_remap(_remap, image.size(), _rotation, _translation, _scale, _resize_mode) ) {
          CF_ERROR("c_image_transform_routine: create_transformation_remap() fails");
          return false;
        }
      }

      if ( !image.empty() ) {
        cv::remap(image.getMat(), image,
            _remap, cv::noArray(),
            _interpolation,
            _border_type,
            _border_value);
      }


      if ( mask.needed() ) {

        if ( !mask.empty() ) {
          cv::remap(mask.getMat(), mask,
              _remap, cv::noArray(),
              cv::INTER_AREA,
              cv::BORDER_CONSTANT);

        }
        else {
          cv::remap(cv::Mat1b(_previous_image_size, 255), mask,
              _remap, cv::noArray(),
              cv::INTER_AREA,
              cv::BORDER_CONSTANT);
        }

        cv::compare(mask.getMat(), 250, mask,
            cv::CMP_GE);
      }
    }
  }

  return true;
}

static cv::Matx23f build_forward_transformation_matrix(const cv::Point2f & src_center,
    double rotation,
    const cv::Size2f & scale)
{
  // forward transform is:
  //   new_image = scale * (rotation * (image - src_center) + src_center)
  //   new_image = [scale * rotation] * image - scale * (rotation * src_center - src_center)

  double sa, ca;
  sincos(-rotation, &sa, &ca);

  const cv::Matx22f S(scale.width, 0,
      0, scale.height);

  const cv::Matx22f R(ca, sa,
      -sa, ca);

  const cv::Matx22f SR =
      S * R;

  const cv::Vec2f T =
      - S * (R * cv::Vec2f(src_center) - cv::Vec2f(src_center));

  return cv::Matx23f(
      SR(0, 0), SR(0, 1), T(0),
      SR(1, 0), SR(1, 1), T(1));
}

bool c_affine_transform_routine::create_transformation_remap(cv::Mat2f & dst,
    const cv::Size & image_size,
    double rotation_degrees,
    const cv::Point2f & translation,
    const cv::Size2f & scale,
    image_resize_mode resize_mode)
{
  // forward transform is:
  //   new_image = scale * (rotation * (image - center) + center) + translation

  const cv::Point2f image_center(image_size.width / 2,
      image_size.height / 2);

  const cv::Vec3f corners[4] = {
      cv::Vec3f(0, 0, 1),
      cv::Vec3f(image_size.width, 0, 1),
      cv::Vec3f(image_size.width, image_size.height, 1),
      cv::Vec3f(0, image_size.height, 1),
  };

  const cv::Matx23f M =
      build_forward_transformation_matrix(image_center,
          rotation_degrees * CV_PI / 180,
          scale);


  int xmin, xmax, ymin, ymax;
  cv::Size rotated_image_size;

  cv::Matx23f Mi;
  cv::invertAffineTransform(M, Mi);

  switch (resize_mode) {

  case resize_keep: {
    xmin = 0;
    xmax = image_size.width;
    ymin = 0;

    ymax = image_size.height;
    rotated_image_size = image_size;
    break;
  }

  case resize_scale: {
    xmin = 0;
    xmax = image_size.width * scale.width;
    ymin = 0;
    ymax = image_size.height * scale.height;

    rotated_image_size.width = (xmax - xmin);
    rotated_image_size.height = (ymax - ymin);
    break;
  }

  case resize_adjust: {

    xmin = INT_MAX;
    xmax = INT_MIN;
    ymin = INT_MAX;
    ymax = INT_MIN;

    for( uint i = 0; i < 4; ++i ) {

      const cv::Vec2f c =
          M * corners[i];

      if( c(0) < xmin ) {
        xmin = c(0);
      }
      if( c(0) > xmax ) {
        xmax = c(0);
      }
      if( c(1) < ymin ) {
        ymin = c(1);
      }
      if( c(1) > ymax ) {
        ymax = c(1);
      }
    }

    rotated_image_size.width = (xmax - xmin);
    rotated_image_size.height = (ymax - ymin);
    break;
  }

  }

  dst.create(rotated_image_size);
  dst.setTo(cv::Vec2f(-translation.x, -translation.y));

  for( int y = 0; y < dst.rows; ++y ) {
    for( int x = 0; x < dst.cols; ++x ) {
      dst[y][x] += Mi * cv::Vec3f(x + xmin, y + ymin, 1);
    }
  }


  return true;
}
