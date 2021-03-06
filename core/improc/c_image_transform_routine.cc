/*
 * c_image_transform_routine.cc
 *
 *  Created on: Jun 12, 2022
 *      Author: amyznikov
 */

#include "c_image_transform_routine.h"

template<>
const c_enum_member * members_of<c_image_transform_routine::image_resize_mode>()
{
  static constexpr c_enum_member members[] = {
      {c_image_transform_routine::resize_keep, "KEEP", ""},
      {c_image_transform_routine::resize_adjust, "ADJUST", ""},
      {c_image_transform_routine::resize_keep},
  };

  return members;
}


c_image_transform_routine::c_class_factory c_image_transform_routine::class_factory;


c_image_transform_routine::c_image_transform_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_image_transform_routine::ptr c_image_transform_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_image_transform_routine::set_resize_mode(image_resize_mode v)
{
  resize_mode_ = v;
}

c_image_transform_routine::image_resize_mode c_image_transform_routine::resize_mode() const
{
  return resize_mode_;
}

void c_image_transform_routine::set_interpolation(cv::InterpolationFlags v)
{
  interpolation_ = v;
}

cv::InterpolationFlags c_image_transform_routine::interpolation() const
{
  return interpolation_;
}

void c_image_transform_routine::set_border_type(cv::BorderTypes v)
{
  border_type_ = v;
}

cv::BorderTypes c_image_transform_routine::border_type() const
{
  return border_type_;
}

void c_image_transform_routine::set_border_value(const cv::Scalar & v)
{
  border_value_ = v;
}

const cv::Scalar & c_image_transform_routine::border_value() const
{
  return border_value_;
}

void c_image_transform_routine::set_rotation(double v)
{
  rotation_ = v;
}

double c_image_transform_routine::rotation() const
{
  return rotation_;
}

void c_image_transform_routine::set_translation(const cv::Point2f & v)
{
  translation_ = v;
}

const cv::Point2f & c_image_transform_routine::translation() const
{
  return translation_;
}

void c_image_transform_routine::set_scale(const cv::Size2f & v)
{
  scale_ = v;
}

const cv::Size2f c_image_transform_routine::scale() const
{
  return scale_;
}

bool c_image_transform_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, rotation);
  LOAD_PROPERTY(settings, this, translation);
  LOAD_PROPERTY(settings, this, scale);
  LOAD_PROPERTY(settings, this, interpolation);
  LOAD_PROPERTY(settings, this, border_type);
  LOAD_PROPERTY(settings, this, border_value);

  return true;
}

bool c_image_transform_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, rotation);
  SAVE_PROPERTY(settings, *this, translation);
  SAVE_PROPERTY(settings, *this, scale);
  SAVE_PROPERTY(settings, *this, interpolation);
  SAVE_PROPERTY(settings, *this, border_type);
  SAVE_PROPERTY(settings, *this, border_value);

  return true;
}

bool c_image_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {
    if( rotation_ || translation_.x || translation_.y || scale_.width != 1 || scale_.height != 1 ) {

      cv::Mat2f M;

      if ( !create_transformation_remap(M, image.size(),rotation_, translation_, scale_, resize_mode_) ) {
        CF_ERROR("c_image_transform_routine: create_transformation_remap() fails");
        return false;
      }

      if ( !image.empty() ) {
        //cv::Mat new_image;
        cv::remap(image.getMat(), image, M, cv::noArray(), interpolation_, border_type_, border_value_);
        //image.move(new_image);
      }

      if ( !mask.empty() ) {
        //cv::Mat new_mask;
        cv::remap(mask.getMat(), mask, M, cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT);
        cv::compare(mask, 255, mask, cv::CMP_EQ);
        //mask.move(new_mask);
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

static cv::Matx23f invert(const cv::Matx23f & M)
{
  // v' = R * v + T
  // v'- T = R * v
  // Ri * ( v'- T) = v
  // Ri * v'- Ri * T = v


  const cv::Matx22f R(M(0, 0), M(0, 1),
      M(1, 0), M(1, 1));

  const cv::Vec2f T(M(0,2), M(1, 2));

  const cv::Matx22f Ri =
      R.inv();

  const cv::Vec2f Ti =
      -Ri * T;

  return cv::Matx23f(
      Ri(0, 0), Ri(0, 1), Ti(0),
      Ri(1, 0), Ri(1, 1), Ti(1));
}

bool c_image_transform_routine::create_transformation_remap(cv::Mat2f & dst,
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

  const cv::Matx23f M =
      build_forward_transformation_matrix(image_center,
          rotation_degrees * CV_PI / 180,
          scale);


  const cv::Matx23f Mi =
      invert(M);

  const cv::Vec3f corners[4] = {
      cv::Vec3f(0, 0, 1),
      cv::Vec3f(image_size.width, 0, 1),
      cv::Vec3f(image_size.width, image_size.height, 1),
      cv::Vec3f(0, image_size.height, 1),
  };


  int xmin = INT_MAX;
  int xmax = INT_MIN;
  int ymin = INT_MAX;
  int ymax = INT_MIN;

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

  cv::Size rotated_image_size;

  switch (resize_mode) {
  case resize_keep:
    rotated_image_size = image_size;
    break;

  case resize_adjust: {
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
