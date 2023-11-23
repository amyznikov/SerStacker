/*
 * locate_extremes.cc
 *
 *  Created on: Nov 21, 2023
 *      Author: amyznikov
 */
#include "locate_extremes.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_locate_extremes_options::neighbor_filter_type>()
{
  static constexpr c_enum_member members[] = {
      { c_locate_extremes_options::filter_morph, "MORPH", "" },
      { c_locate_extremes_options::filter_mean, "mean", "" },
      { c_locate_extremes_options::filter_morph },
  };

  return members;
}

static cv::Mat get_filter_kernel(c_locate_extremes_options::neighbor_filter_type filter_type,
    cv::MorphShapes shape, const cv::Size & size, const cv::Point & anchor)
{

  switch (filter_type) {

    case c_locate_extremes_options::filter_morph: {

      static thread_local cv::Mat1b SE;
      static thread_local cv::MorphShapes se_shape;
      static thread_local cv::Size se_size;
      static thread_local cv::Point se_anchor;

      if( SE.empty() || shape != se_shape || size != se_size || anchor != se_anchor ) {

        SE =
            cv::getStructuringElement(shape,
                size,
                anchor);

        SE[anchor.y][anchor.x] = 0;

        se_shape = shape;
        se_size = size;
        se_anchor = anchor;
      }

      return SE;
    }

    case c_locate_extremes_options::filter_mean: {

      static cv::Mat1f SE;

      static thread_local cv::MorphShapes se_shape;
      static thread_local cv::Size se_size;
      static thread_local cv::Point se_anchor;

      if( SE.empty() || shape != se_shape || size != se_size || anchor != se_anchor ) {

        cv::getStructuringElement(shape, size, anchor).
            convertTo(SE, CV_32F);

        SE[anchor.y][anchor.x] = 0;
        cv::divide(SE, cv::sum(SE), SE);

        se_shape = shape;
        se_size = size;
        se_anchor = anchor;
      }

      return SE;
    }
  }

  return cv::Mat();
}


bool locate_extremes(cv::InputArray image, cv::InputArray input_mask, cv::OutputArray output_mask,
    const c_locate_extremes_options & opts)
{
  if ( !opts.locate_minimums && !opts.locate_maximums ) {
    output_mask.release();
    return true;
  }

  cv::Size se_size =
      opts.se_size;

  cv::Point anchor =
      opts.anchor;

  if ( se_size.width < 1 ) {
    se_size.width = 1;
  }
  if ( se_size.height < 1 ) {
    se_size.height = 1;
  }

  if( anchor.x < 0 || anchor.x >= se_size.width ) {
    anchor.x = se_size.width / 2;
  }

  if( anchor.y < 0 || anchor.y >= se_size.height ) {
    anchor.y = se_size.height / 2;
  }

  const cv::Mat SE =
      get_filter_kernel(opts.filter_type,
          opts.se_shape, se_size, anchor);


  cv::Mat DM, EM;

  if( opts.filter_type == c_locate_extremes_options::filter_mean ) {

    cv::Mat AVG;
    cv::Mat ximage;

    cv::filter2D(image, AVG, CV_32F, SE, anchor, 0, opts.border_type);

    if ( image.depth() == CV_32F ) {
      ximage = image.getMat();
    }
    else {
      image.getMat().convertTo(ximage, CV_32F);
    }

    if( opts.locate_maximums ) {

      if( opts.maximums_alpha == 1 && opts.maximums_beta == 0 ) {
        cv::compare(ximage, AVG, DM, cv::CMP_GT);
      }
      else {

        cv::Mat D;

        AVG.convertTo(D, AVG.depth(),
            opts.maximums_alpha,
            opts.maximums_beta);

        cv::compare(ximage, D, DM,
            cv::CMP_GT);
      }
    }

    if( opts.locate_minimums ) {

      if( opts.minimums_alpha == 1 && opts.minimums_beta == 0 ) {
        cv::compare(ximage, AVG, DM, cv::CMP_LT);
      }
      else {

        cv::Mat E;

        AVG.convertTo(E, AVG.depth(),
            opts.minimums_alpha,
            opts.minimums_beta);

        cv::compare(ximage, E, DM,
            cv::CMP_LT);
      }
    }

  }
  else {

    if( opts.locate_maximums ) {

      cv::Mat D;

      cv::dilate(image, D, SE, anchor, 1, opts.border_type,
            opts.border_value);

      if( opts.maximums_alpha == 1 && opts.maximums_beta == 0 ) {
        cv::compare(image, D, DM, cv::CMP_GT);
      }
      else {
        cv::Mat tmp;

        if( image.depth() < CV_32F ) {
          image.getMat().convertTo(tmp, CV_32F);
        }
        else {
          tmp = image.getMat();
        }

        D.convertTo(D, tmp.depth(),
            opts.maximums_alpha,
            opts.maximums_beta);

        cv::compare(tmp, D, DM, cv::CMP_GT);
      }
    }

    if( opts.locate_minimums ) {

      cv::Mat E;

      if( opts.filter_type == c_locate_extremes_options::filter_morph ) {
        cv::erode(image, E, SE, anchor, 1, opts.border_type,
            opts.border_value);
      }

      if( opts.minimums_alpha == 1 && opts.minimums_beta == 0 ) {
        cv::compare(image, E, EM, cv::CMP_LT);
      }
      else {
        cv::Mat tmp;

        if( image.depth() < CV_32F ) {
          image.getMat().convertTo(tmp, CV_32F);
        }
        else {
          tmp = image.getMat();
        }

        E.convertTo(E, tmp.depth(),
            opts.minimums_alpha,
            opts.minimums_beta);

        cv::compare(tmp, E, EM, cv::CMP_LT);
      }
    }

  }

  if ( opts.locate_maximums && opts.locate_minimums ) {
    cv::bitwise_or(EM, DM, output_mask);
  }
  else if ( opts.locate_maximums ) {
    output_mask.move(DM);
  }
  else if ( opts.locate_minimums ) {
    output_mask.move(EM);
  }

  if ( !input_mask.empty() ) {
    if ( input_mask.channels() == output_mask.channels() ) {
      cv::bitwise_and(input_mask, output_mask, output_mask);
    }
    else if (input_mask.channels() == 1) {
      output_mask.setTo(0, ~input_mask.getMat());
    }
  }

  return true;
}
