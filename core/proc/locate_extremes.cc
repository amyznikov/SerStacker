/*
 * locate_extremes.cc
 *
 *  Created on: Nov 21, 2023
 *      Author: amyznikov
 */
#include "locate_extremes.h"



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

  cv::Mat1b SE =
      cv::getStructuringElement(opts.se_shape,
          se_size,
          anchor);

  SE[anchor.y][anchor.x] = 0;


  cv::Mat DM, EM;

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

      D.convertTo(D, tmp.depth(),
          opts.maximums_alpha,
          opts.maximums_beta);

      cv::compare(tmp, D, DM, cv::CMP_GT);
    }
  }

  if( opts.locate_minimums ) {

    cv::Mat E;

    cv::erode(image, E, SE, anchor, 1, opts.border_type,
        opts.border_value);

    if( opts.minimums_alpha == 1 && opts.minimums_beta == 0 ) {
      cv::compare(image, E, EM, cv::CMP_LT);
    }
    else {
      cv::Mat tmp;

      if( image.depth() < CV_32F ) {
        image.getMat().convertTo(tmp, CV_32F);
      }

      E.convertTo(E, tmp.depth(),
          opts.minimums_alpha,
          opts.minimums_beta);

      cv::compare(tmp, E, EM, cv::CMP_LT);
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
