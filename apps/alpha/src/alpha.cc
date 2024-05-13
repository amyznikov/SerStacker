/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>

#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>

#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>

#include <core/debug.h>

namespace {

class c_eppflow
{
public:
  typedef c_eppflow this_class;

  // made public for debug purposes
  struct pyramid_entry {
    cv::Mat1f current_image, reference_image;
    cv::Mat2f uv, rmap;
    cv::Mat1f Ix, Iy;
    cv::Mat1f Ixx, Iyy, Ixy;
    cv::Mat1f Iw, It, Itx, Ity;
    // cv::Mat4f D;
    int lvl = 0;
  };

  void set_min_image_size(int v);
  int min_image_size() const;

  void set_noise_level(double v);
  double noise_level() const;

  bool compute(cv::InputArray reference_image,
      cv::InputArray current_image);


  const std::vector<pyramid_entry> & image_pyramid();

protected:

  bool set_reference_image(cv::InputArray src);
  bool setup_input_image(cv::InputArray src);
  void compute_mg(cv::InputArray src, cv::OutputArray dst) const;
  void downscale(cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      const cv::Size & dst_size) const;
  void upscale(cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      const cv::Size & dst_size) const;
  void average_filter(cv::Mat1f & img) const;
  void compute_stepest_descent(const cv::Mat1f & Ig, const cv::Mat1f & It, cv::Mat1f & Itg) const;
  bool compute_uv(pyramid_entry & e) const;

protected:
  std::vector<pyramid_entry> pyramid_;
  double noise_level_ = 1e-3;
  int min_image_size_ = 4;
};


static void epp_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  if( ddepth < 0 ) {
    ddepth = std::max(src.depth(), CV_32F);
  }

  cv::sepFilter2D(src, gx, -1, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, -1, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
}

static void epp_create_identity_remap(cv::Mat2f & map, const cv::Size & size)
{
  typedef tbb::blocked_range<int> tbb_range;
  constexpr int tbb_block_size = 512;

  map.create(size);

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec2f * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x;
            m[x][1] = y;
          }
        }
    });
}


/*
 * Pyramid up to specific size
 */
bool epp_upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {

      const cv::Size nextSize((sizes.back().width + 1) / 2,
          (sizes.back().height + 1) / 2);

      if( nextSize == inputSize ) {
        break;
      }

      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }

      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}
const std::vector<c_eppflow::pyramid_entry> & c_eppflow::image_pyramid()
{
  return pyramid_;
}

void c_eppflow::set_noise_level(double v)
{
  noise_level_ = v;
}

double c_eppflow::noise_level() const
{
  return noise_level_;
}

void c_eppflow::set_min_image_size(int v)
{
  min_image_size_ = v;
}

int c_eppflow::min_image_size() const
{
  return min_image_size_;
}

void c_eppflow::downscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{
  cv::pyrDown(src, dst, dst_size);

  if( dst_mask.needed() ) {
    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {
      cv::pyrDown(src_mask, dst_mask, dst_size);
      cv::compare(dst_mask.getMat(), cv::Scalar::all(250), dst_mask, cv::CMP_GE);
    }
  }
}

void c_eppflow::upscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{
  cv::pyrUp(src, dst, dst_size);

  if( dst_mask.needed() ) {

    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {
      cv::pyrUp(src_mask, dst_mask, dst_size);
      cv::compare(dst_mask.getMat(), cv::Scalar::all(250), dst_mask, cv::CMP_GE);
    }
  }
}

void c_eppflow::compute_mg(cv::InputArray src, cv::OutputArray dst) const
{
  cv::Mat tmp;
//  cv::morphologyEx(src, tmp, cv::MORPH_GRADIENT,
//      cv::Mat1b(3, 3, 255),
//      cv::Point(1, 1),
//      1,
//      cv::BORDER_REPLICATE);

  src.getMat().copyTo(tmp);

  if( tmp.depth() != CV_32F ) {
    tmp.convertTo(tmp, CV_32F);
  }

  if( tmp.channels() != 1 ) {
    reduce_color_channels(tmp, dst, cv::REDUCE_AVG);
  }
  else {
    dst.move(tmp);
  }
}

bool c_eppflow::set_reference_image(cv::InputArray src)
{
  pyramid_.clear();
  pyramid_.reserve(32);

  const int min_image_size =
      std::max(4, min_image_size_);

  for( int current_level = 0;; ++current_level ) {

    pyramid_.emplace_back();

    pyramid_entry & current_scale =
        pyramid_.back();

    current_scale.lvl =
        current_level;

    if( current_level == 0 ) {
      compute_mg(src, current_scale.reference_image);
    }
    else {

      const pyramid_entry & previous_scale =
          pyramid_[current_level - 1];

      const cv::Size previous_size =
          previous_scale.reference_image.size();

      const cv::Size next_size((int) ((previous_size.width + 1) / 2),
          (int) ((previous_size.height + 1) / 2));

      if( previous_size == next_size || std::min(next_size.width, next_size.height) <= min_image_size ) {
        pyramid_.pop_back();
        break;
      }

      downscale(previous_scale.reference_image, cv::noArray(),
          current_scale.reference_image, cv::noArray(),
          next_size);
    }

    epp_differentiate(current_scale.reference_image,
        current_scale.Ix, current_scale.Iy);

    tbb::parallel_invoke(
        [this, &current_scale]() {
          cv::multiply(current_scale.Ix, current_scale.Ix, current_scale.Ixx);
          average_filter(current_scale.Ixx);
        },
        [this, &current_scale]() {
          cv::multiply(current_scale.Ix, current_scale.Iy, current_scale.Ixy);
          average_filter(current_scale.Ixy);
        },
        [this, &current_scale]() {
          cv::multiply(current_scale.Iy, current_scale.Iy, current_scale.Iyy);
          average_filter(current_scale.Iyy);
        }
    );


  }

  return true;
}

bool c_eppflow::setup_input_image(cv::InputArray src)
{
  const int num_levels =
      pyramid_.size();

  for( int current_level = 0; current_level < num_levels; ++current_level ) {

    pyramid_entry & current_scale =
        pyramid_[current_level];

    if( current_level == 0 ) {
      compute_mg(src, current_scale.current_image);
    }
    else {

      const pyramid_entry & previous_scale =
          pyramid_[current_level - 1];

      downscale(previous_scale.current_image, cv::noArray(),
          current_scale.current_image, cv::noArray(),
          current_scale.reference_image.size());
    }

    epp_create_identity_remap(current_scale.rmap,
        current_scale.reference_image.size());
  }





  return true;
}

void c_eppflow::average_filter(cv::Mat1f & img) const
{
//  const double sigma = 3;
//  cv::GaussianBlur(img, img, cv::Size(), sigma, sigma, cv::BORDER_CONSTANT);

  cv::Size size =
      img.size();

  for ( int i = 0; i < 2; ++i ) {
    size.width = (size.width + 1) / 2;
    size.height = (size.height + 1) / 2;

    //if ( std::max() )

  }

  cv::resize(img, img, size, 0, 0, cv::INTER_AREA);

  static thread_local const cv::Mat G = cv::getGaussianKernel(5, 0, CV_32F);
  cv::sepFilter2D(img, img, -1, G, G, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);
}

void c_eppflow::compute_stepest_descent(const cv::Mat1f & Ig, const cv::Mat1f & It, cv::Mat1f & Itg) const
{
  cv::multiply(Ig, It, Itg);
  average_filter(Itg);
}

bool c_eppflow::compute_uv(pyramid_entry & e) const
{
  const cv::Size size =
      e.reference_image.size();

  cv::remap(e.current_image, e.Iw,
      e.rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_REPLICATE);

  const cv::Mat1f & I1 =
      e.Iw;

  const cv::Mat1f & I2 =
      e.reference_image;

  cv::subtract(I2, I1, e.It);
  compute_stepest_descent(e.Ix, e.It, e.Itx);
  compute_stepest_descent(e.Iy, e.It, e.Ity);


  //  a00 = Ixx;
  //  a01 = Ixy;
  //  a10 = Ixy;
  //  a11 = Iyy;
  //  b0  = 2 * Itx;
  //  b1  = 2 * Ity;
  //  D = a00 * a11 - a10 * a01
  //  u = 1/D * (a11 * b0 - a01 * b1);
  //  v = 1/D * (a00 * b1 - a10 * b0);

  const cv::Size size2 =
      e.Ity.size();

  e.uv.create(size2);

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, size2.height, 256),
      [&e, size2](const range & r) {

        static constexpr float eps = 1e-25;
        static constexpr float update_multiplier = 1.5;

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0, nx = size2.width; x < nx; ++x ) {

            const float & a00 = e.Ixx[y][x];
            const float & a01 = e.Ixy[y][x];
            const float & a10 = e.Ixy[y][x];
            const float & a11 = e.Iyy[y][x];

            const float & b0 = e.Itx[y][x];
            const float & b1 = e.Ity[y][x];

            const float det = update_multiplier / (eps + fabsf(a00 * a11 - a10 * a01));

            e.uv[y][x][0] = det * (a11 * b0 - a01 * b1);
            e.uv[y][x][1] = det * (a00 * b1 - a10 * b0);
          }
        }
      });


  epp_upscale(e.uv, size);

  return true;
}

bool c_eppflow::compute(cv::InputArray reference_image, cv::InputArray current_image)
{
  if ( !set_reference_image(reference_image) ) {
    CF_ERROR("set_reference_image() fails");
    return false;
  }

  if ( !setup_input_image(current_image) ) {
    CF_ERROR("setup_input_image() fails");
    return false;
  }

  const int num_levels =
      pyramid_.size();

  for( int i = num_levels - 1; i >= 0; --i ) {

    pyramid_entry & current_scale =
        pyramid_[i];

    if( i < num_levels - 1 ) {

      const pyramid_entry & prev_scale =
          pyramid_[i + 1];

      const cv::Size current_size =
          current_scale.current_image.size();

      const cv::Size prev_size =
          prev_scale.current_image.size();

      const cv::Scalar size_ratio((double) current_size.width / (double) prev_size.width,
          (double) current_size.height / (double) prev_size.height);

      cv::multiply(prev_scale.uv, size_ratio, current_scale.uv);
      upscale(current_scale.uv, cv::noArray(), current_scale.uv, cv::noArray(), current_size);
      cv::add(current_scale.rmap, current_scale.uv, current_scale.rmap);
    }

    for( int j = 0; j < 3; ++j ) {
      compute_uv(current_scale);
      cv::add(current_scale.rmap, current_scale.uv, current_scale.rmap);
    }
  }

  return true;
}

}

int main(int argc, char *argv[])
{

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

//  std::string input_file_name =
//      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/2011_09_26_drive_0005_sync.03.avi";

  std::string input_file_name =
      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0009_sync/2011_09_26_drive_0009_sync.03.avi";

//  std::string input_file_name =
//      "/home/data/zenuity/000006_romeo-undistorted.avi";

  c_ffmpeg_input_source::sptr source =
      c_ffmpeg_input_source::create(input_file_name);

  if ( !source->open() ) {
    CF_ERROR("source->open() fails for '%s'", input_file_name.c_str());
    return 1;
  }

  source->seek(321);

  cv::Mat images[2];

  for ( int i = 0; i < 2; ++i ) {

    if ( !source->read(images[i], nullptr, nullptr) ) {
      CF_ERROR("source->read(images[%d]) fails for '%s'", i, input_file_name.c_str());
      return 1;
    }

  }

  source->close();

  c_eppflow f;

  f.set_min_image_size(8);
  f.set_noise_level(1e-3);

  f.compute(images[0], images[1]);

  const auto & pyramid =
      f.image_pyramid();

  for( int i = 0, n = pyramid.size(); i < n; ++i ) {
    save_image(pyramid[i].reference_image, ssprintf("alpha-debug/pyramid/L%03d.reference_mg.tiff", i));
    save_image(pyramid[i].current_image, ssprintf("alpha-debug/pyramid/L%03d.current_mg.tiff", i));
    save_image(pyramid[i].uv, ssprintf("alpha-debug/pyramid/L%03d.uv.flo", i));
    save_image(pyramid[i].Ix, ssprintf("alpha-debug/pyramid/Ix.L%03d.tiff", i));
    save_image(pyramid[i].Iy, ssprintf("alpha-debug/pyramid/Iy.L%03d.tiff", i));
    save_image(pyramid[i].Ixx, ssprintf("alpha-debug/pyramid/Ixx.L%03d.tiff", i));
    save_image(pyramid[i].Iyy, ssprintf("alpha-debug/pyramid/Iyy.L%03d.tiff", i));
    save_image(pyramid[i].Ixy, ssprintf("alpha-debug/pyramid/Ixy.L%03d.tiff", i));
    save_image(pyramid[i].It, ssprintf("alpha-debug/pyramid/It.L%03d.tiff", i));
    save_image(pyramid[i].Itx, ssprintf("alpha-debug/pyramid/Itx.L%03d.tiff", i));
    save_image(pyramid[i].Ity, ssprintf("alpha-debug/pyramid/Ity.L%03d.tiff", i));


    cv::Mat tmp;
    cv::remap(pyramid[i].current_image, tmp,
        pyramid[i].rmap, cv::noArray(),
        cv::INTER_AREA,
        cv::BORDER_REPLICATE);

    save_image(tmp, ssprintf("alpha-debug/pyramid/L%03d.reapped_mg.tiff", i));

    cv::Mat2f flow;
    ecc_remap_to_optflow(pyramid[i].rmap, flow);
    save_image(flow, ssprintf("alpha-debug/pyramid/L%03d.flow.flo", i));


  }

  return 0;
}


#if 0

int main(int argc, char *argv[])
{

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

//  std::string input_file_name =
//      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/2011_09_26_drive_0005_sync.03.avi";

  std::string input_file_name =
      "/mnt/data/kitti/2011_09_26/2011_09_26_drive_0009_sync/2011_09_26_drive_0009_sync.03.avi";

//  std::string input_file_name =
//      "/home/data/zenuity/000006_romeo-undistorted.avi";

  c_ffmpeg_input_source::sptr source =
      c_ffmpeg_input_source::create(input_file_name);

  if ( !source->open() ) {
    CF_ERROR("source->open() fails for '%s'", input_file_name.c_str());
    return 1;
  }

  source->seek(321);

  cv::Mat src_images[2];
  cv::Mat images[2];

  for ( int i = 0; i < 2; ++i ) {

    if ( !source->read(src_images[i], nullptr, nullptr) ) {
      CF_ERROR("source->read(images[%d]) fails for '%s'", i, input_file_name.c_str());
      return 1;
    }

    cv::cvtColor(src_images[i], images[i], cv::COLOR_BGR2GRAY);
    cv::morphologyEx(images[i], images[i], cv::MORPH_GRADIENT, cv::Mat1b(3,3,255));
    images[i].convertTo(images[i], CV_32F, 1./255, 0);
  }

  source->close();

  c_eccflow f;

  f.set_support_scale(3);
  f.set_min_image_size(4);
  f.set_noise_level(1e-3);
  f.set_scale_factor(0.75);
  f.set_update_multiplier(2);

  f.set_reference_image(images[0], cv::noArray());

  const auto & pyramid =
      f.current_pyramid();

  CF_DEBUG("H: pyramid.size()=%zu", pyramid.size());
  for( int i = 0, n = pyramid.size(); i < n; ++i ) {
    save_image(pyramid[i].reference_image, ssprintf("alpha-debug/pyramid/L%03d.reference_mg.tiff", i));
  }

  cv::Mat2f rmap;
  cv::Mat2f optflow;

  f.compute(images[1], rmap,  cv::noArray());
  for( int i = 0, n = pyramid.size(); i < n; ++i ) {
    save_image(pyramid[i].current_image, ssprintf("alpha-debug/pyramid/L%03d.current_mg.tiff", i));

    ecc_remap_to_optflow(pyramid[i].rmap, optflow);
    save_image(optflow, ssprintf("alpha-debug/pyramid/optflow.L%03d.flo", i));
  }

  ecc_remap_to_optflow(rmap, optflow);
  save_image(optflow, ssprintf("alpha-debug/pyramid/optflow.final.flo"));

  save_image(images[0], ssprintf("alpha-debug/pyramid/reference_image.tiff"));
  save_image(images[1], ssprintf("alpha-debug/pyramid/current_image.tiff"));
  cv::remap(images[1], images[1], rmap, cv::noArray(), cv::INTER_LINEAR);
  save_image(images[1], ssprintf("alpha-debug/pyramid/remapped_image.tiff"));

  cv::Mat absdiff_image, mask1, mask2, mask;

//  if ( f.use_usharp() ) {
//    cv::compare(pyramid.front().reference_mg, cv::Scalar::all(0.1), mask1, cv::CMP_GT);
//  }
//  save_image(pyramid.front().reference_mg, mask1, ssprintf("alpha-debug/pyramid/reference_mg.tiff"));
//
//  cv::remap(pyramid.front().current_mg, pyramid.front().current_mg, rmap, cv::noArray(), cv::INTER_LINEAR);
//  if ( f.use_usharp() ) {
//    cv::compare(pyramid.front().current_mg, cv::Scalar::all(0.1), mask2, cv::CMP_GT);
//  }
//  save_image(pyramid.front().current_mg, mask2, ssprintf("alpha-debug/pyramid/remapped_current_mg.tiff"));
//
//  if ( f.use_usharp() ) {
//    cv::bitwise_and(mask1, mask2, mask);
//    morphological_smooth_close(mask, mask, cv::Mat1b(3, 3, 255));
//  }

  cv::remap(src_images[1], src_images[1], rmap, cv::noArray(), cv::INTER_LINEAR);
  save_image(src_images[0], mask, ssprintf("alpha-debug/pyramid/masked_src_image0.tiff"));
  save_image(src_images[1], mask, ssprintf("alpha-debug/pyramid/masked_src_image1.tiff"));



//  std::string options;
//
//  for ( int i = 1; i < argc; ++i ) {
//    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
//      fprintf(stdout, "Usage: alpha path_to_pcap_file.pcap\n");
//      return 0;
//
//    }
//
//    if ( address.empty() ) {
//      address = argv[i];
//      continue;
//    }
//
//    fprintf(stderr, "Invalid arg: '%s'\n", argv[i]);
//    return 1;
//  }



  return 0;
}
#endif


