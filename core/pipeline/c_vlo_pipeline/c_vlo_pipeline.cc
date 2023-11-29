/*
 * c_vlo_pipeline.cc
 *
 *  Created on: Oct 26, 2023
 *      Author: amyznikov
 */

#include "c_vlo_pipeline.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/autoclip.h>
#include <core/proc/colormap.h>
#include <core/mtf/pixinsight-mtf.h>
#include <core/io/save_ply.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

//////////////////////////////
template<>
const c_enum_member* members_of<VLO_INTENSITY_CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { VLO_INTENSITY_PEAK, "PEAK", "PEAK" },
      { VLO_INTENSITY_AREA, "AREA", "AREA" },
      { VLO_INTENSITY_PEAK },
  };

  return members;
}

template<class ScanType>
static bool update_vlo_lookup_table_statistics_(const ScanType & scan,
    c_vlo_lookup_table_statistics & statistics)
{

  typedef typename ScanType::Echo echo_type;

  typedef decltype(ScanType::Echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::Echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::Echo::dist) dist_type;
  constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

  constexpr double max_distance_meters  = 500;
  constexpr double bin_size_in_meters = 1.0;
  constexpr int num_bins = (int) (max_distance_meters / bin_size_in_meters) + 1;


  if ( statistics.empty() ) { // Initialize new statistics table;
    statistics.resize(num_bins);
  }

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      if ( s >= 296 && s <= 325 ) {
        continue;
      }


      echo_type echos[3] = {0};
      int num_echos = 0;

      for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

        const auto & echo =
            scan.slot[s].echo[l][e];

        const auto &distance =
            echo.dist;

        if( distance <= 0 || distance > max_dist_value ) {
          continue;
        }

        const auto &peak =
            echo.peak;

        if( peak <= 0 || peak > max_peak_value ) {
          continue;
        }

        const auto &area =
            echo.area;

        if( area <= 0 || peak > max_area_value ) {
          continue;
        }


        // all data are valid
        echos[num_echos++] = echo;
      }


      if ( num_echos > 0 ) {

        if ( num_echos > 1 ) {

          std::sort(echos, echos + num_echos,
              [](const auto & prev, const auto & next) {
                return next.peak < prev.peak;
              });

        }

        const auto & echo =
            echos[0];

        const int bin_index =
            std::min(num_bins-1, (int)( echo.dist * 0.01 / bin_size_in_meters ));


        c_vlo_lookup_table_item & item =
            statistics[bin_index];

        item.distance += echo.dist * 0.01;
        item.area += echo.area;
        item.peak += echo.peak;
        item.area2 += ((double)echo.area) * ((double)echo.area);
        item.peak2 += ((double)echo.peak) * ((double)echo.peak);
        item.num_measurements += 1;
      }
    }
  }

  return true;
}

static bool update_vlo_lookup_table_statistics(const c_vlo_scan & scan,
    c_vlo_lookup_table_statistics & statistics)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return update_vlo_lookup_table_statistics_(scan.scan1, statistics);
    case VLO_VERSION_3:
      return update_vlo_lookup_table_statistics_(scan.scan3, statistics);
    case VLO_VERSION_5:
      return update_vlo_lookup_table_statistics_(scan.scan5, statistics);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}

static bool save_vlo_lookup_table_statistics(const std::string & filename,
    const c_vlo_lookup_table_statistics & table)
{
  FILE * fp = nullptr;


  if ( !create_path(get_parent_directory(filename)) ) {
    CF_ERROR("create_path() for '%s' fails: %s", filename.c_str(), strerror(errno));
    return false;
  }



  if( !(fp = fopen(filename.c_str(), "w")) ) {
    CF_ERROR("FATAL: can not write file '%s' : %s ", filename.c_str(), strerror(errno));
    return false;
  }



  fprintf(fp, "BIN\tDIST\tAREA_MEAN\tAREA_STDEV\tPEAK_MEAN\tPEAK_STDEV\tNPTS\n");

  for ( int i = 0, n = table.size(); i < n; ++i ) {

    const c_vlo_lookup_table_item & item =
        table[i];

    if ( item.num_measurements < 1 ) {
      continue;
    }

    const double N = item.num_measurements;
    const double mean_distance = item.distance / N;
    const double mean_peak = item.peak / N;
    const double mean_area = item.area / N;
    const double stdev_peak = sqrt(item.peak2 / N - mean_peak * mean_peak);
    const double stdev_area = sqrt(item.area2 / N - mean_area * mean_area);

    fprintf(fp, "%5d\t%6.1f"
        "\t%12f\t%12f"
        "\t%7f\t%7f"
        "\t%15g"
        "\n",
        i,
        mean_distance,
        mean_area, stdev_area,
        mean_peak, stdev_peak,
        N
        );

  }


  fclose(fp);

  return true;
}

//////////////////////////////


c_vlo_pipeline::c_vlo_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_vlo_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
    SERIALIZE_OPTION(section, save, input_options_, sort_echos_by_distance);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "processing_options")) ) {

    SERIALIZE_OPTION(section, save, processing_options_, enable_blom_detection2);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_distance);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, max_distance);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, vlo_walk_error);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_slope);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, max_slope);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, counts_threshold);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_segment_size);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_height);

    SERIALIZE_OPTION(section, save, processing_options_, saturation_level);
    SERIALIZE_OPTION(section, save, processing_options_, enable_filter_segments);

    SERIALIZE_OPTION(section, save, processing_options_, enable_gather_lookup_table_statistics);
    SERIALIZE_OPTION(section, save, processing_options_, vlo_lookup_table_statistics_filename);

    SERIALIZE_OPTION(section, save, processing_options_, vlo_intensity_channel);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    // SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_ply);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_filename);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_intensity_channel);

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_writer_options);

    SERIALIZE_OPTION(section, save, output_options_, save_bloom2_segments);
    SERIALIZE_OPTION(section, save, output_options_, segments_writer_options);

    SERIALIZE_OPTION(section, save, output_options_, save_bloom2_intensity_profiles);
    SERIALIZE_OPTION(section, save, output_options_, intensity_writer_options);

    SERIALIZE_OPTION(section, save, output_options_, save_bloom2_display);
    SERIALIZE_OPTION(section, save, output_options_, display_writer_options);

    SERIALIZE_OPTION(section, save, output_options_, save_walls);
    SERIALIZE_OPTION(section, save, output_options_, walls_writer_options);

//    SERIALIZE_OPTION(section, save, output_options_, save_filtered_profiles);
//    SERIALIZE_OPTION(section, save, output_options_, filtered_writer_options);

  }

  return true;
}


const std::vector<c_image_processing_pipeline_ctrl>& c_vlo_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
      PIPELINE_CTL(ctrls, input_options_.sort_echos_by_distance, "sort_echos_by_distance", "");
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_GROUP(ctrls, "Processing options", "");

      PIPELINE_CTL_GROUP(ctrls, "VLO lookup table statistics", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_gather_lookup_table_statistics,
            "enable_gather_lookup_table_statistics", "");
        PIPELINE_CTLC(ctrls, processing_options_.vlo_intensity_channel, "vlo_intensity_channel", "",
            _this->processing_options_.enable_gather_lookup_table_statistics);
        PIPELINE_CTLC(ctrls, processing_options_.vlo_lookup_table_statistics_filename, "vlo_statistics_filename", "",
            _this->processing_options_.enable_gather_lookup_table_statistics);
      PIPELINE_CTL_END_GROUP(ctrls);


      PIPELINE_CTL_GROUP(ctrls, "Blom Detection2", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_blom_detection2, "enable_bloom_detection2", "");

        PIPELINE_CTL(ctrls, processing_options_.saturation_level, "saturation_level", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_filter_segments, "filter segments", "");

        PIPELINE_CTL_GROUP(ctrls, "Depth segmentation", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_distance, "min_distance", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.max_distance, "max_distance", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.vlo_walk_error, "walk_error", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.counts_threshold, "counts_threshold", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_segment_size, "min_segment_size", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_slope, "min_slope", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.max_slope, "max_slope", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_height, "min_height", "");
        PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      // PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL(ctrls, output_options_.save_cloud3d_ply, "save cloud3d ply", "Set checked if you want to save 3D clouds in PLY format");
      PIPELINE_CTLC(ctrls, output_options_.cloud3d_filename, "cloud3d_filename", "", _this->output_options_.save_cloud3d_ply);
      PIPELINE_CTLC(ctrls, output_options_.cloud3d_intensity_channel, "cloud3d_intensity_channel", "", _this->output_options_.save_cloud3d_ply);


      PIPELINE_CTL_GROUP(ctrls, "save_progress_video", "");
      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.progress_writer_options, _this->output_options_.save_progress_video);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "save_bloom2_display", "");
      PIPELINE_CTL(ctrls, output_options_.save_bloom2_display, "save_bloom2_display", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.display_writer_options, _this->output_options_.save_bloom2_display);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "save_bloom2_segments", "");
      PIPELINE_CTL(ctrls, output_options_.save_bloom2_segments, "save_bloom2_segments", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.segments_writer_options, _this->output_options_.save_bloom2_segments);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "save_intensity_profiles", "");
      PIPELINE_CTL(ctrls, output_options_.save_bloom2_intensity_profiles, "save_intensity_profiles", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.intensity_writer_options, _this->output_options_.save_bloom2_intensity_profiles);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "save_walls", "");
      PIPELINE_CTL(ctrls, output_options_.save_walls, "save_walls", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.walls_writer_options, _this->output_options_.save_walls);
      PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_vlo_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_vlo_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->input_options_ = this->input_options_;
  p->processing_options_ = this->processing_options_;
  p->output_options_ = this->output_options_;

  return true;
}

bool c_vlo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  cv::Mat image =
      c_vlo_file::get_image(current_scan_,
          c_vlo_reader::DATA_CHANNEL_AMBIENT);

  if( !image.empty() ) {
    autoclip(image, cv::noArray(),
        0.5, 99.5,
        0, 255);
    if( image.depth() != CV_8U ) {
      image.convertTo(image, CV_8U);
    }
  }

  display_frame.move(image);
  display_mask.release();

  return true;
}

bool c_vlo_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

  const int num_sources =
      input_sequence_->sources().size();

  for( int i = 0; i < num_sources; ++i ) {

    const c_input_source::sptr &source =
        input_sequence_->source(i);

    if (source->enabled() ) {

      c_vlo_input_source *vlo =
          dynamic_cast<c_vlo_input_source*>(source.get());

      if( !vlo ) {
        CF_ERROR("ERROR: input source %d is not a VLO source", i);
        return false;
      }
    }
  }


  vlo_lookup_table_statistics_.clear();

  return true;
}

void c_vlo_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  if ( !vlo_lookup_table_statistics_.empty() ) {

    const std::string filename =
        generate_output_filename(processing_options_.vlo_lookup_table_statistics_filename,
            "vlo-statistics",
            ".txt");

    CF_DEBUG("SAVING '%s'", filename.c_str());

    if ( !save_vlo_lookup_table_statistics(filename, vlo_lookup_table_statistics_) ) {
      CF_ERROR("save_vlo_lookup_table_statistics('%s') fails", filename.c_str());
    }
  }

}

bool c_vlo_pipeline::run_pipeline()
{
  if( !input_sequence_->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if( is_live_sequence ) {
    total_frames_ = INT_MAX;
  }
  else {

    const int start_pos =
        std::max(input_options_.start_frame_index, 0);

    const int end_pos =
        input_options_.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                input_options_.start_frame_index + input_options_.max_input_frames);

    total_frames_ = end_pos - start_pos;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          total_frames_,
          start_pos,
          end_pos,
          input_sequence_->size(),
          input_options_.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");
  CF_DEBUG("total_frames_=%d", total_frames_);

  processed_frames_ = 0;
  accumulated_frames_ = 0;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    c_vlo_input_source *vlo =
        dynamic_cast<c_vlo_input_source*>(input_sequence_->current_source().get());
    if( !vlo ) {
      CF_ERROR("dynamic_cast<c_vlo_input_source*>(current_source) fails");
      return false;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !vlo->read(&current_scan_) ) {
        CF_ERROR("vlo->read(scan) fails: %s", strerror(errno));
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++accumulated_frames_;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const int read_pos =
        input_sequence_->current_pos() + 1;
    if( !input_sequence_->seek(read_pos) ) {
      CF_ERROR("input_sequence_->seek(read_pos=%d) fails: %s",
          read_pos, strerror(errno));
      break;
    }

  }

  return true;
}


bool c_vlo_pipeline::process_current_frame()
{
  if( input_options_.sort_echos_by_distance && !c_vlo_file::sort_echos_by_distance(current_scan_) ) {
    CF_ERROR("c_vlo_file::sort_echos_by_distance() fails");
    return false;
  }

  if ( !run_blom_detection2() ) {
    CF_ERROR("run_bloom_detection2() fails");
    return false;
  }

  if ( !update_vlo_lookup_table_statistics() ) {
    CF_ERROR("update_vlo_lookup_table_statistics() fails");
    return false;
  }

  if( !save_progress_video() ) {
    CF_ERROR("save_progress_video() fails");
    return false;
  }

  if( !save_cloud3d_ply() ) {
    CF_ERROR("save_cloud3d_ply() fails");
    return false;
  }

  return true;
}

bool c_vlo_pipeline::run_blom_detection2()
{
  if( !processing_options_.enable_blom_detection2 ) {
    return true; // silently ignore
  }

  cv::Mat3f clouds[3];
  cv::Mat4f histogram;
  cv::Mat3w segments;

  bool fOk =
      c_vlo_file::get_clouds3d(current_scan_,
          clouds);

  if ( !fOk ) {
    CF_ERROR("c_vlo_file::get_clouds3d() fails");
    return false;
  }


  fOk =
      vlo_depth_segmentation(clouds,
          histogram,
          segments,
          processing_options_.depth_segmentation_);

  if( !fOk ) {
    CF_ERROR("vlo_depth_segmentation() fails");
    return false;
  }

  ///////////////////////////////////////////////////////////////////////////
  // Filter segments using intensity profiles

  if ( processing_options_.enable_filter_segments ) {

    cv::Mat3f intensities;

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_ECHO_PEAK).convertTo(intensities,
            CV_32F);

    struct c_segment_point
    {
      int y;
      float intens;
    };

    struct c_segment
    {
      std::vector<c_segment_point> points;
      bool saturated = false;
    };

    std::vector<c_segment> wall_segments;
    std::vector<c_segment> chunks;

    wall_segments.reserve(32);
    chunks.reserve(32);

    CF_DEBUG("processing_options_.saturation_level=%g", processing_options_.saturation_level);

    for( int x = 0; x < segments.cols; ++x ) {

      ///////

      wall_segments.clear();

      for( int y = 0; y < segments.rows; ++y ) {

        for( int e = 0; e < 3; ++e ) {

          const uint16_t seg_id =
              segments[y][x][e];

          if( seg_id ) {

            if( wall_segments.size() < seg_id ) {
              wall_segments.resize(seg_id);
            }

            c_segment & s =
                wall_segments[seg_id - 1];

            const float & intens =
                intensities[y][x][e];

            if( s.points.empty() || s.points.back().y != y ) {
              s.points.emplace_back(c_segment_point { y, intens });
            }
            else if( intens > s.points.back().intens ) {
              s.points.back().intens = intens;
            }
          }
        }
      }

      ///////

      for( uint16_t seg_id = 1; seg_id <= wall_segments.size(); ++seg_id ) {

        c_segment & seg =
            wall_segments[seg_id - 1];

        if( seg.points.empty() ) {
          continue;
        }

        chunks.clear();

        for( int p = 0; p < seg.points.size(); ++p ) {

          // collect saturated chunk if exists
          if( seg.points[p].intens >= processing_options_.saturation_level ) {

            chunks.emplace_back();
            c_segment & chunk = chunks.back();

            chunk.saturated = true;

            while (p < seg.points.size() && seg.points[p].intens >= processing_options_.saturation_level) {

              const c_segment_point & sp =
                  seg.points[p];

              chunk.points.emplace_back(sp);

              for( int e = 0; e < 3; ++e ) {
                if( segments[sp.y][x][e] == seg_id ) {
                  segments[sp.y][x][e] = 0;
                }
              }

              ++p;
            }
          }

          // collect unsaturated chunk if exists
          if( p + 1 < seg.points.size() ) {

            chunks.emplace_back();
            c_segment & chunk = chunks.back();

            while (p < seg.points.size() && seg.points[p].intens < processing_options_.saturation_level) {
              chunk.points.emplace_back(seg.points[p++]);
            }
          }
        }

      }

    }
  }

  // END OF Filter segments using intensity profiles
  ///////////////////////////////////////////////////////////////////////////


  if( output_options_.save_bloom2_segments ) {

    if( !add_output_writer(bloom2_segments_writer_, output_options_.segments_writer_options, "segments", ".ser") ) {
      CF_ERROR("bloom2_segments_writer_.open(%s) fails",
          bloom2_segments_writer_.filename().c_str());
      return false;
    }


    if( !bloom2_segments_writer_.write(segments) ) {
      CF_ERROR("bloom2_segments_writer_.write(%s) fails",
          bloom2_segments_writer_.filename().c_str());
      return false;
    }
  }

  ///////////////////////////////////////////////////////////////////////////
  // Save intensity profiles for individual segments

  if ( output_options_.save_bloom2_intensity_profiles ) {

    cv::Mat3f intensity_image;

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_ECHO_PEAK).convertTo(intensity_image,
            CV_32F);


    cv::Mat1f intensity(clouds[0].size());

    for( int segment_id = 1;; ++segment_id ) {

      intensity.setTo(0);

      int num_pixels_labeled = 0;

      for( int y = 0; y < segments.rows; ++y ) {
        for( int x = 0; x < segments.cols; ++x ) {

          int ne = 0;

          for( int e = 0; e < 3; ++e ) {

            if( segments[y][x][e] == segment_id ) {

              intensity[y][x] += intensity_image[y][x][e];
              ++ne;
              ++num_pixels_labeled;

            }
          }

          if( ne > 1 ) {
            intensity[y][x] /= ne;
          }
        }
      }

      if( !num_pixels_labeled ) {
        break;
      }

      cv::putText(intensity, ssprintf("F %d L %d", input_sequence_->current_pos(), segment_id),
          cv::Point(16, 32),
          cv::FONT_HERSHEY_COMPLEX,
          0.75,
          cv::Scalar::all(140),
          1, cv::LINE_AA,
          false);

      if( !add_output_writer(bloom2_intensity_writer_, output_options_.intensity_writer_options, "intensity", ".ser") ) {
        CF_ERROR("bloom2_intensity_writer_.open(%s) fails",
            bloom2_intensity_writer_.filename().c_str());
        return false;
      }

      if( !bloom2_intensity_writer_.write(intensity) ) {
        CF_ERROR("bloom2_intensity_writer_.write(%s) fails",
            bloom2_segments_writer_.filename().c_str());
        return false;
      }
    }
  }

  // END OF Save intensity profiles for individual segments
  ///////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////
  // Label light walls

  if ( output_options_.save_walls ) {

    static const auto label_wall =
        [](const cv::Mat3f & intensity_image, const cv::Mat3f & depth_image, const cv::Mat3w & segments_image,
            double mean_depth,
            cv::Mat1f & wall_image, cv::Mat3b & mask)
            {
              for (int y = 0; y < intensity_image.rows; ++y ) {
                for (int x = 0; x < intensity_image.cols; ++x ) {
                  for (int e = 0; e < 3; ++e ) {
                    if ( segments_image[y][x][e] && std::abs(depth_image[y][x][e] - mean_depth) < 100 ) {
                      wall_image[y][x] = intensity_image[y][x][e];
                      mask[y][x][e] = 255;
                    }
                  }
                }
              }
            };



    cv::Mat3f intensity_image;
    cv::Mat3f depth_image;

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_ECHO_PEAK).convertTo(intensity_image,
            CV_32F);

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_DEPTH).convertTo(depth_image,
            CV_32F);



    cv::Mat1f wall_image(segments.size());
    cv::Mat3b mask(segments.size(), cv::Vec3b::all(0));

    int wall_id = 0;

    for( int x = 0; x < segments.cols; ++x ) {

      for( int y = 0; y < segments.rows; ++y ) {

        for( int e = 0; e < 3; ++e ) {

          if( segments[y][x][e] && !mask[y][x][e] ) {

            double mean_segment_depth = 0;
            int n = 0;

            const int segment_id =
                segments[y][x][e];

            for( int yy = y; yy < segments.rows; ++yy ) {
              for( int ee = 0; ee < 3; ++ee ) {
                if ( segments[yy][x][ee] == segment_id ) {
                  mean_segment_depth += depth_image[yy][x][ee];
                  ++n;
                }
              }
            }

            if( n > 30 ) {

              mean_segment_depth /= n;
              ++wall_id;

              wall_image.setTo(cv::Scalar::all(0));

              label_wall(intensity_image, depth_image, segments,
                  mean_segment_depth,
                  wall_image,
                  mask);

              cv::putText(wall_image, ssprintf("F %d L %d", input_sequence_->current_pos(), segment_id),
                  cv::Point(16, 32),
                  cv::FONT_HERSHEY_COMPLEX,
                  0.75,
                  cv::Scalar::all(140),
                  1, cv::LINE_AA,
                  false);

              if( !add_output_writer(bloom2_walls_writer_, output_options_.walls_writer_options, "walls", ".ser") ) {
                CF_ERROR("bloom2_walls_writer_.open(%s) fails",
                    bloom2_walls_writer_.filename().c_str());
                return false;
              }

              if( !bloom2_walls_writer_.write(wall_image) ) {
                CF_ERROR("bloom2_walls_writer_.write('%s') fails",
                    bloom2_walls_writer_.filename().c_str());
                return false;
              }

            }
          }
        }
      }

    }


  }

  // END OF Label light walls
  ///////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////
  // Save BLOM2 Output Display

  if( output_options_.save_bloom2_display ) {
    // segments & counts

    std::vector<cv::Mat1f> histogram_channels;
    cv::split(histogram, histogram_channels);


    const cv::Size display_size =
        cv::Size(std::max(std::max(clouds[0].cols, segments.cols), histogram_channels[0].cols),
            clouds[0].rows + segments.rows + histogram_channels[0].rows);

    const cv::Rect roi[3] = {
        cv::Rect(0, 0, clouds[0].cols, clouds[0].rows), // depths
        cv::Rect(0, clouds[0].rows, segments.cols, segments.rows), // segments
        cv::Rect(0, clouds[0].rows + segments.rows, histogram_channels[0].cols, histogram_channels[0].rows), // counts
    };

    cv::Mat3f display_image(display_size,
        cv::Vec3w::all(0));

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_DEPTH).convertTo(display_image(roi[0]),
        display_image.depth());

    segments.convertTo(display_image(roi[1]),
        display_image.depth());

    std::vector<cv::Mat1f> counts_channels( {
        histogram_channels[0],
        histogram_channels[2],
        cv::Mat1f(histogram_channels[0].size(), 0.f)
    });

    cv::merge(counts_channels, display_image(roi[2]));

    if( !add_output_writer(blom2_display_writer_, output_options_.display_writer_options, "display", ".ser") ) {
      CF_ERROR("blom2_display_writer_.open(%s) fails",
          blom2_display_writer_.filename().c_str());
      return false;
    }

    if( !blom2_display_writer_.write(display_image) ) {
      CF_ERROR("blom2_display_writer_.write('%s') fails",
          blom2_display_writer_.filename().c_str());
      return false;
    }
  }

  // END OF Save BLOM2 Output Display
  ///////////////////////////////////////////////////////////////////////////

  return true;
}

bool c_vlo_pipeline::update_vlo_lookup_table_statistics()
{
  if ( !processing_options_.enable_gather_lookup_table_statistics ) {
    return true; // silently ignore
  }

  // update here
  if ( !::update_vlo_lookup_table_statistics(current_scan_, vlo_lookup_table_statistics_) ) {
    CF_ERROR("update_vlo_lookup_table_statistics() fails");
    return false;
  }

  // success
  return true;
}


bool c_vlo_pipeline::save_progress_video()
{
  if( !output_options_.save_progress_video ) {
    return true; // silently ignore
  }

  cv::Mat display_image, display_mask;

  if ( !get_display_image(display_image, display_mask) ) {
    CF_ERROR("get_display_image() fails");
    return false;
  }

  if( !add_output_writer(progress_writer_, output_options_.progress_writer_options, "progress", ".avi") ) {
    CF_ERROR("progress_writer_.open(%s) fails",
        progress_writer_.filename().c_str());
    return false;
  }

  if( !progress_writer_.write(display_image, display_mask) ) {
    CF_ERROR("progress_writer_.write(%s) fails",
        progress_writer_.filename().c_str());
    return false;
  }

  return true;
}

bool c_vlo_pipeline::save_cloud3d_ply()
{
  if( !output_options_.save_cloud3d_ply) {
    return true; // silently ignore
  }

  std::vector<cv::Point3f> points;
  std::vector<cv::Vec3b> colors;

  bool fOk =
      c_vlo_file::get_cloud3d(current_scan_,
          output_options_.cloud3d_intensity_channel,
          points,
          colors);

  if( !fOk ) {
    CF_ERROR("get_cloud3d() fails");
    return false;
  }

  const std::string filename =
      ssprintf("%s/cloud3d/%s/cloud3d.%03d.ply",
          output_path_.c_str(),
          input_sequence_->name().c_str(),
          input_sequence_->current_pos() - 1);

  if ( !save_ply(points, colors, filename) ) {
    CF_ERROR("save_ply('%s') fails", filename.c_str());
    return false;
  }

  return true;
}

