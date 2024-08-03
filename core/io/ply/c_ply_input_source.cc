/*
 * c_ply_input_source.cc
 *
 *  Created on: Jan 24, 2024
 *      Author: amyznikov
 */

#include "c_ply_input_source.h"
#include "c_ply_frame.h"
#include <core/debug.h>

static bool drop_spaces(char *& s)
{
  while (*s && isspace(*s)) {
    ++s;
  }

  s[strcspn(s, "\r\n")] = 0;

  return *s;
}

static bool skip_spaces(const char *& s)
{
  while (*s && isspace(*s)) {
    ++s;
  }
  return *s;
}

static std::string copy_trim_extra_spaces(const char * s)
{
  std::string ss;

  for( skip_spaces(s); *s; ) {
    ss += *s;
    if( isspace(*s) ) {
      skip_spaces(s);
    }
    else {
      ++s;
    }
  }

  return ss;
}


static inline bool starts_with(const char * prefix, const std::string & s)
{
  // https://stackoverflow.com/questions/1878001/how-do-i-check-if-a-c-stdstring-starts-with-a-certain-string-and-convert-a
  return s.rfind(prefix, 0) == 0;  // pos=0 limits the search to the prefix
}

static bool read_ply_header_lines(FILE * fp, std::vector<std::string> & header)
{
  //ply
  //format ascii 1.0
  //element vertex 221424
  //property double x
  //property double y
  //property double z
  //property uchar red
  //property uchar green
  //property uchar blue
  //end_header

  char line[4096] = "";
  char * s;

  while (fgets(s = line, sizeof(line) - 1, fp)) {
    if( !drop_spaces(s) || *s == '#' ) {
      continue;
    }
    break;
  }

  if( strcasecmp(s, "ply") != 0 ) {
    CF_ERROR("No 'ply' header detected");
    return false;
  }

  while (fgets(s = line, sizeof(line) - 1, fp)) {

    if( !drop_spaces(s) || *s == '#' ) {
      continue;
    }

    //CF_DEBUG("s='%s'", s);

    if( strcasecmp(s, "end_header") == 0 ) {
      return true;
    }

    if( strncasecmp(s, "element", 7) == 0 || strncasecmp(s, "property", 8) == 0 ) {
      header.emplace_back(copy_trim_extra_spaces(s));
      continue;
    }

    if( strcasecmp(s, "format ascii 1.0") == 0 ) {
      continue;
    }

    CF_ERROR("Not supported keyword '%s' in ply file'", s);
    break;
  }


  return false;
}


static double get_property_scale(const char * proptype)
{
  if( strcasecmp(proptype, "float") == 0 ) {
    return 1;
  }
  if( strcasecmp(proptype, "double") == 0 ) {
    return 1;
  }
  if( strcasecmp(proptype, "uchar") == 0 ) {
    return 1;//. / 255;
  }

  return -1;
}

static bool loadPlyPointCloud(const std::string & filename,
    std::vector<cv::Vec3f> & output_points,
    std::vector<cv::Vec3f> & output_colors)
{
  FILE * fp = nullptr;
  std::vector<std::string> header;
  std::string line_format;

  int expected_vertext_count = -1;

  double x_property_scale = -1;
  double y_property_scale = -1;
  double z_property_scale = -1;
  double r_scale = -1;
  double g_scale = -1;
  double b_scale = -1;

  int x_pos = -1;
  int y_pos = -1;
  int z_pos = -1;
  int r_pos = -1;
  int g_pos = -1;
  int b_pos = -1;

  int prop_pos = 0;

  bool have_colors = true;

  bool fOk = false;

  output_points.clear();
  output_colors.clear();

  if( !(fp = fopen(filename.c_str(), "r")) ) {
    CF_ERROR("fopen('%s') fails: %s", filename.c_str(), strerror(errno));
    goto end;
  }

  if ( !read_ply_header_lines(fp, header) ) {
    CF_ERROR("read_ply_header_lines('%s') fails", filename.c_str());
    goto end;
  }



  for ( const std::string & line : header ) {

    if( starts_with("element vertex", line) ) {
      if( sscanf(line.c_str(), "element vertex %d", &expected_vertext_count) != 1 ) {
        CF_ERROR("Syntax error in ply headerf line '%s'", line.c_str());
        goto end;
      }
      continue;
    }

    if( starts_with("property", line) ) {

      char proptype[256] = "";
      char propname[256] = "";

      if( sscanf(line.c_str(), "property %255s %255s", proptype, propname) != 2 ) {
        CF_ERROR("Syntax error in ply header line '%s'", line.c_str());
        goto end;
      }

      if( strcasecmp(propname, "x") == 0 ) {
        x_pos = prop_pos++;
        if( (x_property_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else if( strcasecmp(propname, "y") == 0 ) {
        y_pos = prop_pos++;
        if( (y_property_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else if( strcasecmp(propname, "z") == 0 ) {
        z_pos = prop_pos++;
        if( (z_property_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else if( strcasecmp(propname, "red") == 0 ) {
        r_pos = prop_pos++;
        if( (r_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else if( strcasecmp(propname, "green") == 0 ) {
        g_pos = prop_pos++;
        if( (g_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else if( strcasecmp(propname, "blue") == 0 ) {
        b_pos = prop_pos++;
        if( (b_scale = get_property_scale(proptype)) < 0 ) {
          CF_ERROR("Unsupported property type '%s'", line.c_str());
          goto end;
        }
      }
      else {
        ++prop_pos;
      }
    }
  }

  if( x_pos < 0 || y_pos < 0 || z_pos < 0 ) {
    CF_ERROR("No valid 3D point coordinates defined in ply header: xpos=%d ypos=%d zpos=%d",
        x_pos, y_pos, z_pos);
    goto end;
  }

  if( r_pos < 0 || g_pos < 0 || b_pos < 0 ) {
    CF_WARNING("No valid RGB point colors defined in ply header: rpos=%d gpos=%d bpos=%d",
        r_pos, g_pos, b_pos);

    r_pos = g_pos = b_pos = -1;
    have_colors = false;
  }

  for ( int p = 0; p < prop_pos; ++p ) {
    if ( p == x_pos || p == y_pos || p == z_pos || p == r_pos || p == g_pos || p == b_pos ) {
      line_format.append("%lf ");
    }
    else {
      line_format.append("%*s ");
    }
  }

  line_format[line_format.size() - 1] = 0;

  if ( have_colors ) {


    double x, y, z, r, g, b;

    const char * format =
        line_format.c_str();

    if ( expected_vertext_count > 0 ) {
      output_points.reserve(expected_vertext_count);
      output_colors.reserve(expected_vertext_count);
    }

    while ( !feof(fp) )  {
      if ( fscanf(fp, format, &x, &y, &z, &r, &g, &b ) == 6 ) {
        output_points.emplace_back(x, y, z);
        output_colors.emplace_back(r * r_scale, g * g_scale, b * b_scale);
      }
    }


  }
  else {

    double x, y, z;

    const char * format =
        line_format.c_str();

    if ( expected_vertext_count > 0 ) {
      output_points.reserve(expected_vertext_count);
    }

    while ( !feof(fp) )  {
      if ( fscanf(fp, format, &x, &y, &z ) == 3 ) {
        output_points.emplace_back(x, y, z);
        output_colors.emplace_back(1, 1, 1);
      }
    }

  }

//  CF_DEBUG("output_points.size=%zu output_colors.size=%zu",
//      output_points.size(), output_colors.size());

  fOk = true;

end:
  if( fp ) {
    fclose(fp);
  }

  return fOk;
}

c_ply_input_source::c_ply_input_source(const std::string & filename) :
  base(filename)
{
  size_ = 1;
}


c_ply_input_source::sptr c_ply_input_source::create(const std::string & filename)
{
  return sptr(new this_class(filename));
}

const std::vector<std::string> & c_ply_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".ply"
  };

  return suffixes_;
}

bool c_ply_input_source::open()
{
  curpos_ = 0;
  return !filename_.empty();
}

void c_ply_input_source::close()
{
  curpos_ = -1;
}

bool c_ply_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_ply_frame * f =
      dynamic_cast<c_ply_frame*>(output_frame.get());

  if( !f ) {
    output_frame.reset(f = new c_ply_frame());
  }

  f->filename_ = this->filename_;


  if ( !loadPlyPointCloud(f->filename_, f->points_, f->colors_) ) {
    CF_ERROR("loadPlyPointCloud('%s') fails", f->filename_.c_str());
  }

  return true;
}

bool c_ply_input_source::read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpc)
{
  return false;
}

bool c_ply_input_source::seek(int pos)
{
  return ((curpos_ = pos) == 0);
}

int c_ply_input_source::curpos()
{
  return curpos_;
}

bool c_ply_input_source::is_open() const
{
  return curpos_ >= 0;
}
