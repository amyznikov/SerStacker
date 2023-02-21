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
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/debug.h>

class c_ply_file
{
public:
  typedef c_ply_file this_class;

  enum ply_file_format
  {
    ply_format_unknown = -1,
    ply_format_ascii,
    ply_format_binary_little_endian,
    ply_format_binary_big_endian,
  };

  // http://paulbourke.net/dataformats/ply/
  // https://en.wikipedia.org/wiki/PLY_(file_format)
  enum ply_property_type
  {
    ply_property_unknown = -1,
    ply_property_int8,    //      character                 1
    ply_property_uint8,   //      unsigned character        1
    ply_property_int16,   //      short integer             2
    ply_property_uint16,  //      unsigned short integer    2
    ply_property_int32,   //      integer                   4
    ply_property_uint32,  //      unsigned integer          4
    ply_property_float32, //      single-precision float    4
    ply_property_float64, //      double-precision float    8
  };

  struct property {
    std::string name;
    ply_property_type type = ply_property_unknown;
    ply_property_type list_size_type = ply_property_unknown;
  };

  struct element {
    std::string name;
    std::vector<property> properties;
    int size = 0;
  };

  const std::string & filename() const
  {
    return filename_;
  }

  enum ply_file_format format() const
  {
    return format_;
  }

  const std::vector<element>& elems() const
  {
    return elems_;
  }

  const struct element & elem(int index) const
  {
    return elems_[index];
  }

  static int datasize(const ply_property_type & prop_type)
  {
    switch (prop_type) {
      case ply_property_int8:    //      character                 1
        return 1;
      case ply_property_uint8:   //      unsigned character        1
        return 1;
      case ply_property_int16:   //      short integer             2
        return 2;
      case ply_property_uint16:  //      unsigned short integer    2
        return 2;
      case ply_property_int32:   //      integer                   4
        return 4;
      case ply_property_uint32:  //      unsigned integer          4
        return 4;
      case ply_property_float32: //      single-precision float    4
        return 4;
      case ply_property_float64: //      double-precision float    8
        return 8;
      default:
        break;
    }
    return -1;
  }

//  int datasize(const property & prop) const
//  {
//    if ( prop.list_size_type == ply_property_unknown ) {
//      return datasize(prop.type);
//    }
//
//    //return datasize(prop.
//
//  }

  int datasize(const element & elem) const
  {
    switch (format_) {
      case ply_format_ascii:
        return elem.size; // return number of lines

      case ply_format_binary_little_endian:
      case ply_format_binary_big_endian: {

        int propsize = 0;
        int size = 0;

        for ( int i = 0, n = elem.properties.size(); i < n; ++i ) {

        }


        break;
      }

      default:
        break;
    }

    return -1;
  }

  static enum ply_property_type parse_property_type(const char * s)
  {
    if( strcasecmp(s, "char") == 0 || strcasecmp(s, "int8") == 0 ) {
      return ply_property_int8; // character  1 byte
    }
    if( strcasecmp(s, "uchar") == 0 || strcasecmp(s, "uint8") == 0 ) {
      return ply_property_uint8; // unsigned character  1 byte
    }
    if( strcasecmp(s, "short") == 0 || strcasecmp(s, "int16") == 0 ) {
      return ply_property_int16; // short integer 2 bytes
    }
    if( strcasecmp(s, "ushort") == 0 || strcasecmp(s, "uint16") == 0 ) {
      return ply_property_uint16; // unsigned short integer 2 bytes
    }
    if( strcasecmp(s, "int") == 0 || strcasecmp(s, "int32") == 0 ) {
      return ply_property_int32; // integer  4 bytes
    }
    if( strcasecmp(s, "uint") == 0 || strcasecmp(s, "uint32") == 0 ) {
      return ply_property_uint32; // unsigned integer  4 bytes
    }
    if( strcasecmp(s, "float") == 0 || strcasecmp(s, "float32") == 0 ) {
      return ply_property_float32; // single-precision float 4 bytes
    }
    if( strcasecmp(s, "double") == 0 || strcasecmp(s, "float64") == 0 ) {
      return ply_property_float64; // double-precision float 8 bytes
    }

    return ply_property_unknown;
  }

  static enum ply_property_type parse_property_type(const std::string & text)
  {
    return parse_property_type(text.c_str());
  }


  const element* find_element(const std::string & name, int * filepos = nullptr) const
  {
//    const char * s = name.c_str();
//    const element * e = nullptr;
//    int datapos = 0;
//
//    for( int i = 0, n = elems_.size(); i < n; ++i ) {
//
//      const element &elem =
//          elems_[i];
//
//      if( strcasecmp(elem.name.c_str(), s) == 0 ) {
//        break;
//      }
//
//      datapos +=
//      start_line +=
//          elem.size();
//    }
//
//    if( start_line < 0 ) {
//      CF_ERROR("Element 'vertex' not found");
//      return false;
//    }

    return nullptr;

  }

  virtual ~c_ply_file() = default;

protected:
  c_ply_file()
  {
  }

  c_ply_file(const std::string & filename) :
    filename_(filename)
  {
  }

protected:
  std::string filename_;
  enum ply_file_format format_ = ply_format_unknown;
  std::vector<struct element> elems_;
};



class c_ply_reader :
    public c_ply_file
{
public:
  typedef c_ply_reader this_class;
  typedef c_ply_file base;

  c_ply_reader();
  c_ply_reader(const std::string & filename);
  ~c_ply_reader() override;

  bool open(const std::string & filename = "");
  bool is_open() const;
  void close();

//  template<class T>
//  bool read_vertices(std::vector<T> & pts,
//      const std::vector<std::string> & props = {"x", "y", "z"});

protected:
  void set_format(enum ply_file_format format);
  static bool read_line(FILE * fp, std::vector<std::string> * tokens);

//  template<class ContainerType>
//  bool read_vertices_ascii(FILE * fp, ContainerType & c);

protected:
  FILE * fp = nullptr;
  int start_data_pos = -1;
};

template<>
const c_enum_member* members_of<c_ply_file::ply_file_format>()
{
  static constexpr c_enum_member members[] {
      { c_ply_file::ply_format_ascii, "ascii", "ascii 1.0" },
      { c_ply_file::ply_format_binary_little_endian, "binary_little_endian", "binary little endian 1.0" },
      { c_ply_file::ply_format_binary_big_endian, "binary_big_endian", "binary big endian 1.0" },
      { c_ply_file::ply_format_unknown },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_ply_file::ply_property_type>()
{
  static constexpr c_enum_member members[] {
      { c_ply_file::ply_property_int8, "int8", "character 1 byte" },
      { c_ply_file::ply_property_uint8, "uint8", "unsigned character 1 byte" },
      { c_ply_file::ply_property_int16, "int16", "short integer 2 bytes" },
      { c_ply_file::ply_property_uint16, "uint16", "unsigned short integer 2 bytes" },
      { c_ply_file::ply_property_int32, "int32", "integer 4 bytes" },
      { c_ply_file::ply_property_uint32, "uint32", "unsigned integer 4 bytes" },
      { c_ply_file::ply_property_float32, "float32", "single-precision float 4 bytes" },
      { c_ply_file::ply_property_float64, "float64", "double-precision float 8 bytes" },
      { c_ply_file::ply_property_unknown },
  };

  return members;
}

c_ply_reader::c_ply_reader()
{
}

c_ply_reader::c_ply_reader(const std::string & filename) :
    base(filename)
{
}

c_ply_reader::~c_ply_reader()
{
  close();
}

void c_ply_reader::close()
{
  if ( fp ) {
    fclose(fp);
    fp = nullptr;
  }
}

bool c_ply_reader::is_open() const
{
  return fp != nullptr;
}

void c_ply_reader::set_format(enum ply_file_format format)
{
  this->format_ = format;
}


bool c_ply_reader::read_line(FILE * fp, std::vector<std::string> * tokens)
{
  int ch;
  std::string token;

  tokens->clear();

  while ((ch = fgetc(fp)) != EOF) {

    if( ch == '\n' ) {
      break;
    }

    if( !isspace(ch) ) {
      token.append(1, ch);
    }
    else if( !token.empty() ) {
      tokens->emplace_back(token);
      token.clear();
    }
  }

  if( !token.empty() ) {
    tokens->emplace_back(token);
    token.clear();
  }

  return !EOF || !tokens->empty();
}

bool c_ply_reader::open(const std::string & filename)
{
  std::vector<std::string> tokens;
  struct element elem;
  struct property prop;
  bool fOk = false;
  bool end_header = false;

  close();
  elems_.clear();
  set_format(ply_format_unknown);
  start_data_pos = -1;

  if ( !filename.empty() ) {
    this->filename_ = filename;
  }

  if( this->filename_.empty() ) {
    CF_ERROR("No input ply file name specified");
    return false;
  }

  if( !(fp = fopen(this->filename_.c_str(), "rb")) ) {
    CF_ERROR("fopen('%s', mode='rb') fails: %s",
        this->filename_.c_str(),
        strerror(errno));
    return false;
  }

  if( !read_line(fp, &tokens) || tokens.empty() ) {
    CF_ERROR("readline() fails for ply header: %s", strerror(errno));
    goto end;
  }

  if( strcasecmp(tokens[0].c_str(), "ply") != 0 ) {
    CF_ERROR("Not a ply file: first line starts from '%s'", tokens[0].c_str());
    goto end;
  }

  for( int line = 2; read_line(fp, &tokens); ++line ) {

    ///////////////////////////////////////////////////////////////////////////////////////////////
    if( tokens.empty() ) {
      continue; // empty line
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    if( strcasecmp(tokens[0].c_str(), "comment") == 0 ) {
      continue;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    if( strcasecmp(tokens[0].c_str(), "format") == 0 ) {

      if( strcasecmp(tokens[1].c_str(), "ascii") == 0 ) {
        set_format(ply_format_ascii);
      }
      else if( strcasecmp(tokens[1].c_str(), "binary_little_endian") == 0 ) {
        set_format(ply_format_binary_little_endian);
      }
      else if( strcasecmp(tokens[1].c_str(), "binary_big_endian") == 0 ) {
        set_format(ply_format_binary_big_endian);
      }
      else {
        CF_ERROR("Not supported ply file format '%s' at line %d", tokens[1].c_str(), line);
        goto end;
      }

      continue;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    while (strcasecmp(tokens[0].c_str(), "element") == 0) {

      elem.properties.clear();

      if( tokens[0].size() < 3 ) {
        CF_ERROR("Invalid element specification at line '%d'", line);
        goto end;
      }

      elem.name = tokens[1];
      if( sscanf(tokens[2].c_str(), "%d", &elem.size) != 1 ) {
        CF_ERROR("Can not parse element size '%s' at line '%d'", tokens[2].c_str(), line);
        goto end;
      }

      /////////////////////////////////////////////////////////////////////////////////////////////
      for( ++line; read_line(fp, &tokens); ++line ) {

        if ( tokens.empty() ) {
          continue;
        }

        if( strcasecmp(tokens[0].c_str(), "comment") == 0 ) {
          continue;
        }

        if( strcasecmp(tokens[0].c_str(), "property") == 0 ) {

          if( tokens.size() < 3 ) {
            CF_ERROR("Parse error at line %d", line);
            goto end;
          }

          else if( strcasecmp(tokens[1].c_str(), "list") != 0 ) {
            prop.name = tokens[2];
            if( (prop.type = parse_property_type(tokens[1])) == ply_property_unknown ) {
              CF_ERROR("Parse error at line %d", line);
              goto end;
            }
          }

          else if( tokens.size() < 5 ) {
            CF_ERROR("Parse error at line %d", line);
            goto end;
          }

          else {
            prop.name = tokens[4];

            if( (prop.list_size_type = parse_property_type(tokens[2])) == ply_property_unknown ) {
              CF_ERROR("Parse error at line %d", line);
              goto end;
            }

            if( (prop.type = parse_property_type(tokens[3])) == ply_property_unknown ) {
              CF_ERROR("Parse error at line %d", line);
              goto end;
            }
          }

          elem.properties.emplace_back(prop);
          continue;
        } // property

        break;
      }

      elems_.emplace_back(elem);
    } // while ("element")


    ///////////////////////////////////////////////////////////////////////////////////////////////
    if( strcasecmp(tokens[0].c_str(), "end_header") == 0 ) {
      end_header = true; // end of ply header
      start_data_pos = ftell(fp);
      break;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////

    CF_WARNING("WARNING: Ignore unparsed keyword '%s' at line %d of '%s'",
        tokens[0].c_str(),
        line,
        filename_.c_str());
  }

  if( !end_header ) {
    CF_ERROR("The 'end_header' keyword not found in ply file '%s'.\n"
        "Bad ply file format ?",
        this->filename_.c_str());
    goto end;
  }

  if( this->format_ == ply_format_unknown ) {
    CF_ERROR("The ply file format was not correctly specified in file header.\n"
        "Bad ply file format ?",
        this->filename_.c_str());
    goto end;
  }


  fOk = true;

end:
  if ( !fOk ) {
    close();
  }

  return fOk;
}


//template<class T>
//bool c_ply_reader::read_vertices(std::vector<T> & pts, const std::vector<std::string> & props)
//{
//  if( !is_open() ) {
//    CF_ERROR("ply file is not open");
//    return false;
//  }
//
//  switch (format_) {
//
//    case ply_format_ascii: {
//
//      return read_vertices_ascii(fp, c);
//    }
//
//    case ply_format_binary_big_endian:
//      break;
//    case ply_format_binary_little_endian:
//      break;
//    default:
//      break;
//  }
//
//  return false;
//}
//
//template<class ContainerType>
//bool c_ply_reader::read_vertices_ascii(FILE * fp, ContainerType & c)
//{
//  int start_line = -1;
//  int num_lines = -1;
//
//  // search the 'vertex' element
//
//  start_line = 0;
//  for ( int i = 0, n = elems_.size() ; i < n; ++i ) {
//
//    const element & elem =
//        elems_[i];
//
//    if ( strcasecmp(elem.name.c_str(), "vertex") == 0 ) {
//      num_lines =
//          elem.size();
//      break;
//    }
//
//    start_line +=
//        elem.size();
//  }
//
//  if ( start_line < 0 ) {
//    CF_ERROR("Element 'vertex' not found");
//    return false;
//  }
//
//  fseek(fp, start_data_pos, SEEK_SET);
//
//  // skip lines
//  for( int i = 0; i < start_line; ++i ) {
//
//    int c;
//    while ((c = fgetc(fp)) != EOF && c != '\n') {
//    }
//
//    if( c == EOF ) {
//      CF_ERROR("Unexpected en of file");
//      return false;
//    }
//  }
//
//  // read vertices
//  for( int i = 0; i < num_lines; ++i ) {
//
//  }
//
//
//  return false;
//}
//
//


int main(int argc, char *argv[])
{

  fprintf(stdout, "0x%0X (%d)\n", '\n', '\n');
  return 0;


  std::string input_file_name;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Test for ply file read.\n"
          "Usage:\n"
          "   alpha <input_ply_file.ply>\n"
          "\n");

      return 0;
    }

    if ( input_file_name.empty() ) {
      input_file_name = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if( input_file_name.empty() ) {
    fprintf(stderr, "input ply file expected\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  c_ply_reader ply;

  if ( !ply.open(input_file_name) ) {
    CF_ERROR("ply.open('%s') fails", input_file_name.c_str());
    return 1;
  }


  CF_DEBUG("ply format: %s", toString(ply.format()) );

  CF_DEBUG("Elements: count=%zu", ply.elems().size());

  for ( int i = 0, n = ply.elems().size(); i < n; ++i ) {

    const c_ply_file::element & elem =
        ply.elem(i);

    CF_DEBUG("elem[%d]: name=%s size=%d properties.size=%zu",
        i,
        elem.name.c_str(),
        elem.size,
        elem.properties.size());

    for ( int j = 0, m = elem.properties.size(); j < m; ++j ) {

      const c_ply_file::property & p =
          elem.properties[j];

      CF_DEBUG("   prop[%d]: name=%s type=%s lst=%s", j, p.name.c_str(), toString(p.type), toString(p.list_size_type));

    }

  }


  const c_ply_file::element *e = ply.find_element("vertex");
  if( !e ) {
    CF_ERROR("element 'vertex' nor found in ply file");
    return 1;
  }


//  start_data_pos


  return 0;
}


