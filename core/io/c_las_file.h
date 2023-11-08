/*
 * c_las_file.h
 *
 *  Created on: Nov 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_las_file_h__
#define __c_las_file_h__

#if HAVE_LASLIB

#include <laslib/lasreader.hpp>
#include <laslib/laswriter.hpp>
#include <string>


class c_las_file
{
public:
  typedef c_las_file this_class;

  c_las_file();

  const std::string & filename() const;

protected:
  std::string filename_;
};

class c_las_reader :
    public c_las_file
{
public:
  typedef c_las_reader this_class;
  typedef c_las_file base;

  c_las_reader();

  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;
  const LASpoint * read_point();

  const LASheader * header() const;


//  const LASpoint * point() const;
//  I64 npoints;
//  I64 p_count;
//  LASTransformMatrix transform_matrix;


protected:
  LASreader * las_ = nullptr;
};

class c_las_writer :
    public c_las_file
{
public:
  typedef c_las_writer this_class;
  typedef c_las_file base;

  c_las_writer();

  bool create(const std::string & filename);
  void close();
  bool is_open() const;

protected:
  LASwriter * las_ = nullptr;
};

#endif // HAVE_LASLIB
#endif /* __c_las_file_h__ */
