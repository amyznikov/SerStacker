/*
 * QImageSaveOptions.h
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageSaveOptions_h__
#define __QImageSaveOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class QImageSavePNGOptions :
    public QSettingsWidget
{
public:
  typedef QImageSavePNGOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageSavePNGOptions(QWidget * parent = nullptr);

  void setPixelDepth(PIXEL_DEPTH v);
  PIXEL_DEPTH pixelDepth() const;

protected:
  QComboBox * pixtype_ctl = nullptr;
};

class QImageSaveTIFFOptions :
    public QSettingsWidget
{
public:
  typedef QImageSaveTIFFOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<PIXEL_DEPTH> QPixelDepthCombo;


  QImageSaveTIFFOptions(QWidget * parent = nullptr);

  void setPixelDepth(PIXEL_DEPTH v);
  PIXEL_DEPTH pixelDepth() const;

  void setTiffCompression(int v);
  int tiffCompression() const;

  void setEmbedAlphaMask(bool v);
  bool embedAlphaMask() const;

  QCheckBox * embedAlphaMaskCtl() const;

protected:
  QPixelDepthCombo * pixelDepth_ctl = nullptr;
  QCheckBox * embedAlphaMask_ctl = nullptr;
  QComboBox * compression_ctl = nullptr;
};

class QImageSaveJPEGOptions :
    public QSettingsWidget
{
public:
  typedef QImageSaveJPEGOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageSaveJPEGOptions(QWidget * parent = nullptr);

  void setJpegQuality(double v);
  double jpegQuality() const;

protected:
  QNumericBox * quality_ctl = nullptr;
};


enum QImageSaveFormat {
  QImageSaveFormatUnknown = -1,
  QImageSaveTIFF = 0,
  QImageSavePNG = 1,
  QImageSaveJPEG = 2,
  QImageSaveFLO = 3,
};
//
//const extern struct QImageSaveFormat_desc {
//  const char * name;
//  enum QImageSaveFormat value;
//} QImageSaveFormats[];
//
//QString toString(enum QImageSaveFormat m);
//enum QImageSaveFormat fromString(const QString & s,
//    enum QImageSaveFormat defval);



class QImageSaveOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageSaveOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<QImageSaveFormat> QImageSaveFormatCombo;


  QImageSaveOptions(QWidget * parent = nullptr);

  QImageSaveFormat format() const;
  void set_format(QImageSaveFormat v);

  QImageSavePNGOptions * pngOptions() const;
  QImageSaveTIFFOptions * tiffOptions() const;
  QImageSaveJPEGOptions * jpegOptions() const;
  QCheckBox * saveProcessorConfigCtl() const;


protected:
  std::vector<int> imwrite_params_;

  QImageSaveFormatCombo * format_ctl = nullptr;
  QStackedWidget * stack_ctl = nullptr;
  QImageSaveTIFFOptions * tiffOptions_ctl = nullptr;
  QImageSavePNGOptions * pngOptions_ctl = nullptr;
  QImageSaveJPEGOptions * jpegOptions_ctl = nullptr;
  QCheckBox * save_also_processor_config_ctl = nullptr;
};


class QImageSaveOptionsDialog :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageSaveOptionsDialog ThisClass;
  typedef QDialog Base;

  QImageSaveOptionsDialog(QWidget * parent = nullptr);

  QImageSaveFormat format() const;
  void set_format(QImageSaveFormat v);

  QImageSavePNGOptions * pngOptions() const;
  QImageSaveTIFFOptions * tiffOptions() const;
  QImageSaveJPEGOptions * jpegOptions() const;
  QCheckBox * saveProcessorConfigCtl() const;

protected:
  QImageSaveOptions * options_ctl = nullptr;
  QPushButton * ok_ctl = nullptr;
  QPushButton * cancel_ctl = nullptr;
};

QString saveImageFileAs(QWidget * parent,
    const cv::Mat & currentImage,
    const cv::Mat & currentMask,
    const c_image_processor::sptr & currentProcesor = nullptr,
    const QString & fileName = "");

#endif /* __QImageSaveOptions_h__ */
