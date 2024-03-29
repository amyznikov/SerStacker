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

  QImageSavePNGOptions(QWidget * parent = Q_NULLPTR);

  void setPixelDepth(PIXEL_DEPTH v);
  PIXEL_DEPTH pixelDepth() const;

protected:
  QComboBox * pixtype_ctl = Q_NULLPTR;
};

class QImageSaveTIFFOptions :
    public QSettingsWidget
{
public:
  typedef QImageSaveTIFFOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<PIXEL_DEPTH> QPixelDepthCombo;


  QImageSaveTIFFOptions(QWidget * parent = Q_NULLPTR);

  void setPixelDepth(PIXEL_DEPTH v);
  PIXEL_DEPTH pixelDepth() const;

  void setTiffCompression(int v);
  int tiffCompression() const;

  void setEmbedAlphaMask(bool v);
  bool embedAlphaMask() const;

  QCheckBox * embedAlphaMaskCtl() const;

protected:
  QPixelDepthCombo * pixelDepth_ctl = Q_NULLPTR;
  QCheckBox * embedAlphaMask_ctl = Q_NULLPTR;
  QComboBox * compression_ctl = Q_NULLPTR;
};

class QImageSaveJPEGOptions :
    public QSettingsWidget
{
public:
  typedef QImageSaveJPEGOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageSaveJPEGOptions(QWidget * parent = Q_NULLPTR);

  void setJpegQuality(double v);
  double jpegQuality() const;

protected:
  QNumericBox * quality_ctl = Q_NULLPTR;
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


  QImageSaveOptions(QWidget * parent = Q_NULLPTR);

  QImageSaveFormat format() const;
  void set_format(QImageSaveFormat v);

  QImageSavePNGOptions * pngOptions() const;
  QImageSaveTIFFOptions * tiffOptions() const;
  QImageSaveJPEGOptions * jpegOptions() const;
  QCheckBox * saveProcessorConfigCtl() const;


protected:
  std::vector<int> imwrite_params_;

  QImageSaveFormatCombo * format_ctl = Q_NULLPTR;
  QStackedWidget * stack_ctl = Q_NULLPTR;
  QImageSaveTIFFOptions * tiffOptions_ctl = Q_NULLPTR;
  QImageSavePNGOptions * pngOptions_ctl = Q_NULLPTR;
  QImageSaveJPEGOptions * jpegOptions_ctl = Q_NULLPTR;
  QCheckBox * save_also_processor_config_ctl = Q_NULLPTR;
};


class QImageSaveOptionsDialog :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageSaveOptionsDialog ThisClass;
  typedef QDialog Base;

  QImageSaveOptionsDialog(QWidget * parent = Q_NULLPTR);

  QImageSaveFormat format() const;
  void set_format(QImageSaveFormat v);

  QImageSavePNGOptions * pngOptions() const;
  QImageSaveTIFFOptions * tiffOptions() const;
  QImageSaveJPEGOptions * jpegOptions() const;
  QCheckBox * saveProcessorConfigCtl() const;

protected:
  QImageSaveOptions * options_ctl = Q_NULLPTR;
  QPushButton * ok_ctl = Q_NULLPTR;
  QPushButton * cancel_ctl = Q_NULLPTR;
};

QString saveImageFileAs(QWidget * parent,
    const cv::Mat & currentImage,
    const cv::Mat & currentMask,
    const c_image_processor::sptr & currentProcesor = nullptr,
    const QString & fileName = "");

#endif /* __QImageSaveOptions_h__ */
