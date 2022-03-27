/*
 * QImageSaveOptions.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#include "QImageSaveOptions.h"
#include <gui/qimageview/cv2qt.h>
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/normalize.h>
#include <core/io/save_image.h>
#include <tiff.h>
#include <tiffio.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<QImageSaveFormat>()
{
  static constexpr c_enum_member members[] = {
      { QImageSaveTIFF, "TIFF", },
      { QImageSavePNG, "PNG", },
      { QImageSaveJPEG, "JPEG", },
      { QImageSaveFormatUnknown, nullptr, }  // must  be last
  };

  return members;
}

//const struct QImageSaveFormat_desc QImageSaveFormats[] = {
//};
//
//QString toString(enum QImageSaveFormat v)
//{
//  for ( uint i = 0; QImageSaveFormats[i].name; ++i ) {
//    if ( QImageSaveFormats[i].value == v ) {
//      return QImageSaveFormats[i].name;
//    }
//  }
//  return "";
//}
//
//enum QImageSaveFormat fromString(const QString & s, enum QImageSaveFormat defval)
//{
//  const QByteArray utf8 = s.toUtf8();
//  const char * cstr = utf8.data();
//
//  for ( uint i = 0; QImageSaveFormats[i].name; ++i ) {
//    if ( strcasecmp(QImageSaveFormats[i].name, cstr) == 0 ) {
//      return QImageSaveFormats[i].value;
//    }
//  }
//  return defval;
//}

//static QString toString(const PIXEL_DEPTH v)
//{
//  return toStdString(v).c_str();
//}

//static PIXEL_DEPTH fromString(const QString & s, PIXEL_DEPTH defval)
//{
//  return fromStdString(s.toStdString(), defval);
//}
//

///////////////////////////////////////////////////////////////////////////////

QImageSavePNGOptions::QImageSavePNGOptions(QWidget * parent) :
    Base("QImageSavePNGOptions", parent)
{
  pixtype_ctl = add_combobox(
      "Pixel depth:");

  pixtype_ctl->addItem(toString(PIXEL_DEPTH_8U), QVariant::fromValue((int) (PIXEL_DEPTH_8U)));
  pixtype_ctl->addItem(toString(PIXEL_DEPTH_16U), QVariant::fromValue((int) (PIXEL_DEPTH_16U)));
  pixtype_ctl->setCurrentIndex(0);
}

void QImageSavePNGOptions::setPixelDepth(PIXEL_DEPTH v)
{
  int bestIndex;

  switch ( v ) {
  case PIXEL_DEPTH_8U :
    case PIXEL_DEPTH_8S :
    bestIndex = 0;  // 8 bit
    break;
  default :
    bestIndex = 1;  // 16 bit
    break;
  }

  pixtype_ctl->setCurrentIndex(bestIndex);
}

PIXEL_DEPTH QImageSavePNGOptions::pixelDepth() const
{
  return (PIXEL_DEPTH) pixtype_ctl->currentData().toInt();
}

QImageSaveTIFFOptions::QImageSaveTIFFOptions(QWidget * parent) :
    Base("QImageSaveTIFFOptions", parent)
{
  pixelDepth_ctl =
      add_enum_combobox<PIXEL_DEPTH>("Pixel type:",
          [this](PIXEL_DEPTH v) {
          });

  compression_ctl = 
      add_combobox( "TIFF Compression: ",
          std::function<void(int)>());


  compression_ctl->addItem("NONE", COMPRESSION_NONE);
  compression_ctl->addItem("LZW", COMPRESSION_LZW);
  compression_ctl->addItem("LZMA", COMPRESSION_LZMA);

  compression_ctl->setCurrentIndex(std::max(0,
      compression_ctl->findData(
          default_tiff_compression())));

  embedAlphaMask_ctl =
      add_checkbox("Embed alpha mask");

  embedAlphaMask_ctl->setChecked(true);
}

void QImageSaveTIFFOptions::setPixelDepth(PIXEL_DEPTH v)
{
  pixelDepth_ctl->setCurrentItem(v);
}

PIXEL_DEPTH QImageSaveTIFFOptions::pixelDepth() const
{
  return pixelDepth_ctl->currentItem();
}

void QImageSaveTIFFOptions::setEmbedAlphaMask(bool v)
{
  embedAlphaMask_ctl->setChecked(true);
}

bool QImageSaveTIFFOptions::embedAlphaMask() const
{
  return embedAlphaMask_ctl->isChecked();
}

QCheckBox * QImageSaveTIFFOptions::embedAlphaMaskCtl() const
{
  return embedAlphaMask_ctl;
}

void QImageSaveTIFFOptions::setTiffCompression(int v)
{
  int index = compression_ctl->findData(v);
  if ( index >= 0 ) {
    compression_ctl->setCurrentIndex(index);
  }
}

int QImageSaveTIFFOptions::tiffCompression() const
{
  return compression_ctl->currentData().toInt();
}

QImageSaveJPEGOptions::QImageSaveJPEGOptions(QWidget * parent) :
    Base("QImageSaveJPEGOptions", parent)
{
  quality_ctl = 
    add_numeric_box<double>("Jpeg quality",
      [this](double v) {
      });

  quality_ctl->setValue(99);
}

void QImageSaveJPEGOptions::setJpegQuality(double v)
{
  quality_ctl->setValue(v);
}

double QImageSaveJPEGOptions::jpegQuality() const
{
  double v = 99;
  fromString(quality_ctl->text(), &v);
  return v;
}

///////////////////////////////////////////////////////////////////////////////

QImageSaveOptions::QImageSaveOptions(QWidget * parent) :
    Base("QImageSaveOptions", parent)
{
  format_ctl =
      add_enum_combobox<QImageSaveFormat>("Format:",
          [this](QImageSaveFormat format) {

        switch (format) {
          case QImageSaveTIFF:
          stack_ctl->setVisible(true);
          stack_ctl->setCurrentWidget(tiffOptions_ctl);
          break;
          case QImageSavePNG:
          stack_ctl->setVisible(true);
          stack_ctl->setCurrentWidget(pngOptions_ctl);
          break;
          case QImageSaveJPEG:
          stack_ctl->setVisible(true);
          stack_ctl->setCurrentWidget(jpegOptions_ctl);
          break;
          default:
          stack_ctl->setVisible(false);
          break;
        }
      });

  save_also_processor_config_ctl =
      add_checkbox("Save also proc.cfg:");

  stack_ctl = add_widget<QStackedWidget>(QString());
  stack_ctl->addWidget(tiffOptions_ctl = new QImageSaveTIFFOptions());
  stack_ctl->addWidget(pngOptions_ctl = new QImageSavePNGOptions());
  stack_ctl->addWidget(jpegOptions_ctl = new QImageSaveJPEGOptions());


  updateControls();
}

QImageSaveFormat QImageSaveOptions::format() const
{
  return format_ctl->currentItem();
}

void QImageSaveOptions::set_format(QImageSaveFormat v)
{
  format_ctl->setCurrentItem(v);
  format_ctl->setEnabled(false);
}

QImageSavePNGOptions * QImageSaveOptions::pngOptions() const
{
  return pngOptions_ctl;

}

QImageSaveTIFFOptions * QImageSaveOptions::tiffOptions() const
{
  return tiffOptions_ctl;
}

QImageSaveJPEGOptions * QImageSaveOptions::jpegOptions() const
{
  return jpegOptions_ctl;
}

QCheckBox * QImageSaveOptions::saveProcessorConfigCtl() const
{
  return save_also_processor_config_ctl;
}

///////////////////////////////////////////////////////////////////////////////

QImageSaveOptionsDialog::QImageSaveOptionsDialog(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Save Options...");

  QVBoxLayout * layout =
      new QVBoxLayout(this);

  QHBoxLayout * hbox =
      new QHBoxLayout();

  hbox->addWidget(ok_ctl =
      new QPushButton("OK", this));

  hbox->addWidget(cancel_ctl =
      new QPushButton("Cancel", this));

  layout->addWidget(options_ctl = new QImageSaveOptions(this));
  layout->addLayout(hbox);

  ok_ctl->setDefault(true);
  ok_ctl->setAutoDefault(true);

  connect(ok_ctl, &QPushButton::clicked,
      this, &QDialog::accept);

  connect(cancel_ctl, &QPushButton::clicked,
      this, &QDialog::reject);
}

QImageSaveFormat QImageSaveOptionsDialog::format() const
{
  return options_ctl->format();

}

void QImageSaveOptionsDialog::set_format(QImageSaveFormat v)
{
  options_ctl->set_format(v);
}

QImageSavePNGOptions * QImageSaveOptionsDialog::pngOptions() const
{
  return options_ctl->pngOptions();
}

QImageSaveTIFFOptions * QImageSaveOptionsDialog::tiffOptions() const
{
  return options_ctl->tiffOptions();
}

QImageSaveJPEGOptions * QImageSaveOptionsDialog::jpegOptions() const
{
  return options_ctl->jpegOptions();
}

QCheckBox * QImageSaveOptionsDialog::saveProcessorConfigCtl() const
{
  return options_ctl->saveProcessorConfigCtl();
}

///////////////////////////////////////////////////////////////////////////////

QString saveImageFileAs(QWidget * parent,
    const cv::Mat & currentImage,
    const cv::Mat & currentMask,
    const c_image_processor::ptr & currentProcesor,
    const QString & fileName)
{

  // QString * savedFileName

  QSettings settings;

  static const QString keyName =
      "previousPathForSaveImageAs";

  QString filter =
      "Image files *.tiff *.tif *.png *.jpg(*.tiff *.tif *.png *.jpg);;\n";

  if ( currentImage.channels() == 2 ) {
    filter.append("Optical flow files *.flo(*.flo);;\n");
  }

  filter.append("All files (*);;");

  QString previousPathForSaveImageAs = fileName;
  if ( previousPathForSaveImageAs.isEmpty() ) {
    previousPathForSaveImageAs = settings.value(keyName).toString();
  }

  while ( 42 ) {

    QString selectedFileName =
        QFileDialog::getSaveFileName(parent,
            "Save image as...",
            previousPathForSaveImageAs,
            filter);

    if ( selectedFileName.isEmpty() ) {
      break;
    }

    enum QImageSaveFormat format =
        QImageSaveFormatUnknown;

    bool embedAlphaMask = true;
    bool saveProcessorConfig = false;

    std::vector<int> imwrite_params;
    PIXEL_DEPTH selectedPixelDepth = PIXEL_DEPTH_NO_CHANGE;

    if ( selectedFileName.endsWith(".flo", Qt::CaseInsensitive) ) {

      if ( currentImage.channels() != 2 ) {
        QMessageBox::critical(parent, "ERROR",
            QString("Top save optical flow the image must be 2-channel floating point.\n"
                "Current image channels is %1\n").arg(currentImage.channels()));

        continue;
      }

      format = QImageSaveFLO;
    }
    else if ( selectedFileName.endsWith(".png", Qt::CaseInsensitive) ) {
      format = QImageSavePNG;
    }
    else if ( selectedFileName.endsWith(".tiff", Qt::CaseInsensitive)
        || selectedFileName.endsWith(".tif", Qt::CaseInsensitive) ) {
      format = QImageSaveTIFF;
    }
    else if ( selectedFileName.endsWith(".jpg", Qt::CaseInsensitive)
        || selectedFileName.endsWith(".jpeg", Qt::CaseInsensitive) ) {
      format = QImageSaveJPEG;
    }
    else {
      QMessageBox::critical(parent, "ERROR",
          QString("Sorry about this...\n"
              "Not yet supported image format selected"));
      continue;
    }

    if ( format != QImageSaveFLO ) {

      static QImageSaveOptionsDialog * dlgbox;
      if ( !dlgbox ) {
        dlgbox = new QImageSaveOptionsDialog(parent);
        //dlgbox->setWindowTitle("Save Options...");
      }

      dlgbox->setParent(parent);
      dlgbox->set_format(format);
      dlgbox->saveProcessorConfigCtl()->setEnabled(currentProcesor != nullptr);
      dlgbox->tiffOptions()->embedAlphaMaskCtl()->setEnabled(!currentMask.empty());

      if ( dlgbox->exec() != QDialog::Accepted ) {
        break;
      }


      if ( currentProcesor ) {
        saveProcessorConfig = dlgbox->saveProcessorConfigCtl()->isChecked();
      }

      switch (format) {
      case QImageSaveFLO:
      {
        break;
      }
      case QImageSaveTIFF:
      {

        const QImageSaveTIFFOptions *tiffOptions =
            dlgbox->tiffOptions();

        selectedPixelDepth = tiffOptions->pixelDepth();
        embedAlphaMask = !currentMask.empty() && tiffOptions->embedAlphaMask();

        imwrite_params.emplace_back(cv::ImwriteFlags::IMWRITE_TIFF_COMPRESSION);
        imwrite_params.emplace_back(tiffOptions->tiffCompression());

        break;
      }
      case QImageSavePNG:
      {

        const QImageSavePNGOptions *pngOptions =
            dlgbox->pngOptions();

        selectedPixelDepth = pngOptions->pixelDepth();

        if ( embedAlphaMask && currentImage.channels() == 1 ) {
          embedAlphaMask = false;
        }

        break;
      }
      case QImageSaveJPEG:
      {

        const QImageSaveJPEGOptions *jpegOptions =
            dlgbox->jpegOptions();

        selectedPixelDepth = PIXEL_DEPTH_8U;

        imwrite_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
        imwrite_params.emplace_back(jpegOptions->jpegQuality());
        break;
      }
      }
    }

    QWaitCursor wait(parent);

    cv::Mat image;
    const cv::Mat mask = currentMask;


    if ( format == QImageSaveFLO ) {
      image = currentImage;
    }
    else if ( format == QImageSaveJPEG ) {

      if ( currentImage.channels() == 2 ) {  // optical flow
        flow2HSV(currentImage, image, 0, true);
      }
      else if ( currentImage.depth() == CV_8U ) {
        image = currentImage;
      }
      else {
        normalize_minmax(currentImage, image, 0, UINT8_MAX, mask);
        image.convertTo(image, CV_8U);
      }
    }
    else if ( format == QImageSavePNG ) {

      if ( currentImage.channels() == 2 ) {  // optical flow
        flow2HSV(currentImage, image, 0, true);
      }
      else if ( currentImage.depth() == selectedPixelDepth ) {
        image = currentImage;
      }
      else {
        double minv, maxv;
        get_data_range_for_pixel_depth(selectedPixelDepth, &minv, &maxv);
        normalize_minmax(currentImage, image, minv, maxv, mask);
        image.convertTo(image, selectedPixelDepth);
      }
    }
    else {  // format == QImageSaveTIFF
      if ( currentImage.channels() == 2 ) {  // optical flow
        flow2HSV(currentImage, image, 0, true);
      }
      else if ( selectedPixelDepth == PIXEL_DEPTH_NO_CHANGE || currentImage.depth() == selectedPixelDepth ) {
        image = currentImage;
      }
      else {
        double minv, maxv;
        get_data_range_for_pixel_depth(selectedPixelDepth, &minv, &maxv);
        normalize_minmax(currentImage, image, minv, maxv, mask);
        image.convertTo(image, selectedPixelDepth);
      }
    }

    if ( !save_image(image, embedAlphaMask ? mask : cv::noArray(), selectedFileName.toStdString(), imwrite_params) ) {
      QMessageBox::critical(parent, "ERROR", QString("save_image('%s') fails").arg(selectedFileName));
      continue;
    }

    settings.setValue(keyName, selectedFileName);

    if( currentProcesor && saveProcessorConfig ) {
      currentProcesor->save(QString("%1.cfg").arg(selectedFileName).toStdString(),
          QFileInfo(selectedFileName).completeBaseName().toStdString(),
          false);
    }


    return selectedFileName;
  }

  return QString();
}
