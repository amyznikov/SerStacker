/*
 * QImageFileEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageFileEditor.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/bad_pixels.h>
#include <core/debug.h>

QImageFileEditor::QImageFileEditor(QImageScene * scene, QWidget * parent) :
  Base(scene, parent)
{
  input_sequence_ = c_input_sequence::create();
  //input_sequence_->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence_->set_auto_apply_color_matrix(true);

  Base::_layout->addWidget(playControls =
      new QPlaySequenceControl(this));

  connect(playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);
}

QImageFileEditor::QImageFileEditor(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

void QImageFileEditor::setDebayerAlgorithm(DEBAYER_ALGORITHM algo)
{
  debayerAlgorithm_ = algo;
  Q_EMIT debayerAlgorithmChanged();
}

DEBAYER_ALGORITHM QImageFileEditor::debayerAlgorithm() const
{
  return debayerAlgorithm_;
}

void QImageFileEditor::setDropBadPixels(bool v)
{
  filterBadPixels_ = v;
  Q_EMIT dropBadPixelsChanged();
}

bool QImageFileEditor::dropBadPixels() const
{
  return filterBadPixels_;
}

void QImageFileEditor::setBadPixelsVariationThreshold(double v)
{
  badPixelsVariationThreshold_ = v;
  Q_EMIT badPixelsVariationThresholdChanged();
}

double QImageFileEditor::badPixelsVariationThreshold() const
{
  return badPixelsVariationThreshold_;
}


const c_input_sequence::sptr & QImageFileEditor::input_sequence() const
{
  return input_sequence_;
}

void QImageFileEditor::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  closeCurrentSequence();
  Base::setImage(image, mask, imageData, make_copy);
}

void QImageFileEditor::editImage(cv::InputArray image, cv::InputArray mask, bool make_copy)
{
  closeCurrentSequence();
  Base::editImage(image, mask, make_copy);
}

void QImageFileEditor::openImage(const std::string & pathfilename)
{
  input_sequence_->close(true);
  input_sequence_->add_source(pathfilename);
  setCurrentFileName(pathfilename.c_str());
  startDisplay();
}

void QImageFileEditor::openImage(const QString & pathfilename)
{
  openImage(pathfilename.toStdString());
}

void QImageFileEditor::openImages(const std::vector<std::string> & pathfilenames)
{
  input_sequence_->close(true);
  input_sequence_->add_sources(pathfilenames);
  setCurrentFileName(pathfilenames.empty() ? "" : pathfilenames.front().c_str());
  startDisplay();
}

void QImageFileEditor::openImages(const QStringList & pathfilenames)
{
  std::vector<std::string> std_names;

  std_names.reserve(pathfilenames.size());
  for ( const QString & s : pathfilenames ) {
    std_names.emplace_back(s.toStdString());
  }

  return openImages(std_names);
}

void QImageFileEditor::closeCurrentSequence()
{
  if( playControls->state() != QPlaySequenceControl::Stopped ) {
    playControls->setState(QPlaySequenceControl::Stopped);
  }

  playControls->hide();

  if( input_sequence_->is_open() ) {
    input_sequence_->close(true);
  }

  clear();
  setCurrentFileName("");
}


void QImageFileEditor::startDisplay()
{
  QWaitCursor wait(this);

  Base::setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  playControls->setState(QPlaySequenceControl::Stopped);
  playControls->hide();

  if ( !input_sequence_->is_open() && !input_sequence_->open() ) {
    return;
  }

  const int num_frames = input_sequence_->size();
  if ( num_frames < 1 ) {
    QMessageBox::critical(this, "ERROR",
        "Can not determine number of frames and seek range for given source.\n"
        "Image can not be displayed correctly, seems such input source is not supported");
    return;
  }

  if( num_frames > 1 ) {
    playControls->show();
    playControls->setSeekRange(0, num_frames - 1);
    playControls->setCurpos(0);
  }

  loadNextFrame();
}

void QImageFileEditor::onSeek(int pos)
{
  if ( input_sequence_ && input_sequence_->is_open() ) {
    if ( pos != input_sequence_->current_pos() ) {
      input_sequence_->seek(pos);
    }
    loadNextFrame();
  }
}

void QImageFileEditor::loadNextFrame()
{
  if ( input_sequence_ && input_sequence_->is_open() ) {

    c_input_source::sptr current_source =
        input_sequence_->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

      input_sequence_->read(_inputImage, &_inputMask);

//      CF_DEBUG("inputImage_: %dx%d channels=%d depth=%d inputMask_: %dx%d channels=%d depth=%d",
//          inputImage_.cols, inputImage_.rows, inputImage_.channels(), inputImage_.depth(),
//          inputMask_.cols, inputMask_.rows, inputMask_.channels(), inputMask_.depth());


      Q_EMIT onInputImageLoad(_inputImage, _inputMask,
          input_sequence_->colorid(),
          input_sequence_->bpp());


      if ( filterBadPixels_ && badPixelsVariationThreshold_ > 0 ) {

        if( !is_bayer_pattern(input_sequence_->colorid()) ) {
          median_filter_hot_pixels(_inputImage, badPixelsVariationThreshold_, false);
        }
        else if( !extract_bayer_planes(_inputImage, _inputImage, input_sequence_->colorid()) ) {
          CF_ERROR("ERROR: extract_bayer_planes() fails");
        }
        else {
          median_filter_hot_pixels(_inputImage, badPixelsVariationThreshold_, true);
          if( !nninterpolation(_inputImage, _inputImage, input_sequence_->colorid()) ) {
            CF_ERROR("nninterpolation() fails");
          }
        }
      }
      else if( is_bayer_pattern(input_sequence_->colorid()) ) {
        debayer(_inputImage, _inputImage, input_sequence_->colorid(),
            debayerAlgorithm_);
      }

      setCurrentFileName(current_source->filename().c_str());
      updateImage();
    }
  }
}

void QImageFileEditor::hideEvent(QHideEvent *e)
{
  closeCurrentSequence();
  Base::hideEvent(e);
}

